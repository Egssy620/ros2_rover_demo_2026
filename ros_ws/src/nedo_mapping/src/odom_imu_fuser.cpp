#include <array>
#include <mutex>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// livox_ros_driver2 outputs linear_acceleration in g (not m/s²).
// Must match imuGravity in params_mid360.yaml.
static constexpr double G_TO_MPS2 = 9.80511;

// Injects the EKF-fused orientation (from /odometry/filtered) into every raw
// IMU message and converts linear_acceleration from g to m/s².
// LIO-SAM uses acc+gyro for GTSAM preintegration; orientation is used only for
// initial heading — supplying the wheel-odom-corrected value improves yaw init.
//
// The injected orientation is made RELATIVE to the first received EKF
// orientation. This prevents accumulated wheel-odometry yaw (e.g. 152° from a
// previous session) from shifting LIO-SAM's initial coordinate frame, which
// would cause ICP to diverge.
//
// When no EKF message has arrived yet, publishes with orientation_covariance[0]
// = -1 (ROS convention: orientation unknown), so LIO-SAM ignores orientation.
class OdomImuFuser : public rclcpp::Node
{
public:
  OdomImuFuser()
  : Node("odom_imu_fuser")
  {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { onImu(msg); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom_filtered", rclcpp::SystemDefaultsQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); });

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "imu_fused", rclcpp::SensorDataQoS());
  }

private:
  // Returns q_a_inv * q_b  (i.e. rotation of b relative to a)
  static geometry_msgs::msg::Quaternion relativeQ(
    const geometry_msgs::msg::Quaternion & q_a,
    const geometry_msgs::msg::Quaternion & q_b)
  {
    // q_a_inv = (w_a, -x_a, -y_a, -z_a)  for a unit quaternion
    // result = q_a_inv * q_b
    geometry_msgs::msg::Quaternion r;
    r.w =  q_a.w * q_b.w + q_a.x * q_b.x + q_a.y * q_b.y + q_a.z * q_b.z;
    r.x =  q_a.w * q_b.x - q_a.x * q_b.w - q_a.y * q_b.z + q_a.z * q_b.y;
    r.y =  q_a.w * q_b.y + q_a.x * q_b.z - q_a.y * q_b.w - q_a.z * q_b.x;
    r.z =  q_a.w * q_b.z - q_a.x * q_b.y + q_a.y * q_b.x - q_a.z * q_b.w;
    return r;
  }

  void onOdom(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_initial_q_) {
      initial_q_ = msg->pose.pose.orientation;
      has_initial_q_ = true;
    }

    // Publish orientation relative to the first EKF sample so that LIO-SAM
    // always starts at identity, regardless of accumulated wheel-odom yaw.
    latest_q_ = relativeQ(initial_q_, msg->pose.pose.orientation);

    // pose.covariance is row-major 6×6; diagonal indices for roll/pitch/yaw: 21,28,35
    orient_cov_[0] = msg->pose.covariance[21];  // roll-roll
    orient_cov_[4] = msg->pose.covariance[28];  // pitch-pitch
    orient_cov_[8] = msg->pose.covariance[35];  // yaw-yaw
    has_orientation_ = true;
  }

  void onImu(sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu out = *msg;
    out.header.stamp = this->now();  // re-stamp: align to ROS clock

    // livox_ros_driver2 reports linear_acceleration in g, not m/s².
    // LIO-SAM's GTSAM preintegration (MakeSharedU) expects m/s².
    out.linear_acceleration.x *= G_TO_MPS2;
    out.linear_acceleration.y *= G_TO_MPS2;
    out.linear_acceleration.z *= G_TO_MPS2;

    std::lock_guard<std::mutex> lock(mutex_);
    if (has_orientation_) {
      out.orientation = latest_q_;
      out.orientation_covariance = orient_cov_;
    } else {
      out.orientation_covariance[0] = -1.0;  // signal: orientation unknown
      // Livox driver leaves orientation as {0,0,0,0}; identity prevents
      // LIO-SAM's imuConverter from calling rclcpp::shutdown() on norm < 0.1.
      out.orientation.w = 1.0;
      out.orientation.x = 0.0;
      out.orientation.y = 0.0;
      out.orientation.z = 0.0;
    }

    imu_pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::mutex mutex_;
  geometry_msgs::msg::Quaternion initial_q_{};  // first EKF orientation (reference)
  geometry_msgs::msg::Quaternion latest_q_{};
  std::array<double, 9> orient_cov_{};
  bool has_initial_q_{false};
  bool has_orientation_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomImuFuser>());
  rclcpp::shutdown();
  return 0;
}
