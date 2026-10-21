// rover_odo_to_twist_node.cpp
//
// 目的
// - メカナムローバの車輪オドメトリ /rover_odo (geometry_msgs/TwistStamped) を
//   robot_localization (EKF) が購読できる geometry_msgs/TwistWithCovarianceStamped
//   に変換する。
// - robot_localization の入力型は odomN=nav_msgs/Odometry, poseN=PoseWithCovarianceStamped,
//   twistN=TwistWithCovarianceStamped, imuN=sensor_msgs/Imu の 4 種のみ。TwistStamped は
//   remap しても型が違うため接続が成立せず、エラーも出ずに無入力になる。
//
// 設計上の注意
// - stamp は「元メッセージの header.stamp をそのまま使う」のが原則 (sensor_timeout 判定と
//   IMU との時刻同期がここで決まる)。ただし stamp が 0 (未設定) の場合のみ now() で補う。
// - frame_id は base_link。twistN 入力は機体座標系の速度として解釈される。
// - 共分散は有効化する成分に必ず非ゼロを入れる。0 のままだと EKF のゲインが発散する。
// - kvx/kvy/kth はスケール較正係数。rover_odom_to_pose2d_node と同じ意味・同じ値を使うこと
//   (両者で違う値を使うと odom 系の縮尺がずれる)。
//
// パラメータ
//   input_topic        : 購読する TwistStamped (default /rover_odo)
//   output_topic       : 出力する TwistWithCovarianceStamped (default /rover_odo_cov)
//   frame_id           : 出力ヘッダの frame (default base_link)
//   kvx, kvy, kth      : vx, vy, vyaw のスケール較正係数 (default 1.0)
//   linear_x_variance  : vx の分散 [(m/s)^2]
//   linear_y_variance  : vy の分散 [(m/s)^2]
//   angular_z_variance : vyaw の分散 [(rad/s)^2]

#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

class RoverOdoToTwistNode : public rclcpp::Node
{
public:
  RoverOdoToTwistNode() : Node("rover_odo_to_twist_node")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/rover_odo");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/rover_odo_cov");
    frame_id_     = this->declare_parameter<std::string>("frame_id", "base_link");

    kvx_ = this->declare_parameter<double>("kvx", 1.0);
    kvy_ = this->declare_parameter<double>("kvy", 1.0);
    kth_ = this->declare_parameter<double>("kth", 1.0);

    // 実測オドメなので cmd_vel 疑似速度より締める (σ≈0.1 m/s 相当)。
    // メカナムは横滑りが乗るので vy は vx より緩め。実機のスリップを見て調整。
    linear_x_variance_  = this->declare_parameter<double>("linear_x_variance", 0.01);   // [(m/s)^2]
    linear_y_variance_  = this->declare_parameter<double>("linear_y_variance", 0.02);   // [(m/s)^2]
    angular_z_variance_ = this->declare_parameter<double>("angular_z_variance", 0.01);  // [(rad/s)^2]

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      output_topic_, rclcpp::SystemDefaultsQoS());

    // 入力は best-effort (SensorDataQoS)。ドライバが reliable でも購読は成立する。
    // rover_odom_to_pose2d_node と同じ QoS。
    subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RoverOdoToTwistNode::rover_odo_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "rover_odo_to_twist: %s -> %s (frame=%s, k=[%.3f %.3f %.3f], "
                "var vx=%.4f vy=%.4f vyaw=%.4f)",
                input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(),
                kvx_, kvy_, kth_,
                linear_x_variance_, linear_y_variance_, angular_z_variance_);
  }

private:
  void rover_odo_callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped out;

    // 元の stamp を保持する。未設定 (0) のときだけ受信時刻で補う。
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      out.header.stamp = this->now();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "%s の header.stamp が 0。受信時刻で代替中 (EKF の時刻同期精度が落ちる)",
                           input_topic_.c_str());
    } else {
      out.header.stamp = msg->header.stamp;
    }
    out.header.frame_id = frame_id_;

    out.twist.twist.linear.x  = kvx_ * msg->twist.linear.x;
    out.twist.twist.linear.y  = kvy_ * msg->twist.linear.y;
    out.twist.twist.angular.z = kth_ * msg->twist.angular.z;

    // 6x6 (vx,vy,vz,vroll,vpitch,vyaw)。対角のみ設定。
    std::fill(out.twist.covariance.begin(), out.twist.covariance.end(), 0.0);
    out.twist.covariance[0]  = linear_x_variance_;   // vx
    out.twist.covariance[7]  = linear_y_variance_;   // vy
    out.twist.covariance[35] = angular_z_variance_;  // vyaw
    // 使わない vz/vroll/vpitch は EKF 側 config で false にするため値は無視される。

    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double kvx_;
  double kvy_;
  double kth_;
  double linear_x_variance_;
  double linear_y_variance_;
  double angular_z_variance_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverOdoToTwistNode>());
  rclcpp::shutdown();
  return 0;
}
