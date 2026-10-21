#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

double wrapAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool finiteQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(norm_squared) && norm_squared > 1.0e-12;
}

double quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  const double sin_yaw = 2.0 * (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(sin_yaw, cos_yaw);
}

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(0.5 * yaw);
  quaternion.w = std::cos(0.5 * yaw);
  return quaternion;
}

struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct PlanarTwist
{
  double vx_map{0.0};
  double vy_map{0.0};
  double yaw_rate{0.0};
};

}  // namespace

class MocapLocalizationAdapter : public rclcpp::Node
{
public:
  MocapLocalizationAdapter()
  : Node("mocap_localization_adapter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/rover_1021/pose");
    input_frame_ = declare_parameter<std::string>("input_frame", "world");
    odometry_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/odometry/filtered");
    robot_pose_topic_ = declare_parameter<std::string>("robot_pose_topic", "/robot_pose");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);

    map_from_world_x_ = declare_parameter<double>("map_from_world_x", 0.0);
    map_from_world_y_ = declare_parameter<double>("map_from_world_y", 0.0);
    map_from_world_z_ = declare_parameter<double>("map_from_world_z", 0.0);
    map_from_world_yaw_ = declare_parameter<double>("map_from_world_yaw", 0.0);

    rigid_body_to_base_x_ = declare_parameter<double>("rigid_body_to_base_x", 0.0);
    rigid_body_to_base_y_ = declare_parameter<double>("rigid_body_to_base_y", 0.0);
    rigid_body_to_base_z_ = declare_parameter<double>("rigid_body_to_base_z", 0.0);
    rigid_body_to_base_yaw_ = declare_parameter<double>("rigid_body_to_base_yaw", 0.0);

    position_variance_ = declare_parameter<double>("position_variance", 9.0e-6);
    yaw_variance_ = declare_parameter<double>("yaw_variance", 2.7415568e-5);
    unobserved_variance_ = declare_parameter<double>("unobserved_variance", 1.0e6);
    twist_filter_time_constant_ = declare_parameter<double>(
      "twist_filter_time_constant", 0.10);
    minimum_derivative_dt_ = declare_parameter<double>("minimum_derivative_dt", 0.001);
    maximum_derivative_dt_ = declare_parameter<double>("maximum_derivative_dt", 0.10);
    linear_velocity_variance_ = declare_parameter<double>(
      "linear_velocity_variance", 0.0025);
    yaw_rate_variance_ = declare_parameter<double>("yaw_rate_variance", 0.007615435);

    validateParameters();

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
    robot_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      robot_pose_topic_, 10);

    const auto mocap_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, mocap_qos,
      std::bind(&MocapLocalizationAdapter::onPose, this, std::placeholders::_1));

    if (publish_tf_) {
      static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
      dynamic_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      publishMapToOdomTransform();
    }

    RCLCPP_INFO(
      get_logger(),
      "Mocap adapter: %s [%s] -> %s [%s -> %s], robot pose %s, TF=%s",
      input_topic_.c_str(), input_frame_.c_str(), odometry_topic_.c_str(),
      map_frame_.c_str(), base_frame_.c_str(), robot_pose_topic_.c_str(),
      publish_tf_ ? "enabled" : "disabled");
  }

private:
  void validateParameters() const
  {
    const std::array<double, 16> numeric_parameters = {
      map_from_world_x_, map_from_world_y_, map_from_world_z_, map_from_world_yaw_,
      rigid_body_to_base_x_, rigid_body_to_base_y_, rigid_body_to_base_z_,
      rigid_body_to_base_yaw_, position_variance_, yaw_variance_, unobserved_variance_,
      twist_filter_time_constant_, minimum_derivative_dt_, maximum_derivative_dt_,
      linear_velocity_variance_, yaw_rate_variance_};
    for (const double value : numeric_parameters) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("mocap adapter parameters must be finite");
      }
    }
    if (input_topic_.empty() || odometry_topic_.empty() || robot_pose_topic_.empty() ||
      input_frame_.empty() || map_frame_.empty() || odom_frame_.empty() || base_frame_.empty())
    {
      throw std::invalid_argument("mocap adapter topic and frame parameters must not be empty");
    }
    if (position_variance_ < 0.0 || yaw_variance_ < 0.0 ||
      unobserved_variance_ < 0.0 || linear_velocity_variance_ < 0.0 ||
      yaw_rate_variance_ < 0.0 || twist_filter_time_constant_ < 0.0 ||
      minimum_derivative_dt_ <= 0.0 || maximum_derivative_dt_ <= minimum_derivative_dt_)
    {
      throw std::invalid_argument("mocap adapter variance and timing parameters are invalid");
    }
  }

  PlanarPose transformPose(const geometry_msgs::msg::PoseStamped & message) const
  {
    const double world_yaw = quaternionToYaw(message.pose.orientation);
    const double map_rigid_yaw = wrapAngle(map_from_world_yaw_ + world_yaw);

    const double map_cos = std::cos(map_from_world_yaw_);
    const double map_sin = std::sin(map_from_world_yaw_);
    const double rigid_x = map_from_world_x_ +
      map_cos * message.pose.position.x - map_sin * message.pose.position.y;
    const double rigid_y = map_from_world_y_ +
      map_sin * message.pose.position.x + map_cos * message.pose.position.y;

    const double rigid_cos = std::cos(map_rigid_yaw);
    const double rigid_sin = std::sin(map_rigid_yaw);

    PlanarPose pose;
    pose.x = rigid_x +
      rigid_cos * rigid_body_to_base_x_ - rigid_sin * rigid_body_to_base_y_;
    pose.y = rigid_y +
      rigid_sin * rigid_body_to_base_x_ + rigid_cos * rigid_body_to_base_y_;
    pose.z = map_from_world_z_ + message.pose.position.z + rigid_body_to_base_z_;
    pose.yaw = wrapAngle(map_rigid_yaw + rigid_body_to_base_yaw_);
    return pose;
  }

  void updateTwist(const rclcpp::Time & stamp, const PlanarPose & pose)
  {
    if (!previous_stamp_.has_value() || !previous_pose_.has_value()) {
      previous_stamp_ = stamp;
      previous_pose_ = pose;
      filtered_twist_ = PlanarTwist{};
      return;
    }

    const double dt = (stamp - *previous_stamp_).seconds();
    if (dt <= minimum_derivative_dt_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignored non-increasing or too-close mocap stamp: dt=%.6f s", dt);
      return;
    }

    if (dt > maximum_derivative_dt_) {
      filtered_twist_ = PlanarTwist{};
      previous_stamp_ = stamp;
      previous_pose_ = pose;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Mocap gap %.3f s exceeded derivative limit; twist was reset", dt);
      return;
    }

    const PlanarTwist raw_twist = {
      (pose.x - previous_pose_->x) / dt,
      (pose.y - previous_pose_->y) / dt,
      wrapAngle(pose.yaw - previous_pose_->yaw) / dt};
    const double alpha = twist_filter_time_constant_ <= 0.0 ?
      1.0 : dt / (twist_filter_time_constant_ + dt);
    filtered_twist_.vx_map += alpha * (raw_twist.vx_map - filtered_twist_.vx_map);
    filtered_twist_.vy_map += alpha * (raw_twist.vy_map - filtered_twist_.vy_map);
    filtered_twist_.yaw_rate += alpha * (raw_twist.yaw_rate - filtered_twist_.yaw_rate);

    previous_stamp_ = stamp;
    previous_pose_ = pose;
  }

  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (message->header.frame_id != input_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected mocap frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), input_frame_.c_str());
      return;
    }
    if (!finiteQuaternion(message->pose.orientation) ||
      !std::isfinite(message->pose.position.x) ||
      !std::isfinite(message->pose.position.y) ||
      !std::isfinite(message->pose.position.z))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected non-finite mocap pose");
      return;
    }

    const rclcpp::Time stamp(message->header.stamp);
    if (stamp.nanoseconds() <= 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected mocap pose with zero stamp");
      return;
    }
    if (previous_stamp_.has_value() && stamp <= *previous_stamp_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected non-increasing mocap timestamp");
      return;
    }

    const PlanarPose pose = transformPose(*message);
    updateTwist(stamp, pose);
    publishLocalization(message->header.stamp, pose);
  }

  void publishLocalization(
    const builtin_interfaces::msg::Time & stamp,
    const PlanarPose & pose)
  {
    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header.stamp = stamp;
    robot_pose.header.frame_id = map_frame_;
    robot_pose.pose.position.x = pose.x;
    robot_pose.pose.position.y = pose.y;
    robot_pose.pose.position.z = pose.z;
    robot_pose.pose.orientation = yawToQuaternion(pose.yaw);
    robot_pose_publisher_->publish(robot_pose);

    nav_msgs::msg::Odometry odometry;
    odometry.header = robot_pose.header;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose = robot_pose.pose;
    odometry.pose.covariance.fill(0.0);
    odometry.pose.covariance[0] = position_variance_;
    odometry.pose.covariance[7] = position_variance_;
    odometry.pose.covariance[14] = unobserved_variance_;
    odometry.pose.covariance[21] = unobserved_variance_;
    odometry.pose.covariance[28] = unobserved_variance_;
    odometry.pose.covariance[35] = yaw_variance_;

    const double body_cos = std::cos(pose.yaw);
    const double body_sin = std::sin(pose.yaw);
    odometry.twist.twist.linear.x =
      body_cos * filtered_twist_.vx_map + body_sin * filtered_twist_.vy_map;
    odometry.twist.twist.linear.y =
      -body_sin * filtered_twist_.vx_map + body_cos * filtered_twist_.vy_map;
    odometry.twist.twist.angular.z = filtered_twist_.yaw_rate;
    odometry.twist.covariance.fill(0.0);
    odometry.twist.covariance[0] = linear_velocity_variance_;
    odometry.twist.covariance[7] = linear_velocity_variance_;
    odometry.twist.covariance[14] = unobserved_variance_;
    odometry.twist.covariance[21] = unobserved_variance_;
    odometry.twist.covariance[28] = unobserved_variance_;
    odometry.twist.covariance[35] = yaw_rate_variance_;
    odometry_publisher_->publish(odometry);

    if (dynamic_tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = robot_pose.header;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = pose.x;
      transform.transform.translation.y = pose.y;
      transform.transform.translation.z = pose.z;
      transform.transform.rotation = robot_pose.pose.orientation;
      dynamic_tf_broadcaster_->sendTransform(transform);
    }
  }

  void publishMapToOdomTransform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(transform);
  }

  std::string input_topic_;
  std::string input_frame_;
  std::string odometry_topic_;
  std::string robot_pose_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{false};

  double map_from_world_x_{0.0};
  double map_from_world_y_{0.0};
  double map_from_world_z_{0.0};
  double map_from_world_yaw_{0.0};
  double rigid_body_to_base_x_{0.0};
  double rigid_body_to_base_y_{0.0};
  double rigid_body_to_base_z_{0.0};
  double rigid_body_to_base_yaw_{0.0};
  double position_variance_{9.0e-6};
  double yaw_variance_{2.7415568e-5};
  double unobserved_variance_{1.0e6};
  double twist_filter_time_constant_{0.10};
  double minimum_derivative_dt_{0.001};
  double maximum_derivative_dt_{0.10};
  double linear_velocity_variance_{0.0025};
  double yaw_rate_variance_{0.007615435};

  PlanarTwist filtered_twist_{};
  std::optional<rclcpp::Time> previous_stamp_;
  std::optional<PlanarPose> previous_pose_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapLocalizationAdapter>());
  rclcpp::shutdown();
  return 0;
}
