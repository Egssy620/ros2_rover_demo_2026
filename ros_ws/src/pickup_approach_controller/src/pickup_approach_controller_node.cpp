#include "pickup_approach_controller/controller_core.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace pickup_approach_controller
{
namespace
{

double quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  const double sin_yaw = 2.0 *
    (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 *
    (y * y + z * z);
  return std::atan2(sin_yaw, cos_yaw);
}

bool finiteQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm_squared =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::isfinite(norm_squared) && norm_squared > 1.0e-8;
}

double stampToSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

geometry_msgs::msg::Twist makeTwist(
  const double vx_body,
  const double vy_body,
  const double omega)
{
  geometry_msgs::msg::Twist message;
  message.linear.x = vx_body;
  message.linear.y = vy_body;
  message.angular.z = omega;
  return message;
}

bool positiveSemidefinitePlanarCovariance(const double covariance[3][3])
{
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      if (!std::isfinite(covariance[row][column])) {
        return false;
      }
    }
  }
  const double a = covariance[0][0];
  const double d = covariance[1][1];
  const double f = covariance[2][2];
  const double b = 0.5 * (covariance[0][1] + covariance[1][0]);
  const double c = 0.5 * (covariance[0][2] + covariance[2][0]);
  const double e = 0.5 * (covariance[1][2] + covariance[2][1]);
  constexpr double tolerance = 1.0e-12;
  const double determinant = a * d * f + 2.0 * b * c * e -
    a * e * e - d * c * c - f * b * b;
  return a >= -tolerance && d >= -tolerance && f >= -tolerance &&
         a * d - b * b >= -tolerance &&
         a * f - c * c >= -tolerance &&
         d * f - e * e >= -tolerance && determinant >= -tolerance;
}

}  // namespace

class PickupApproachControllerNode : public rclcpp::Node
{
public:
  PickupApproachControllerNode()
  : Node("pickup_approach_controller"),
    config_(readControllerConfig()),
    controller_(config_)
  {
    map_frame_ = declare_parameter<std::string>("frames.map", "map");
    base_frame_ = declare_parameter<std::string>("frames.base", "base_link");
    odometry_topic_ = declare_parameter<std::string>(
      "topics.odometry", "/odometry/filtered");
    goal_topic_ = declare_parameter<std::string>(
      "topics.goal", "/goal_pose");
    path_topic_ = declare_parameter<std::string>(
      "topics.path", "/pickup_approach/path");
    too_close_topic_ = declare_parameter<std::string>(
      "topics.too_close", "/scrap_pile_skirt_node/too_close");
    command_topic_ = declare_parameter<std::string>(
      "topics.command", "/pickup_approach/cmd_vel");
    debug_command_topic_ = declare_parameter<std::string>(
      "topics.debug_command", "/pickup_approach/debug_cmd_vel");
    result_topic_ = declare_parameter<std::string>(
      "topics.result", "/pickup_approach/result");
    status_topic_ = declare_parameter<std::string>(
      "topics.status", "/pickup_approach/status");

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    maximum_localization_age_ = declare_parameter<double>(
      "maximum_localization_age", 0.10);
    maximum_environment_age_ = declare_parameter<double>(
      "maximum_environment_age", 0.25);
    maximum_position_variance_ = declare_parameter<double>(
      "maximum_position_variance", 0.01);
    maximum_yaw_variance_ = declare_parameter<double>(
      "maximum_yaw_variance", 0.007615435);
    endpoint_position_tolerance_ = declare_parameter<double>(
      "endpoint_position_tolerance", 0.03);
    endpoint_yaw_tolerance_ = declare_parameter<double>(
      "endpoint_yaw_tolerance", 0.034906585);
    straight_path_spacing_ = declare_parameter<double>(
      "straight_path_spacing", 0.02);
    require_environment_guard_ = declare_parameter<bool>(
      "require_environment_guard", true);
    enable_straight_line_fallback_ = declare_parameter<bool>(
      "enable_straight_line_fallback", false);
    publish_motion_commands_ = declare_parameter<bool>(
      "publish_motion_commands", false);

    validateParameters();

    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(command_topic_, 10);
    debug_command_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(debug_command_topic_, 10);
    result_publisher_ = create_publisher<std_msgs::msg::String>(result_topic_, 10);
    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PickupApproachControllerNode::onOdometry, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      std::bind(&PickupApproachControllerNode::onGoal, this, std::placeholders::_1));
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      std::bind(&PickupApproachControllerNode::onPath, this, std::placeholders::_1));
    too_close_subscription_ = create_subscription<std_msgs::msg::Bool>(
      too_close_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PickupApproachControllerNode::onTooClose, this, std::placeholders::_1));

    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "~/set_enabled",
      std::bind(
        &PickupApproachControllerNode::onSetEnabled, this,
        std::placeholders::_1, std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "~/cancel",
      std::bind(
        &PickupApproachControllerNode::onCancel, this,
        std::placeholders::_1, std::placeholders::_2));

    const auto timer_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&PickupApproachControllerNode::onControlTimer, this));

    publishZeroCommand();
    publishStatus("DISABLED", "STARTUP_SAFE_STATE", true);
    RCLCPP_WARN(
      get_logger(),
      "Started with motion authority OFF. publish_motion_commands=%s, straight fallback=%s",
      publish_motion_commands_ ? "true" : "false",
      enable_straight_line_fallback_ ? "true" : "false");
  }

private:
  void validateParameters() const
  {
    const bool timing_valid =
      control_rate_hz_ > 0.0 && maximum_localization_age_ > 0.0 &&
      maximum_environment_age_ > 0.0 && maximum_position_variance_ > 0.0 &&
      maximum_yaw_variance_ > 0.0;
    const bool geometry_valid =
      endpoint_position_tolerance_ > 0.0 && endpoint_yaw_tolerance_ > 0.0 &&
      straight_path_spacing_ > 0.0;
    const bool limits_valid =
      config_.maximum_speed > 0.0 && config_.maximum_yaw_rate > 0.0 &&
      config_.maximum_acceleration > 0.0 &&
      config_.maximum_angular_acceleration > 0.0 &&
      config_.maximum_jerk > 0.0 && config_.maximum_angular_jerk > 0.0 &&
      config_.maximum_snap > 0.0 && config_.maximum_angular_snap > 0.0 &&
      config_.maximum_lateral_acceleration > 0.0 &&
      config_.path_deceleration > 0.0 && config_.maximum_progress_rate > 0.0;
    const bool arrival_valid =
      config_.arrival_position_tolerance > 0.0 &&
      config_.arrival_yaw_tolerance > 0.0 &&
      config_.arrival_window_seconds > 0.0 &&
      config_.arrival_minimum_window_seconds > 0.0 &&
      config_.arrival_minimum_window_seconds <= config_.arrival_window_seconds &&
      config_.arrival_minimum_samples >= 2 && config_.confidence_sigma >= 0.0 &&
      config_.maximum_position_drift_rate > 0.0 &&
      config_.maximum_yaw_drift_rate > 0.0;
    if (!timing_valid || !geometry_valid || !limits_valid || !arrival_valid) {
      throw std::invalid_argument("pickup approach parameters must be positive and consistent");
    }
  }

  ControllerConfig readControllerConfig()
  {
    ControllerConfig config;
    config.maximum_speed = declare_parameter<double>(
      "controller.maximum_speed", config.maximum_speed);
    config.maximum_yaw_rate = declare_parameter<double>(
      "controller.maximum_yaw_rate", config.maximum_yaw_rate);
    config.maximum_acceleration = declare_parameter<double>(
      "controller.maximum_acceleration", config.maximum_acceleration);
    config.maximum_angular_acceleration = declare_parameter<double>(
      "controller.maximum_angular_acceleration", config.maximum_angular_acceleration);
    config.maximum_jerk = declare_parameter<double>(
      "controller.maximum_jerk", config.maximum_jerk);
    config.maximum_angular_jerk = declare_parameter<double>(
      "controller.maximum_angular_jerk", config.maximum_angular_jerk);
    config.maximum_snap = declare_parameter<double>(
      "controller.maximum_snap", config.maximum_snap);
    config.maximum_angular_snap = declare_parameter<double>(
      "controller.maximum_angular_snap", config.maximum_angular_snap);
    config.maximum_lateral_acceleration = declare_parameter<double>(
      "controller.maximum_lateral_acceleration", config.maximum_lateral_acceleration);
    config.path_deceleration = declare_parameter<double>(
      "controller.path_deceleration", config.path_deceleration);
    config.maximum_longitudinal_recovery_speed = declare_parameter<double>(
      "controller.maximum_longitudinal_recovery_speed",
      config.maximum_longitudinal_recovery_speed);
    config.position_gain = declare_parameter<double>(
      "controller.position_gain", config.position_gain);
    config.yaw_gain = declare_parameter<double>(
      "controller.yaw_gain", config.yaw_gain);
    config.maximum_correction_speed = declare_parameter<double>(
      "controller.maximum_correction_speed", config.maximum_correction_speed);
    config.maximum_correction_yaw_rate = declare_parameter<double>(
      "controller.maximum_correction_yaw_rate", config.maximum_correction_yaw_rate);
    config.projection_search_back = declare_parameter<double>(
      "controller.projection_search_back", config.projection_search_back);
    config.projection_search_ahead = declare_parameter<double>(
      "controller.projection_search_ahead", config.projection_search_ahead);
    config.progress_correction_time_constant = declare_parameter<double>(
      "controller.progress_correction_time_constant",
      config.progress_correction_time_constant);
    config.maximum_progress_rate = declare_parameter<double>(
      "controller.maximum_progress_rate", config.maximum_progress_rate);
    config.curvature_feedforward_gain = declare_parameter<double>(
      "controller.curvature_feedforward_gain", config.curvature_feedforward_gain);
    config.velocity_shaping_gain = declare_parameter<double>(
      "controller.velocity_shaping_gain", config.velocity_shaping_gain);
    config.acceleration_shaping_gain = declare_parameter<double>(
      "controller.acceleration_shaping_gain", config.acceleration_shaping_gain);
    config.yaw_rate_shaping_gain = declare_parameter<double>(
      "controller.yaw_rate_shaping_gain", config.yaw_rate_shaping_gain);
    config.angular_acceleration_shaping_gain = declare_parameter<double>(
      "controller.angular_acceleration_shaping_gain",
      config.angular_acceleration_shaping_gain);
    config.arrival_position_tolerance = declare_parameter<double>(
      "arrival.position_tolerance", config.arrival_position_tolerance);
    config.arrival_yaw_tolerance = declare_parameter<double>(
      "arrival.yaw_tolerance", config.arrival_yaw_tolerance);
    config.arrival_command_speed_tolerance = declare_parameter<double>(
      "arrival.command_speed_tolerance", config.arrival_command_speed_tolerance);
    config.arrival_command_yaw_rate_tolerance = declare_parameter<double>(
      "arrival.command_yaw_rate_tolerance", config.arrival_command_yaw_rate_tolerance);
    config.arrival_window_seconds = declare_parameter<double>(
      "arrival.window_seconds", config.arrival_window_seconds);
    config.arrival_minimum_window_seconds = declare_parameter<double>(
      "arrival.minimum_window_seconds", config.arrival_minimum_window_seconds);
    config.arrival_minimum_samples = declare_parameter<int>(
      "arrival.minimum_samples", config.arrival_minimum_samples);
    config.confidence_sigma = declare_parameter<double>(
      "arrival.confidence_sigma", config.confidence_sigma);
    config.maximum_position_drift_rate = declare_parameter<double>(
      "arrival.maximum_position_drift_rate", config.maximum_position_drift_rate);
    config.maximum_yaw_drift_rate = declare_parameter<double>(
      "arrival.maximum_yaw_drift_rate", config.maximum_yaw_drift_rate);
    return config;
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    LocalizationSample sample;
    sample.stamp = stampToSeconds(message->header.stamp);
    const bool stamp_regressed = have_localization_stamp_ &&
      sample.stamp < last_localization_stamp_ - 1.0e-9;
    const bool stamp_advanced = !have_localization_stamp_ ||
      sample.stamp > last_localization_stamp_ + 1.0e-9;
    if (stamp_regressed || stamp_advanced) {
      ++localization_sequence_;
      last_localization_stamp_ = sample.stamp;
      have_localization_stamp_ = true;
    }
    sample.sequence = localization_sequence_;
    sample.pose.x = message->pose.pose.position.x;
    sample.pose.y = message->pose.pose.position.y;
    sample.pose.yaw = quaternionToYaw(message->pose.pose.orientation);
    sample.vx_body = message->twist.twist.linear.x;
    sample.vy_body = message->twist.twist.linear.y;
    sample.omega = message->twist.twist.angular.z;

    const auto & covariance = message->pose.covariance;
    sample.covariance[0][0] = covariance[0];
    sample.covariance[0][1] = covariance[1];
    sample.covariance[0][2] = covariance[5];
    sample.covariance[1][0] = covariance[6];
    sample.covariance[1][1] = covariance[7];
    sample.covariance[1][2] = covariance[11];
    sample.covariance[2][0] = covariance[30];
    sample.covariance[2][1] = covariance[31];
    sample.covariance[2][2] = covariance[35];

    const bool frame_valid =
      message->header.frame_id == map_frame_ && message->child_frame_id == base_frame_;
    const bool pose_valid =
      std::isfinite(sample.pose.x) && std::isfinite(sample.pose.y) &&
      std::isfinite(sample.pose.yaw) && finiteQuaternion(message->pose.pose.orientation);
    const bool covariance_valid =
      positiveSemidefinitePlanarCovariance(sample.covariance) &&
      sample.covariance[0][0] <= maximum_position_variance_ &&
      sample.covariance[1][1] <= maximum_position_variance_ &&
      sample.covariance[2][2] <= maximum_yaw_variance_;
    sample.healthy = frame_valid && pose_valid && covariance_valid &&
      std::isfinite(sample.stamp) && sample.stamp > 0.0 && !stamp_regressed;

    latest_localization_ = sample;
    have_localization_ = true;
    localization_receive_time_ = now();
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!enabled_) {
      RCLCPP_WARN(get_logger(), "Ignored pickup goal because motion authority is OFF");
      publishStatus("DISABLED", "GOAL_WITHOUT_AUTHORITY", true);
      return;
    }
    if (message->header.frame_id != map_frame_ ||
      !std::isfinite(message->pose.position.x) ||
      !std::isfinite(message->pose.position.y) ||
      !finiteQuaternion(message->pose.orientation))
    {
      RCLCPP_ERROR(get_logger(), "Rejected pickup goal: invalid map-frame pose");
      publishStatus("WAIT_GOAL", "GOAL_INVALID", true);
      return;
    }

    goal_.x = message->pose.position.x;
    goal_.y = message->pose.position.y;
    goal_.yaw = quaternionToYaw(message->pose.orientation);
    have_goal_ = true;
    arrival_result_sent_ = false;
    last_arrival_decision_ = "COLLECTING";
    last_arrival_reason_ = "WINDOW_NOT_READY";
    controller_.clearPath();
    publishZeroCommand();

    if (enable_straight_line_fallback_ && have_localization_ && latest_localization_.healthy) {
      std::string error;
      const auto path = makeStraightPath(
        latest_localization_.pose, goal_, straight_path_spacing_);
      if (controller_.setPath(path, &error)) {
        publishResult("planned_straight_fallback");
        publishStatus("READY", "STRAIGHT_FALLBACK_PATH", true);
        return;
      }
      RCLCPP_ERROR(get_logger(), "Failed to create fallback path: %s", error.c_str());
    }
    publishStatus("WAIT_PATH", "GOAL_ACCEPTED", true);
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (!have_goal_) {
      RCLCPP_WARN(get_logger(), "Rejected path because no pickup goal is active");
      publishStatus("WAIT_GOAL", "PATH_WITHOUT_GOAL", true);
      return;
    }
    if (message->header.frame_id != map_frame_ || message->poses.size() < 2U) {
      RCLCPP_ERROR(get_logger(), "Rejected path: frame or point count is invalid");
      publishStatus("WAIT_PATH", "PATH_INVALID", true);
      return;
    }

    std::vector<Pose2D> path;
    path.reserve(message->poses.size());
    for (const auto & stamped_pose : message->poses) {
      if ((!stamped_pose.header.frame_id.empty() &&
        stamped_pose.header.frame_id != map_frame_) ||
        !std::isfinite(stamped_pose.pose.position.x) ||
        !std::isfinite(stamped_pose.pose.position.y) ||
        !finiteQuaternion(stamped_pose.pose.orientation))
      {
        RCLCPP_ERROR(get_logger(), "Rejected path: one or more poses are invalid");
        publishStatus("WAIT_PATH", "PATH_INVALID", true);
        return;
      }
      path.push_back(Pose2D{
        stamped_pose.pose.position.x,
        stamped_pose.pose.position.y,
        quaternionToYaw(stamped_pose.pose.orientation)});
    }

    const double endpoint_distance = std::hypot(
      path.back().x - goal_.x, path.back().y - goal_.y);
    const double endpoint_yaw_error = std::abs(wrapAngle(path.back().yaw - goal_.yaw));
    if (endpoint_distance > endpoint_position_tolerance_ ||
      endpoint_yaw_error > endpoint_yaw_tolerance_)
    {
      RCLCPP_ERROR(
        get_logger(), "Rejected path: endpoint does not match pickup goal");
      publishStatus("WAIT_PATH", "PATH_ENDPOINT_MISMATCH", true);
      return;
    }

    std::string error;
    if (!controller_.setPath(path, &error)) {
      RCLCPP_ERROR(get_logger(), "Rejected path: %s", error.c_str());
      publishStatus("WAIT_PATH", "PATH_GEOMETRY_INVALID", true);
      return;
    }
    arrival_result_sent_ = false;
    last_arrival_decision_ = "COLLECTING";
    last_arrival_reason_ = "WINDOW_NOT_READY";
    publishResult("planned");
    publishStatus(enabled_ ? "READY" : "DISABLED", "PATH_ACCEPTED", true);
  }

  void onTooClose(const std_msgs::msg::Bool::SharedPtr message)
  {
    too_close_ = message->data;
    have_environment_guard_ = true;
    environment_receive_time_ = now();
  }

  void onSetEnabled(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    enabled_ = request->data;
    if (!enabled_) {
      controller_.clearPath();
      have_goal_ = false;
      arrival_result_sent_ = false;
      last_arrival_decision_ = "COLLECTING";
      last_arrival_reason_ = "AUTHORITY_RELEASED";
      publishZeroCommand();
      publishResult("authority_released");
      publishStatus("DISABLED", "AUTHORITY_RELEASED", true);
      response->message = "Pickup approach motion authority released; active goal cleared";
    } else {
      controller_.clearPath();
      have_goal_ = false;
      arrival_result_sent_ = false;
      last_arrival_decision_ = "COLLECTING";
      last_arrival_reason_ = "WINDOW_NOT_READY";
      publishZeroCommand();
      publishResult("authority_acquired");
      publishStatus("WAIT_GOAL", "AUTHORITY_ACQUIRED", true);
      response->message = publish_motion_commands_ ?
        "Pickup approach authority acquired; waiting for a new goal" :
        "Authority acquired in calculation-only mode; waiting for a new goal";
    }
    response->success = true;
  }

  void onCancel(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    controller_.clearPath();
    have_goal_ = false;
    arrival_result_sent_ = false;
    last_arrival_decision_ = "COLLECTING";
    last_arrival_reason_ = "CANCELLED";
    publishZeroCommand();
    publishResult("cancelled");
    publishStatus(enabled_ ? "WAIT_GOAL" : "DISABLED", "CANCELLED", true);
    response->success = true;
    response->message = "Active pickup approach goal cancelled";
  }

  void onControlTimer()
  {
    const rclcpp::Time current_time = now();
    const double localization_receive_age = have_localization_ ?
      (current_time - localization_receive_time_).seconds() :
      std::numeric_limits<double>::infinity();
    const double localization_stamp_age = have_localization_ ?
      current_time.seconds() - latest_localization_.stamp :
      std::numeric_limits<double>::infinity();
    const bool localization_fresh = have_localization_ &&
      localization_receive_age >= 0.0 &&
      localization_receive_age <= maximum_localization_age_ &&
      localization_stamp_age >= 0.0 &&
      localization_stamp_age <= maximum_localization_age_;
    const double environment_age = have_environment_guard_ ?
      (current_time - environment_receive_time_).seconds() :
      std::numeric_limits<double>::infinity();
    const bool environment_fresh = !require_environment_guard_ ||
      (have_environment_guard_ && environment_age >= 0.0 &&
      environment_age <= maximum_environment_age_);

    LocalizationSample localization = latest_localization_;
    localization.healthy = localization.healthy && localization_fresh;
    const bool effective_too_close = have_environment_guard_ ?
      too_close_ : require_environment_guard_;
    const ControllerOutput output = controller_.step(
      current_time.seconds(), localization, enabled_, environment_fresh, effective_too_close);

    debug_command_publisher_->publish(
      makeTwist(output.vx_body, output.vy_body, output.omega));
    if (publish_motion_commands_ && enabled_) {
      command_publisher_->publish(
        makeTwist(output.vx_body, output.vy_body, output.omega));
    } else {
      publishZeroCommand();
    }

    if (output.arrival_pulse && !arrival_result_sent_) {
      publishResult("arrived");
      arrival_result_sent_ = true;
    }
    last_arrival_decision_ = output.arrival_decision;
    last_arrival_reason_ = output.arrival_reason;
    if (enabled_ && !have_goal_) {
      publishStatus("WAIT_GOAL", "NO_GOAL", false);
    } else {
      publishStatus(output.state, output.reason, false);
    }
  }

  void publishZeroCommand()
  {
    command_publisher_->publish(makeTwist(0.0, 0.0, 0.0));
  }

  void publishResult(const std::string & result)
  {
    std_msgs::msg::String message;
    message.data = result;
    result_publisher_->publish(message);
  }

  void publishStatus(
    const std::string & state,
    const std::string & reason,
    const bool force)
  {
    std::ostringstream stream;
    stream << state << "|reason=" << reason
           << "|enabled=" << (enabled_ ? "true" : "false")
           << "|motion_output=" << (publish_motion_commands_ ? "true" : "false")
           << "|goal=" << (have_goal_ ? "true" : "false")
           << "|path=" << (controller_.hasPath() ? "true" : "false")
           << "|arrival=" << last_arrival_decision_
           << "|arrival_reason=" << last_arrival_reason_;
    const std::string status = stream.str();
    const rclcpp::Time current_time = now();
    const bool heartbeat_due =
      last_status_publish_time_.nanoseconds() == 0 ||
      (current_time - last_status_publish_time_).seconds() >= 1.0;
    if (!force && status == last_status_ && !heartbeat_due) {
      return;
    }
    std_msgs::msg::String message;
    message.data = status;
    status_publisher_->publish(message);
    last_status_ = status;
    last_status_publish_time_ = current_time;
  }

  ControllerConfig config_;
  ControllerCore controller_;
  std::string map_frame_;
  std::string base_frame_;
  std::string odometry_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string too_close_topic_;
  std::string command_topic_;
  std::string debug_command_topic_;
  std::string result_topic_;
  std::string status_topic_;
  double control_rate_hz_{50.0};
  double maximum_localization_age_{0.10};
  double maximum_environment_age_{0.25};
  double maximum_position_variance_{0.01};
  double maximum_yaw_variance_{0.007615435};
  double endpoint_position_tolerance_{0.03};
  double endpoint_yaw_tolerance_{0.034906585};
  double straight_path_spacing_{0.02};
  bool require_environment_guard_{true};
  bool enable_straight_line_fallback_{false};
  bool publish_motion_commands_{false};
  bool enabled_{false};
  bool have_goal_{false};
  bool have_localization_{false};
  bool have_environment_guard_{false};
  bool too_close_{true};
  bool arrival_result_sent_{false};
  Pose2D goal_{};
  LocalizationSample latest_localization_{};
  std::uint64_t localization_sequence_{0U};
  double last_localization_stamp_{0.0};
  bool have_localization_stamp_{false};
  rclcpp::Time localization_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time environment_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_publish_time_{0, 0, RCL_ROS_TIME};
  std::string last_status_;
  std::string last_arrival_decision_{"COLLECTING"};
  std::string last_arrival_reason_{"WINDOW_NOT_READY"};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr debug_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr too_close_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace pickup_approach_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pickup_approach_controller::PickupApproachControllerNode>());
  rclcpp::shutdown();
  return 0;
}
