#include "pickup_approach_controller/controller_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pickup_approach_controller
{
namespace
{

constexpr double kSmall = 1.0e-9;

double clampValue(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

void limitVector(double & x, double & y, const double limit)
{
  const double magnitude = std::hypot(x, y);
  if (magnitude > limit && magnitude > kSmall) {
    const double scale = limit / magnitude;
    x *= scale;
    y *= scale;
  }
}

void limitJerkBeforeAccelerationBoundary(
  double & jerk_x,
  double & jerk_y,
  const double acceleration_x,
  const double acceleration_y,
  const double maximum_acceleration,
  const double maximum_snap,
  const double dt)
{
  const double acceleration_norm = std::hypot(acceleration_x, acceleration_y);
  if (acceleration_norm <= kSmall) {
    return;
  }
  const double direction_x = acceleration_x / acceleration_norm;
  const double direction_y = acceleration_y / acceleration_norm;
  const double outward_jerk = direction_x * jerk_x + direction_y * jerk_y;
  const double acceleration_margin = std::max(maximum_acceleration - acceleration_norm, 0.0);
  const double continuous_limit = std::sqrt(2.0 * maximum_snap * acceleration_margin);
  const double discrete_margin = maximum_snap * dt;
  const double maximum_outward_jerk = std::max(continuous_limit - discrete_margin, 0.0);
  if (outward_jerk > maximum_outward_jerk) {
    jerk_x -= (outward_jerk - maximum_outward_jerk) * direction_x;
    jerk_y -= (outward_jerk - maximum_outward_jerk) * direction_y;
  }
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  const double upper = *middle;
  if ((values.size() % 2U) != 0U) {
    return upper;
  }
  const auto lower_middle = std::max_element(values.begin(), middle);
  return 0.5 * (upper + *lower_middle);
}

double robustSigma(const std::vector<double> & values)
{
  if (values.size() < 2U) {
    return 0.0;
  }
  const double center = median(values);
  std::vector<double> deviations;
  deviations.reserve(values.size());
  for (const double value : values) {
    deviations.push_back(std::abs(value - center));
  }
  return 1.4826 * median(deviations);
}

double circularMean(const std::vector<double> & values)
{
  double sine_sum = 0.0;
  double cosine_sum = 0.0;
  for (const double value : values) {
    sine_sum += std::sin(value);
    cosine_sum += std::cos(value);
  }
  return std::atan2(sine_sum, cosine_sum);
}

double largestPlanarVariance(const double covariance[3][3])
{
  const double a = covariance[0][0];
  const double d = covariance[1][1];
  const double b = 0.5 * (covariance[0][1] + covariance[1][0]);
  const double discriminant = std::sqrt(std::max(
    (a - d) * (a - d) + 4.0 * b * b, 0.0));
  return std::max(0.5 * (a + d + discriminant), 0.0);
}

bool finitePose(const Pose2D & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

}  // namespace

double wrapAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

ControllerCore::ControllerCore(const ControllerConfig & config)
: config_(config)
{
}

bool ControllerCore::setPath(const std::vector<Pose2D> & poses, std::string * error)
{
  if (poses.size() < 2U) {
    if (error != nullptr) {
      *error = "経路には2点以上が必要です";
    }
    return false;
  }
  for (const auto & pose : poses) {
    if (!finitePose(pose)) {
      if (error != nullptr) {
        *error = "経路に有限でない姿勢が含まれています";
      }
      return false;
    }
  }

  std::vector<PathPoint> candidate(poses.size());
  candidate.front().pose = poses.front();
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const double segment = std::hypot(
      poses[i].x - poses[i - 1U].x,
      poses[i].y - poses[i - 1U].y);
    if (segment <= kSmall) {
      if (error != nullptr) {
        *error = "経路に重複した連続点が含まれています";
      }
      return false;
    }
    candidate[i].pose = poses[i];
    candidate[i].pose.yaw = candidate[i - 1U].pose.yaw +
      wrapAngle(poses[i].yaw - candidate[i - 1U].pose.yaw);
    candidate[i].s = candidate[i - 1U].s + segment;
  }

  std::vector<double> tangent_yaw(candidate.size(), 0.0);
  for (std::size_t i = 0; i < candidate.size(); ++i) {
    const std::size_t before = (i == 0U) ? 0U : i - 1U;
    const std::size_t after = (i + 1U >= candidate.size()) ? candidate.size() - 1U : i + 1U;
    double dx = candidate[after].pose.x - candidate[before].pose.x;
    double dy = candidate[after].pose.y - candidate[before].pose.y;
    const double norm = std::hypot(dx, dy);
    if (norm <= kSmall) {
      if (error != nullptr) {
        *error = "経路の折返し点で接線方向を定義できません";
      }
      return false;
    }
    dx /= norm;
    dy /= norm;
    candidate[i].tangent_x = dx;
    candidate[i].tangent_y = dy;
    tangent_yaw[i] = std::atan2(dy, dx);
    if (i > 0U) {
      tangent_yaw[i] = tangent_yaw[i - 1U] + wrapAngle(tangent_yaw[i] - tangent_yaw[i - 1U]);
    }
  }

  for (std::size_t i = 0; i < candidate.size(); ++i) {
    const std::size_t before = (i == 0U) ? 0U : i - 1U;
    const std::size_t after = (i + 1U >= candidate.size()) ? candidate.size() - 1U : i + 1U;
    const double ds = std::max(candidate[after].s - candidate[before].s, kSmall);
    candidate[i].curvature = (tangent_yaw[after] - tangent_yaw[before]) / ds;
    candidate[i].yaw_derivative =
      (candidate[after].pose.yaw - candidate[before].pose.yaw) / ds;

    double speed_limit = config_.maximum_speed;
    if (std::abs(candidate[i].curvature) > kSmall) {
      speed_limit = std::min(
        speed_limit,
        std::sqrt(config_.maximum_lateral_acceleration / std::abs(candidate[i].curvature)));
    }
    if (std::abs(candidate[i].yaw_derivative) > kSmall) {
      speed_limit = std::min(
        speed_limit,
        config_.maximum_yaw_rate / std::abs(candidate[i].yaw_derivative));
    }
    candidate[i].speed_limit = speed_limit;
  }

  trajectory_ = std::move(candidate);
  path_progress_ = 0.0;
  last_sequence_ = 0U;
  have_sequence_ = false;
  have_step_time_ = false;
  arrived_ = false;
  arrival_window_.clear();
  hardStop();
  return true;
}

void ControllerCore::clearPath()
{
  trajectory_.clear();
  arrival_window_.clear();
  path_progress_ = 0.0;
  arrived_ = false;
  have_sequence_ = false;
  have_step_time_ = false;
  hardStop();
}

bool ControllerCore::hasPath() const
{
  return trajectory_.size() >= 2U;
}

const std::vector<PathPoint> & ControllerCore::trajectory() const
{
  return trajectory_;
}

void ControllerCore::hardStop()
{
  world_velocity_x_ = 0.0;
  world_velocity_y_ = 0.0;
  world_acceleration_x_ = 0.0;
  world_acceleration_y_ = 0.0;
  world_jerk_x_ = 0.0;
  world_jerk_y_ = 0.0;
  omega_command_ = 0.0;
  angular_acceleration_ = 0.0;
  angular_jerk_ = 0.0;
}

double ControllerCore::projectProgress(const Pose2D & pose) const
{
  double best_distance_squared = std::numeric_limits<double>::infinity();
  double best_progress = path_progress_;
  const double search_start = have_sequence_ ?
    std::max(0.0, path_progress_ - config_.projection_search_back) : 0.0;
  const double search_end = have_sequence_ ?
    std::min(trajectory_.back().s, path_progress_ + config_.projection_search_ahead) :
    trajectory_.back().s;
  for (std::size_t i = 0; i + 1U < trajectory_.size(); ++i) {
    if (trajectory_[i + 1U].s < search_start || trajectory_[i].s > search_end) {
      continue;
    }
    const double ax = trajectory_[i].pose.x;
    const double ay = trajectory_[i].pose.y;
    const double bx = trajectory_[i + 1U].pose.x;
    const double by = trajectory_[i + 1U].pose.y;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double length_squared = dx * dx + dy * dy;
    const double ratio = clampValue(
      ((pose.x - ax) * dx + (pose.y - ay) * dy) / length_squared,
      0.0, 1.0);
    const double px = ax + ratio * dx;
    const double py = ay + ratio * dy;
    const double distance_squared = (pose.x - px) * (pose.x - px) +
      (pose.y - py) * (pose.y - py);
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_progress = trajectory_[i].s + ratio * (trajectory_[i + 1U].s - trajectory_[i].s);
    }
  }
  return clampValue(best_progress, search_start, search_end);
}

PathPoint ControllerCore::interpolate(const double progress) const
{
  if (progress <= 0.0) {
    return trajectory_.front();
  }
  if (progress >= trajectory_.back().s) {
    return trajectory_.back();
  }
  const auto upper = std::upper_bound(
    trajectory_.begin(), trajectory_.end(), progress,
    [](const double value, const PathPoint & point) {return value < point.s;});
  const std::size_t after = static_cast<std::size_t>(std::distance(trajectory_.begin(), upper));
  const std::size_t before = after - 1U;
  const double ratio = (progress - trajectory_[before].s) /
    (trajectory_[after].s - trajectory_[before].s);

  PathPoint result;
  result.s = progress;
  result.pose.x = trajectory_[before].pose.x + ratio *
    (trajectory_[after].pose.x - trajectory_[before].pose.x);
  result.pose.y = trajectory_[before].pose.y + ratio *
    (trajectory_[after].pose.y - trajectory_[before].pose.y);
  result.pose.yaw = trajectory_[before].pose.yaw + ratio *
    (trajectory_[after].pose.yaw - trajectory_[before].pose.yaw);
  result.tangent_x = trajectory_[before].tangent_x + ratio *
    (trajectory_[after].tangent_x - trajectory_[before].tangent_x);
  result.tangent_y = trajectory_[before].tangent_y + ratio *
    (trajectory_[after].tangent_y - trajectory_[before].tangent_y);
  const double tangent_norm = std::hypot(result.tangent_x, result.tangent_y);
  if (tangent_norm > kSmall) {
    result.tangent_x /= tangent_norm;
    result.tangent_y /= tangent_norm;
  } else {
    result.tangent_x = trajectory_[before].tangent_x;
    result.tangent_y = trajectory_[before].tangent_y;
  }
  result.curvature = trajectory_[before].curvature + ratio *
    (trajectory_[after].curvature - trajectory_[before].curvature);
  result.yaw_derivative = trajectory_[before].yaw_derivative + ratio *
    (trajectory_[after].yaw_derivative - trajectory_[before].yaw_derivative);
  result.speed_limit = trajectory_[before].speed_limit + ratio *
    (trajectory_[after].speed_limit - trajectory_[before].speed_limit);
  return result;
}

ControllerOutput ControllerCore::step(
  const double now_seconds,
  const LocalizationSample & localization,
  const bool motion_authority,
  const bool environment_valid,
  const bool too_close)
{
  ControllerOutput output;
  output.path_progress = path_progress_;

  if (!motion_authority) {
    hardStop();
    have_step_time_ = false;
    arrival_window_.clear();
    output.state = "DISABLED";
    output.reason = "NO_MOTION_AUTHORITY";
    return output;
  }
  if (!hasPath()) {
    hardStop();
    have_step_time_ = false;
    output.state = "WAIT_PATH";
    output.reason = "NO_PATH";
    return output;
  }
  if (!localization.healthy || !finitePose(localization.pose)) {
    hardStop();
    have_step_time_ = false;
    arrival_window_.clear();
    output.state = "LOCALIZATION_HOLD";
    output.reason = "LOCALIZATION_INVALID";
    return output;
  }
  if (!environment_valid) {
    hardStop();
    have_step_time_ = false;
    arrival_window_.clear();
    output.state = "ENVIRONMENT_HOLD";
    output.reason = "ENVIRONMENT_STALE";
    return output;
  }
  if (too_close) {
    hardStop();
    have_step_time_ = false;
    arrival_window_.clear();
    output.state = "SAFETY_STOP";
    output.reason = "PILE_TOO_CLOSE";
    return output;
  }

  double dt = 0.02;
  if (have_step_time_) {
    dt = now_seconds - last_step_time_;
    if (!(dt > 0.0) || dt > 0.25) {
      hardStop();
      arrival_window_.clear();
      have_step_time_ = false;
      output.state = "TIMING_HOLD";
      output.reason = "CONTROL_PERIOD_INVALID";
      return output;
    }
  }
  last_step_time_ = now_seconds;
  have_step_time_ = true;

  const bool first_localization = !have_sequence_;
  const bool new_localization = first_localization || localization.sequence != last_sequence_;
  const double projected_progress = projectProgress(localization.pose);
  if (first_localization) {
    path_progress_ = projected_progress;
  } else {
    const PathPoint previous_reference = interpolate(path_progress_);
    const double cosine = std::cos(localization.pose.yaw);
    const double sine = std::sin(localization.pose.yaw);
    double along_track_speed = 0.0;
    if (std::isfinite(localization.vx_body) && std::isfinite(localization.vy_body)) {
      const double measured_world_vx =
        cosine * localization.vx_body - sine * localization.vy_body;
      const double measured_world_vy =
        sine * localization.vx_body + cosine * localization.vy_body;
      along_track_speed = previous_reference.tangent_x * measured_world_vx +
        previous_reference.tangent_y * measured_world_vy;
      along_track_speed = clampValue(
        along_track_speed,
        -config_.maximum_progress_rate,
        config_.maximum_progress_rate);
    }
    const double predicted_progress = path_progress_ + along_track_speed * dt;
    double candidate_progress = predicted_progress;
    if (new_localization) {
      const double correction_weight = dt /
        (config_.progress_correction_time_constant + dt);
      candidate_progress += correction_weight * (projected_progress - predicted_progress);
    }
    const double maximum_change = config_.maximum_progress_rate * dt;
    path_progress_ += clampValue(
      candidate_progress - path_progress_, -maximum_change, maximum_change);
    path_progress_ = clampValue(path_progress_, 0.0, trajectory_.back().s);
  }
  if (new_localization) {
    last_sequence_ = localization.sequence;
    have_sequence_ = true;
  }

  const PathPoint reference = interpolate(path_progress_);
  const Pose2D & goal = trajectory_.back().pose;
  const double remaining = std::max(trajectory_.back().s - path_progress_, 0.0);
  const double goal_dx = goal.x - localization.pose.x;
  const double goal_dy = goal.y - localization.pose.y;
  const double goal_distance = std::hypot(goal_dx, goal_dy);

  const double terminal_speed_gain = config_.path_deceleration /
    std::max(config_.maximum_speed, kSmall);
  const double terminal_control_distance = reference.speed_limit /
    std::max(terminal_speed_gain, kSmall);
  const double longitudinal_distance = remaining > terminal_control_distance ?
    remaining : std::min(remaining, goal_distance);
  double path_speed = std::min(
    reference.speed_limit, terminal_speed_gain * longitudinal_distance);
  const double goal_along_error = reference.tangent_x * goal_dx +
    reference.tangent_y * goal_dy;
  if (remaining <= config_.arrival_position_tolerance &&
    goal_along_error < -config_.arrival_position_tolerance)
  {
    path_speed = -std::min(
      config_.maximum_longitudinal_recovery_speed,
      terminal_speed_gain * std::abs(goal_along_error));
  }

  const double error_x = reference.pose.x - localization.pose.x;
  const double error_y = reference.pose.y - localization.pose.y;
  const double along_error = error_x * reference.tangent_x + error_y * reference.tangent_y;
  double normal_error_x = error_x - along_error * reference.tangent_x;
  double normal_error_y = error_y - along_error * reference.tangent_y;
  normal_error_x *= config_.position_gain;
  normal_error_y *= config_.position_gain;
  limitVector(normal_error_x, normal_error_y, config_.maximum_correction_speed);

  double desired_vx = path_speed * reference.tangent_x + normal_error_x;
  double desired_vy = path_speed * reference.tangent_y + normal_error_y;
  limitVector(desired_vx, desired_vy, config_.maximum_speed);

  const double normal_x = -reference.tangent_y;
  const double normal_y = reference.tangent_x;
  double feedforward_acceleration_x = config_.curvature_feedforward_gain *
    reference.curvature * path_speed * path_speed * normal_x;
  double feedforward_acceleration_y = config_.curvature_feedforward_gain *
    reference.curvature * path_speed * path_speed * normal_y;
  limitVector(
    feedforward_acceleration_x,
    feedforward_acceleration_y,
    config_.maximum_lateral_acceleration);

  const double yaw_error = wrapAngle(reference.pose.yaw - localization.pose.yaw);
  const double yaw_correction = clampValue(
    config_.yaw_gain * yaw_error,
    -config_.maximum_correction_yaw_rate,
    config_.maximum_correction_yaw_rate);
  const double desired_omega = clampValue(
    reference.yaw_derivative * path_speed + yaw_correction,
    -config_.maximum_yaw_rate,
    config_.maximum_yaw_rate);

  double target_jerk_x = config_.velocity_shaping_gain *
    (desired_vx - world_velocity_x_) + config_.acceleration_shaping_gain *
    (feedforward_acceleration_x - world_acceleration_x_);
  double target_jerk_y = config_.velocity_shaping_gain *
    (desired_vy - world_velocity_y_) + config_.acceleration_shaping_gain *
    (feedforward_acceleration_y - world_acceleration_y_);
  limitVector(target_jerk_x, target_jerk_y, config_.maximum_jerk);
  limitJerkBeforeAccelerationBoundary(
    target_jerk_x, target_jerk_y,
    world_acceleration_x_, world_acceleration_y_,
    config_.maximum_acceleration, config_.maximum_snap, dt);
  double snap_x = (target_jerk_x - world_jerk_x_) / dt;
  double snap_y = (target_jerk_y - world_jerk_y_) / dt;
  limitVector(snap_x, snap_y, config_.maximum_snap);
  world_jerk_x_ += snap_x * dt;
  world_jerk_y_ += snap_y * dt;
  limitVector(world_jerk_x_, world_jerk_y_, config_.maximum_jerk);
  const double previous_acceleration_x = world_acceleration_x_;
  const double previous_acceleration_y = world_acceleration_y_;
  world_acceleration_x_ += world_jerk_x_ * dt;
  world_acceleration_y_ += world_jerk_y_ * dt;
  limitVector(world_acceleration_x_, world_acceleration_y_, config_.maximum_acceleration);
  world_jerk_x_ = (world_acceleration_x_ - previous_acceleration_x) / dt;
  world_jerk_y_ = (world_acceleration_y_ - previous_acceleration_y) / dt;
  world_velocity_x_ += world_acceleration_x_ * dt;
  world_velocity_y_ += world_acceleration_y_ * dt;
  limitVector(world_velocity_x_, world_velocity_y_, config_.maximum_speed);

  const double target_angular_jerk = clampValue(
    config_.yaw_rate_shaping_gain * (desired_omega - omega_command_) -
    config_.angular_acceleration_shaping_gain * angular_acceleration_,
    -config_.maximum_angular_jerk,
    config_.maximum_angular_jerk);
  const double angular_snap = clampValue(
    (target_angular_jerk - angular_jerk_) / dt,
    -config_.maximum_angular_snap,
    config_.maximum_angular_snap);
  angular_jerk_ += angular_snap * dt;
  angular_jerk_ = clampValue(
    angular_jerk_, -config_.maximum_angular_jerk, config_.maximum_angular_jerk);
  angular_acceleration_ += angular_jerk_ * dt;
  angular_acceleration_ = clampValue(
    angular_acceleration_,
    -config_.maximum_angular_acceleration,
    config_.maximum_angular_acceleration);
  omega_command_ += angular_acceleration_ * dt;
  omega_command_ = clampValue(
    omega_command_, -config_.maximum_yaw_rate, config_.maximum_yaw_rate);

  const double cosine = std::cos(localization.pose.yaw);
  const double sine = std::sin(localization.pose.yaw);
  output.vx_body = cosine * world_velocity_x_ + sine * world_velocity_y_;
  output.vy_body = -sine * world_velocity_x_ + cosine * world_velocity_y_;
  output.omega = omega_command_;
  output.path_progress = path_progress_;
  output.position_error = goal_distance;
  output.yaw_error = wrapAngle(goal.yaw - localization.pose.yaw);
  output.state = "TRACKING";
  output.reason = "OK";

  if (new_localization) {
    updateArrivalWindow(
      localization,
      std::hypot(output.vx_body, output.vy_body),
      std::abs(output.omega));
  }
  double arrival_position_error = output.position_error;
  double arrival_yaw_error = std::abs(output.yaw_error);
  const bool command_stopped =
    std::hypot(output.vx_body, output.vy_body) <=
    config_.arrival_command_speed_tolerance &&
    std::abs(output.omega) <= config_.arrival_command_yaw_rate_tolerance;
  bool arrival_now = false;
  if (command_stopped) {
    arrival_now = evaluateArrival(
      arrival_position_error,
      arrival_yaw_error,
      output.arrival_decision,
      output.arrival_reason);
  } else {
    output.arrival_decision = "COLLECTING";
    output.arrival_reason = "COMMAND_MOVING";
  }
  output.arrival_pulse = arrival_now && !arrived_;
  arrived_ = arrived_ || arrival_now;
  output.arrived = arrived_;
  if (arrived_) {
    hardStop();
    output.vx_body = 0.0;
    output.vy_body = 0.0;
    output.omega = 0.0;
    output.state = "ARRIVED";
    output.reason = "CONFIDENCE_REGION_INSIDE";
  } else if (output.arrival_decision == "INDETERMINATE") {
    output.state = "ARRIVAL_HOLD";
    output.reason = output.arrival_reason;
  }
  return output;
}

void ControllerCore::updateArrivalWindow(
  const LocalizationSample & localization,
  const double command_speed,
  const double command_yaw_rate)
{
  const bool command_stopped =
    command_speed <= config_.arrival_command_speed_tolerance &&
    command_yaw_rate <= config_.arrival_command_yaw_rate_tolerance;
  if (!command_stopped) {
    arrival_window_.clear();
    return;
  }

  ArrivalSample sample;
  sample.stamp = localization.stamp;
  sample.pose = localization.pose;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      sample.covariance[row][column] = localization.covariance[row][column];
    }
  }
  arrival_window_.push_back(sample);
  while (!arrival_window_.empty() &&
    sample.stamp - arrival_window_.front().stamp > config_.arrival_window_seconds)
  {
    arrival_window_.pop_front();
  }
}

bool ControllerCore::evaluateArrival(
  double & position_error,
  double & yaw_error,
  std::string & decision,
  std::string & reason) const
{
  if (arrival_window_.size() < static_cast<std::size_t>(config_.arrival_minimum_samples)) {
    decision = "COLLECTING";
    reason = "WINDOW_NOT_READY";
    return false;
  }
  const double span = arrival_window_.back().stamp - arrival_window_.front().stamp;
  if (span < config_.arrival_minimum_window_seconds) {
    decision = "COLLECTING";
    reason = "WINDOW_NOT_READY";
    return false;
  }

  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> yaws;
  xs.reserve(arrival_window_.size());
  ys.reserve(arrival_window_.size());
  yaws.reserve(arrival_window_.size());
  const Pose2D & goal = trajectory_.back().pose;
  double reported_position_std = 0.0;
  double reported_yaw_std = 0.0;
  for (const auto & sample : arrival_window_) {
    xs.push_back(sample.pose.x);
    ys.push_back(sample.pose.y);
    yaws.push_back(sample.pose.yaw);
    reported_position_std = std::max(
      reported_position_std,
      std::sqrt(largestPlanarVariance(sample.covariance)));
    reported_yaw_std = std::max(
      reported_yaw_std,
      std::sqrt(std::max(sample.covariance[2][2], 0.0)));
  }

  const double center_x = median(xs);
  const double center_y = median(ys);
  const double center_yaw = circularMean(yaws);
  position_error = std::hypot(goal.x - center_x, goal.y - center_y);
  yaw_error = std::abs(wrapAngle(goal.yaw - center_yaw));
  const double window_position_std = std::max(robustSigma(xs), robustSigma(ys));
  std::vector<double> yaw_deviations;
  yaw_deviations.reserve(yaws.size());
  for (const double yaw : yaws) {
    yaw_deviations.push_back(wrapAngle(yaw - center_yaw));
  }
  const double window_yaw_std = robustSigma(yaw_deviations);
  const double position_confidence = config_.confidence_sigma *
    std::max(reported_position_std, window_position_std);
  const double yaw_confidence = config_.confidence_sigma *
    std::max(reported_yaw_std, window_yaw_std);

  const std::size_t group_count = std::max<std::size_t>(arrival_window_.size() / 4U, 1U);
  std::vector<double> first_times;
  std::vector<double> last_times;
  std::vector<double> first_xs;
  std::vector<double> first_ys;
  std::vector<double> first_yaws;
  std::vector<double> last_xs;
  std::vector<double> last_ys;
  std::vector<double> last_yaws;
  first_times.reserve(group_count);
  last_times.reserve(group_count);
  first_xs.reserve(group_count);
  first_ys.reserve(group_count);
  first_yaws.reserve(group_count);
  last_xs.reserve(group_count);
  last_ys.reserve(group_count);
  last_yaws.reserve(group_count);
  for (std::size_t index = 0; index < group_count; ++index) {
    const auto & first = arrival_window_[index];
    const auto & last = arrival_window_[arrival_window_.size() - group_count + index];
    first_times.push_back(first.stamp);
    last_times.push_back(last.stamp);
    first_xs.push_back(first.pose.x);
    first_ys.push_back(first.pose.y);
    first_yaws.push_back(first.pose.yaw);
    last_xs.push_back(last.pose.x);
    last_ys.push_back(last.pose.y);
    last_yaws.push_back(last.pose.yaw);
  }
  const double drift_duration = std::max(
    median(last_times) - median(first_times), kSmall);
  const double position_drift_rate = std::hypot(
    median(last_xs) - median(first_xs),
    median(last_ys) - median(first_ys)) / drift_duration;
  const double yaw_drift_rate = std::abs(wrapAngle(
    circularMean(last_yaws) - circularMean(first_yaws))) / drift_duration;
  if (position_confidence > config_.arrival_position_tolerance ||
    yaw_confidence > config_.arrival_yaw_tolerance)
  {
    decision = "INDETERMINATE";
    reason = "UNCERTAINTY_TOO_LARGE";
    return false;
  }
  if (position_drift_rate > config_.maximum_position_drift_rate ||
    yaw_drift_rate > config_.maximum_yaw_drift_rate)
  {
    decision = "INDETERMINATE";
    reason = "LOCALIZATION_DRIFT";
    return false;
  }
  if (position_error + position_confidence <= config_.arrival_position_tolerance &&
    yaw_error + yaw_confidence <= config_.arrival_yaw_tolerance)
  {
    decision = "ARRIVED";
    reason = "CONFIDENCE_REGION_INSIDE";
    return true;
  }
  if (position_error - position_confidence > config_.arrival_position_tolerance ||
    yaw_error - yaw_confidence > config_.arrival_yaw_tolerance)
  {
    decision = "NOT_ARRIVED";
    reason = "CONFIDENCE_REGION_OUTSIDE";
    return false;
  }
  decision = "INDETERMINATE";
  reason = "CONFIDENCE_REGION_OVERLAP";
  return false;
}

std::vector<Pose2D> makeStraightPath(
  const Pose2D & start,
  const Pose2D & goal,
  const double spacing)
{
  const double distance = std::hypot(goal.x - start.x, goal.y - start.y);
  const std::size_t segment_count = static_cast<std::size_t>(
    std::max(1.0, std::ceil(distance / std::max(spacing, 0.001))));
  std::vector<Pose2D> path;
  path.reserve(segment_count + 1U);
  const double yaw_change = wrapAngle(goal.yaw - start.yaw);
  for (std::size_t i = 0; i <= segment_count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(segment_count);
    const double smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio);
    Pose2D pose;
    pose.x = start.x + ratio * (goal.x - start.x);
    pose.y = start.y + ratio * (goal.y - start.y);
    pose.yaw = start.yaw + smooth_ratio * yaw_change;
    path.push_back(pose);
  }
  return path;
}

}  // namespace pickup_approach_controller
