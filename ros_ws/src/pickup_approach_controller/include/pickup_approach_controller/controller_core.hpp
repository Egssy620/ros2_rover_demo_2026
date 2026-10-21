#ifndef PICKUP_APPROACH_CONTROLLER__CONTROLLER_CORE_HPP_
#define PICKUP_APPROACH_CONTROLLER__CONTROLLER_CORE_HPP_

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace pickup_approach_controller
{

// 地図座標系における車体中心の平面姿勢を表します。
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 上流EKFが出す一つの融合自己位置メッセージを、制御コア用に整形した値です。
struct LocalizationSample
{
  double stamp{0.0};
  std::uint64_t sequence{0};
  Pose2D pose{};
  double covariance[3][3]{};
  double vx_body{0.0};
  double vy_body{0.0};
  double omega{0.0};
  bool healthy{false};
};

// MATLAB版の空間軌道表に対応する一つの節点です。
struct PathPoint
{
  double s{0.0};
  Pose2D pose{};
  double tangent_x{1.0};
  double tangent_y{0.0};
  double curvature{0.0};
  double yaw_derivative{0.0};
  double speed_limit{0.0};
};

// 実機調整値をまとめます。単位はm、s、radです。
struct ControllerConfig
{
  double maximum_speed{0.25};
  double maximum_yaw_rate{0.35};
  double maximum_acceleration{0.35};
  double maximum_angular_acceleration{0.60};
  double maximum_jerk{0.80};
  double maximum_angular_jerk{1.50};
  double maximum_snap{3.0};
  double maximum_angular_snap{5.0};
  double maximum_lateral_acceleration{0.25};
  double path_deceleration{0.20};
  double maximum_longitudinal_recovery_speed{0.04};
  double position_gain{0.8};
  double yaw_gain{1.2};
  double maximum_correction_speed{0.08};
  double maximum_correction_yaw_rate{0.18};
  double projection_search_back{0.60};
  double projection_search_ahead{0.80};
  double progress_correction_time_constant{0.10};
  double maximum_progress_rate{0.80};
  double curvature_feedforward_gain{1.0};
  double velocity_shaping_gain{25.0};
  double acceleration_shaping_gain{10.0};
  double yaw_rate_shaping_gain{9.0};
  double angular_acceleration_shaping_gain{6.0};
  double arrival_position_tolerance{0.020};
  double arrival_yaw_tolerance{0.020943951};  // 1.2 deg
  double arrival_command_speed_tolerance{0.010};
  double arrival_command_yaw_rate_tolerance{0.010};
  double arrival_window_seconds{0.8};
  double arrival_minimum_window_seconds{0.5};
  int arrival_minimum_samples{10};
  double confidence_sigma{2.0};
  double maximum_position_drift_rate{0.005};
  double maximum_yaw_drift_rate{0.008726646};
};

// 一周期の制御計算結果です。速度は車体座標系です。
struct ControllerOutput
{
  double vx_body{0.0};
  double vy_body{0.0};
  double omega{0.0};
  double path_progress{0.0};
  double position_error{0.0};
  double yaw_error{0.0};
  bool arrived{false};
  bool arrival_pulse{false};
  std::string arrival_decision{"COLLECTING"};
  std::string arrival_reason{"WINDOW_NOT_READY"};
  std::string state{"IDLE"};
  std::string reason{"NO_PATH"};
};

// ROS 2に依存しない追従コアです。単体試験とログ再生から同じ処理を呼べます。
class ControllerCore
{
public:
  explicit ControllerCore(const ControllerConfig & config = ControllerConfig{});

  // 地図座標の節点列を受理し、距離基準の空間軌道表を生成します。
  bool setPath(const std::vector<Pose2D> & poses, std::string * error = nullptr);

  // 目標・到着履歴・指令整形器をすべて安全な停止状態へ戻します。
  void clearPath();

  // 最新の融合自己位置から一制御周期分の速度指令を計算します。
  ControllerOutput step(
    double now_seconds,
    const LocalizationSample & localization,
    bool motion_authority,
    bool environment_valid,
    bool too_close);

  bool hasPath() const;
  const std::vector<PathPoint> & trajectory() const;

private:
  struct ArrivalSample
  {
    double stamp{0.0};
    Pose2D pose{};
    double covariance[3][3]{};
  };

  PathPoint interpolate(double progress) const;
  double projectProgress(const Pose2D & pose) const;
  void hardStop();
  void updateArrivalWindow(
    const LocalizationSample & localization,
    double command_speed,
    double command_yaw_rate);
  bool evaluateArrival(
    double & position_error,
    double & yaw_error,
    std::string & decision,
    std::string & reason) const;

  ControllerConfig config_{};
  std::vector<PathPoint> trajectory_{};
  std::deque<ArrivalSample> arrival_window_{};
  double path_progress_{0.0};
  double last_step_time_{0.0};
  std::uint64_t last_sequence_{0};
  bool have_step_time_{false};
  bool have_sequence_{false};
  bool arrived_{false};
  double world_velocity_x_{0.0};
  double world_velocity_y_{0.0};
  double world_acceleration_x_{0.0};
  double world_acceleration_y_{0.0};
  double world_jerk_x_{0.0};
  double world_jerk_y_{0.0};
  double omega_command_{0.0};
  double angular_acceleration_{0.0};
  double angular_jerk_{0.0};
};

// 試験時だけ使う直線経路を生成します。実運用では山回避計画器の出力を使います。
std::vector<Pose2D> makeStraightPath(
  const Pose2D & start,
  const Pose2D & goal,
  double spacing);

double wrapAngle(double angle);

}  // namespace pickup_approach_controller

#endif  // PICKUP_APPROACH_CONTROLLER__CONTROLLER_CORE_HPP_
