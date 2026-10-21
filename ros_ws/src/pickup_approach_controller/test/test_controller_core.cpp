#include "pickup_approach_controller/controller_core.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace pickup_approach_controller
{
namespace
{

LocalizationSample makeLocalization(
  const double stamp,
  const std::uint64_t sequence,
  const Pose2D & pose)
{
  LocalizationSample sample;
  sample.stamp = stamp;
  sample.sequence = sequence;
  sample.pose = pose;
  sample.covariance[0][0] = 9.0e-6;
  sample.covariance[1][1] = 9.0e-6;
  sample.covariance[2][2] = std::pow(0.3 * 3.14159265358979323846 / 180.0, 2.0);
  sample.healthy = true;
  return sample;
}

TEST(ControllerCore, RequiresMotionAuthority)
{
  ControllerCore controller;
  std::string error;
  ASSERT_TRUE(controller.setPath({Pose2D{0.0, 0.0, 0.0}, Pose2D{1.0, 0.0, 0.0}}, &error));
  const auto output = controller.step(
    0.0, makeLocalization(0.0, 1U, Pose2D{0.0, 0.0, 0.0}), false, true, false);
  EXPECT_DOUBLE_EQ(output.vx_body, 0.0);
  EXPECT_DOUBLE_EQ(output.vy_body, 0.0);
  EXPECT_DOUBLE_EQ(output.omega, 0.0);
  EXPECT_EQ(output.state, "DISABLED");
}

TEST(ControllerCore, GeneratesForwardCommandAndHonorsPileStop)
{
  ControllerCore controller;
  ASSERT_TRUE(controller.setPath({Pose2D{0.0, 0.0, 0.0}, Pose2D{1.0, 0.0, 0.0}}));
  ControllerOutput output;
  for (std::uint64_t index = 0U; index < 20U; ++index) {
    const double time = 0.02 * static_cast<double>(index);
    output = controller.step(
      time, makeLocalization(time, index + 1U, Pose2D{0.0, 0.0, 0.0}), true, true, false);
  }
  EXPECT_GT(output.vx_body, 0.0);
  EXPECT_NEAR(output.vy_body, 0.0, 1.0e-12);

  output = controller.step(
    0.42, makeLocalization(0.42, 22U, Pose2D{0.0, 0.0, 0.0}), true, true, true);
  EXPECT_DOUBLE_EQ(output.vx_body, 0.0);
  EXPECT_DOUBLE_EQ(output.vy_body, 0.0);
  EXPECT_DOUBLE_EQ(output.omega, 0.0);
  EXPECT_EQ(output.state, "SAFETY_STOP");
}

TEST(ControllerCore, DeclaresArrivalFromStationaryConfidenceWindow)
{
  ControllerConfig config;
  config.arrival_window_seconds = 1.0;
  config.arrival_minimum_samples = 10;
  ControllerCore controller(config);
  ASSERT_TRUE(controller.setPath({Pose2D{0.0, 0.0, 0.0}, Pose2D{1.0, 0.0, 0.0}}));

  bool received_pulse = false;
  ControllerOutput output;
  for (std::uint64_t index = 0U; index <= 60U; ++index) {
    const double time = 0.02 * static_cast<double>(index);
    output = controller.step(
      time, makeLocalization(time, index + 1U, Pose2D{1.0, 0.0, 0.0}), true, true, false);
    received_pulse = received_pulse || output.arrival_pulse;
  }
  EXPECT_TRUE(received_pulse);
  EXPECT_TRUE(output.arrived);
  EXPECT_EQ(output.state, "ARRIVED");
  EXPECT_EQ(output.arrival_decision, "ARRIVED");
}

TEST(ControllerCore, StraightPathUsesSmoothSpatialYawTransition)
{
  const auto path = makeStraightPath(
    Pose2D{0.0, 0.0, 0.0},
    Pose2D{1.0, 0.0, 1.0},
    0.1);
  ASSERT_GT(path.size(), 3U);
  EXPECT_DOUBLE_EQ(path.front().yaw, 0.0);
  EXPECT_DOUBLE_EQ(path.back().yaw, 1.0);
  EXPECT_LT(path[1].yaw - path[0].yaw, path[path.size() / 2U].yaw - path[path.size() / 2U - 1U].yaw);
}

TEST(ControllerCore, RejectsArrivalWhileLocalizationCenterDrifts)
{
  ControllerCore controller;
  ASSERT_TRUE(controller.setPath({Pose2D{0.0, 0.0, 0.0}, Pose2D{1.0, 0.0, 0.0}}));

  ControllerOutput output;
  for (std::uint64_t index = 0U; index <= 50U; ++index) {
    const double time = 0.02 * static_cast<double>(index);
    const double x = 0.995 + 0.010 * static_cast<double>(index) / 50.0;
    output = controller.step(
      time, makeLocalization(time, index + 1U, Pose2D{x, 0.0, 0.0}), true, true, false);
  }
  EXPECT_FALSE(output.arrived);
  EXPECT_NE(output.state, "ARRIVED");
  EXPECT_EQ(output.arrival_decision, "INDETERMINATE");
  EXPECT_EQ(output.arrival_reason, "LOCALIZATION_DRIFT");
}

}  // namespace
}  // namespace pickup_approach_controller
