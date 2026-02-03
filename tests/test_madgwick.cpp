#include <doctest/doctest.h>

#include <cmath>

#include "flight/estimators/madgwick.h"

namespace {

float QuaternionNorm(const flight::core::Quaternionf& q) {
  return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

}  // namespace

TEST_CASE("Madgwick initializes to identity orientation") {
  flight::estimators::MadgwickEstimator estimator;
  REQUIRE(estimator.Initialize());

  const auto output = estimator.Update({});
  CHECK(output.pose.orientation.w == doctest::Approx(1.0f));
  CHECK(output.pose.orientation.x == doctest::Approx(0.0f));
  CHECK(output.pose.orientation.y == doctest::Approx(0.0f));
  CHECK(output.pose.orientation.z == doctest::Approx(0.0f));
}

TEST_CASE("Madgwick updates gyro and timestamp on first sample") {
  flight::estimators::MadgwickEstimator estimator;
  REQUIRE(estimator.Initialize());

  flight::estimators::EstimatorInput input{};
  input.imu = flight::sensors::ImuSample{
      {0.0f, 0.0f, 9.80665f}, {0.1f, -0.2f, 0.3f}, 1000};

  const auto output = estimator.Update(input);
  CHECK(output.pose.angular_velocity_rps.x == doctest::Approx(0.1f));
  CHECK(output.pose.angular_velocity_rps.y == doctest::Approx(-0.2f));
  CHECK(output.pose.angular_velocity_rps.z == doctest::Approx(0.3f));
  CHECK(output.timestamp_us == 1000);
  CHECK(output.pose.orientation.w == doctest::Approx(1.0f));
}

TEST_CASE("Madgwick integrates yaw from gyro with valid accel") {
  flight::estimators::MadgwickEstimator estimator;
  REQUIRE(estimator.Initialize());

  flight::estimators::EstimatorInput input{};
  input.imu = flight::sensors::ImuSample{
      {0.0f, 0.0f, 9.80665f}, {0.0f, 0.0f, 1.0f}, 1000000};
  estimator.Update(input);

  input.imu->timestamp_us = 2000000;
  const auto output = estimator.Update(input);

  const float expected_w = std::cos(0.5f);
  const float expected_z = std::sin(0.5f);

  CHECK(output.pose.orientation.w == doctest::Approx(expected_w).epsilon(0.01f));
  CHECK(output.pose.orientation.x == doctest::Approx(0.0f).epsilon(0.01f));
  CHECK(output.pose.orientation.y == doctest::Approx(0.0f).epsilon(0.01f));
  CHECK(output.pose.orientation.z == doctest::Approx(expected_z).epsilon(0.01f));
  CHECK(QuaternionNorm(output.pose.orientation) == doctest::Approx(1.0f).epsilon(0.01f));
}

TEST_CASE("Madgwick uses magnetometer path when available") {
  flight::estimators::MadgwickEstimator estimator;
  REQUIRE(estimator.Initialize());

  flight::estimators::EstimatorInput input{};
  input.imu = flight::sensors::ImuSample{
      {0.0f, 0.0f, 9.80665f}, {0.0f, 0.0f, 0.0f}, 1000000};
  input.mag = flight::sensors::MagSample{{1.0f, 0.0f, 0.0f}, 1000000};
  estimator.Update(input);

  input.imu->timestamp_us = 2000000;
  input.mag->timestamp_us = 2000000;
  const auto output = estimator.Update(input);

  CHECK(QuaternionNorm(output.pose.orientation) == doctest::Approx(1.0f).epsilon(0.01f));
}
