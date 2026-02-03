/**
 * @file madgwick.cpp
 * @brief Placeholder Madgwick estimator implementation.
 */

#include "flight/estimators/madgwick.h"

namespace flight::estimators {

/** @brief Initialize estimator state. */
bool MadgwickEstimator::Initialize() {
  state_ = {};
  return true;
}

/** @brief Update estimator using available inputs. */
EstimatorOutput MadgwickEstimator::Update(const EstimatorInput& input) {
  if (input.imu) {
    state_.pose.angular_velocity_rps = input.imu->gyro_rps;
  }
  if (input.imu) {
    state_.timestamp_us = input.imu->timestamp_us;
  }
  return state_;
}

}  // namespace flight::estimators
