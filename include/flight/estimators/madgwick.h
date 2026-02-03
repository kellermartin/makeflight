/**
 * @file madgwick.h
 * @brief Madgwick AHRS/IMU filter estimator interface.
 *
 * The Madgwick filter fuses gyroscope measurements with accelerometer and
 * optional magnetometer data to estimate orientation. This class provides a
 * lightweight, real-time implementation suitable for embedded systems.
 */
#pragma once

#include "flight/estimators/estimators.h"

namespace flight::estimators {

/**
 * @brief Madgwick orientation estimator.
 *
 * The estimator uses a gradient-descent correction step (gain @c beta) to
 * reduce drift from gyro integration while aligning the estimated gravity
 * (and magnetic field if available) with measured vectors.
 *
 * Inputs:
 * - IMU: gyroscope (rad/s) and accelerometer (m/s^2) samples.
 * - Magnetometer (optional): magnetic field in microtesla.
 *
 * Outputs:
 * - Orientation as a quaternion (w, x, y, z).
 * - Angular velocity copied from the IMU gyro.
 *
 * Notes:
 * - The update step is skipped if no IMU sample is provided.
 * - The quaternion is normalized every update.
 * - If accelerometer or magnetometer normalization fails, the correction term
 *   is skipped and the filter behaves as pure gyro integration for that step.
 */
class MadgwickEstimator final : public IStateEstimator {
 public:
  /**
   * @brief Initialize estimator state.
   *
   * Resets orientation to identity quaternion and zeroes all state fields.
   *
   * @return true on success.
   */
  bool Initialize() override;

  /**
   * @brief Update estimator using latest available sensor inputs.
   *
   * The time delta is derived from IMU timestamps. A maximum dt clamp is used
   * to avoid large integration steps if timestamps jump.
   *
   * @param input Combined estimator inputs.
   * @return Latest estimator output.
   */
  EstimatorOutput Update(const EstimatorInput& input) override;

 private:
  /**
   * @brief Latest estimator output.
   */
  EstimatorOutput state_{};
};

}  // namespace flight::estimators
