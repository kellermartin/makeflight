#pragma once

#include <optional>

#include "flight/core/types.h"
#include "flight/sensors/sensors.h"
#include "flight/actuators/telemetry.h"

namespace flight::estimators {

/** @brief Combined sensor inputs to the estimator. */
struct EstimatorInput {
  std::optional<sensors::ImuSample> imu;
  std::optional<sensors::MagSample> mag;
  std::optional<sensors::BaroSample> baro;
  std::optional<sensors::GpsSample> gps;
  std::optional<actuators::DshotTelemetryFrame> esc_telemetry;
};

/** @brief State estimator output. */
struct EstimatorOutput {
  core::Pose pose;
  core::TimestampUs timestamp_us = 0;
};

/** @brief Estimator interface (Madgwick, Mahony, EKF, etc.). */
class IStateEstimator {
 public:
  virtual ~IStateEstimator() = default;
  /** @brief Initialize estimator state. */
  virtual bool Initialize() = 0;
  /** @brief Update estimator using latest inputs. */
  virtual EstimatorOutput Update(const EstimatorInput& input) = 0;
};

}  // namespace flight::estimators
