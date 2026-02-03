#pragma once

#include <cstdint>
#include <optional>

#include "flight/actuators/telemetry.h"
#include "flight/controllers/controllers.h"
#include "flight/core/types.h"

namespace flight::telemetry {

/** @brief Snapshot of runtime state for diagnostics/telemetry. */
struct TelemetrySnapshot {
  core::TimestampUs timestamp_us = 0;
  core::Pose pose{};
  controllers::ControlSetpoint setpoint{};
  controllers::ControlOutput output{};
  std::optional<actuators::DshotTelemetryFrame> esc_telemetry;
};

/** @brief Telemetry sink interface. */
class ITelemetrySink {
 public:
  virtual ~ITelemetrySink() = default;
  virtual bool Initialize() = 0;
  virtual void Publish(const TelemetrySnapshot& snapshot) = 0;
};

}  // namespace flight::telemetry
