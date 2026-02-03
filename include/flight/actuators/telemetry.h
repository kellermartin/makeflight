#pragma once

#include <cstdint>
#include <optional>

namespace flight::actuators {

/** @brief Raw DShot telemetry frame. */
struct DshotTelemetryFrame {
  uint8_t channel = 0;
  uint16_t data = 0;
  uint16_t raw = 0;
  bool crc_ok = false;
};

/** @brief Telemetry receiver interface. */
class ITelemetryReceiver {
 public:
  virtual ~ITelemetryReceiver() = default;
  virtual bool Initialize() = 0;
  virtual std::optional<DshotTelemetryFrame> Read() = 0;
};

}  // namespace flight::actuators
