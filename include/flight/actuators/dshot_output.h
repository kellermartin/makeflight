#pragma once

#include "flight/actuators/actuators.h"

namespace flight::actuators {

class DshotOutput final : public IActuatorOutput {
 public:
  struct Config {
    uint16_t min_throttle = 48;
    uint16_t max_throttle = 2047;
    bool enable_telemetry = false;
  };

  DshotOutput();
  explicit DshotOutput(const Config& config);

  bool Initialize() override { return true; }
  bool Write(const ActuatorCommand* commands, uint8_t count) override;

  static uint16_t PackCommand(uint16_t throttle, bool telemetry) {
    const uint16_t data =
        static_cast<uint16_t>((throttle << 1) | (telemetry ? 1 : 0));
    const uint16_t csum = (data ^ (data >> 4) ^ (data >> 8)) & 0xF;
    return static_cast<uint16_t>((data << 4) | csum);
  }

 private:
  uint16_t ValueToThrottle(float value) const;

  Config config_{};
  ActuatorCommand last_commands_[8] = {};
  uint16_t last_packets_[8] = {};
  uint8_t last_count_ = 0;
};

}  // namespace flight::actuators
