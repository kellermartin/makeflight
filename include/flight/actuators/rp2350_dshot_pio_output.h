#pragma once

#include <cstdint>

#include "flight/actuators/actuators.h"

namespace flight::actuators {

class Rp2350DshotPioOutput final : public IActuatorOutput {
 public:
  static constexpr uint32_t kMaxChannels = 4;

  enum class DshotSpeed : uint32_t {
    k150 = 150,
    k300 = 300,
    k600 = 600,
    k1200 = 1200,
  };

  struct Config {
    uint8_t pio_index = 0;
    uint8_t state_machine = 0;
    uint8_t channel_count = 4;
    uint32_t pins[kMaxChannels] = {0};
    DshotSpeed speed = DshotSpeed::k300;
    bool enable_telemetry = false;
    uint16_t min_throttle = 48;
    uint16_t max_throttle = 2047;
  };

  Rp2350DshotPioOutput();
  explicit Rp2350DshotPioOutput(const Config& config);

  bool Initialize() override;
  bool Write(const ActuatorCommand* commands, uint8_t count) override;

 private:
  uint16_t ValueToThrottle(float value) const;
  void BuildSymbolWords(uint16_t packet, uint32_t* words, uint8_t& word_count) const;

  Config config_{};
  uint8_t active_sms_[kMaxChannels] = {};
  uint8_t last_count_ = 0;
};

}  // namespace flight::actuators
