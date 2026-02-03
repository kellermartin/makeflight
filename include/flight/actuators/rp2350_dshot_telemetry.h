#pragma once

#include <cstdint>

#include "flight/actuators/telemetry.h"

namespace flight::actuators {

class Rp2350DshotTelemetryReceiver final : public ITelemetryReceiver {
 public:
  struct Config {
    uint8_t pio_index = 1;
    uint8_t state_machine = 0;
    uint8_t channel_count = 4;
    uint32_t pins[4] = {0};
    uint32_t dshot_khz = 300;
    uint8_t samples_per_bit = 10;
  };

  explicit Rp2350DshotTelemetryReceiver(const Config& config = {});

  bool Initialize() override;
  std::optional<DshotTelemetryFrame> Read() override;

 private:
  bool DecodeFrame(uint8_t channel, DshotTelemetryFrame& frame);

  Config config_{};
  uint8_t active_sms_[4] = {};
  bool initialized_ = false;
  uint32_t sample_words_[4][6] = {};
  uint16_t sample_count_[4] = {};
};

}  // namespace flight::actuators
