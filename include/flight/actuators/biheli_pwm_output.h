#pragma once

#include "flight/actuators/actuators.h"

namespace flight::actuators {

/**
 * @brief Biheli PWM output (bidirectional ESC).
 *
 * Normalized command [-1, 1] is mapped to PWM microseconds.
 */
class BiheliPwmOutput final : public IActuatorOutput {
 public:
  /** @brief Pulse width configuration. */
  struct Config {
    float min_pulse_us = 1100.0f;
    float max_pulse_us = 1900.0f;
    float neutral_pulse_us = 1500.0f;
  };

  /** @brief Construct with PWM config. */
  explicit BiheliPwmOutput(const Config& config);

  bool Initialize() override { return true; }
  /** @brief Write actuator commands. */
  bool Write(const ActuatorCommand* commands, uint8_t count) override;

  /** @brief Last mapped pulse width for diagnostics. */
  float PulseUs(uint8_t index) const { return last_pulse_us_[index]; }

 private:
  Config config_{};
  float last_pulse_us_[8] = {};
  uint8_t last_count_ = 0;
};

}  // namespace flight::actuators
