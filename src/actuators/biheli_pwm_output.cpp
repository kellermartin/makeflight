/**
 * @file biheli_pwm_output.cpp
 * @brief Biheli PWM output implementation.
 */

#include "flight/actuators/biheli_pwm_output.h"

namespace flight::actuators {

namespace {

/** @brief Clamp helper. */
float Clamp(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

}  // namespace

/** @brief Construct with PWM configuration. */
BiheliPwmOutput::BiheliPwmOutput(const Config& config) : config_(config) {}

/** @brief Map normalized commands to PWM pulse widths. */
bool BiheliPwmOutput::Write(const ActuatorCommand* commands, uint8_t count) {
  last_count_ = count > 8 ? 8 : count;
  for (uint8_t i = 0; i < last_count_; ++i) {
    const float value = Clamp(commands[i].value, -1.0f, 1.0f);
    const float range = config_.max_pulse_us - config_.neutral_pulse_us;
    const float pulse = config_.neutral_pulse_us + (value * range);
    last_pulse_us_[i] = Clamp(pulse, config_.min_pulse_us, config_.max_pulse_us);
  }
  return true;
}

}  // namespace flight::actuators
