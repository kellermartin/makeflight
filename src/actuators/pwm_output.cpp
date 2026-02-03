/**
 * @file pwm_output.cpp
 * @brief PWM actuator output placeholder.
 */

#include "flight/actuators/pwm_output.h"

namespace flight::actuators {

/** @brief Store last commands (placeholder). */
bool PwmOutput::Write(const ActuatorCommand* commands, uint8_t count) {
  last_count_ = count > 8 ? 8 : count;
  for (uint8_t i = 0; i < last_count_; ++i) {
    last_commands_[i] = commands[i];
  }
  return true;
}

}  // namespace flight::actuators
