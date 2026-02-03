#pragma once

#include <cstdint>

namespace flight::actuators {

/** @brief Normalized actuator command in [-1, 1]. */
struct ActuatorCommand {
  float value = 0.0f;
};

/** @brief Actuator output interface (PWM, DShot, CAN ESC, etc.). */
class IActuatorOutput {
 public:
  virtual ~IActuatorOutput() = default;
  /** @brief Initialize output hardware. */
  virtual bool Initialize() = 0;
  /** @brief Send actuator commands. */
  virtual bool Write(const ActuatorCommand* commands, uint8_t count) = 0;
};

}  // namespace flight::actuators
