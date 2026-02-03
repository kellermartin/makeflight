#pragma once

#include <cstdint>

#include "flight/core/types.h"

namespace flight::controllers {

/** @brief Desired control setpoint. */
struct ControlSetpoint {
  core::Vector3f position_m;
  core::Vector3f velocity_mps;
  core::Vector3f attitude_rpy_rad;
  core::Vector3f body_rates_rps;
  float thrust = 0.0f;
};

/** @brief Controller output in normalized actuator units. */
struct ControlOutput {
  float motors[8] = {0};
  uint8_t motor_count = 0;
};

/** @brief Controller interface (PID, LQR, cascaded, etc.). */
class IController {
 public:
  virtual ~IController() = default;
  /** @brief Initialize controller state. */
  virtual bool Initialize() = 0;
  /** @brief Update controller given the estimated state and setpoint. */
  virtual ControlOutput Update(const core::Pose& state,
                               const ControlSetpoint& setpoint,
                               float dt_s) = 0;
};

}  // namespace flight::controllers
