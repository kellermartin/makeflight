/**
 * @file rov_controller.cpp
 * @brief ROV mixer implementation for 4 thrusters.
 */

#include "flight/controllers/rov_controller.h"

namespace flight::controllers {

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

/** @brief Construct with mixer configuration. */
RovController::RovController(const RovMixConfig& config) : config_(config) {}

/** @brief Mix surge/yaw/heave into thruster commands. */
ControlOutput RovController::Update(const core::Pose&,
                                    const ControlSetpoint& setpoint,
                                    float) {
  ControlOutput output{};
  output.motor_count = 4;

  const float surge = setpoint.velocity_mps.x * config_.surge_gain;
  const float yaw = setpoint.body_rates_rps.z * config_.yaw_gain;
  const float heave = setpoint.velocity_mps.z * config_.heave_gain;

  output.motors[0] = Clamp(surge + yaw, -1.0f, 1.0f);  // horizontal left
  output.motors[1] = Clamp(surge - yaw, -1.0f, 1.0f);  // horizontal right
  output.motors[2] = Clamp(heave, -1.0f, 1.0f);        // vertical front
  output.motors[3] = Clamp(heave, -1.0f, 1.0f);        // vertical rear

  return output;
}

}  // namespace flight::controllers
