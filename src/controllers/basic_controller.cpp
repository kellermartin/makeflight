/**
 * @file basic_controller.cpp
 * @brief Basic controller placeholder implementation.
 */

#include "flight/controllers/basic_controller.h"

namespace flight::controllers {

/** @brief Compute a simple thrust-only output. */
ControlOutput BasicController::Update(const core::Pose&,
                                      const ControlSetpoint& setpoint,
                                      float) {
  ControlOutput output{};
  output.motor_count = 4;
  for (uint8_t i = 0; i < output.motor_count; ++i) {
    output.motors[i] = setpoint.thrust;
  }
  return output;
}

}  // namespace flight::controllers
