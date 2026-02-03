#pragma once

#include "flight/controllers/controllers.h"

namespace flight::controllers {

/** @brief ROV mixer gains for surge/yaw/heave. */
struct RovMixConfig {
  float surge_gain = 1.0f;
  float yaw_gain = 1.0f;
  float heave_gain = 1.0f;
};

/**
 * @brief 4-thruster ROV controller (2 horizontal, 2 vertical).
 *
 * Uses setpoint velocity and yaw rate to generate motor outputs.
 */
class RovController final : public IController {
 public:
  /** @brief Construct with mix gains. */
  explicit RovController(const RovMixConfig& config);

  bool Initialize() override { return true; }
  /** @brief Compute motor outputs from setpoint. */
  ControlOutput Update(const core::Pose& state,
                       const ControlSetpoint& setpoint,
                       float dt_s) override;

 private:
  RovMixConfig config_{};
};

}  // namespace flight::controllers
