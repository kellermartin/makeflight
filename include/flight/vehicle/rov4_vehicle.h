#pragma once

#include "flight/vehicle/vehicle.h"

namespace flight::vehicle {

/**
 * @brief 4-thruster ROV vehicle implementation.
 *
 * Channel mapping:
 * - ch0: surge (forward/back)
 * - ch1: yaw (turn)
 * - ch2: heave (up/down)
 */
class Rov4Vehicle final : public IVehicle {
 public:
  /** @brief Construct with dependencies. */
  explicit Rov4Vehicle(const VehicleDependencies& deps);

  VehicleType Type() const override { return VehicleType::kRov4Thruster; }
  /** @brief Initialize vehicle dependencies. */
  bool Initialize() override;
  /** @brief Run control update. */
 void Update(float dt_s) override;

 private:
  enum class ArmState : uint8_t { kDisarmed = 0, kArmed = 1 };

  void UpdateArming(const receiver::CommandFrame& frame, float dt_s);

  VehicleDependencies deps_;
  ArmState arm_state_ = ArmState::kDisarmed;
  float arm_hold_s_ = 0.0f;
  float disarm_hold_s_ = 0.0f;
};

}  // namespace flight::vehicle
