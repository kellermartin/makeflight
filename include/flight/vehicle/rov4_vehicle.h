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
  VehicleDependencies deps_;
};

}  // namespace flight::vehicle
