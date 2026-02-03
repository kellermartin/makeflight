/**
 * @file vehicle.cpp
 * @brief Vehicle factory implementations.
 */

#include "flight/vehicle/rov4_vehicle.h"
#include "flight/vehicle/vehicle.h"

namespace flight::vehicle {

/** @brief Create a vehicle instance by type. */
std::unique_ptr<IVehicle> VehicleRegistry::Create(VehicleType type,
                                                  const VehicleDependencies& deps) const {
  switch (type) {
    case VehicleType::kRov4Thruster:
      return std::make_unique<Rov4Vehicle>(deps);
    case VehicleType::kVtolPlane:
      return nullptr;
    case VehicleType::kQuadcopter:
      return nullptr;
  }
  return nullptr;
}

}  // namespace flight::vehicle
