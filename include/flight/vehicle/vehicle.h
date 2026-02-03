#pragma once

#include <memory>

#include "flight/actuators/actuators.h"
#include "flight/actuators/telemetry.h"
#include "flight/controllers/controllers.h"
#include "flight/estimators/estimators.h"
#include "flight/receiver/receiver.h"
#include "flight/scheduler/scheduler.h"
#include "flight/telemetry/telemetry.h"

namespace flight::vehicle {

/** @brief Supported vehicle archetypes. */
enum class VehicleType {
  kRov4Thruster,
  kVtolPlane,
  kQuadcopter,
};

/** @brief Vehicle interface for high-level update. */
class IVehicle {
 public:
  virtual ~IVehicle() = default;
  /** @brief Vehicle type identifier. */
  virtual VehicleType Type() const = 0;
  /** @brief Initialize dependencies. */
  virtual bool Initialize() = 0;
  /** @brief Update vehicle control loop. */
  virtual void Update(float dt_s) = 0;
};

/** @brief Dependencies required by a vehicle instance. */
struct VehicleDependencies {
  estimators::IStateEstimator* estimator = nullptr;
  controllers::IController* controller = nullptr;
  actuators::IActuatorOutput* actuators = nullptr;
  actuators::ITelemetryReceiver* telemetry = nullptr;
  telemetry::ITelemetrySink* telemetry_sink = nullptr;
  receiver::ICommandReceiver* receiver = nullptr;
};

/** @brief Factory for vehicle instances. */
class VehicleRegistry {
 public:
  /** @brief Create a vehicle by type. */
  std::unique_ptr<IVehicle> Create(VehicleType type, const VehicleDependencies& deps) const;
};

}  // namespace flight::vehicle
