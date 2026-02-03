/**
 * @file rov4_vehicle.cpp
 * @brief 4-thruster ROV vehicle implementation.
 */

#include "flight/vehicle/rov4_vehicle.h"
#include "flight/telemetry/telemetry.h"

namespace flight::vehicle {

/** @brief Construct a 4-thruster ROV vehicle. */
Rov4Vehicle::Rov4Vehicle(const VehicleDependencies& deps) : deps_(deps) {}

/** @brief Initialize vehicle dependencies. */
bool Rov4Vehicle::Initialize() {
  bool ok = true;
  if (deps_.estimator) {
    ok = deps_.estimator->Initialize() && ok;
  }
  if (deps_.controller) {
    ok = deps_.controller->Initialize() && ok;
  }
  if (deps_.actuators) {
    ok = deps_.actuators->Initialize() && ok;
  }
  if (deps_.receiver) {
    ok = deps_.receiver->Initialize() && ok;
  }
  return ok;
}

/** @brief Run control update for the ROV. */
void Rov4Vehicle::Update(float dt_s) {
  if (!deps_.estimator || !deps_.controller || !deps_.actuators) {
    return;
  }

  estimators::EstimatorInput input;
  if (deps_.telemetry) {
    input.esc_telemetry = deps_.telemetry->Read();
  }
  const auto estimate = deps_.estimator->Update(input);

  controllers::ControlSetpoint setpoint{};
  if (deps_.receiver) {
    const auto frame = deps_.receiver->Read();
    if (frame && frame->channel_count >= 3) {
      setpoint.velocity_mps.x = frame->channels[0];
      setpoint.body_rates_rps.z = frame->channels[1];
      setpoint.velocity_mps.z = frame->channels[2];
    }
  }

  auto output = deps_.controller->Update(estimate.pose, setpoint, dt_s);
  output.motor_count = 4;

  actuators::ActuatorCommand commands[4];
  for (uint8_t i = 0; i < 4; ++i) {
    commands[i].value = output.motors[i];
  }
  deps_.actuators->Write(commands, 4);

  if (deps_.telemetry_sink) {
    telemetry::TelemetrySnapshot snapshot{};
    snapshot.timestamp_us = estimate.timestamp_us;
    snapshot.pose = estimate.pose;
    snapshot.setpoint = setpoint;
    snapshot.output = output;
    snapshot.esc_telemetry = input.esc_telemetry;
    deps_.telemetry_sink->Publish(snapshot);
  }
}

}  // namespace flight::vehicle
