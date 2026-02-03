/**
 * @file rov4_vehicle.cpp
 * @brief 4-thruster ROV vehicle implementation.
 */

#include "flight/vehicle/rov4_vehicle.h"
#include "flight/telemetry/telemetry.h"

#include <cmath>

namespace flight::vehicle {

namespace {

constexpr float kArmYawThreshold = 0.9f;
constexpr float kDisarmYawThreshold = -0.9f;
constexpr float kNeutralThreshold = 0.1f;
constexpr float kHoldTimeS = 1.0f;

}  // namespace

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
  bool has_frame = false;
  receiver::CommandFrame frame{};
  if (deps_.receiver) {
    const auto received = deps_.receiver->Read();
    if (received && received->channel_count >= 3) {
      frame = *received;
      has_frame = true;
      setpoint.velocity_mps.x = frame.channels[0];
      setpoint.body_rates_rps.z = frame.channels[1];
      setpoint.velocity_mps.z = frame.channels[2];
    }
  }

  if (has_frame) {
    UpdateArming(frame, dt_s);
  } else {
    arm_hold_s_ = 0.0f;
    disarm_hold_s_ = 0.0f;
  }

  controllers::ControlOutput output{};
  output.motor_count = 4;
  if (arm_state_ == ArmState::kArmed) {
    output = deps_.controller->Update(estimate.pose, setpoint, dt_s);
    output.motor_count = 4;
  } else {
    for (uint8_t i = 0; i < output.motor_count; ++i) {
      output.motors[i] = 0.0f;
    }
  }

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
    snapshot.armed = arm_state_ == ArmState::kArmed;
    deps_.telemetry_sink->Publish(snapshot);
  }
}

void Rov4Vehicle::UpdateArming(const receiver::CommandFrame& frame, float dt_s) {
  const float surge = frame.channels[0];
  const float yaw = frame.channels[1];
  const float heave = frame.channels[2];

  const bool neutral = std::abs(surge) < kNeutralThreshold &&
                       std::abs(heave) < kNeutralThreshold;

  if (neutral && yaw > kArmYawThreshold) {
    arm_hold_s_ += dt_s;
    disarm_hold_s_ = 0.0f;
  } else if (neutral && yaw < kDisarmYawThreshold) {
    disarm_hold_s_ += dt_s;
    arm_hold_s_ = 0.0f;
  } else {
    arm_hold_s_ = 0.0f;
    disarm_hold_s_ = 0.0f;
  }

  if (arm_hold_s_ >= kHoldTimeS) {
    arm_state_ = ArmState::kArmed;
  }
  if (disarm_hold_s_ >= kHoldTimeS) {
    arm_state_ = ArmState::kDisarmed;
  }
}

}  // namespace flight::vehicle
