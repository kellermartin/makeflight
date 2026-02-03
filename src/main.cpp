/**
 * @file main.cpp
 * @brief Minimal demo wiring for the ROV vehicle.
 */

#include "flight/actuators/biheli_pwm_output.h"
#include "flight/controllers/rov_controller.h"
#include "flight/estimators/madgwick.h"
#include "flight/receiver/udp_receiver.h"
#include "flight/telemetry/udp_telemetry.h"
#include "flight/vehicle/vehicle.h"

int main() {
  flight::estimators::MadgwickEstimator estimator;
  flight::controllers::RovController controller(flight::controllers::RovMixConfig{});
  flight::actuators::BiheliPwmOutput actuators(flight::actuators::BiheliPwmOutput::Config{});
  flight::receiver::UdpReceiver receiver(flight::receiver::UdpReceiver::Config{});
  flight::telemetry::UdpTelemetrySender telemetry(flight::telemetry::UdpTelemetrySender::Config{});

  flight::vehicle::VehicleDependencies deps;
  deps.estimator = &estimator;
  deps.controller = &controller;
  deps.actuators = &actuators;
  deps.receiver = &receiver;
  deps.telemetry_sink = &telemetry;

  flight::vehicle::VehicleRegistry registry;
  auto vehicle = registry.Create(flight::vehicle::VehicleType::kRov4Thruster, deps);
  if (!vehicle || !vehicle->Initialize()) {
    return 1;
  }

  telemetry.Initialize();

  vehicle->Update(0.002f);
  return 0;
}
