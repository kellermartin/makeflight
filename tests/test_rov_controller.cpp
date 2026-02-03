#include <doctest/doctest.h>

#include "flight/controllers/rov_controller.h"

TEST_CASE("ROV controller mixes surge and yaw") {
  flight::controllers::RovController controller({});
  flight::controllers::ControlSetpoint setpoint{};

  setpoint.velocity_mps.x = 0.5f;
  setpoint.body_rates_rps.z = 0.25f;

  auto output = controller.Update({}, setpoint, 0.01f);

  CHECK(output.motor_count == 4);
  CHECK(output.motors[0] == doctest::Approx(0.75f));
  CHECK(output.motors[1] == doctest::Approx(0.25f));
}

TEST_CASE("ROV controller clamps outputs") {
  flight::controllers::RovController controller({});
  flight::controllers::ControlSetpoint setpoint{};

  setpoint.velocity_mps.x = 0.0f;
  setpoint.body_rates_rps.z = 2.0f;

  auto output = controller.Update({}, setpoint, 0.01f);

  CHECK(output.motors[0] == doctest::Approx(1.0f));
  CHECK(output.motors[1] == doctest::Approx(-1.0f));
}
