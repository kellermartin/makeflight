#include <doctest/doctest.h>

#include "flight/actuators/biheli_pwm_output.h"

TEST_CASE("Biheli PWM maps normalized values") {
  flight::actuators::BiheliPwmOutput output({});
  flight::actuators::ActuatorCommand commands[3];
  commands[0].value = -1.0f;
  commands[1].value = 0.0f;
  commands[2].value = 1.0f;

  REQUIRE(output.Write(commands, 3));

  CHECK(output.PulseUs(0) == doctest::Approx(1100.0f));
  CHECK(output.PulseUs(1) == doctest::Approx(1500.0f));
  CHECK(output.PulseUs(2) == doctest::Approx(1900.0f));
}
