#include <doctest/doctest.h>

#include "flight/config/in_memory_config.h"

TEST_CASE("Config store set/get") {
  flight::config::InMemoryConfigStore store;

  REQUIRE(store.Set("rov.surge_gain", 1.5f));

  float value = 0.0f;
  CHECK(store.Get("rov.surge_gain", value));
  CHECK(value == doctest::Approx(1.5f));
}
