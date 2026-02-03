#include <doctest/doctest.h>

#include "flight/scheduler/scheduler.h"

TEST_CASE("Scheduler runs tasks at configured rates") {
  flight::scheduler::Scheduler scheduler;

  int fast_count = 0;
  int slow_count = 0;

  scheduler.AddTask({"fast", 100, [&](float) { ++fast_count; }});
  scheduler.AddTask({"slow", 10, [&](float) { ++slow_count; }});

  for (int i = 0; i < 100; ++i) {
    scheduler.Tick(0.01f);
  }

  CHECK(fast_count == 100);
  CHECK(slow_count == 10);
}
