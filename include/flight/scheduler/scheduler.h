#pragma once

#include <cstdint>
#include <functional>
#include <vector>

namespace flight::scheduler {

/** @brief Scheduled task definition. */
struct Task {
  const char* name = nullptr;
  uint32_t rate_hz = 0;
  std::function<void(float)> callback;
  float accumulator_s = 0.0f;
};

/**
 * @brief Simple fixed-rate scheduler.
 *
 * Each task runs at its configured rate when Tick() advances time.
 */
class Scheduler {
 public:
  /** @brief Register a new task. */
  void AddTask(const Task& task);
  /** @brief Advance scheduler by dt seconds. */
  void Tick(float dt_s);

 private:
  std::vector<Task> tasks_;
};

}  // namespace flight::scheduler
