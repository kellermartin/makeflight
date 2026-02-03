/**
 * @file scheduler.cpp
 * @brief Implementation of the fixed-rate scheduler.
 */

#include "flight/scheduler/scheduler.h"

namespace flight::scheduler {

/** @brief Register a task with the scheduler. */
void Scheduler::AddTask(const Task& task) {
  tasks_.push_back(task);
}

/** @brief Advance scheduler time and run due tasks. */
void Scheduler::Tick(float dt_s) {
  for (auto& task : tasks_) {
    if (task.rate_hz == 0 || !task.callback) {
      continue;
    }
    task.accumulator_s += dt_s;
    const float period_s = 1.0f / static_cast<float>(task.rate_hz);
    while (task.accumulator_s + 1e-6f >= period_s) {
      task.callback(period_s);
      task.accumulator_s -= period_s;
    }
  }
}

}  // namespace flight::scheduler
