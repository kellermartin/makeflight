/**
 * @file dshot_output.cpp
 * @brief DShot actuator output encoder placeholder.
 */

#include "flight/actuators/dshot_output.h"

#include <algorithm>

namespace flight::actuators {

namespace {

float Clamp(float value, float min_value, float max_value) {
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace

DshotOutput::DshotOutput() = default;

DshotOutput::DshotOutput(const Config& config) : config_(config) {}

uint16_t DshotOutput::ValueToThrottle(float value) const {
  const float clamped = Clamp(value, -1.0f, 1.0f);
  const float normalized = (clamped + 1.0f) * 0.5f;
  const float range = static_cast<float>(config_.max_throttle - config_.min_throttle);
  const float mapped = static_cast<float>(config_.min_throttle) + (normalized * range);
  return static_cast<uint16_t>(Clamp(mapped,
                                     static_cast<float>(config_.min_throttle),
                                     static_cast<float>(config_.max_throttle)));
}

/** @brief Store last commands and encoded packets (placeholder). */
bool DshotOutput::Write(const ActuatorCommand* commands, uint8_t count) {
  last_count_ = count > 8 ? 8 : count;
  for (uint8_t i = 0; i < last_count_; ++i) {
    last_commands_[i] = commands[i];
    const uint16_t throttle = ValueToThrottle(commands[i].value);
    last_packets_[i] = PackCommand(throttle, config_.enable_telemetry);
  }
  return true;
}

}  // namespace flight::actuators
