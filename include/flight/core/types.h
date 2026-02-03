#pragma once

#include <cstdint>

namespace flight::core {

/**
 * @brief 3D vector in float precision.
 */
struct Vector3f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

/**
 * @brief Quaternion (w, x, y, z) in float precision.
 */
struct Quaternionf {
  float w = 1.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

/** @brief Timestamp in microseconds. */
using TimestampUs = uint64_t;

/**
 * @brief Estimated vehicle pose and rates.
 */
struct Pose {
  Vector3f position_m;
  Vector3f velocity_mps;
  Quaternionf orientation;
  Vector3f angular_velocity_rps;
};

}  // namespace flight::core
