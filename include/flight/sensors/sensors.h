#pragma once

#include <cstdint>
#include <optional>

#include "flight/core/types.h"

namespace flight::sensors {

/** @brief IMU sample containing accelerometer and gyroscope data. */
struct ImuSample {
  core::Vector3f accel_mps2;
  core::Vector3f gyro_rps;
  core::TimestampUs timestamp_us = 0;
};

/** @brief Barometer sample containing pressure and altitude. */
struct BaroSample {
  float pressure_pa = 0.0f;
  float temperature_c = 0.0f;
  float altitude_m = 0.0f;
  core::TimestampUs timestamp_us = 0;
};

/** @brief Magnetometer sample. */
struct MagSample {
  core::Vector3f magnetic_ut;
  core::TimestampUs timestamp_us = 0;
};

/** @brief GPS sample. */
struct GpsSample {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  float altitude_m = 0.0f;
  float ground_speed_mps = 0.0f;
  float course_deg = 0.0f;
  uint8_t satellites = 0;
  bool fix = false;
  core::TimestampUs timestamp_us = 0;
};

/** @brief IMU interface. */
class IImu {
 public:
  virtual ~IImu() = default;
  /** @brief Initialize the sensor. */
  virtual bool Initialize() = 0;
  /** @brief Read a sample if available. */
  virtual std::optional<ImuSample> Read() = 0;
};

/** @brief Barometer interface. */
class IBarometer {
 public:
  virtual ~IBarometer() = default;
  /** @brief Initialize the sensor. */
  virtual bool Initialize() = 0;
  /** @brief Read a sample if available. */
  virtual std::optional<BaroSample> Read() = 0;
};

/** @brief Magnetometer interface. */
class IMagnetometer {
 public:
  virtual ~IMagnetometer() = default;
  /** @brief Initialize the sensor. */
  virtual bool Initialize() = 0;
  /** @brief Read a sample if available. */
  virtual std::optional<MagSample> Read() = 0;
};

/** @brief GPS interface. */
class IGps {
 public:
  virtual ~IGps() = default;
  /** @brief Initialize the sensor. */
  virtual bool Initialize() = 0;
  /** @brief Read a sample if available. */
  virtual std::optional<GpsSample> Read() = 0;
};

}  // namespace flight::sensors
