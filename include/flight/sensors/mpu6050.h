#pragma once

#include "flight/hal/hal.h"
#include "flight/sensors/sensors.h"

namespace flight::sensors {

/**
 * @brief MPU6050 IMU driver (accelerometer + gyroscope).
 */
class Mpu6050Imu final : public IImu {
 public:
  /** @brief Device configuration. */
  struct Config {
    uint8_t address = 0x68;
  };

  /** @brief Construct with I2C bus and config. */
  Mpu6050Imu(hal::II2c* i2c, const Config& config);

  /** @brief Initialize device registers. */
  bool Initialize() override;
  /** @brief Read latest IMU sample. */
  std::optional<ImuSample> Read() override;

 private:
  hal::II2c* i2c_ = nullptr;
  Config config_{};
};

}  // namespace flight::sensors
