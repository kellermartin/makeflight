/**
 * @file mpu6050.cpp
 * @brief MPU6050 IMU driver implementation.
 */

#include "flight/sensors/mpu6050.h"

namespace flight::sensors {

namespace {

/** @brief Power management register. */
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
/** @brief First accelerometer output register. */
constexpr uint8_t kRegAccelXOut = 0x3B;

}  // namespace

/** @brief Construct with I2C bus and config. */
Mpu6050Imu::Mpu6050Imu(hal::II2c* i2c, const Config& config) : i2c_(i2c), config_(config) {}

/** @brief Initialize the sensor by waking it up. */
bool Mpu6050Imu::Initialize() {
  if (!i2c_) {
    return false;
  }
  uint8_t payload[2] = {kRegPwrMgmt1, 0x00};
  return i2c_->Write(config_.address, payload, sizeof(payload));
}

/** @brief Read accelerometer and gyro values. */
std::optional<ImuSample> Mpu6050Imu::Read() {
  if (!i2c_) {
    return std::nullopt;
  }

  uint8_t reg = kRegAccelXOut;
  uint8_t data[14] = {0};
  if (!i2c_->WriteRead(config_.address, &reg, 1, data, sizeof(data))) {
    return std::nullopt;
  }

  ImuSample sample{};
  sample.timestamp_us = 0;

  const int16_t ax = static_cast<int16_t>((data[0] << 8) | data[1]);
  const int16_t ay = static_cast<int16_t>((data[2] << 8) | data[3]);
  const int16_t az = static_cast<int16_t>((data[4] << 8) | data[5]);
  const int16_t gx = static_cast<int16_t>((data[8] << 8) | data[9]);
  const int16_t gy = static_cast<int16_t>((data[10] << 8) | data[11]);
  const int16_t gz = static_cast<int16_t>((data[12] << 8) | data[13]);

  constexpr float kAccelScale = 9.80665f / 16384.0f;
  constexpr float kGyroScale = 3.1415926f / (180.0f * 131.0f);

  sample.accel_mps2.x = ax * kAccelScale;
  sample.accel_mps2.y = ay * kAccelScale;
  sample.accel_mps2.z = az * kAccelScale;

  sample.gyro_rps.x = gx * kGyroScale;
  sample.gyro_rps.y = gy * kGyroScale;
  sample.gyro_rps.z = gz * kGyroScale;

  return sample;
}

}  // namespace flight::sensors
