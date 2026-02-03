/**
 * @file main_pico.cpp
 * @brief RP2350 Pico SDK entry point for MPU6050 bring-up.
 */

#include <cstdio>

#include <pico/stdlib.h>

#include "flight/hal/pico_config.h"
#include "flight/hal/rp2350_pico_hal.h"
#include "flight/sensors/mpu6050.h"

int main() {
  stdio_init_all();
  sleep_ms(1500);

  printf("I2C%d SDA=%u SCL=%u\n",
         flight::hal::PicoConfig::kI2cIndex,
         flight::hal::PicoConfig::kI2cSdaGpio,
         flight::hal::PicoConfig::kI2cSclGpio);

  flight::hal::Rp2350I2c i2c(flight::hal::PicoConfig::kI2cIndex,
                             flight::hal::PicoConfig::kI2cSdaGpio,
                             flight::hal::PicoConfig::kI2cSclGpio,
                             {});
  i2c.Initialize();

  flight::sensors::Mpu6050Imu imu(&i2c, {});
  if (!imu.Initialize()) {
    printf("MPU6050 init failed\n");
  } else {
    printf("MPU6050 init ok\n");
  }

  while (true) {
    auto sample = imu.Read();
    if (sample) {
      printf("accel: %.2f %.2f %.2f | gyro: %.2f %.2f %.2f\n",
             sample->accel_mps2.x,
             sample->accel_mps2.y,
             sample->accel_mps2.z,
             sample->gyro_rps.x,
             sample->gyro_rps.y,
             sample->gyro_rps.z);
    } else {
      printf("MPU6050 read failed\n");
    }
    sleep_ms(200);
  }
}
