/**
 * @file main_pico.cpp
 * @brief RP2350 Pico SDK entry point for MPU6050 bring-up.
 */

#include <cstdio>

#include <pico/stdlib.h>

#include "flight/hal/pico_config.h"
#include "flight/hal/rp2350_pico_hal.h"
#include "flight/actuators/rp2350_dshot_pio_output.h"
#include "flight/actuators/rp2350_dshot_telemetry.h"
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

  flight::actuators::Rp2350DshotPioOutput::Config dshot_cfg{};
  dshot_cfg.channel_count = flight::hal::PicoConfig::kDshotChannelCount;
  for (uint32_t i = 0; i < flight::hal::PicoConfig::kDshotChannelCount; ++i) {
    dshot_cfg.pins[i] = flight::hal::PicoConfig::kDshotPins[i];
  }
  dshot_cfg.speed = flight::actuators::Rp2350DshotPioOutput::DshotSpeed::k300;
  dshot_cfg.enable_telemetry = true;

  flight::actuators::Rp2350DshotPioOutput dshot(dshot_cfg);
  if (!dshot.Initialize()) {
    printf("DShot init failed\n");
  } else {
    printf("DShot init ok (DShot300)\n");
  }

  flight::actuators::Rp2350DshotTelemetryReceiver::Config telem_cfg{};
  telem_cfg.channel_count = flight::hal::PicoConfig::kDshotChannelCount;
  for (uint32_t i = 0; i < flight::hal::PicoConfig::kDshotChannelCount; ++i) {
    telem_cfg.pins[i] = flight::hal::PicoConfig::kDshotPins[i];
  }
  telem_cfg.pio_index = 1;
  telem_cfg.state_machine = 0;
  telem_cfg.dshot_khz = 300;
  telem_cfg.samples_per_bit = 10;

  flight::actuators::Rp2350DshotTelemetryReceiver telemetry(telem_cfg);
  telemetry.Initialize();

  uint32_t last_print_us = time_us_32();
  float throttle = -1.0f;
  float step = 0.05f;

  while (true) {
    auto sample = imu.Read();
    if (sample && (time_us_32() - last_print_us) > 200000) {
      printf("accel: %.2f %.2f %.2f | gyro: %.2f %.2f %.2f\n",
             sample->accel_mps2.x,
             sample->accel_mps2.y,
             sample->accel_mps2.z,
             sample->gyro_rps.x,
             sample->gyro_rps.y,
             sample->gyro_rps.z);
      last_print_us = time_us_32();
    }

    flight::actuators::ActuatorCommand commands[4] = {};
    for (auto& command : commands) {
      command.value = throttle;
    }
    dshot.Write(commands, 4);

    throttle += step;
    if (throttle > 1.0f || throttle < -1.0f) {
      step = -step;
      throttle = throttle > 1.0f ? 1.0f : -1.0f;
    }

    const auto frame = telemetry.Read();
    if (frame) {
      printf("dshot telem ch%u data=%u raw=%u crc=%u\n",
             frame->channel,
             frame->data,
             frame->raw,
             frame->crc_ok ? 1 : 0);
    }

    sleep_ms(20);
  }
}
