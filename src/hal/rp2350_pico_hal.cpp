/**
 * @file rp2350_pico_hal.cpp
 * @brief RP2350 Pico SDK HAL implementations.
 */

#include "flight/hal/rp2350_pico_hal.h"

#include <hardware/i2c.h>
#include <pico/stdlib.h>

namespace flight::hal {

uint64_t Rp2350Time::NowUs() const {
  return time_us_64();
}

void Rp2350Time::SleepUs(uint64_t duration_us) {
  sleep_us(duration_us);
}

Rp2350I2c::Rp2350I2c(uint8_t i2c_index, uint32_t sda_gpio, uint32_t scl_gpio, const Config& config)
    : i2c_index_(i2c_index), sda_gpio_(sda_gpio), scl_gpio_(scl_gpio), config_(config) {}

bool Rp2350I2c::Initialize() {
  i2c_inst_t* instance = (i2c_index_ == 0) ? i2c0 : i2c1;
  i2c_init(instance, config_.baud_hz);

  gpio_set_function(sda_gpio_, GPIO_FUNC_I2C);
  gpio_set_function(scl_gpio_, GPIO_FUNC_I2C);
  gpio_pull_up(sda_gpio_);
  gpio_pull_up(scl_gpio_);
  return true;
}

bool Rp2350I2c::Write(uint8_t address, const uint8_t* data, size_t length) {
  i2c_inst_t* instance = (i2c_index_ == 0) ? i2c0 : i2c1;
  int written = i2c_write_blocking(instance, address, data, length, false);
  return written == static_cast<int>(length);
}

bool Rp2350I2c::Read(uint8_t address, uint8_t* out, size_t length) {
  i2c_inst_t* instance = (i2c_index_ == 0) ? i2c0 : i2c1;
  int read = i2c_read_blocking(instance, address, out, length, false);
  return read == static_cast<int>(length);
}

bool Rp2350I2c::WriteRead(uint8_t address,
                          const uint8_t* data,
                          size_t data_length,
                          uint8_t* out,
                          size_t out_length) {
  i2c_inst_t* instance = (i2c_index_ == 0) ? i2c0 : i2c1;
  int written = i2c_write_blocking(instance, address, data, data_length, true);
  if (written != static_cast<int>(data_length)) {
    return false;
  }
  int read = i2c_read_blocking(instance, address, out, out_length, false);
  return read == static_cast<int>(out_length);
}

}  // namespace flight::hal
