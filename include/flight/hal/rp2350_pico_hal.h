#pragma once

#include <cstdint>

#include "flight/hal/hal.h"

namespace flight::hal {

/**
 * @brief RP2350 Pico SDK time implementation.
 */
class Rp2350Time final : public ITime {
 public:
  uint64_t NowUs() const override;
  void SleepUs(uint64_t duration_us) override;
};

/**
 * @brief RP2350 Pico SDK I2C implementation.
 */
class Rp2350I2c final : public II2c {
 public:
  struct Config {
    uint32_t baud_hz = 400000;
  };

  Rp2350I2c(uint8_t i2c_index, uint32_t sda_gpio, uint32_t scl_gpio, const Config& config);

  bool Initialize();

  bool Write(uint8_t address, const uint8_t* data, size_t length) override;
  bool Read(uint8_t address, uint8_t* out, size_t length) override;
  bool WriteRead(uint8_t address,
                 const uint8_t* data,
                 size_t data_length,
                 uint8_t* out,
                 size_t out_length) override;

 private:
  uint8_t i2c_index_ = 0;
  uint32_t sda_gpio_ = 0;
  uint32_t scl_gpio_ = 1;
  Config config_{};
};

}  // namespace flight::hal
