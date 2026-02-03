#pragma once

#include <cstdint>

namespace flight::hal {

/**
 * @brief Pico SDK board configuration defaults.
 *
 * Update these to match your wiring.
 */
struct PicoConfig {
  static constexpr uint8_t kI2cIndex = 0;
  static constexpr uint32_t kI2cSdaGpio = 4;
  static constexpr uint32_t kI2cSclGpio = 5;
  static constexpr uint32_t kDshotChannelCount = 4;
  static constexpr uint32_t kDshotPins[kDshotChannelCount] = {2, 3, 6, 7};
};

}  // namespace flight::hal
