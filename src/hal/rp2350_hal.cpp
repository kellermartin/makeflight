/**
 * @file rp2350_hal.cpp
 * @brief RP2350 HAL stubs.
 */

#include "flight/hal/rp2350_hal.h"

namespace flight::hal {

/** @brief Return current time in microseconds (stub). */
uint64_t Rp2350Time::NowUs() const {
  return 0;
}

/** @brief Sleep for the specified duration (stub). */
void Rp2350Time::SleepUs(uint64_t) {
}

}  // namespace flight::hal
