#pragma once

#include "flight/hal/hal.h"

namespace flight::hal {

class Rp2350Time final : public ITime {
 public:
  uint64_t NowUs() const override;
  void SleepUs(uint64_t duration_us) override;
};

}  // namespace flight::hal
