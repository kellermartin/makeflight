#pragma once

#include "flight/actuators/actuators.h"

namespace flight::actuators {

class DshotOutput final : public IActuatorOutput {
 public:
  bool Initialize() override { return true; }
  bool Write(const ActuatorCommand* commands, uint8_t count) override;

 private:
  ActuatorCommand last_commands_[8] = {};
  uint8_t last_count_ = 0;
};

}  // namespace flight::actuators
