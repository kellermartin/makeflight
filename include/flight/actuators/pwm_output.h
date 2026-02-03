#pragma once

#include "flight/actuators/actuators.h"

namespace flight::actuators {

class PwmOutput final : public IActuatorOutput {
 public:
  bool Initialize() override { return true; }
  bool Write(const ActuatorCommand* commands, uint8_t count) override;

  const ActuatorCommand* LastCommands() const { return last_commands_; }
  uint8_t LastCount() const { return last_count_; }

 private:
  ActuatorCommand last_commands_[8] = {};
  uint8_t last_count_ = 0;
};

}  // namespace flight::actuators
