#pragma once

#include "flight/controllers/controllers.h"

namespace flight::controllers {

class BasicController final : public IController {
 public:
  bool Initialize() override { return true; }
  ControlOutput Update(const core::Pose& state,
                       const ControlSetpoint& setpoint,
                       float dt_s) override;
};

}  // namespace flight::controllers
