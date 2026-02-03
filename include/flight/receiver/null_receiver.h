#pragma once

#include "flight/receiver/receiver.h"

namespace flight::receiver {

class NullReceiver final : public ICommandReceiver {
 public:
  bool Initialize() override { return true; }
  std::optional<CommandFrame> Read() override { return std::nullopt; }
};

}  // namespace flight::receiver
