#pragma once

#include <cstdint>
#include <optional>

namespace flight::receiver {

/** @brief Normalized command channels coming from a receiver. */
struct CommandFrame {
  float channels[16] = {0};
  uint8_t channel_count = 0;
  bool failsafe = false;
};

/** @brief Receiver interface (ELRS, SBUS, UDP, etc.). */
class ICommandReceiver {
 public:
  virtual ~ICommandReceiver() = default;
  /** @brief Initialize receiver hardware or transport. */
  virtual bool Initialize() = 0;
  /** @brief Read a command frame if available. */
  virtual std::optional<CommandFrame> Read() = 0;
};

}  // namespace flight::receiver
