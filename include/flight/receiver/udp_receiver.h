#pragma once

#include <cstdint>
#include <optional>

#include "flight/receiver/receiver.h"

namespace flight::receiver {

/**
 * @brief UDP receiver for command frames.
 *
 * Packet format:
 * - magic: 0x4D465454 ("MFTT")
 * - version: 1
 * - channel_count
 * - channels[16]
 */
class UdpReceiver final : public ICommandReceiver {
 public:
  /** @brief UDP configuration options. */
  struct Config {
    uint16_t port = 14550;
  };

  /** @brief Construct receiver with config. */
  explicit UdpReceiver(const Config& config);

  /** @brief Initialize UDP socket. */
  bool Initialize() override;
  /** @brief Receive latest frame (non-blocking). */
  std::optional<CommandFrame> Read() override;

 private:
  Config config_{};
  int socket_fd_ = -1;
};

}  // namespace flight::receiver
