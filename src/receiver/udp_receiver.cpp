/**
 * @file udp_receiver.cpp
 * @brief UDP-based command receiver implementation.
 */

#include "flight/receiver/udp_receiver.h"

#include <cstring>

#if defined(__linux__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace flight::receiver {

namespace {

/** @brief Packed UDP command payload. */
struct UdpPacket {
  uint32_t magic = 0x4D465454;  // "MFTT"
  uint8_t version = 1;
  uint8_t channel_count = 0;
  float channels[16] = {0};
};

}  // namespace

/** @brief Construct with configuration. */
UdpReceiver::UdpReceiver(const Config& config) : config_(config) {}

/** @brief Initialize UDP socket (non-blocking). */
bool UdpReceiver::Initialize() {
#if defined(__linux__)
  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(config_.port);

  if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  int flags = fcntl(socket_fd_, F_GETFL, 0);
  fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  return true;
#else
  return false;
#endif
}

/** @brief Read next command frame if available. */
std::optional<CommandFrame> UdpReceiver::Read() {
#if defined(__linux__)
  if (socket_fd_ < 0) {
    return std::nullopt;
  }

  UdpPacket packet{};
  ssize_t received = ::recv(socket_fd_, &packet, sizeof(packet), 0);
  if (received <= 0) {
    return std::nullopt;
  }
  if (packet.magic != 0x4D465454 || packet.version != 1) {
    return std::nullopt;
  }

  CommandFrame frame{};
  frame.channel_count = packet.channel_count > 16 ? 16 : packet.channel_count;
  for (uint8_t i = 0; i < frame.channel_count; ++i) {
    frame.channels[i] = packet.channels[i];
  }
  frame.failsafe = false;
  return frame;
#else
  return std::nullopt;
#endif
}

}  // namespace flight::receiver
