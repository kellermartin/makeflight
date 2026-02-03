/**
 * @file udp_telemetry.cpp
 * @brief UDP telemetry sender implementation.
 */

#include "flight/telemetry/udp_telemetry.h"

#include <cstring>

#if defined(__linux__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace flight::telemetry {

namespace {

#pragma pack(push, 1)
struct UdpTelemetryPacket {
  uint32_t magic = 0x4D46544C;  // "MFTL"
  uint8_t version = 1;
  uint8_t motor_count = 0;
  uint16_t reserved = 0;
  uint64_t timestamp_us = 0;
  flight::core::Quaternionf orientation{};
  flight::core::Vector3f angular_velocity_rps{};
  flight::core::Vector3f position_m{};
  flight::core::Vector3f velocity_mps{};
  flight::core::Vector3f setpoint_velocity_mps{};
  flight::core::Vector3f setpoint_body_rates_rps{};
  float setpoint_thrust = 0.0f;
  float motors[8] = {0};
  uint8_t esc_channel = 0;
  uint16_t esc_data = 0;
  uint16_t esc_raw = 0;
  uint8_t esc_crc_ok = 0;
  uint8_t esc_present = 0;
};
#pragma pack(pop)

}  // namespace

UdpTelemetrySender::UdpTelemetrySender(const Config& config) : config_(config) {}

bool UdpTelemetrySender::Initialize() {
#if defined(__linux__)
  socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  return socket_fd_ >= 0;
#else
  return false;
#endif
}

void UdpTelemetrySender::Publish(const TelemetrySnapshot& snapshot) {
#if defined(__linux__)
  if (socket_fd_ < 0) {
    return;
  }
  if (config_.rate_hz > 0 && snapshot.timestamp_us > 0) {
    const uint64_t min_interval_us = 1000000ull / config_.rate_hz;
    if ((snapshot.timestamp_us - last_send_us_) < min_interval_us) {
      return;
    }
  }

  UdpTelemetryPacket packet{};
  packet.timestamp_us = snapshot.timestamp_us;
  packet.orientation = snapshot.pose.orientation;
  packet.angular_velocity_rps = snapshot.pose.angular_velocity_rps;
  packet.position_m = snapshot.pose.position_m;
  packet.velocity_mps = snapshot.pose.velocity_mps;
  packet.setpoint_velocity_mps = snapshot.setpoint.velocity_mps;
  packet.setpoint_body_rates_rps = snapshot.setpoint.body_rates_rps;
  packet.setpoint_thrust = snapshot.setpoint.thrust;
  packet.motor_count = snapshot.output.motor_count;
  for (uint8_t i = 0; i < packet.motor_count && i < 8; ++i) {
    packet.motors[i] = snapshot.output.motors[i];
  }

  if (snapshot.esc_telemetry) {
    packet.esc_present = 1;
    packet.esc_channel = snapshot.esc_telemetry->channel;
    packet.esc_data = snapshot.esc_telemetry->data;
    packet.esc_raw = snapshot.esc_telemetry->raw;
    packet.esc_crc_ok = snapshot.esc_telemetry->crc_ok ? 1 : 0;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(config_.port);
  addr.sin_addr.s_addr = inet_addr(config_.address.c_str());

  ::sendto(socket_fd_,
           reinterpret_cast<const void*>(&packet),
           sizeof(packet),
           0,
           reinterpret_cast<sockaddr*>(&addr),
           sizeof(addr));

  last_send_us_ = snapshot.timestamp_us;
#else
  (void)snapshot;
#endif
}

}  // namespace flight::telemetry
