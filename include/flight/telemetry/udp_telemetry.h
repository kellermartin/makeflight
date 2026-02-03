#pragma once

#include <cstdint>
#include <string>

#include "flight/telemetry/telemetry.h"

namespace flight::telemetry {

class UdpTelemetrySender final : public ITelemetrySink {
 public:
  struct Config {
    std::string address = "127.0.0.1";
    uint16_t port = 14560;
    uint16_t rate_hz = 100;
  };

  explicit UdpTelemetrySender(const Config& config);

  bool Initialize() override;
  void Publish(const TelemetrySnapshot& snapshot) override;

 private:
  Config config_{};
  int socket_fd_ = -1;
  uint64_t last_send_us_ = 0;
};

}  // namespace flight::telemetry
