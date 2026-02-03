#include <doctest/doctest.h>

#include "flight/actuators/dshot_output.h"

TEST_CASE("DShot packs checksum correctly") {
  const uint16_t throttle = 1000;
  const uint16_t packet = flight::actuators::DshotOutput::PackCommand(throttle, false);

  const uint16_t data = static_cast<uint16_t>(throttle << 1);
  const uint16_t expected_checksum = (data ^ (data >> 4) ^ (data >> 8)) & 0xF;
  const uint16_t expected_packet = static_cast<uint16_t>((data << 4) | expected_checksum);

  CHECK(packet == expected_packet);
}

TEST_CASE("DShot packs telemetry bit") {
  const uint16_t throttle = 48;
  const uint16_t packet_no_tlm = flight::actuators::DshotOutput::PackCommand(throttle, false);
  const uint16_t packet_tlm = flight::actuators::DshotOutput::PackCommand(throttle, true);

  CHECK(packet_no_tlm != packet_tlm);
  CHECK(((packet_no_tlm ^ packet_tlm) & 0x1F0) != 0);
}
