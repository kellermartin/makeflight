#include <doctest/doctest.h>

#include <chrono>
#include <thread>

#if defined(__linux__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "flight/receiver/udp_receiver.h"

TEST_CASE("UDP receiver reads a command frame") {
#if defined(__linux__)
  flight::receiver::UdpReceiver receiver({14551});
  REQUIRE(receiver.Initialize());

  int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  REQUIRE(sock >= 0);

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(14551);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  struct Packet {
    uint32_t magic;
    uint8_t version;
    uint8_t channel_count;
    float channels[16];
  } packet{};

  packet.magic = 0x4D465454;
  packet.version = 1;
  packet.channel_count = 3;
  packet.channels[0] = 0.1f;
  packet.channels[1] = -0.2f;
  packet.channels[2] = 0.3f;

  ssize_t sent = ::sendto(sock, &packet, sizeof(packet), 0,
                          reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  CHECK(sent > 0);

  std::optional<flight::receiver::CommandFrame> frame;
  for (int i = 0; i < 50; ++i) {
    frame = receiver.Read();
    if (frame.has_value()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  REQUIRE(frame.has_value());
  CHECK(frame->channel_count == 3);
  CHECK(frame->channels[0] == doctest::Approx(0.1f));
  CHECK(frame->channels[1] == doctest::Approx(-0.2f));
  CHECK(frame->channels[2] == doctest::Approx(0.3f));

  ::close(sock);
#else
  CHECK(true);
#endif
}
