#include <doctest/doctest.h>

#include <array>
#include <cstring>
#include <vector>

#include "flight/hal/hal.h"
#include "flight/sensors/mpu6050.h"

namespace {

class FakeI2c final : public flight::hal::II2c {
 public:
  bool Write(uint8_t address, const uint8_t* data, size_t length) override {
    last_address = address;
    last_write.assign(data, data + length);
    return true;
  }

  bool Read(uint8_t, uint8_t*, size_t) override { return false; }

  bool WriteRead(uint8_t address,
                 const uint8_t* data,
                 size_t data_length,
                 uint8_t* out,
                 size_t out_length) override {
    last_address = address;
    last_write.assign(data, data + data_length);
    if (out_length > response.size()) {
      return false;
    }
    std::memcpy(out, response.data(), out_length);
    return true;
  }

  uint8_t last_address = 0;
  std::vector<uint8_t> last_write;
  std::array<uint8_t, 14> response{};
};

}  // namespace

TEST_CASE("MPU6050 initializes by waking the device") {
  FakeI2c i2c;
  flight::sensors::Mpu6050Imu imu(&i2c, {});

  REQUIRE(imu.Initialize());
  REQUIRE(i2c.last_write.size() == 2);
  CHECK(i2c.last_write[0] == 0x6B);
  CHECK(i2c.last_write[1] == 0x00);
}

TEST_CASE("MPU6050 reads and scales accel/gyro") {
  FakeI2c i2c;
  flight::sensors::Mpu6050Imu imu(&i2c, {});

  // Accel: 16384 -> 1g => 9.80665 m/s^2
  // Gyro: 131 -> 1 deg/s => 0.0174533 rad/s
  i2c.response = {0x40, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00,
                  0x00, 0x00, 0x83, 0x00, 0x83, 0x00, 0x83};

  auto sample = imu.Read();
  REQUIRE(sample.has_value());

  CHECK(sample->accel_mps2.x == doctest::Approx(9.80665f));
  CHECK(sample->accel_mps2.y == doctest::Approx(0.0f));
  CHECK(sample->accel_mps2.z == doctest::Approx(-9.80665f));

  CHECK(sample->gyro_rps.x == doctest::Approx(0.0174533f).epsilon(0.01f));
  CHECK(sample->gyro_rps.y == doctest::Approx(0.0174533f).epsilon(0.01f));
  CHECK(sample->gyro_rps.z == doctest::Approx(0.0174533f).epsilon(0.01f));
}
