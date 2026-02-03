#pragma once

#include <cstdint>
#include <cstddef>

namespace flight::hal {

/**
 * @brief Time abstraction for scheduling and timestamps.
 */
class ITime {
 public:
  virtual ~ITime() = default;
  /** @brief Current time in microseconds. */
  virtual uint64_t NowUs() const = 0;
  /** @brief Sleep or delay for the given duration. */
  virtual void SleepUs(uint64_t duration_us) = 0;
};

/**
 * @brief Flash storage abstraction for configuration persistence.
 */
class IFlash {
 public:
  virtual ~IFlash() = default;
  /** @brief Read bytes from flash. */
  virtual bool Read(uint32_t address, uint8_t* out, size_t length) = 0;
  /** @brief Write bytes to flash. */
  virtual bool Write(uint32_t address, const uint8_t* data, size_t length) = 0;
  /** @brief Erase flash region. */
  virtual bool Erase(uint32_t address, uint32_t length) = 0;
};

/**
 * @brief I2C bus abstraction used by sensors like IMUs and barometers.
 */
class II2c {
 public:
  virtual ~II2c() = default;
  /** @brief Write bytes to a device. */
  virtual bool Write(uint8_t address, const uint8_t* data, size_t length) = 0;
  /** @brief Read bytes from a device. */
  virtual bool Read(uint8_t address, uint8_t* out, size_t length) = 0;
  /** @brief Combined write then read (common for register access). */
  virtual bool WriteRead(uint8_t address,
                         const uint8_t* data,
                         size_t data_length,
                         uint8_t* out,
                         size_t out_length) = 0;
};

/**
 * @brief SPI bus abstraction.
 */
class ISpi {
 public:
  virtual ~ISpi() = default;
  /** @brief Full-duplex transfer. */
  virtual bool Transfer(const uint8_t* tx, size_t tx_length, uint8_t* rx, size_t rx_length) = 0;
};

/**
 * @brief UART abstraction for serial receivers or GPS.
 */
class IUart {
 public:
  virtual ~IUart() = default;
  /** @brief Write bytes to UART. */
  virtual bool Write(const uint8_t* data, size_t length) = 0;
  /** @brief Read bytes from UART. */
  virtual bool Read(uint8_t* out, size_t length) = 0;
};

/**
 * @brief GPIO abstraction for simple digital IO.
 */
class IGpio {
 public:
  virtual ~IGpio() = default;
  /** @brief Drive output high or low. */
  virtual void Write(bool high) = 0;
  /** @brief Read current level. */
  virtual bool Read() const = 0;
};

}  // namespace flight::hal
