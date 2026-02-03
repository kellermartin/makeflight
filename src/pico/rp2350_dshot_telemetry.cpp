/**
 * @file rp2350_dshot_telemetry.cpp
 * @brief RP2350 DShot telemetry receiver scaffold.
 */

#include "flight/actuators/rp2350_dshot_telemetry.h"

#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <pico/stdlib.h>

#include "dshot_telem_rx.pio.h"

namespace flight::actuators {

Rp2350DshotTelemetryReceiver::Rp2350DshotTelemetryReceiver() = default;

Rp2350DshotTelemetryReceiver::Rp2350DshotTelemetryReceiver(const Config& config)
    : config_(config) {}

bool Rp2350DshotTelemetryReceiver::Initialize() {
  if (config_.channel_count == 0 || config_.channel_count > 4) {
    return false;
  }
  if (config_.dshot_khz == 0 || config_.samples_per_bit < 6) {
    return false;
  }

  PIO pio = (config_.pio_index == 0) ? pio0 : pio1;
  const uint offset = pio_add_program(pio, &dshot_telem_rx_program);

  const uint32_t sys_hz = clock_get_hz(clk_sys);
  const uint32_t sample_hz =
      config_.dshot_khz * 1000u * static_cast<uint32_t>(config_.samples_per_bit);
  const float clkdiv = static_cast<float>(sys_hz) /
                       static_cast<float>(sample_hz);
  if (clkdiv < 1.0f) {
    return false;
  }

  for (uint8_t i = 0; i < config_.channel_count; ++i) {
    const uint8_t sm = static_cast<uint8_t>(config_.state_machine + i);
    if (sm >= 4) {
      return false;
    }
    active_sms_[i] = sm;
    pio_sm_claim(pio, sm);
    pio_sm_set_consecutive_pindirs(pio, sm, config_.pins[i], 1, false);

    pio_sm_config cfg = dshot_telem_rx_program_get_default_config(offset);
    sm_config_set_in_pins(&cfg, config_.pins[i]);
    sm_config_set_in_shift(&cfg, true, true, 32);
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&cfg, clkdiv);

    pio_sm_init(pio, sm, offset, &cfg);
    pio_sm_set_enabled(pio, sm, true);
  }

  for (uint8_t i = 0; i < config_.channel_count; ++i) {
    sample_count_[i] = 0;
    for (uint8_t w = 0; w < 6; ++w) {
      sample_words_[i][w] = 0;
    }
  }

  initialized_ = true;
  return true;
}

std::optional<DshotTelemetryFrame> Rp2350DshotTelemetryReceiver::Read() {
  if (!initialized_) {
    return std::nullopt;
  }

  PIO pio = (config_.pio_index == 0) ? pio0 : pio1;
  for (uint8_t i = 0; i < config_.channel_count; ++i) {
    const uint8_t sm = active_sms_[i];
    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
      const uint32_t sample = pio_sm_get_blocking(pio, sm);
      const uint8_t word_index = static_cast<uint8_t>(sample_count_[i] / 32);
      if (word_index < 6) {
        sample_words_[i][word_index] = sample;
      }
      sample_count_[i] += 32;
      const uint16_t needed_samples =
          static_cast<uint16_t>(16u * config_.samples_per_bit);
      if (sample_count_[i] >= needed_samples) {
        DshotTelemetryFrame frame{};
        if (DecodeFrame(i, frame)) {
          sample_count_[i] = 0;
          for (uint8_t w = 0; w < 6; ++w) {
            sample_words_[i][w] = 0;
          }
          return frame;
        }
        sample_count_[i] = 0;
        for (uint8_t w = 0; w < 6; ++w) {
          sample_words_[i][w] = 0;
        }
        break;
      }
    }
  }
  return std::nullopt;
}

bool Rp2350DshotTelemetryReceiver::DecodeFrame(uint8_t channel,
                                               DshotTelemetryFrame& frame) {
  const uint8_t samples_per_bit = config_.samples_per_bit;
  const uint16_t bits = 16;
  const uint16_t total_samples = bits * samples_per_bit;
  if (total_samples > 192) {
    return false;
  }

  uint16_t raw = 0;
  for (uint16_t bit = 0; bit < bits; ++bit) {
    uint16_t ones = 0;
    for (uint8_t s = 0; s < samples_per_bit; ++s) {
      const uint16_t sample_index = bit * samples_per_bit + s;
      const uint16_t word = sample_index / 32;
      const uint16_t bit_in_word = sample_index % 32;
      const bool sample = (sample_words_[channel][word] >> bit_in_word) & 0x1;
      ones += sample ? 1 : 0;
    }
    const bool bit_set = ones >= (samples_per_bit / 2 + 1);
    raw = static_cast<uint16_t>((raw << 1) | (bit_set ? 1 : 0));
  }

  frame.channel = channel;
  frame.raw = raw;
  const uint16_t data = static_cast<uint16_t>(raw >> 4);
  const uint16_t crc = static_cast<uint16_t>(raw & 0xF);
  const uint16_t expected_crc =
      static_cast<uint16_t>((data ^ (data >> 4) ^ (data >> 8)) & 0xF);
  frame.data = data;
  frame.crc_ok = crc == expected_crc;
  return true;
}

}  // namespace flight::actuators
