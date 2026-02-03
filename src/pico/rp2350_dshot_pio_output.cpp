/**
 * @file rp2350_dshot_pio_output.cpp
 * @brief RP2350 PIO-based DShot output scaffold.
 */

#include "flight/actuators/rp2350_dshot_pio_output.h"

#include <algorithm>
#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <pico/stdlib.h>

#include "dshot_tx.pio.h"
#include "flight/actuators/dshot_output.h"

namespace flight::actuators {

namespace {

constexpr uint8_t kSymbolTicksPerBit = 3;
constexpr uint8_t kInterFrameTicks = 6;

inline float Clamp(float value, float min_value, float max_value) {
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace

Rp2350DshotPioOutput::Rp2350DshotPioOutput(const Config& config) : config_(config) {}

bool Rp2350DshotPioOutput::Initialize() {
  // TODO: Load PIO program and configure state machine timings for DShot.
  // This implementation outputs DShot300-compatible waveforms using a simple
  // 3-tick symbol encoding (110 for '1', 100 for '0').
  if (config_.channel_count == 0 || config_.channel_count > kMaxChannels) {
    return false;
  }

  PIO pio = (config_.pio_index == 0) ? pio0 : pio1;
  const uint offset = pio_add_program(pio, &dshot_tx_program);

  const uint32_t sys_hz = clock_get_hz(clk_sys);
  const float target_hz = static_cast<float>(static_cast<uint32_t>(config_.speed)) *
                          static_cast<float>(kSymbolTicksPerBit);
  const float clkdiv = static_cast<float>(sys_hz) / target_hz;
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
    pio_sm_set_consecutive_pindirs(pio, sm, config_.pins[i], 1, true);

    pio_sm_config cfg = dshot_tx_program_get_default_config(offset);
    sm_config_set_out_pins(&cfg, config_.pins[i], 1);
    sm_config_set_out_shift(&cfg, true, true, 32);
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&cfg, clkdiv);

    pio_sm_init(pio, sm, offset, &cfg);
    pio_sm_set_enabled(pio, sm, true);
  }

  return true;
}

uint16_t Rp2350DshotPioOutput::ValueToThrottle(float value) const {
  const float clamped = Clamp(value, -1.0f, 1.0f);
  const float normalized = (clamped + 1.0f) * 0.5f;
  const float range = static_cast<float>(config_.max_throttle - config_.min_throttle);
  const float mapped = static_cast<float>(config_.min_throttle) + (normalized * range);
  return static_cast<uint16_t>(Clamp(mapped,
                                     static_cast<float>(config_.min_throttle),
                                     static_cast<float>(config_.max_throttle)));
}

void Rp2350DshotPioOutput::BuildSymbolWords(uint16_t packet,
                                            uint32_t* words,
                                            uint8_t& word_count) const {
  const uint16_t bits = 16;
  const uint16_t total_ticks = bits * kSymbolTicksPerBit + kInterFrameTicks;
  word_count = static_cast<uint8_t>((total_ticks + 31) / 32);
  for (uint8_t i = 0; i < word_count; ++i) {
    words[i] = 0;
  }

  uint16_t tick_index = 0;
  for (int bit = 15; bit >= 0; --bit) {
    const bool bit_set = (packet >> bit) & 0x1;
    const uint8_t symbol = bit_set ? 0b110 : 0b100;
    for (uint8_t s = 0; s < kSymbolTicksPerBit; ++s) {
      const uint8_t symbol_bit = (symbol >> (kSymbolTicksPerBit - 1 - s)) & 0x1;
      const uint16_t word = tick_index / 32;
      const uint16_t bit_in_word = tick_index % 32;
      if (symbol_bit) {
        words[word] |= (1u << bit_in_word);
      }
      ++tick_index;
    }
  }
  // Remaining ticks are already zero to create inter-frame low gap.
}

bool Rp2350DshotPioOutput::Write(const ActuatorCommand* commands, uint8_t count) {
  last_count_ = count > config_.channel_count ? config_.channel_count : count;
  if (last_count_ == 0) {
    return true;
  }

  PIO pio = (config_.pio_index == 0) ? pio0 : pio1;

  for (uint8_t i = 0; i < last_count_; ++i) {
    const uint16_t throttle = ValueToThrottle(commands[i].value);
    const uint16_t packet =
        DshotOutput::PackCommand(throttle, config_.enable_telemetry);

    uint32_t words[2] = {0, 0};
    uint8_t word_count = 0;
    BuildSymbolWords(packet, words, word_count);

    const uint8_t sm = active_sms_[i];
    for (uint8_t w = 0; w < word_count; ++w) {
      pio_sm_put_blocking(pio, sm, words[w]);
    }
  }
  return true;
}

}  // namespace flight::actuators
