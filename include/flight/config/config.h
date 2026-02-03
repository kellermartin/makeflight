#pragma once

#include <cstdint>
#include <string_view>

namespace flight::config {

/** @brief Simple key/value parameter. */
struct Parameter {
  const char* key = nullptr;
  float value = 0.0f;
};

/** @brief Configuration storage interface (flash-backed). */
class IConfigStore {
 public:
  virtual ~IConfigStore() = default;
  /** @brief Load parameters from storage. */
  virtual bool Load() = 0;
  /** @brief Save parameters to storage. */
  virtual bool Save() = 0;
  /** @brief Set parameter value by key. */
  virtual bool Set(std::string_view key, float value) = 0;
  /** @brief Get parameter value by key. */
  virtual bool Get(std::string_view key, float& value_out) const = 0;
};

}  // namespace flight::config
