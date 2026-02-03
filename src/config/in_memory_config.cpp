/**
 * @file in_memory_config.cpp
 * @brief In-memory config store implementation (non-persistent).
 */

#include "flight/config/in_memory_config.h"

namespace flight::config {

/** @brief Set a parameter value. */
bool InMemoryConfigStore::Set(std::string_view key, float value) {
  values_[std::string(key)] = value;
  return true;
}

/** @brief Get a parameter value. */
bool InMemoryConfigStore::Get(std::string_view key, float& value_out) const {
  auto it = values_.find(std::string(key));
  if (it == values_.end()) {
    return false;
  }
  value_out = it->second;
  return true;
}

}  // namespace flight::config
