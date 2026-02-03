#pragma once

#include <string>
#include <unordered_map>

#include "flight/config/config.h"

namespace flight::config {

class InMemoryConfigStore final : public IConfigStore {
 public:
  bool Load() override { return true; }
  bool Save() override { return true; }
  bool Set(std::string_view key, float value) override;
  bool Get(std::string_view key, float& value_out) const override;

 private:
  std::unordered_map<std::string, float> values_;
};

}  // namespace flight::config
