#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace esphome {

template<typename T> class optional {
 public:
  optional() = default;
  optional(const T &value) : value_(value), has_value_(true) {}  // NOLINT
  bool has_value() const { return this->has_value_; }
  const T &operator*() const { return this->value_; }
  T *operator->() { return &this->value_; }

 protected:
  T value_{};
  bool has_value_{false};
};

template<typename T> T clamp(T value, T min, T max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

/// Same shape as ESPHome's helper: "AA.BB.CC (3)", the count only past 4 bytes.
inline std::string format_hex_pretty(const std::vector<uint8_t> &data) {
  std::string out;
  char byte[4];
  for (size_t i = 0; i < data.size(); i++) {
    snprintf(byte, sizeof(byte), i == 0 ? "%02X" : ".%02X", data[i]);
    out += byte;
  }
  if (data.size() > 4)
    out += " (" + std::to_string(data.size()) + ")";
  return out;
}

}  // namespace esphome
