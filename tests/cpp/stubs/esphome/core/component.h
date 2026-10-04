#pragma once

#include <cstdint>

#include "esphome/core/hal.h"

namespace esphome {

namespace setup_priority {
inline constexpr float DATA = 600.0f;
inline constexpr float PROCESSOR = 400.0f;
}  // namespace setup_priority

class Component {
 public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void loop() {}
  virtual void dump_config() {}
  virtual float get_setup_priority() const { return setup_priority::DATA; }
};

}  // namespace esphome
