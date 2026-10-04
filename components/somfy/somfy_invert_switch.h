#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SWITCH

#include "somfy_time_based_cover.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome {
namespace somfy {

/// Runtime toggle for a somfy cover's invert_direction.
///
/// The choice is kept across reboots by the switch's own preference. Until it
/// has been toggled once, the cover's `invert_direction:` from YAML applies.
class SomfyInvertSwitch : public switch_::Switch, public Component {
 public:
  void setup() override;
  void dump_config() override;
  // After the covers: their setup() restores the position this switch must
  // leave alone at boot.
  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

  void set_cover(SomfyTimeBasedCover *cover) { this->cover_ = cover; }

 protected:
  void write_state(bool state) override;

  SomfyTimeBasedCover *cover_{nullptr};
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SWITCH
