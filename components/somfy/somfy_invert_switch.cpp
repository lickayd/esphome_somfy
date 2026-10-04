#include "somfy_invert_switch.h"

#ifdef USE_SWITCH

#include "esphome/core/log.h"

namespace esphome {
namespace somfy {

static const char *const TAG = "somfy.invert_switch";

void SomfyInvertSwitch::setup() {
  const optional<bool> stored = this->get_initial_state();
  const bool invert = stored.has_value() ? *stored : this->cover_->get_invert_direction();
  this->cover_->restore_invert_direction(invert);
  this->publish_state(invert);
}

void SomfyInvertSwitch::dump_config() { LOG_SWITCH("", "Somfy Invert Direction", this); }

void SomfyInvertSwitch::write_state(bool state) {
  this->cover_->apply_invert_direction(state);
  this->publish_state(state);
}

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SWITCH
