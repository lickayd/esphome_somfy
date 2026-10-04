#pragma once

// Vendored copy of ESPHome's time_based cover position/trigger engine.
//
// Since ESPHome 2026.07 esphome::time_based::TimeBasedCover is declared `final`
// and can no longer be used as a base class. The somfy covers only ever used it
// as a convenience for (a) duration-based position estimation and (b) the
// open/close/stop triggers that drive the radio commands. Vendoring that logic
// here keeps the component self-contained and immune to future changes in the
// upstream time_based sub-platform.

#include "esphome/components/cover/cover.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

namespace esphome {
namespace somfy {

class SomfyTimeBasedCover : public cover::Cover, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  Trigger<> *get_open_trigger() { return &this->open_trigger_; }
  Trigger<> *get_close_trigger() { return &this->close_trigger_; }
  Trigger<> *get_stop_trigger() { return &this->stop_trigger_; }
  void set_open_duration(uint32_t open_duration) { this->open_duration_ = open_duration; }
  void set_close_duration(uint32_t close_duration) { this->close_duration_ = close_duration; }
  cover::CoverTraits get_traits() override;
  void set_has_built_in_endstop(bool value) { this->has_built_in_endstop_ = value; }
  void set_manual_control(bool value) { this->manual_control_ = value; }
  void set_assumed_state(bool value) { this->assumed_state_ = value; }
  cover::CoverOperation get_last_operation() const { return this->last_operation_; }

  // Invert direction: swap which physical command Home Assistant's open and
  // close send, for an installation where "open" would otherwise run the
  // motor the wrong way (typically an awning, where the remote's Up retracts).
  // Everything the entity shows stays in Home Assistant's terms: position,
  // open_duration and close_duration describe HA's open and close.
  //
  // From config, before setup(): just the flag.
  void set_invert_direction(bool invert) { this->invert_direction_ = invert; }
  bool get_invert_direction() const { return this->invert_direction_; }
  // Restoring a stored choice at boot: the restored position was saved under
  // that same choice, so it is left alone.
  void restore_invert_direction(bool invert);
  // Changing it while running: the motor has not moved, so the same physical
  // place is now the mirrored HA position. Stops a movement in progress.
  void apply_invert_direction(bool invert);

 protected:
  void control(const cover::CoverCall &call) override;
  void stop_prev_trigger_();
  bool is_at_target_() const;
  void start_direction_(cover::CoverOperation dir);
  void recompute_position_();
  // A derived cover that animates the entity on its own while a physical
  // remote drives the motor ends that animation here, without transmitting,
  // and leaves current_operation IDLE.
  virtual void stop_remote_animation_() {}
  // Called whenever the effective invert setting changed after configuration,
  // for settings that are HA positions of a fixed physical place.
  virtual void on_invert_direction_changed_() {}

  Trigger<> open_trigger_;
  uint32_t open_duration_;
  Trigger<> close_trigger_;
  uint32_t close_duration_;
  Trigger<> stop_trigger_;

  Trigger<> *prev_command_trigger_{nullptr};
  uint32_t last_recompute_time_{0};
  uint32_t start_dir_time_{0};
  uint32_t last_publish_time_{0};
  float target_position_{0};
  bool has_built_in_endstop_{false};
  bool manual_control_{false};
  bool assumed_state_{false};
  bool invert_direction_{false};
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};
};

}  // namespace somfy
}  // namespace esphome
