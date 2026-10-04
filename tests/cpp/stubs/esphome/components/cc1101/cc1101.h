#pragma once

#include "esphome/core/component.h"

#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

// Recording stand-in for ESPHome's native cc1101 component. Every call the
// production code makes is reported to `sink` in order, so a suite can assert
// on the exact register/TX sequence the radio would have seen.
namespace esphome {
namespace cc1101 {

enum class CC1101Error { NONE = 0, TIMEOUT, PARAMS, CRC_ERROR, FIFO_OVERFLOW, PLL_LOCK };
enum class SyncMode : uint8_t { SYNC_MODE_NONE, SYNC_MODE_15_16, SYNC_MODE_16_16, SYNC_MODE_30_32 };
enum class Modulation : uint8_t { MODULATION_2_FSK, MODULATION_GFSK, MODULATION_ASK_OOK };
enum class MagnTarget : uint8_t { MAGN_TARGET_24DB, MAGN_TARGET_27DB, MAGN_TARGET_30DB, MAGN_TARGET_33DB };
enum class MaxLnaGain : uint8_t { MAX_LNA_GAIN_DEFAULT, MAX_LNA_GAIN_MINUS_2P6DB };
enum class MaxDvgaGain : uint8_t { MAX_DVGA_GAIN_DEFAULT, MAX_DVGA_GAIN_MINUS_1 };
enum class CarrierSenseRelThr : uint8_t {
  CARRIER_SENSE_REL_THR_DEFAULT,
  CARRIER_SENSE_REL_THR_PLUS_6DB,
  CARRIER_SENSE_REL_THR_PLUS_10DB,
  CARRIER_SENSE_REL_THR_PLUS_14DB,
};

class CC1101Listener {
 public:
  virtual ~CC1101Listener() = default;
  virtual void on_packet(const std::vector<uint8_t> &packet, float freq_offset, float rssi, uint8_t lqi) = 0;
};

class CC1101Component : public Component {
 public:
  void begin_rx() { this->record_("begin_rx()"); }

  void set_frequency(float value) { this->record_num_("set_frequency", value); }
  void set_filter_bandwidth(float value) { this->record_num_("set_filter_bandwidth", value); }
  void set_fsk_deviation(float value) { this->record_num_("set_fsk_deviation", value); }
  void set_symbol_rate(float value) { this->record_num_("set_symbol_rate", value); }
  void set_sync_mode(SyncMode value) { this->record_num_("set_sync_mode", static_cast<int>(value)); }
  void set_carrier_sense_above_threshold(bool value) { this->record_num_("set_carrier_sense_above_threshold", value); }
  void set_modulation_type(Modulation value) { this->record_num_("set_modulation_type", static_cast<int>(value)); }
  void set_manchester(bool value) { this->record_num_("set_manchester", value); }
  void set_sync1(uint8_t value) { this->record_num_("set_sync1", value); }
  void set_sync0(uint8_t value) { this->record_num_("set_sync0", value); }
  void set_magn_target(MagnTarget value) { this->record_num_("set_magn_target", static_cast<int>(value)); }
  void set_max_lna_gain(MaxLnaGain value) { this->record_num_("set_max_lna_gain", static_cast<int>(value)); }
  void set_max_dvga_gain(MaxDvgaGain value) { this->record_num_("set_max_dvga_gain", static_cast<int>(value)); }
  void set_carrier_sense_abs_thr(int8_t value) { this->record_num_("set_carrier_sense_abs_thr", value); }
  void set_carrier_sense_rel_thr(CarrierSenseRelThr value) {
    this->record_num_("set_carrier_sense_rel_thr", static_cast<int>(value));
  }
  void set_lna_priority(bool value) { this->record_num_("set_lna_priority", value); }
  void set_packet_length(uint8_t value) { this->record_num_("set_packet_length", value); }
  void set_crc_enable(bool value) { this->record_num_("set_crc_enable", value); }

  CC1101Error transmit_packet(const std::vector<uint8_t> &packet) {
    std::string line = "transmit_packet(";
    char byte[3];
    for (const uint8_t value : packet) {
      snprintf(byte, sizeof(byte), "%02X", value);
      line += byte;
    }
    this->record_(line + ")");
    if (this->fail_transmit_at > 0 && --this->fail_transmit_at == 0)
      return CC1101Error::TIMEOUT;
    return CC1101Error::NONE;
  }
  void register_listener(CC1101Listener *listener) {
    this->record_("register_listener()");
    this->listeners_.push_back(listener);
  }

  // Test controls ---------------------------------------------------------
  std::function<void(const std::string &)> sink;
  /// Make the Nth transmit_packet() from now fail (1 = the next one).
  int fail_transmit_at{0};
  /// What the real component does when its FIFO holds a completed packet.
  void inject_packet(const std::vector<uint8_t> &packet, float freq_offset, float rssi, uint8_t lqi) {
    for (auto *listener : this->listeners_)
      listener->on_packet(packet, freq_offset, rssi, lqi);
  }

 protected:
  void record_(const std::string &line) {
    if (this->sink)
      this->sink("cc1101." + line);
  }
  void record_num_(const char *name, double value) {
    char line[96];
    snprintf(line, sizeof(line), "%s(%.10g)", name, value);
    this->record_(line);
  }

  std::vector<CC1101Listener *> listeners_;
};

}  // namespace cc1101
}  // namespace esphome
