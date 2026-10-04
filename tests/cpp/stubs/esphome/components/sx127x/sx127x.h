#pragma once

#include "esphome/core/component.h"

#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

// Recording stand-in for ESPHome's native sx127x component. It mirrors the two
// constraints of the real one that shape the io-homecontrol backend: a packet
// is only transmitted when its size equals the configured payload_length, and
// a changed setting only takes effect on configure().
namespace esphome {
namespace sx127x {

enum class SX127xError { NONE = 0, TIMEOUT, INVALID_PARAMS };

class SX127xListener {
 public:
  virtual ~SX127xListener() = default;
  virtual void on_packet(const std::vector<uint8_t> &packet, float rssi, float snr) = 0;
};

class SX127x : public Component {
 public:
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }
  // The real setup() brings up the SPI device and the reset pin. Until then a
  // configure() cannot reach the chip: its version check fails and the
  // component is marked failed for good, which stops reception.
  void setup() override { this->set_up_ = true; }
  void set_frequency(uint32_t frequency) {
    this->pending_frequency_ = frequency;
    this->record_("set_frequency(" + std::to_string(frequency) + ")");
  }
  void set_payload_length(uint8_t payload_length) {
    this->pending_payload_length_ = payload_length;
    this->record_("set_payload_length(" + std::to_string(payload_length) + ")");
  }
  void set_preamble_size(uint16_t preamble_size) {
    this->pending_preamble_size_ = preamble_size;
    this->record_("set_preamble_size(" + std::to_string(preamble_size) + ")");
  }
  void configure() {
    if (!this->set_up_)
      this->failed = true;
    this->frequency = this->pending_frequency_;
    this->preamble_size = this->pending_preamble_size_;
    this->payload_length = this->pending_payload_length_;
    this->configure_count++;
    this->record_("configure()");
  }
  void set_mode_rx() { this->record_("set_mode_rx()"); }

  SX127xError transmit_packet(const std::vector<uint8_t> &packet) {
    std::string line = "transmit_packet(";
    char byte[3];
    for (const uint8_t value : packet) {
      snprintf(byte, sizeof(byte), "%02X", value);
      line += byte;
    }
    this->record_(line + ")");
    if (packet.size() != this->payload_length)
      return SX127xError::INVALID_PARAMS;
    if (this->fail_transmit_at > 0 && --this->fail_transmit_at == 0)
      return SX127xError::TIMEOUT;
    this->transmitted.push_back(packet);
    this->transmitted_preamble.push_back(this->preamble_size);
    return SX127xError::NONE;
  }
  void register_listener(SX127xListener *listener) {
    this->record_("register_listener()");
    this->listeners_.push_back(listener);
  }

  // Test controls ---------------------------------------------------------
  std::vector<std::string> calls;
  std::vector<std::vector<uint8_t>> transmitted;
  // The preamble size in force when each packet in `transmitted` went out.
  std::vector<uint16_t> transmitted_preamble;
  // The configuration the chip is actually running, i.e. as of the last configure().
  uint32_t frequency{0};
  uint8_t payload_length{0};
  uint16_t preamble_size{0};
  int configure_count{0};
  bool failed{false};
  /// Make the Nth transmit_packet() from now fail (1 = the next one).
  int fail_transmit_at{0};
  /// What the real component does when a fixed-length capture is complete.
  /// FSK mode reports neither RSSI nor SNR, hence the zeros.
  void inject_packet(const std::vector<uint8_t> &packet) {
    for (auto *listener : this->listeners_)
      listener->on_packet(packet, 0.0f, 0.0f);
  }

 protected:
  void record_(const std::string &line) { this->calls.push_back(line); }

  bool set_up_{false};
  uint32_t pending_frequency_{0};
  uint8_t pending_payload_length_{0};
  uint16_t pending_preamble_size_{0};
  std::vector<SX127xListener *> listeners_;
};

}  // namespace sx127x
}  // namespace esphome
