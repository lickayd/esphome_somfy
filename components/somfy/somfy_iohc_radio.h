#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_IOHC

#include "esphome/core/component.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome {
namespace somfy {

namespace iohc {

// Radio parameters every io-homecontrol transceiver has to match, whichever
// chip it is. Chip-specific tuning (filter bandwidth, sync word alignment,
// capture window) lives with the backend that needs it.

// 1W
static constexpr float FREQUENCY_1W = 868.95e6f;
static constexpr float SYMBOL_RATE = 38400.0f;
static constexpr float FSK_DEVIATION = 19200.0f;

// 2W - 3 channel frequency hopping
static constexpr float FREQUENCY_2W_CH0 = 868.25e6f;
static constexpr float FREQUENCY_2W_CH1 = 868.95e6f;
static constexpr float FREQUENCY_2W_CH2 = 869.85e6f;
static constexpr float FREQUENCY_2W[] = {FREQUENCY_2W_CH0, FREQUENCY_2W_CH1, FREQUENCY_2W_CH2};
static constexpr uint32_t CHANNEL_DWELL_US = 2700;

}  // namespace iohc

// What the radio measured for one received packet.
struct IohcRadioMeta {
  float freq_offset{0.0f};
  float rssi{0.0f};
  uint8_t lqi{0};
};

/// Receives packets from an IohcRadio. Implemented by the hub.
class IohcRadioListener {
 public:
  // Called once per packet the radio captured, valid frame or not. `packet`
  // holds logical io-homecontrol bytes starting at ctrl0 (physical framing
  // already removed) and may run past the end of the frame when the radio
  // captures a fixed window. The buffer belongs to the radio and is valid only
  // during the call; the listener may trim it in place.
  virtual void on_radio_packet(std::vector<uint8_t> &packet, const IohcRadioMeta &meta) = 0;
};

/// The transceiver under the io-homecontrol hub.
///
/// The hub speaks in logical frames (ctrl0 .. CRC) and owns the protocol: CRC,
/// repeat policy, 2W sessions and hop timing. A backend owns one chip: its
/// register setup and the physical encoding of a logical frame on air. Frames
/// passed to the transmit methods always include their CRC.
class IohcRadio {
 public:
  // The hub configures the chip from its own setup(), so it has to be set up
  // after the ESPHome component that owns the chip. Return a priority below
  // that component's when it sets up later than the default.
  virtual float hub_setup_priority() const { return setup_priority::DATA; }

  // Called once from the hub's setup(), after the first configure_1w().
  virtual void setup_radio(IohcRadioListener *listener) = 0;

  // Whether the radio can retune fast enough for 2W channel hopping. The hub
  // refuses 2W commands on a radio that cannot, and then never calls
  // configure_2w() or transmit_2w().
  virtual bool supports_2w() const { return true; }

  // Apply the complete 1W receive configuration on `frequency` (Hz). Called
  // before and after every 1W transmit, so it must restore anything a transmit
  // or a 2W excursion changed.
  virtual void configure_1w(float frequency) = 0;
  // Retune to a 2W channel (Hz). Called on every hop, so keep it cheap.
  virtual void configure_2w(float frequency) = 0;
  // Start listening with the current configuration.
  virtual void begin_rx() = 0;

  // Send one 1W burst: `first` once, then `repeat` until `copies` frames went
  // out in total. Blocks until the burst is done or a copy fails. Leaves the
  // radio in a transmit configuration; the hub reconfigures before listening.
  virtual void transmit_1w(const std::vector<uint8_t> &first, const std::vector<uint8_t> &repeat, uint8_t copies) = 0;
  // Send one 2W frame on the current channel and restore the 2W receive
  // configuration. Returns the number of bytes put on air.
  virtual size_t transmit_2w(const std::vector<uint8_t> &frame) = 0;

  virtual void dump_config() = 0;
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC
