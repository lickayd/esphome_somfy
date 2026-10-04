#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_IOHC_SX127X

#include "somfy_iohc_radio.h"
#include "esphome/components/sx127x/sx127x.h"
#include <cstdint>
#include <vector>

namespace esphome {
namespace somfy {

/// io-homecontrol 1W on ESPHome's native sx127x component in FSK packet mode.
///
/// Like the CC1101, the chip is used as a plain bit pipe: it generates the
/// preamble and matches the preamble-tail-aligned sync word, and the UART-8N1
/// framing of the payload is done here in software.
///
/// The modem settings (bit rate, deviation, sync word, preamble, bandwidth, PA)
/// come from the `sx127x:` block and are checked at config time; this backend
/// only changes what differs between listening and sending: the carrier
/// frequency and the fixed packet length. The native component transmits only
/// packets of exactly the configured length and resets the chip on every
/// configure(), which takes about 18 ms on an ESP32-S3. A 1W burst therefore
/// costs one reconfiguration before it and one after it.
///
/// A physical io remote sends the first copy of a burst, the one carrying the
/// wake bit, behind a much longer preamble than its repeats (a Situo: about
/// 254 ms against 13 ms). A non-zero wake_preamble makes this backend do the same,
/// for receivers that need the long preamble to wake up. It costs one more
/// reconfiguration, between the first copy and the repeats.
///
/// 2W is not supported: a hop would need a reconfiguration several times
/// longer than the 2.7 ms channel dwell.
class IohcRadioSX127x : public IohcRadio, public sx127x::SX127xListener {
 public:
  // preamble_size: transmit preamble in bytes. wake_preamble: preamble bytes in
  // front of the first copy of a 1W burst; 0 = same as the repeats.
  IohcRadioSX127x(sx127x::SX127x *radio, uint8_t rx_window, uint16_t preamble_size, uint16_t wake_preamble = 0)
      : radio_(radio), rx_window_(rx_window), preamble_size_(preamble_size), wake_preamble_(wake_preamble) {}

  // The native sx127x component sets up its SPI device and reset pin at
  // PROCESSOR priority, later than the hub's default. Configuring the chip
  // before that fails its version check and marks the component failed.
  float hub_setup_priority() const override { return setup_priority::PROCESSOR - 1.0f; }
  void setup_radio(IohcRadioListener *listener) override;
  bool supports_2w() const override { return false; }
  void configure_1w(float frequency) override;
  void configure_2w(float frequency) override;
  void begin_rx() override;
  void transmit_1w(const std::vector<uint8_t> &first, const std::vector<uint8_t> &repeat, uint8_t copies) override;
  size_t transmit_2w(const std::vector<uint8_t> &frame) override;
  void dump_config() override;

  // SX127xListener interface
  void on_packet(const std::vector<uint8_t> &raw, float rssi, float snr) override;

 protected:
  sx127x::SX127x *radio_;
  IohcRadioListener *listener_{nullptr};
  // Fixed-length RX capture window: raw on-air bytes collected after a sync
  // match. Taken from the sx127x block's payload_length.
  uint8_t rx_window_;
  // Transmit preamble in bytes, from the sx127x block's preamble_size.
  uint16_t preamble_size_;
  uint16_t wake_preamble_;

  // What the chip is currently configured for, so the hub's "restore the 1W
  // receive configuration" calls only cost a reconfiguration when a transmit
  // actually changed something.
  bool rx_configured_{false};
  uint32_t rx_frequency_{0};
  // While not in the RX configuration: the packet length and preamble the chip
  // was last configured to transmit with.
  size_t tx_length_{0};
  uint16_t tx_preamble_{0};

  // Reusable UART-codec buffers, so TX and RX do not allocate per packet.
  std::vector<uint8_t> tx_payload_;
  std::vector<uint8_t> rx_frame_;

  bool send_(const std::vector<uint8_t> &frame, uint16_t preamble);
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC_SX127X
