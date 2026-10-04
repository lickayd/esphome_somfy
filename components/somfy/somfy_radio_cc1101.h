#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_IOHC_CC1101

#include "somfy_iohc_radio.h"
#include "esphome/components/cc1101/cc1101.h"
#include <cstdint>
#include <vector>

namespace esphome {
namespace somfy {

namespace iohc {

static constexpr float FILTER_BW = 100000.0f;

// Logical sync bytes are 0xFF 0x33. On air they are UART-encoded, so the CC1101
// hardware sync word is programmed to iohc_proto::PHY_HW_SYNC1/0 (0x57FD), a
// preamble-tail-aligned window of the encoded sequence — see iohc_protocol.h.

// Fixed-length RX capture windows (raw on-air bytes collected after a sync
// match). The hardware-validated 1W receiver uses 60 bytes so a complete frame
// can be recovered from the remote's repeated burst despite CC1101 sync/FIFO
// alignment. Shortening it to the encoded application-frame size prevented
// the FIFO from completing on real Situo presses. Both windows must stay <= 64
// bytes (the CC1101 FIFO depth); the software UART decoder and ctrl0 length
// field discard trailing bytes.
static constexpr uint8_t RX_FIFO_WINDOW_1W = 60;
static constexpr uint8_t RX_FIFO_WINDOW_2W = 60;

}  // namespace iohc

/// io-homecontrol on ESPHome's native cc1101 component in packet mode. The chip
/// cannot UART-frame the payload, so this backend does it in software.
class IohcRadioCC1101 : public IohcRadio, public cc1101::CC1101Listener {
 public:
  explicit IohcRadioCC1101(cc1101::CC1101Component *cc1101) : cc1101_(cc1101) {}

  void setup_radio(IohcRadioListener *listener) override;
  void configure_1w(float frequency) override;
  void configure_2w(float frequency) override;
  void begin_rx() override;
  void transmit_1w(const std::vector<uint8_t> &first, const std::vector<uint8_t> &repeat, uint8_t copies) override;
  size_t transmit_2w(const std::vector<uint8_t> &frame) override;
  void dump_config() override;

  // CC1101Listener interface
  void on_packet(const std::vector<uint8_t> &raw, float freq_offset, float rssi, uint8_t lqi) override;

 protected:
  cc1101::CC1101Component *cc1101_;
  IohcRadioListener *listener_{nullptr};

  // Reusable UART-codec buffers, so TX and RX do not allocate per packet.
  std::vector<uint8_t> tx_payload_;
  std::vector<uint8_t> rx_frame_;
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC_CC1101
