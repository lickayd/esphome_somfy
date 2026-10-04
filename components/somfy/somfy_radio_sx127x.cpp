#include "somfy_radio_sx127x.h"

#ifdef USE_SOMFY_IOHC_SX127X

#include "iohc_protocol.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace somfy {

static const char *const TAG = "somfy.iohc.sx127x";

void IohcRadioSX127x::setup_radio(IohcRadioListener *listener) {
  this->listener_ = listener;
  this->radio_->register_listener(this);
}

void IohcRadioSX127x::dump_config() {
  ESP_LOGCONFIG(TAG, "  SX127x: configured, RX window %u bytes, 1W only", this->rx_window_);
  if (this->wake_preamble_ > 0) {
    ESP_LOGCONFIG(TAG, "  Wake preamble: %u bytes on the first copy, %u on repeats", this->wake_preamble_,
                  this->preamble_size_);
  }
}

// ---------------------------------------------------------------------------
// Radio configuration
// ---------------------------------------------------------------------------

void IohcRadioSX127x::configure_1w(float frequency) {
  const auto hz = static_cast<uint32_t>(frequency + 0.5f);
  if (this->rx_configured_ && this->rx_frequency_ == hz)
    return;
  this->radio_->set_frequency(hz);
  this->radio_->set_payload_length(this->rx_window_);
  this->radio_->set_preamble_size(this->preamble_size_);
  // Leaves the chip in RX: the sx127x block is required to use rx_start.
  this->radio_->configure();
  this->rx_configured_ = true;
  this->rx_frequency_ = hz;
}

void IohcRadioSX127x::configure_2w(float frequency) {
  ESP_LOGW(TAG, "2W is not supported on the SX127x backend (retune to %.3f MHz ignored)", frequency / 1.0e6f);
}

void IohcRadioSX127x::begin_rx() { this->radio_->set_mode_rx(); }

// ---------------------------------------------------------------------------
// TX
// ---------------------------------------------------------------------------

bool IohcRadioSX127x::send_(const std::vector<uint8_t> &frame, uint16_t preamble) {
  // Wrap the logical frame in the io-homecontrol UART-8N1 physical encoding.
  // The chip sends its preamble and the 0x57FD sync word; the codec emits the
  // 0x99 residue and the logical frame, as on the CC1101.
  auto &payload = this->tx_payload_;
  iohc_proto::uart_encode(frame.data(), frame.size(), payload);
  if (this->rx_configured_ || payload.size() != this->tx_length_ || preamble != this->tx_preamble_) {
    this->radio_->set_payload_length(static_cast<uint8_t>(payload.size()));
    this->radio_->set_preamble_size(preamble);
    this->radio_->configure();
    this->rx_configured_ = false;
    this->tx_length_ = payload.size();
    this->tx_preamble_ = preamble;
  }
  const auto err = this->radio_->transmit_packet(payload);
  if (err != sx127x::SX127xError::NONE) {
    ESP_LOGW(TAG, "TX error: %d", static_cast<int>(err));
    return false;
  }
  return true;
}

void IohcRadioSX127x::transmit_1w(const std::vector<uint8_t> &first, const std::vector<uint8_t> &repeat,
                                  uint8_t copies) {
  const bool wake = this->wake_preamble_ > 0;
  ESP_LOGD(TAG, "TX 1W: %u logical bytes, %d copies%s", static_cast<unsigned>(first.size()), copies,
           wake ? ", wake preamble" : "");
  ESP_LOGV(TAG, "TX 1W first logical: %s", format_hex_pretty(first).c_str());

  if (!this->send_(first, wake ? this->wake_preamble_ : this->preamble_size_))
    return;
  // The first copy and its repeats differ in one flag bit, so they encode to
  // the same length and the repeats go out without another reconfiguration,
  // unless the first copy had its own preamble length. In that case the
  // reconfiguration before the first repeat already is the pause.
  for (uint8_t i = 1; i < copies; i++) {
    if (!(wake && i == 1))
      delay(14);
    if (!this->send_(repeat, this->preamble_size_))
      return;
  }
}

size_t IohcRadioSX127x::transmit_2w(const std::vector<uint8_t> &frame) {
  ESP_LOGW(TAG, "2W is not supported on the SX127x backend (%u byte frame dropped)",
           static_cast<unsigned>(frame.size()));
  return 0;
}

// ---------------------------------------------------------------------------
// RX
// ---------------------------------------------------------------------------

void IohcRadioSX127x::on_packet(const std::vector<uint8_t> &raw, float rssi, float /*snr*/) {
  // A fixed-size window of raw on-air bytes captured after the 0x57FD sync
  // match. Strip the UART 8N1 framing to recover the logical frame bytes.
  auto &packet = this->rx_frame_;
  iohc_proto::uart_decode(raw.data(), raw.size(), packet);
  ESP_LOGV(TAG, "RX raw: on_air=%u decoded=%u data=%s", static_cast<unsigned>(raw.size()),
           static_cast<unsigned>(packet.size()), format_hex_pretty(raw).c_str());

  // The native component reports no RSSI in FSK mode (always 0) and has no
  // frequency-offset or link-quality reading, so the meta stays at its zeros.
  IohcRadioMeta meta;
  meta.rssi = rssi;
  this->listener_->on_radio_packet(packet, meta);
}

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC_SX127X
