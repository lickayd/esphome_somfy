#include "somfy_radio_cc1101.h"

#ifdef USE_SOMFY_IOHC_CC1101

#include "iohc_protocol.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace somfy {

// Same tag as the hub: these lines were the hub's before the radio was split
// out, and log filters and troubleshooting notes refer to them by this tag.
static const char *const TAG = "somfy.iohc.hub";

void IohcRadioCC1101::setup_radio(IohcRadioListener *listener) {
  this->listener_ = listener;
  this->cc1101_->register_listener(this);
}

void IohcRadioCC1101::dump_config() { ESP_LOGCONFIG(TAG, "  CC1101: configured"); }

// ---------------------------------------------------------------------------
// Radio configuration
// ---------------------------------------------------------------------------

void IohcRadioCC1101::configure_1w(float frequency) {
  this->cc1101_->set_frequency(frequency);
  this->cc1101_->set_modulation_type(cc1101::Modulation::MODULATION_2_FSK);
  this->cc1101_->set_symbol_rate(iohc::SYMBOL_RATE);
  this->cc1101_->set_fsk_deviation(iohc::FSK_DEVIATION);
  this->cc1101_->set_filter_bandwidth(iohc::FILTER_BW);
  this->cc1101_->set_manchester(false);
  // The logical 0xFF 0x33 sync is UART-encoded on air. The hardware-validated
  // CC1101 alignment locks on preamble tail + wrapped 0xFF (0x57FD), leaving
  // a 0x99 residue at the FIFO head. Use full front-end gain, TI's 33 dB
  // magnitude target, the lowest absolute offset, and the 6 dB relative-rise
  // detector. The relative detector admits weak remotes when they rise above
  // the local noise floor, while the carrier-qualified sync prevents false
  // 16-bit noise matches from continuously occupying the FIFO. Hardware CRC
  // remains disabled because IOHC's CRC is checked after UART decoding.
  this->cc1101_->set_sync1(iohc_proto::PHY_HW_SYNC1);
  this->cc1101_->set_sync0(iohc_proto::PHY_HW_SYNC0);
  this->cc1101_->set_magn_target(cc1101::MagnTarget::MAGN_TARGET_33DB);
  this->cc1101_->set_max_lna_gain(cc1101::MaxLnaGain::MAX_LNA_GAIN_DEFAULT);
  this->cc1101_->set_max_dvga_gain(cc1101::MaxDvgaGain::MAX_DVGA_GAIN_DEFAULT);
  this->cc1101_->set_lna_priority(true);
  this->cc1101_->set_carrier_sense_abs_thr(-8);
  this->cc1101_->set_carrier_sense_rel_thr(cc1101::CarrierSenseRelThr::CARRIER_SENSE_REL_THR_PLUS_6DB);
  this->cc1101_->set_sync_mode(cc1101::SyncMode::SYNC_MODE_16_16);
  this->cc1101_->set_carrier_sense_above_threshold(true);
  this->cc1101_->set_crc_enable(false);
  this->cc1101_->set_packet_length(iohc::RX_FIFO_WINDOW_1W);
}

void IohcRadioCC1101::configure_2w(float frequency) {
  this->cc1101_->set_frequency(frequency);
  this->cc1101_->set_packet_length(iohc::RX_FIFO_WINDOW_2W);
}

void IohcRadioCC1101::begin_rx() { this->cc1101_->begin_rx(); }

// ---------------------------------------------------------------------------
// TX
// ---------------------------------------------------------------------------

void IohcRadioCC1101::transmit_1w(const std::vector<uint8_t> &first, const std::vector<uint8_t> &repeat,
                                  uint8_t copies) {
  // Wrap the logical frame in the io-homecontrol UART-8N1 physical encoding and
  // hand the CC1101 a fixed-length packet (no variable-length prefix byte goes
  // on air). The 0x57FD hardware sync starts four preamble bits before the
  // UART-framed 0xFF; the codec emits the 0x99 residue and logical frame.
  auto &payload = this->tx_payload_;
  iohc_proto::uart_encode(first.data(), first.size(), payload);

  ESP_LOGD(TAG, "TX 1W: %u logical / %u on-air bytes, %d repeats", static_cast<unsigned>(repeat.size()),
           static_cast<unsigned>(payload.size()), copies);
  ESP_LOGV(TAG, "TX 1W first logical: %s", format_hex_pretty(first).c_str());
  ESP_LOGV(TAG, "TX 1W first on-air: %s", format_hex_pretty(payload).c_str());

  this->cc1101_->set_sync1(iohc_proto::PHY_HW_SYNC1);
  this->cc1101_->set_sync0(iohc_proto::PHY_HW_SYNC0);
  this->cc1101_->set_sync_mode(cc1101::SyncMode::SYNC_MODE_16_16);
  this->cc1101_->set_packet_length(static_cast<uint8_t>(payload.size()));

  auto err = this->cc1101_->transmit_packet(payload);
  if (err != cc1101::CC1101Error::NONE) {
    ESP_LOGW(TAG, "TX error on first copy: %d", static_cast<int>(err));
  } else if (copies > 1) {
    iohc_proto::uart_encode(repeat.data(), repeat.size(), payload);
    this->cc1101_->set_packet_length(static_cast<uint8_t>(payload.size()));
    for (uint8_t i = 1; i < copies; i++) {
      delay(14);
      err = this->cc1101_->transmit_packet(payload);
      if (err != cc1101::CC1101Error::NONE) {
        ESP_LOGW(TAG, "TX error on repeat %u: %d", i + 1, static_cast<int>(err));
        break;
      }
    }
  }
}

size_t IohcRadioCC1101::transmit_2w(const std::vector<uint8_t> &frame) {
  // Apply the same UART-8N1 physical encoding + fixed-length packet as 1W.
  auto &payload = this->tx_payload_;
  iohc_proto::uart_encode(frame.data(), frame.size(), payload);
  this->cc1101_->set_packet_length(static_cast<uint8_t>(payload.size()));
  auto err = this->cc1101_->transmit_packet(payload);
  if (err != cc1101::CC1101Error::NONE) {
    ESP_LOGW(TAG, "2W TX error: %d", static_cast<int>(err));
  }
  this->cc1101_->set_packet_length(iohc::RX_FIFO_WINDOW_2W);
  return payload.size();
}

// ---------------------------------------------------------------------------
// RX
// ---------------------------------------------------------------------------

void IohcRadioCC1101::on_packet(const std::vector<uint8_t> &raw, float freq_offset, float rssi, uint8_t lqi) {
  // The CC1101 captures a fixed-size window of raw on-air bytes after the
  // hardware sync match (0x57FD). Strip the io-homecontrol UART 8N1 framing to
  // recover the logical frame bytes (this is what the documented captures show).
  auto &packet = this->rx_frame_;
  iohc_proto::uart_decode(raw.data(), raw.size(), packet);
  ESP_LOGV(TAG,
           "RX raw: on_air=%u decoded=%u rssi=%.1f offset=%.0f lqi=%u data=%s",
           static_cast<unsigned>(raw.size()), static_cast<unsigned>(packet.size()),
           rssi, freq_offset, lqi, format_hex_pretty(raw).c_str());

  IohcRadioMeta meta;
  meta.freq_offset = freq_offset;
  meta.rssi = rssi;
  meta.lqi = lqi;
  this->listener_->on_radio_packet(packet, meta);
}

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC_CC1101
