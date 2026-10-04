#pragma once

#include "esphome/core/defines.h"

#ifdef USE_SOMFY_IOHC

#include "esphome/core/component.h"
#include "somfy_iohc_radio.h"
#include <cstdint>
#include <functional>
#include <vector>

namespace esphome {
namespace somfy {

// io-homecontrol protocol constants
namespace iohc {

// Broadcast address
static constexpr uint32_t BROADCAST_ADDR = 0x00003F;

// 2W protocol timing
static constexpr uint32_t SESSION_TIMEOUT_MS = 3000;
static constexpr uint8_t SESSION_MAX_RETRIES = 2;

// 2W command IDs
static constexpr uint8_t CMD_EXECUTE = 0x00;
static constexpr uint8_t CMD_PRIVATE_ACK = 0x21;
static constexpr uint8_t CMD_ASK_CHALLENGE = 0x31;
static constexpr uint8_t CMD_KEY_TRANSFER = 0x32;
static constexpr uint8_t CMD_KEY_TRANSFER_ACK = 0x33;
static constexpr uint8_t CMD_LAUNCH_KEY_TRANSFER = 0x38;
static constexpr uint8_t CMD_CHALLENGE_REQUEST = 0x3C;
static constexpr uint8_t CMD_CHALLENGE_RESPONSE = 0x3D;
static constexpr uint8_t CMD_STATUS = 0xFE;

// 2W frame control byte flags
static constexpr uint8_t CTRL1_2W = 0x00;       // no Start/End framing bits

}  // namespace iohc

// Decoded CRC-valid iohc RX packet. The frame/data pointers remain valid only
// for the duration of the registered RX callbacks; consumers that need them
// later must copy the bytes.
struct IohcDecodedPacket {
  uint8_t ctrl0{0};
  uint8_t ctrl1{0};
  uint32_t dest_node{0};
  uint32_t src_node{0};
  uint8_t cmd{0};
  const uint8_t *data{nullptr};
  size_t data_len{0};
  const uint8_t *frame{nullptr};
  size_t frame_len{0};
  float rssi{0};
  uint8_t lqi{0};
};

// Callback for iohc RX packet notifications
using IohcRxCallback = std::function<void(const IohcDecodedPacket &pkt)>;

// 2W session completion callback: success flag + optional response data
using Session2WCallback = std::function<void(bool success, const IohcDecodedPacket *response)>;

// CRC-16-KERMIT helper (shared between hub and devices)
uint16_t crc16_kermit(const uint8_t *data, size_t len);

// AES-128 ECB helper (shared between hub and devices)
void aes128_ecb_encrypt(const uint8_t key[16], const uint8_t plaintext[16], uint8_t ciphertext[16]);

// 2W challenge response computation
void compute_2w_response(const uint8_t key[16], const uint8_t *frame_data, size_t frame_len,
                         const uint8_t challenge[6], uint8_t response[6]);

// 2W state machine states
enum class Session2WState : uint8_t {
  IDLE,
  CMD_SENT,         // Command sent, waiting for 0x3C challenge
  CHALLENGE_RCVD,   // Challenge received, response being sent
  WAIT_STATUS,      // Waiting for status/ack from actuator
  COMPLETE,         // Session done
  FAILED,           // Session timed out or error
};

// A pending 2W session (one per command exchange)
struct Session2W {
  Session2WState state{Session2WState::IDLE};
  uint32_t src_node{0};
  uint32_t dest_node{0};
  uint8_t cmd{0};
  std::vector<uint8_t> cmd_data;
  std::vector<uint8_t> frame_payload;  // cmd+data (authenticated MAC input)
  uint8_t key[16]{};
  uint8_t challenge[6]{};
  uint32_t started_ms{0};
  uint8_t retries{0};
  Session2WCallback callback;
};

/// iohc radio hub — owns the protocol side of one radio: framing, CRC, 2W
/// sessions and hop timing. The chip itself sits behind IohcRadio.
/// Devices register for RX callbacks and call transmit_packet() for TX.
class SomfyIohcHub : public Component,
                     public IohcRadioListener {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Configuration
  void set_radio(IohcRadio *radio) { this->radio_ = radio; }
  void set_frequency_1w(float frequency) { this->frequency_1w_ = frequency; }

  // TX: transmit a raw packet (frame bytes including CRC) with repeats (1W mode)
  void transmit_packet(const std::vector<uint8_t> &frame, uint8_t repeat_count);

  // 2W TX: send a command with challenge/response authentication
  void send_2w_command(uint32_t src_node, uint32_t dest_node, uint8_t cmd,
                       const uint8_t *data, size_t data_len,
                       const uint8_t key[16], Session2WCallback callback);

  // After TX, return to RX mode
  void begin_rx();

  // Radio mode configuration
  void configure_radio_1w();
  void configure_radio_2w(uint8_t channel);

  // 2W listening control
  void start_2w_listen();
  void stop_2w_listen();

  // RX: register a device to receive decoded packets
  void register_rx_callback(IohcRxCallback callback) {
    this->rx_callbacks_.push_back(std::move(callback));
  }

  // Monotonic boot-local counters for reception diagnostics. A raw packet has
  // passed the radio's sync/FIFO layer; a valid frame also passed physical
  // decoding, length validation, and protocol CRC.
  uint32_t get_rx_raw_packet_count() const { return this->rx_raw_packet_count_; }
  uint32_t get_rx_valid_frame_count() const { return this->rx_valid_frame_count_; }
  float get_last_raw_frequency_offset() const { return this->last_raw_frequency_offset_; }
  float get_last_raw_rssi() const { return this->last_raw_rssi_; }
  float get_last_valid_rssi() const { return this->last_valid_rssi_; }

  // IohcRadioListener interface
  void on_radio_packet(std::vector<uint8_t> &packet, const IohcRadioMeta &meta) override;

 protected:
  IohcRadio *radio_{nullptr};
  float frequency_1w_{iohc::FREQUENCY_1W};

  // 2W hopping state
  bool listening_2w_{false};
  uint8_t current_2w_channel_{0};
  uint32_t last_hop_us_{0};

  // RX dispatch
  std::vector<IohcRxCallback> rx_callbacks_;
  uint32_t rx_raw_packet_count_{0};
  uint32_t rx_valid_frame_count_{0};
  float last_raw_frequency_offset_{0.0f};
  float last_raw_rssi_{0.0f};
  float last_valid_rssi_{0.0f};

  // 2W session management
  Session2W session_;
  void session_loop_();
  void handle_2w_packet_(const IohcDecodedPacket &pkt);
  void send_2w_frame_(uint32_t src, uint32_t dest, uint8_t cmd,
                      const uint8_t *data, size_t data_len);
  std::vector<uint8_t> build_2w_frame_(uint32_t src, uint32_t dest, uint8_t cmd,
                                        const uint8_t *data, size_t data_len);
};

}  // namespace somfy
}  // namespace esphome

#endif  // USE_SOMFY_IOHC
