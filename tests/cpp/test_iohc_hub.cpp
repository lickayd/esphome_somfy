// Host-side characterization test for the io-homecontrol hub on a CC1101.
//
// Compiles the REAL production sources against the recording cc1101 stub in
// ./stubs and replays a fixed script: boot, a 1W transmit burst, good and bad
// receptions, a complete 2W challenge/response session with channel hopping,
// and a 2W session that times out. Everything the radio would have seen (every
// register setter, every transmitted on-air byte), every log line and every
// packet handed to the registered devices is written to one ordered trace.
//
// That trace is compared with golden/iohc_hub_cc1101.trace. The golden file
// pins the hardware-validated CC1101 behaviour, so a change to the hub or its
// radio backend that alters what reaches the chip fails here instead of on a
// motor. Regenerate it deliberately with UPDATE_GOLDEN=1 and review the diff.

#include "../../components/somfy/iohc_protocol.h"
#include "../../components/somfy/somfy_hub_iohc.h"
#include "../../components/somfy/somfy_radio_cc1101.h"

#include "esphome/components/cc1101/cc1101.h"
#include "esphome/core/log.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace esphome;
using namespace esphome::somfy;

// ---------------------------------------------------------------------------
// Stub runtime
// ---------------------------------------------------------------------------

static std::string g_trace;
static void trace(const std::string &line) { g_trace += line + "\n"; }

namespace esphome {

static uint32_t g_millis = 1000;
static uint32_t g_micros = 1000000;
uint32_t millis() { return g_millis; }
uint32_t micros() { return g_micros; }
void delay(uint32_t ms) {
  trace("delay(" + std::to_string(ms) + ")");
  g_millis += ms;
  g_micros += ms * 1000;
}

}  // namespace esphome

static void advance_ms(uint32_t ms) {
  g_millis += ms;
  g_micros += ms * 1000;
}

static void on_log(char level, const char *tag, const char *message) {
  trace(std::string("log ") + level + " [" + tag + "] " + message);
}

static std::string to_hex(const uint8_t *data, size_t len) {
  std::string out;
  char byte[3];
  for (size_t i = 0; i < len; i++) {
    snprintf(byte, sizeof(byte), "%02X", data[i]);
    out += byte;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Frame helpers
// ---------------------------------------------------------------------------

// Logical frame ctrl0..CRC, as a 1W remote or a 2W actuator puts it on air.
static std::vector<uint8_t> logical_frame(uint8_t ctrl0_flags, uint8_t ctrl1, uint32_t dest, uint32_t src, uint8_t cmd,
                                          const std::vector<uint8_t> &data) {
  std::vector<uint8_t> frame{0x00, ctrl1};
  for (const uint32_t node : {dest, src}) {
    frame.push_back(static_cast<uint8_t>(node >> 16));
    frame.push_back(static_cast<uint8_t>(node >> 8));
    frame.push_back(static_cast<uint8_t>(node));
  }
  frame.push_back(cmd);
  frame.insert(frame.end(), data.begin(), data.end());
  frame[0] = static_cast<uint8_t>(ctrl0_flags | ((frame.size() - 1) & 0x1F));
  const uint16_t crc = crc16_kermit(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>(crc >> 8));
  return frame;
}

// What the CC1101 FIFO holds after its 0x57FD sync match: the UART-encoded
// frame, then whatever followed on air until the fixed capture window is full.
static std::vector<uint8_t> fifo_capture(const std::vector<uint8_t> &logical, size_t window = 60) {
  std::vector<uint8_t> raw;
  iohc_proto::uart_encode(logical.data(), logical.size(), raw);
  while (raw.size() < window)
    raw.push_back(0x55);
  return raw;
}

// ---------------------------------------------------------------------------
// Script
// ---------------------------------------------------------------------------

static const uint32_t REMOTE = 0x26A15C;
static const uint32_t BRIDGE = 0x4A1B7C;
static const uint32_t ACTUATOR = 0x123456;
static const uint8_t KEY[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16};

static void section(const char *name) { trace(std::string("== ") + name); }

static void counters(const SomfyIohcHub &hub) {
  char line[160];
  snprintf(line, sizeof(line), "counters raw=%u valid=%u raw_offset=%.0f raw_rssi=%.1f valid_rssi=%.1f",
           static_cast<unsigned>(hub.get_rx_raw_packet_count()), static_cast<unsigned>(hub.get_rx_valid_frame_count()),
           hub.get_last_raw_frequency_offset(), hub.get_last_raw_rssi(), hub.get_last_valid_rssi());
  trace(line);
}

static void run_script() {
  cc1101::CC1101Component radio;
  radio.sink = trace;
  IohcRadioCC1101 backend(&radio);
  SomfyIohcHub hub;
  hub.set_radio(&backend);
  hub.set_frequency_1w(868.925e6f);  // a calibrated, non-nominal frequency
  hub.register_rx_callback([](const IohcDecodedPacket &pkt) {
    char line[256];
    snprintf(line, sizeof(line),
             "device rx ctrl0=%02X ctrl1=%02X dst=%06X src=%06X cmd=%02X data=%s frame=%s rssi=%.1f lqi=%u",
             pkt.ctrl0, pkt.ctrl1, static_cast<unsigned>(pkt.dest_node), static_cast<unsigned>(pkt.src_node), pkt.cmd,
             to_hex(pkt.data, pkt.data_len).c_str(), to_hex(pkt.frame, pkt.frame_len).c_str(), pkt.rssi, pkt.lqi);
    trace(line);
  });

  section("setup");
  hub.setup();

  section("1W transmit, 4 copies");
  const auto command = logical_frame(0xC0, 0x00, iohc::BROADCAST_ADDR, BRIDGE, 0x00,
                                     {0x01, 0x61, 0xD2, 0x00, 0x00, 0x00, 0x12, 0x34, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF});
  hub.transmit_packet(command, 4);

  section("1W transmit, single copy");
  hub.transmit_packet(command, 1);

  section("1W transmit, second copy fails");
  radio.fail_transmit_at = 2;
  hub.transmit_packet(command, 4);

  section("1W transmit, first copy fails");
  radio.fail_transmit_at = 1;
  hub.transmit_packet(command, 4);

  section("1W transmit rejected");
  hub.transmit_packet(command, 0);
  hub.transmit_packet({0x01, 0x02}, 2);

  section("receive a remote press");
  const auto press = logical_frame(0xC0, 0x20, iohc::BROADCAST_ADDR, REMOTE, 0x00,
                                   {0x01, 0x61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66});
  radio.inject_packet(fifo_capture(press), -1234.0f, -61.5f, 3);
  counters(hub);

  section("receive a frame without data");
  radio.inject_packet(fifo_capture(logical_frame(0x00, 0x00, BRIDGE, ACTUATOR, 0x2E, {})), 250.0f, -70.0f, 9);
  counters(hub);

  section("receive a capture that ends with the frame");
  radio.inject_packet(fifo_capture(press, 0), 20.0f, -98.0f, 0);
  counters(hub);

  section("reject noise, a truncated capture, a short frame and a bad CRC");
  radio.inject_packet(std::vector<uint8_t>(60, 0x00), 10.0f, -99.0f, 0);
  {
    auto truncated = fifo_capture(press, 0);
    truncated.resize(12);
    radio.inject_packet(truncated, 30.0f, -97.0f, 0);
  }
  {
    std::vector<uint8_t> runt{0x05, 0x00, 0x11, 0x22, 0x33, 0x44};
    const uint16_t crc = crc16_kermit(runt.data(), runt.size());
    runt.push_back(static_cast<uint8_t>(crc & 0xFF));
    runt.push_back(static_cast<uint8_t>(crc >> 8));
    radio.inject_packet(fifo_capture(runt), 40.0f, -96.0f, 0);
  }
  {
    auto corrupt = press;
    corrupt[10] ^= 0x01;
    radio.inject_packet(fifo_capture(corrupt), 50.0f, -95.0f, 0);
  }
  counters(hub);

  section("2W session, success");
  const uint8_t execute[] = {0x01, 0xE7, 0xD2, 0x00};
  hub.send_2w_command(BRIDGE, ACTUATOR, iohc::CMD_EXECUTE, execute, sizeof(execute), KEY,
                      [](bool success, const IohcDecodedPacket *response) {
                        trace(std::string("2W callback success=") + (success ? "1" : "0") + " response_cmd=" +
                              (response != nullptr ? to_hex(&response->cmd, 1) : std::string("none")));
                      });
  section("2W busy");
  hub.send_2w_command(BRIDGE, ACTUATOR, iohc::CMD_EXECUTE, execute, sizeof(execute), KEY,
                      [](bool success, const IohcDecodedPacket *) {
                        trace(std::string("2W busy callback success=") + (success ? "1" : "0"));
                      });
  section("2W hop");
  hub.loop();  // dwell not elapsed
  advance_ms(3);
  hub.loop();
  advance_ms(3);
  hub.loop();
  advance_ms(3);
  hub.loop();
  section("2W ignores a foreign packet, then gets the challenge");
  radio.inject_packet(fifo_capture(logical_frame(0x00, 0x00, BRIDGE, 0x654321, iohc::CMD_CHALLENGE_REQUEST,
                                                 {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6})),
                      0.0f, -50.0f, 1);
  radio.inject_packet(fifo_capture(logical_frame(0x00, 0x00, BRIDGE, ACTUATOR, iohc::CMD_CHALLENGE_REQUEST,
                                                 {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6})),
                      0.0f, -51.0f, 1);
  section("2W status completes the session");
  radio.inject_packet(fifo_capture(logical_frame(0x00, 0x00, BRIDGE, ACTUATOR, iohc::CMD_STATUS, {0x00})), 0.0f,
                      -52.0f, 1);
  advance_ms(3);
  hub.loop();  // back in 1W: no hop

  section("2W session, timeout with retries");
  hub.send_2w_command(BRIDGE, ACTUATOR, iohc::CMD_EXECUTE, execute, sizeof(execute), KEY,
                      [](bool success, const IohcDecodedPacket *) {
                        trace(std::string("2W callback success=") + (success ? "1" : "0"));
                      });
  for (int i = 0; i < 3; i++) {
    advance_ms(iohc::SESSION_TIMEOUT_MS + 1);
    hub.loop();
  }
  advance_ms(3);
  hub.loop();  // session over: no hop, no retry

  section("2W transmit error");
  radio.fail_transmit_at = 1;
  hub.send_2w_command(BRIDGE, ACTUATOR, iohc::CMD_EXECUTE, execute, sizeof(execute), KEY, nullptr);
  hub.stop_2w_listen();

  section("final");
  hub.begin_rx();
  counters(hub);
}

// ---------------------------------------------------------------------------
// Golden comparison
// ---------------------------------------------------------------------------

int main() {
  const char *golden_path = "golden/iohc_hub_cc1101.trace";
  log_hook() = on_log;
  run_script();

  if (getenv("UPDATE_GOLDEN") != nullptr) {
    std::ofstream out(golden_path, std::ios::binary);
    out << g_trace;
    printf("  golden file rewritten: %s\n", golden_path);
    return 0;
  }

  std::ifstream in(golden_path, std::ios::binary);
  std::stringstream golden;
  golden << in.rdbuf();
  if (in && golden.str() == g_trace) {
    size_t lines = 0;
    for (const char c : g_trace)
      lines += c == '\n';
    printf("  PASS  hub trace matches %s (%zu lines)\n", golden_path, lines);
    return 0;
  }

  // Point at the first divergence; the full trace is too long to be useful.
  std::istringstream want(golden.str()), got(g_trace);
  std::string want_line, got_line;
  size_t number = 0;
  while (true) {
    const bool has_want = static_cast<bool>(std::getline(want, want_line));
    const bool has_got = static_cast<bool>(std::getline(got, got_line));
    number++;
    if (!has_want && !has_got)
      break;
    if (has_want != has_got || want_line != got_line) {
      printf("  FAIL  hub trace differs from %s at line %zu\n", golden_path, number);
      printf("        expected: %s\n", has_want ? want_line.c_str() : "<end of file>");
      printf("        actual:   %s\n", has_got ? got_line.c_str() : "<end of trace>");
      break;
    }
  }
  return 1;
}
