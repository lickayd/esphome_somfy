// Host-side tests for the io-homecontrol hub on an SX127x.
//
// Compiles the REAL hub and SX127x backend against the recording sx127x stub
// in ./stubs. The receive vectors are raw FIFO captures taken from a physical
// Situo io remote with ESPHome's native sx127x component on an SX1276, so the
// RX path is checked against what the chip really delivers. The transmit
// checks pin the sequence the native component forces on the backend: it only
// sends packets of the configured length and only applies a new length on
// configure(), which costs a chip reset each time.

#include "../../components/somfy/iohc_protocol.h"
#include "../../components/somfy/somfy_hub_iohc.h"
#include "../../components/somfy/somfy_radio_sx127x.h"

#include "esphome/components/sx127x/sx127x.h"

#include <cstdio>
#include <string>
#include <vector>

using namespace esphome;
using namespace esphome::somfy;

// ---------------------------------------------------------------------------
// Stub runtime
// ---------------------------------------------------------------------------

namespace esphome {

static uint32_t g_millis = 1000;
static int g_delays = 0;
uint32_t millis() { return g_millis; }
uint32_t micros() { return g_millis * 1000; }
void delay(uint32_t ms) {
  g_delays++;
  g_millis += ms;
}

}  // namespace esphome

// ---------------------------------------------------------------------------
// Assertions
// ---------------------------------------------------------------------------

static int g_checks = 0;
static int g_passed = 0;

static void check(bool ok, const char *name) {
  g_checks++;
  if (ok) {
    g_passed++;
    printf("  PASS  %s\n", name);
  } else {
    printf("  FAIL  %s\n", name);
  }
}

static std::vector<uint8_t> hex(const std::string &text) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < text.size(); i += 3)
    out.push_back(static_cast<uint8_t>(std::stoul(text.substr(i, 2), nullptr, 16)));
  return out;
}

static std::string join(const std::vector<std::string> &calls) {
  std::string out;
  for (const auto &call : calls)
    out += call + "\n";
  return out;
}

// ---------------------------------------------------------------------------
// Vectors: Situo io remote 0x26A15C, captured on an SX1276 (sync 57 FD,
// 60-byte window). Each is the FIFO content and the logical frame in it.
// ---------------------------------------------------------------------------

// "Up", first copy of the burst (wake bit set in ctrl1).
static const char *const RAW_UP =
    "99.37.C0.90.04.01.7E.4C.94.2C.75.00.50.16.14.01.00.40.10.04.1D.3F.DB.B7.3D.DB.47.58.F3.F4.3D.40.FF.F8.00.00.00.3F."
    "FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.55.55.55.55.55";
static const char *const FRAME_UP = "F6.20.00.00.3F.26.A1.5C.00.01.43.00.00.00.00.70.FE.BB.E7.B7.71.E3.7E.78.81";
// "My", main parameter D200.
static const char *const RAW_MY =
    "99.37.C0.10.04.01.7E.4C.94.2C.75.00.50.16.14.97.00.40.10.05.1D.70.44.32.75.29.01.C5.F2.1C.7F.7B.7F.F8.00.3F.FF.FF."
    "FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.FF.AA.AA.AA.AA.AA.AA.AA";
static const char *const FRAME_MY = "F6.00.00.00.3F.26.A1.5C.00.01.43.D2.00.00.00.71.07.84.72.29.C0.F4.C2.FC.6F";

static const uint8_t RX_WINDOW = 60;
static const uint16_t PREAMBLE = 64;        // bytes, what a Situo io sends before a repeat
static const uint16_t WAKE_PREAMBLE = 1219;  // bytes, about 254 ms as before a Situo's first copy
// A calibrated, non-nominal frequency. The hub holds frequencies as float, which
// resolves 64 Hz steps up here; that is about the chip's own 61 Hz tuning step.
static const float FREQUENCY_1W = 868.925e6f;
static const uint32_t FREQUENCY = static_cast<uint32_t>(FREQUENCY_1W);
static const std::string RX_CONFIG =
    "set_frequency(" + std::to_string(FREQUENCY) + ")\nset_payload_length(60)\nset_preamble_size(64)\nconfigure()\n";

struct Received {
  uint32_t src{0};
  uint32_t dest{0};
  uint8_t cmd{0};
  uint8_t ctrl1{0};
  float rssi{-1.0f};
  std::vector<uint8_t> frame;
};

struct Rig {
  sx127x::SX127x chip;
  IohcRadioSX127x backend;
  SomfyIohcHub hub;
  std::vector<Received> received;

  explicit Rig(uint16_t wake_preamble = 0) : backend(&chip, RX_WINDOW, PREAMBLE, wake_preamble) {
    hub.set_radio(&backend);
    hub.set_frequency_1w(FREQUENCY_1W);
    hub.register_rx_callback([this](const IohcDecodedPacket &pkt) {
      Received r;
      r.src = pkt.src_node;
      r.dest = pkt.dest_node;
      r.cmd = pkt.cmd;
      r.ctrl1 = pkt.ctrl1;
      r.rssi = pkt.rssi;
      r.frame.assign(pkt.frame, pkt.frame + pkt.frame_len);
      received.push_back(r);
    });
    // ESPHome sets components up in descending setup priority.
    if (chip.get_setup_priority() >= hub.get_setup_priority()) {
      chip.setup();
      hub.setup();
    } else {
      hub.setup();
      chip.setup();
    }
  }
};

// A 25-byte 1W command as a cover would hand it to the hub (wake bit clear).
static std::vector<uint8_t> command_frame() {
  std::vector<uint8_t> frame = hex(FRAME_MY);
  frame[5] = 0x4A;  // a bridge identity instead of the remote's
  frame[6] = 0x1B;
  frame[7] = 0x7C;
  const uint16_t crc = crc16_kermit(frame.data(), frame.size() - 2);
  frame[frame.size() - 2] = static_cast<uint8_t>(crc & 0xFF);
  frame[frame.size() - 1] = static_cast<uint8_t>(crc >> 8);
  return frame;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

static void test_setup() {
  printf("setup\n");
  Rig rig;
  check(rig.hub.get_setup_priority() < rig.chip.get_setup_priority(),
        "the hub sets up after the sx127x component that owns the chip");
  check(!rig.chip.failed, "the chip is not configured before its own setup");
  check(join(rig.chip.calls) == RX_CONFIG + "register_listener()\nset_mode_rx()\n",
        "boot applies the hub's 1W frequency and the RX window, then listens");
}

static void test_receive() {
  printf("receive\n");
  Rig rig;
  rig.chip.inject_packet(hex(RAW_UP));
  check(rig.received.size() == 1, "a real Situo capture is dispatched to the devices");
  if (rig.received.size() == 1) {
    const auto &r = rig.received[0];
    check(r.frame == hex(FRAME_UP), "the logical frame is recovered byte for byte");
    check(r.src == 0x26A15C && r.dest == 0x00003F && r.cmd == 0x00, "source, destination and command are parsed");
    check(r.ctrl1 == 0x20, "the wake bit of the first copy is preserved");
    check(r.rssi == 0.0f, "RSSI is reported as 0 (not available in FSK mode)");
  }
  rig.chip.inject_packet(hex(RAW_MY));
  check(rig.received.size() == 2 && rig.received[1].frame == hex(FRAME_MY), "a My press (D200) is recovered");

  auto corrupt = hex(RAW_UP);
  corrupt[12] ^= 0x04;
  rig.chip.inject_packet(corrupt);
  rig.chip.inject_packet(std::vector<uint8_t>(RX_WINDOW, 0xA7));
  check(rig.received.size() == 2, "a corrupted capture and noise are dropped");
  check(rig.hub.get_rx_raw_packet_count() == 4 && rig.hub.get_rx_valid_frame_count() == 2,
        "raw and valid counters tell the two apart");
}

static void test_transmit_burst() {
  printf("transmit burst\n");
  Rig rig;
  rig.chip.calls.clear();
  rig.chip.configure_count = 0;
  g_delays = 0;

  const auto frame = command_frame();
  rig.hub.transmit_packet(frame, 4);

  check(rig.chip.transmitted.size() == 4, "all four copies are accepted by the chip");
  check(rig.chip.configure_count == 2, "the burst costs exactly two reconfigurations");
  check(g_delays == 3, "copies are 14 ms apart");
  if (rig.chip.transmitted.size() == 4) {
    std::vector<uint8_t> first, repeat;
    iohc_proto::uart_decode(rig.chip.transmitted[0].data(), rig.chip.transmitted[0].size(), first);
    iohc_proto::uart_decode(rig.chip.transmitted[1].data(), rig.chip.transmitted[1].size(), repeat);
    check(repeat == frame, "repeats carry the frame unchanged");
    check(first.size() == frame.size() && first[1] == (frame[1] | 0x20), "the first copy carries the wake bit");
    check(crc16_kermit(first.data(), first.size()) == 0, "the first copy's CRC was refreshed");
    check(rig.chip.transmitted[0][0] == 0x99, "the payload starts with the 0x99 sync residue");
    check(rig.chip.transmitted[1] == rig.chip.transmitted[2] && rig.chip.transmitted[2] == rig.chip.transmitted[3],
          "the three repeats are identical");
  }
  const size_t on_air = rig.chip.transmitted.empty() ? 0 : rig.chip.transmitted[0].size();
  const std::string tx_length = "set_payload_length(" + std::to_string(on_air) + ")\n";
  const std::string calls = join(rig.chip.calls);
  check(calls.rfind(tx_length + "set_preamble_size(64)\nconfigure()\ntransmit_packet(", 0) == 0,
        "the TX length is applied by configure() before the first copy");
  check(rig.chip.transmitted_preamble == std::vector<uint16_t>(4, PREAMBLE),
        "without a wake preamble every copy uses the configured preamble");
  const std::string tail = RX_CONFIG + "set_mode_rx()\n";
  check(calls.size() > tail.size() && calls.compare(calls.size() - tail.size(), tail.size(), tail) == 0,
        "afterwards the RX window is restored and the radio listens");
  check(rig.chip.payload_length == RX_WINDOW && rig.chip.frequency == FREQUENCY, "the chip ends in the RX configuration");

  rig.chip.inject_packet(hex(RAW_UP));
  check(rig.received.size() == 1, "reception works again after the burst");
}

static void test_wake_preamble() {
  printf("wake preamble\n");
  Rig rig(WAKE_PREAMBLE);
  rig.chip.configure_count = 0;
  g_delays = 0;

  const auto frame = command_frame();
  rig.hub.transmit_packet(frame, 4);

  check(rig.chip.transmitted.size() == 4, "all four copies go out");
  check(rig.chip.transmitted_preamble == std::vector<uint16_t>({WAKE_PREAMBLE, PREAMBLE, PREAMBLE, PREAMBLE}),
        "only the first copy gets the long wake preamble");
  check(rig.chip.configure_count == 3, "the burst costs three reconfigurations: wake, repeats, back to RX");
  check(g_delays == 2, "the reconfiguration replaces the pause before the first repeat");
  if (rig.chip.transmitted.size() == 4) {
    std::vector<uint8_t> first;
    iohc_proto::uart_decode(rig.chip.transmitted[0].data(), rig.chip.transmitted[0].size(), first);
    check(first.size() == frame.size() && first[1] == (frame[1] | 0x20), "the long preamble is on the copy with the wake bit");
  }
  check(rig.chip.payload_length == RX_WINDOW && rig.chip.preamble_size == PREAMBLE,
        "afterwards the chip is back in the RX configuration with the normal preamble");

  rig.chip.transmitted.clear();
  rig.chip.transmitted_preamble.clear();
  rig.hub.transmit_packet(frame, 1);
  check(rig.chip.transmitted_preamble == std::vector<uint16_t>({WAKE_PREAMBLE}),
        "a single-copy burst still wakes with the long preamble");
  rig.chip.transmitted_preamble.clear();
  rig.hub.transmit_packet(frame, 2);
  check(rig.chip.transmitted_preamble == std::vector<uint16_t>({WAKE_PREAMBLE, PREAMBLE}),
        "the next burst starts with the long preamble again");
}

static void test_transmit_error() {
  printf("transmit error\n");
  Rig rig;
  rig.chip.fail_transmit_at = 2;
  g_delays = 0;
  rig.hub.transmit_packet(command_frame(), 4);
  check(rig.chip.transmitted.size() == 1, "the burst stops at the failed copy");
  check(rig.chip.payload_length == RX_WINDOW, "the RX window is restored after a failed burst");

  rig.chip.transmitted.clear();
  rig.hub.transmit_packet(command_frame(), 2);
  check(rig.chip.transmitted.size() == 2, "the next burst goes out normally");
}

static void test_rejected_frame_costs_nothing() {
  printf("rejected frame\n");
  Rig rig;
  rig.chip.configure_count = 0;
  rig.hub.transmit_packet(command_frame(), 0);
  rig.hub.transmit_packet({0x01, 0x02}, 2);
  check(rig.chip.transmitted.empty() && rig.chip.configure_count == 0,
        "an invalid request neither transmits nor reconfigures the chip");
}

static void test_2w_is_refused() {
  printf("2W\n");
  Rig rig;
  rig.chip.calls.clear();
  const uint8_t key[16] = {};
  const uint8_t data[] = {0x01, 0xE7, 0xD2, 0x00};
  int callbacks = 0;
  bool success = true;
  rig.hub.send_2w_command(0x4A1B7C, 0x123456, iohc::CMD_EXECUTE, data, sizeof(data), key,
                          [&](bool ok, const IohcDecodedPacket *) {
                            callbacks++;
                            success = ok;
                          });
  check(callbacks == 1 && !success, "a 2W command fails immediately through its callback");
  rig.hub.start_2w_listen();
  g_millis += 10;
  rig.hub.loop();
  rig.hub.loop();
  check(rig.chip.calls.empty(), "the chip is neither retuned nor made to hop");
  rig.chip.inject_packet(hex(RAW_UP));
  check(rig.received.size() == 1, "1W reception is unaffected");
}

int main() {
  test_setup();
  test_receive();
  test_transmit_burst();
  test_wake_preamble();
  test_transmit_error();
  test_rejected_frame_costs_nothing();
  test_2w_is_refused();
  printf("\n%d/%d checks passed\n", g_passed, g_checks);
  return g_passed == g_checks ? 0 : 1;
}
