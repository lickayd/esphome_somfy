// Host-side regression tests for the Somfy RTS cover.
//
// These compile the REAL production sources (somfy_rts.cpp, somfy_hub_rts.cpp,
// somfy_time_based_cover.cpp) against minimal ESPHome stubs in ./stubs, so the
// behaviour under test is the code that ships — not a re-implementation.
//
// Scope: the RX state-sync feature, i.e. "pressing a button on the physical
// remote keeps the Home Assistant entity in sync". That path has no coverage in
// the ESPHome compile matrix and can therefore break silently — a successful
// firmware build proves nothing about it.
//
// The 433 MHz demodulator (SomfyRtsHub::decode_frame_) is deliberately NOT
// driven from synthetic timings here; it is exercised by the real radio. Below
// the cover level this only verifies that a frame decoded by the hub reaches
// the covers registered during setup().

#include "../../components/somfy/somfy_rts.h"

#include "esphome/components/button/button.h"
#include "esphome/components/logger/logger.h"
#include "esphome/components/remote_receiver/remote_receiver.h"
#include "esphome/components/remote_transmitter/remote_transmitter.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/application.h"

#include <cmath>
#include <cstdio>
#include <string>

// ---------------------------------------------------------------------------
// Stub runtime
// ---------------------------------------------------------------------------

namespace esphome {

static uint32_t g_millis = 0;
uint32_t millis() { return g_millis; }

Application App;  // NOLINT

namespace logger {
static Logger g_logger;
Logger *global_logger = &g_logger;
}  // namespace logger

}  // namespace esphome

// In-memory replacement for the ESP32 NVS-backed rolling code counter. The real
// implementation is exercised on-device; here codes only need to advance.
static uint16_t g_rolling_code = 1;
NVSRollingCodeStorage::NVSRollingCodeStorage(const char *name, const char *key, uint16_t initial_code)
    : name_(name), key_(key), initial_code_(initial_code) {}
NVSRollingCodeStorage::~NVSRollingCodeStorage() = default;
uint16_t NVSRollingCodeStorage::nextCode() { return g_rolling_code++; }

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

static void check_close(float actual, float expected, float tol, const char *name) {
  const bool ok = std::fabs(actual - expected) <= tol;
  g_checks++;
  if (ok) {
    g_passed++;
    printf("  PASS  %s\n", name);
  } else {
    printf("  FAIL  %s (got %.4f, want %.4f +/- %.4f)\n", name, actual, expected, tol);
  }
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

using namespace esphome;

namespace {

constexpr uint32_t REMOTE_CODE = 0x123456;
constexpr uint32_t FOREIGN_CODE = 0xABCDEF;
constexpr uint32_t BRIDGE_CODE = 0x654321;
constexpr uint32_t TRAVEL_MS = 10000;

/// Exposes the protected surface the tests need to drive and observe.
class TestCover : public somfy::SomfyCover {
 public:
  using SomfyCover::close;
  using SomfyCover::control;
  using SomfyCover::on_rts_frame_;
  using SomfyCover::open;
  using SomfyCover::rx_sync_;
  using SomfyCover::stop;

  bool rx_active() const { return this->rx_sync_.active(); }
};

somfy::RtsDecodedFrame make_frame(uint32_t remote_code, somfy::RtsCommand command) {
  somfy::RtsDecodedFrame frame{};
  frame.remote_code = remote_code;
  frame.command = command;
  frame.rolling_code = 0x0042;
  return frame;
}

/// Turn a transmitted buffer into what the receiver reports off the air.
///
/// The encoder emits one entry per Manchester half-symbol, so two consecutive
/// half-symbols at the same level appear as two entries; on the air they are a
/// single pulse of twice the length. Feeding the raw TX buffer to the decoder
/// therefore does not exercise the demodulator, it just yields an all-zero
/// frame that happens to satisfy the checksum.
remote_base::RawTimings as_received(const remote_base::RawTimings &transmitted) {
  remote_base::RawTimings air;
  for (const int32_t value : transmitted) {
    if (!air.empty() && ((air.back() < 0) == (value < 0)))
      air.back() += value;
    else
      air.push_back(value);
  }
  return air;
}

struct Rig {
  remote_transmitter::RemoteTransmitterComponent tx;
  remote_receiver::RemoteReceiverComponent rx;
  button::Button prog;
  text_sensor::TextSensor detected;
  somfy::SomfyRtsHub hub;
  TestCover cover;

  /// @param with_receiver false models a TX-only hub (RX is optional for RTS).
  /// @param remote_code the code this rig's cover transmits with.
  explicit Rig(bool with_receiver = true, uint32_t remote_code = REMOTE_CODE) {
    g_millis = 1000;
    g_rolling_code = 1;

    this->hub.set_remote_transmitter(&this->tx);
    if (with_receiver)
      this->hub.set_remote_receiver(&this->rx);
    this->hub.setup();

    this->cover.set_hub(&this->hub);
    this->cover.set_prog_button(&this->prog);
    this->cover.set_remote_code(remote_code);
    this->cover.set_storage_namespace("somfy");
    this->cover.set_storage_key("test");
    this->cover.set_repeat_count(2);
    this->cover.set_open_duration(TRAVEL_MS);
    this->cover.set_close_duration(TRAVEL_MS);
    this->cover.set_log_text_sensor(&this->detected);
    this->cover.add_receive_remote_code(REMOTE_CODE);
    this->cover.setup();
  }

  /// Deliver a frame as if the hub had just decoded it off the air.
  void receive(uint32_t remote_code, somfy::RtsCommand command) {
    this->cover.on_rts_frame_(make_frame(remote_code, command));
  }

  /// Advance the simulated clock, running the cover's loop at ~50 Hz.
  void advance(uint32_t ms) {
    for (uint32_t elapsed = 0; elapsed < ms; elapsed += 20) {
      g_millis += 20;
      this->cover.loop();
    }
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// RTS transmit is mandatory; reception is an optional add-on.
static void test_tx_mandatory_rx_optional() {
  printf("TX mandatory / RX optional\n");

  Rig tx_only(/*with_receiver=*/false);
  check(tx_only.rx.listeners.empty(), "TX-only hub registers no RX listener");

  tx_only.cover.open();
  check(tx_only.tx.transmit_count == 1, "open() transmits a frame");
  check(!tx_only.tx.last_data.get_data().empty(), "transmitted frame carries raw timings");

  tx_only.cover.close();
  check(tx_only.tx.transmit_count == 2, "close() transmits a frame");

  tx_only.cover.stop();
  check(tx_only.tx.transmit_count == 3, "stop() transmits a frame");

  Rig with_rx;
  check(with_rx.rx.listeners.size() == 1, "hub with a receiver registers exactly one RX listener");
}

/// setup() must register the cover on the hub, otherwise decoded frames never
/// reach it. Doubles as the encoder/decoder round trip: a frame this component
/// transmits must come back out of its own demodulator unchanged.
static void test_setup_registers_cover_on_hub() {
  printf("Hub -> cover wiring\n");

  // The sender is a second bridge with its own remote code: a hub drops its
  // own transmissions, so the round trip needs a foreign transmitter.
  Rig sender(/*with_receiver=*/false, FOREIGN_CODE);
  sender.cover.open();

  Rig rig;
  g_millis += 1000;
  const int before = rig.detected.publish_count;
  rig.hub.on_receive(remote_base::RemoteReceiveData(as_received(sender.tx.last_data.get_data())));
  check(rig.detected.publish_count > before, "a frame decoded by the hub reaches the cover registered in setup()");
  check(rig.detected.last_state.find("ABCDEF") != std::string::npos, "the transmitted remote code survives the round trip");
  check(rig.detected.last_state.find("UP") != std::string::npos, "the transmitted command survives the round trip");
}

/// With transmitter and receiver on one radio the hub hears itself. Its own
/// frames must not be reported as a detected remote, nor drive the cover.
static void test_own_transmissions_are_ignored() {
  printf("Own transmissions\n");

  Rig rig;
  rig.cover.position = 0.5f;
  rig.cover.open();
  g_millis += 2000;

  const int before = rig.detected.publish_count;
  const bool consumed = rig.hub.on_receive(remote_base::RemoteReceiveData(as_received(rig.tx.last_data.get_data())));
  check(consumed, "the hub recognises its own frame as a valid RTS frame");
  check(rig.detected.publish_count == before, "its own frame is not reported as a detected remote");
  check(!rig.cover.rx_active(), "its own frame does not start a remote animation");

  Rig other(/*with_receiver=*/false, FOREIGN_CODE);
  other.cover.close();
  g_millis += 2000;
  rig.hub.on_receive(remote_base::RemoteReceiveData(as_received(other.tx.last_data.get_data())));
  check(rig.detected.publish_count == before + 1, "a frame from another transmitter is still reported");
  check(rig.detected.last_state.find("ABCDEF") != std::string::npos, "with that transmitter's code");
}

/// The regression guard: a frame from an allow-listed remote must drive the HA
/// entity through an OPENING animation and settle at the open end stop.
static void test_rx_frame_syncs_ha_state() {
  printf("RX state sync (physical remote -> HA entity)\n");

  Rig rig;
  rig.cover.position = 0.5f;
  const int publishes_before = rig.cover.publish_count;

  rig.receive(REMOTE_CODE, somfy::RtsCommand::Up);

  check(rig.cover.rx_active(), "UP frame starts the RX animation");
  check(rig.cover.current_operation == cover::COVER_OPERATION_OPENING, "entity reports OPENING");
  check(rig.cover.publish_count > publishes_before, "state is published immediately");

  // Half open with a 10 s full travel => 5 s of remaining travel.
  rig.advance(2500);
  check_close(rig.cover.position, 0.75f, 0.05f, "position advances while the motor runs");
  check(rig.cover.rx_active(), "animation still running mid-travel");

  rig.advance(3000);
  check_close(rig.cover.position, 1.0f, 0.001f, "position lands on the open end stop");
  check(!rig.cover.rx_active(), "animation finishes");
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "entity returns to IDLE");
  check_close(rig.cover.last_published_position, 1.0f, 0.001f, "final position is published to HA");
}

/// Closing mirrors opening, and travel time scales with the remaining distance.
static void test_rx_close_from_open() {
  printf("RX close\n");

  Rig rig;
  rig.cover.position = 1.0f;

  rig.receive(REMOTE_CODE, somfy::RtsCommand::Down);
  check(rig.cover.current_operation == cover::COVER_OPERATION_CLOSING, "DOWN frame reports CLOSING");

  rig.advance(5000);
  check_close(rig.cover.position, 0.5f, 0.05f, "half-way closed after half the full travel");
  check(rig.cover.rx_active(), "still closing");

  rig.advance(5200);
  check_close(rig.cover.position, 0.0f, 0.001f, "position lands on the closed end stop");
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "entity returns to IDLE");
}

/// A MY frame from the remote must halt the animation where it is.
static void test_rx_stop_halts_animation() {
  printf("RX stop\n");

  Rig rig;
  rig.cover.position = 0.0f;

  rig.receive(REMOTE_CODE, somfy::RtsCommand::Up);
  check(rig.cover.rx_active(), "UP frame starts the RX animation");

  rig.advance(3000);
  const float mid = rig.cover.position;
  check(mid > 0.1f && mid < 0.9f, "cover is part-way open");

  rig.receive(REMOTE_CODE, somfy::RtsCommand::My);
  check(!rig.cover.rx_active(), "MY frame stops the RX animation");
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "entity reports IDLE after stop");

  rig.advance(3000);
  check_close(rig.cover.position, mid, 0.05f, "position holds after stop");
}

/// Position updates must be throttled so a moving cover does not flood the API.
static void test_rx_publishes_are_throttled() {
  printf("RX publish throttling\n");

  Rig rig;
  rig.cover.position = 0.0f;
  rig.receive(REMOTE_CODE, somfy::RtsCommand::Up);

  const int after_start = rig.cover.publish_count;
  rig.advance(5000);  // 250 loop iterations at 50 Hz
  const int during = rig.cover.publish_count - after_start;

  check(during > 0, "position is published while moving");
  check(during <= 30, "publishes are throttled well below the loop rate");
}

/// Frames from remotes outside allowed_remotes must not move the entity, but
/// must still surface on the discovery text sensor so they can be paired.
static void test_foreign_remote_reported_but_ignored() {
  printf("Foreign remote handling\n");

  Rig rig;
  rig.cover.position = 0.5f;
  const int publishes_before = rig.cover.publish_count;

  rig.receive(FOREIGN_CODE, somfy::RtsCommand::Up);

  check(rig.detected.publish_count == 1, "foreign frame is published for discovery");
  check(rig.detected.last_state.find("ABCDEF") != std::string::npos, "discovery text carries the remote code");
  check(rig.detected.last_state.find("UP") != std::string::npos, "discovery text names the command");
  check(!rig.cover.rx_active(), "foreign frame does not start an animation");
  check(rig.cover.publish_count == publishes_before, "foreign frame does not touch entity state");
  check_close(rig.cover.position, 0.5f, 0.001f, "position unchanged by a foreign frame");
}

/// The RTS cover advertises itself as a positionable, assumed-state cover.
static void test_traits() {
  printf("Cover traits\n");

  Rig rig;
  auto traits = rig.cover.get_traits();
  check(!traits.get_supports_tilt(), "tilt is not advertised");
  check(traits.get_is_assumed_state(), "assumed state is advertised");
}

/// A press makes the remote send the same frame several times, ~143 ms apart,
/// and the receiver hands us each copy. Repeats must collapse, but the next
/// press must be acted on immediately however fast it follows — a time-based
/// mute cannot do both, because its dead time is longer than the repeat period.
static void test_repeat_burst_collapses_but_new_press_gets_through() {
  printf("RX burst suppression\n");

  // A second rig plays the allow-listed physical remote, so its encoder yields
  // real on-air bursts. The bridge under test transmits with a code of its own:
  // frames carrying a hub's own code are dropped as its own transmissions.
  Rig remote(/*with_receiver=*/false, REMOTE_CODE);
  Rig rig(/*with_receiver=*/true, BRIDGE_CODE);
  rig.cover.position = 0.0f;

  remote.cover.open();
  const auto up_burst = as_received(remote.tx.last_data.get_data());

  rig.hub.on_receive(remote_base::RemoteReceiveData(up_burst));
  check(rig.cover.rx_active(), "first copy of the burst starts the animation");

  const int cover_publishes = rig.cover.publish_count;
  const int discovery_publishes = rig.detected.publish_count;

  for (int repeat = 0; repeat < 4; repeat++) {
    g_millis += 143;  // measured RTS repeat-frame period
    rig.hub.on_receive(remote_base::RemoteReceiveData(up_burst));
  }
  check(rig.cover.publish_count == cover_publishes, "repeats do not restart the animation");
  check(rig.detected.publish_count == discovery_publishes, "repeats are not re-reported for discovery");

  // Same remote, next rolling code, well inside the old 150 ms dead time.
  remote.cover.stop();
  const auto my_burst = as_received(remote.tx.last_data.get_data());
  g_millis += 20;
  rig.hub.on_receive(remote_base::RemoteReceiveData(my_burst));

  check(!rig.cover.rx_active(), "a following press is acted on, not swallowed");
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "MY from the remote stops the entity");
}

/// While the animation runs it owns current_operation, so a Home Assistant
/// command has to cancel it — otherwise the entity keeps travelling to the end
/// stop and drifts away from what the motor is actually doing.
static void test_ha_command_cancels_remote_animation() {
  printf("HA command vs. physical remote\n");

  Rig rig;
  rig.cover.position = 0.0f;

  rig.receive(REMOTE_CODE, somfy::RtsCommand::Up);
  rig.advance(2000);
  check(rig.cover.rx_active(), "remote animation is running");

  const float at_stop = rig.cover.position;
  cover::CoverCall call;
  call.set_command_stop();
  rig.cover.control(call);

  check(!rig.cover.rx_active(), "HA command cancels the remote animation");
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "entity reports IDLE");

  rig.advance(4000);
  check_close(rig.cover.position, at_stop, 0.01f, "position no longer drifts to the end stop");
}

/// Which command a cover call put on the air, read back through the hub's own
/// demodulator (the discovery sensor reports "<remote> <COMMAND> <rolling>").
static std::string transmitted_command(Rig &rig) {
  // A hub drops its own transmissions, so listen with a separate bridge.
  static Rig *listener = nullptr;
  const uint32_t now = g_millis;
  if (listener == nullptr)
    listener = new Rig(/*with_receiver=*/true, FOREIGN_CODE);  // NOLINT: lives for the whole run
  g_millis = now + 2000;  // outside the repeat-burst window of the previous frame
  listener->hub.on_receive(remote_base::RemoteReceiveData(as_received(rig.tx.last_data.get_data())));
  const std::string &state = listener->detected.last_state;
  const size_t first = state.find(' ');
  const size_t second = state.find(' ', first + 1);
  return state.substr(first + 1, second - first - 1);
}

/// invert_direction swaps the physical commands behind HA's open and close and
/// the meaning of the remote's Up and Down; stop is untouched.
static void test_invert_direction_swaps_commands() {
  printf("Invert direction: commands\n");

  Rig normal;
  normal.cover.open();
  check(transmitted_command(normal) == "UP", "normal: HA open sends UP");
  normal.cover.close();
  check(transmitted_command(normal) == "DOWN", "normal: HA close sends DOWN");

  Rig rig;
  rig.cover.set_invert_direction(true);
  rig.cover.open();
  check(transmitted_command(rig) == "DOWN", "inverted: HA open sends DOWN");
  rig.cover.close();
  check(transmitted_command(rig) == "UP", "inverted: HA close sends UP");
  rig.cover.stop();
  check(transmitted_command(rig) == "MY", "inverted: HA stop still sends MY");
}

static void test_invert_direction_mirrors_remote() {
  printf("Invert direction: physical remote\n");

  Rig rig;
  rig.cover.set_invert_direction(true);
  rig.cover.position = 0.5f;

  rig.receive(REMOTE_CODE, somfy::RtsCommand::Up);
  check(rig.cover.current_operation == cover::COVER_OPERATION_CLOSING, "inverted: remote UP shows as CLOSING");
  rig.advance(2500);
  check_close(rig.cover.position, 0.25f, 0.05f, "inverted: position runs towards closed");

  rig.receive(REMOTE_CODE, somfy::RtsCommand::My);
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "inverted: remote MY still stops");

  rig.receive(REMOTE_CODE, somfy::RtsCommand::Down);
  check(rig.cover.current_operation == cover::COVER_OPERATION_OPENING, "inverted: remote DOWN shows as OPENING");
}

/// Toggling at runtime: the motor has not moved, so the entity's position
/// mirrors; restoring a stored choice at boot leaves the position alone.
static void test_invert_direction_runtime_toggle() {
  printf("Invert direction: runtime toggle\n");

  Rig rig;
  rig.cover.position = 0.8f;
  const int publishes = rig.cover.publish_count;
  rig.cover.apply_invert_direction(true);
  check(rig.cover.get_invert_direction(), "the cover is now inverted");
  check_close(rig.cover.position, 0.2f, 0.001f, "the position is mirrored");
  check(rig.cover.publish_count > publishes, "the new position is published");
  rig.cover.apply_invert_direction(true);
  check_close(rig.cover.position, 0.2f, 0.001f, "applying the same setting again changes nothing");
  rig.cover.apply_invert_direction(false);
  check_close(rig.cover.position, 0.8f, 0.001f, "toggling back restores the position");

  // While a physical-remote animation runs.
  rig.cover.position = 0.5f;
  rig.receive(REMOTE_CODE, somfy::RtsCommand::Up);
  rig.advance(1000);
  check(rig.cover.rx_active(), "remote animation is running");
  const int sent = rig.tx.transmit_count;
  const float before = rig.cover.position;
  rig.cover.apply_invert_direction(true);
  check(!rig.cover.rx_active(), "toggling ends the remote animation");
  check(rig.cover.current_operation == cover::COVER_OPERATION_IDLE, "entity reports IDLE");
  check(rig.tx.transmit_count == sent, "nothing is transmitted for a remote-driven movement");
  check_close(rig.cover.position, 1.0f - before, 0.001f, "position is mirrored from where the animation stood");
  rig.advance(3000);
  check_close(rig.cover.position, 1.0f - before, 0.001f, "position no longer moves");

  // While a movement commanded from Home Assistant runs.
  Rig moving;
  moving.cover.position = 0.0f;
  cover::CoverCall call;
  call.set_position(1.0f);
  moving.cover.control(call);
  moving.advance(2000);
  check(moving.cover.current_operation == cover::COVER_OPERATION_OPENING, "HA movement is running");
  const int sent_before = moving.tx.transmit_count;
  moving.cover.apply_invert_direction(true);
  check(moving.cover.current_operation == cover::COVER_OPERATION_IDLE, "toggling stops the HA movement");
  check(moving.tx.transmit_count == sent_before + 1 && transmitted_command(moving) == "MY",
        "a stop is sent to the motor");

  // Boot-time restore.
  Rig boot;
  boot.cover.position = 0.8f;
  boot.cover.restore_invert_direction(true);
  check(boot.cover.get_invert_direction(), "a stored choice is restored");
  check_close(boot.cover.position, 0.8f, 0.001f, "restoring does not mirror the restored position");
}

int main() {
  printf("Somfy RTS host tests\n\n");

  test_tx_mandatory_rx_optional();
  printf("\n");
  test_setup_registers_cover_on_hub();
  printf("\n");
  test_own_transmissions_are_ignored();
  printf("\n");
  test_rx_frame_syncs_ha_state();
  printf("\n");
  test_rx_close_from_open();
  printf("\n");
  test_rx_stop_halts_animation();
  printf("\n");
  test_rx_publishes_are_throttled();
  printf("\n");
  test_foreign_remote_reported_but_ignored();
  printf("\n");
  test_traits();
  printf("\n");
  test_invert_direction_swaps_commands();
  printf("\n");
  test_invert_direction_mirrors_remote();
  printf("\n");
  test_invert_direction_runtime_toggle();
  printf("\n");
  test_repeat_burst_collapses_but_new_press_gets_through();
  printf("\n");
  test_ha_command_cancels_remote_animation();

  printf("\n%d/%d checks passed\n", g_passed, g_checks);
  return g_passed == g_checks ? 0 : 1;
}
