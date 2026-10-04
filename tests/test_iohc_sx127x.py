"""Config validation for the io-homecontrol hub on ESPHome's native sx127x."""

from pathlib import Path

import pytest
import somfy as hub_mod
from somfy import cover as cover_mod

ROOT = Path(__file__).parent.parent
RADIO_CPP = (ROOT / "components/somfy/somfy_radio_sx127x.cpp").read_text()
HUB_CPP = (ROOT / "components/somfy/somfy_hub_iohc.cpp").read_text()
HUB_PY = (ROOT / "components/somfy/__init__.py").read_text()

# The `sx127x:` block the backend is hardware-tested with (as ESPHome hands it
# to final validation: frequencies in Hz, enums as their YAML names).
GOOD_RADIO = {
    "id": "sx1276_radio",
    "modulation": "FSK",
    "packet_mode": True,
    "bitsync": True,
    "crc_enable": False,
    "rx_start": True,
    "bitrate": 38400,
    "deviation": 19200,
    "preamble_polarity": 0x55,
    "preamble_size": 32,
    "payload_length": 60,
    "sync_value": [0x57, 0xFD],
    "dio0_pin": {"number": 16},
}


def test_accepts_the_hardware_tested_block():
    assert hub_mod.validate_sx127x_radio(dict(GOOD_RADIO)) == GOOD_RADIO


def test_accepts_a_longer_sync_word_with_preamble_bytes_in_front():
    radio = {**GOOD_RADIO, "sync_value": [0x55, 0x55, 0x57, 0xFD]}
    assert hub_mod.validate_sx127x_radio(radio) == radio


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"modulation": "LORA"}, "modulation: FSK"),
        ({"modulation": "OOK"}, "modulation: FSK"),
        ({"packet_mode": False}, "packet_mode: true"),
        ({"bitsync": False}, "bitsync: true"),
        ({"crc_enable": True}, "crc_enable: false"),
        ({"rx_start": False}, "rx_start: true"),
        ({"bitrate": 4800}, "bitrate: 38400"),
        ({"deviation": 5000}, "deviation: 19.2kHz"),
        ({"preamble_polarity": 0xAA}, "preamble_polarity: 0x55"),
        ({"sync_value": []}, "sync_value"),
        ({"sync_value": [0xFF, 0x33]}, "sync_value"),
        ({"sync_value": [0xAA, 0x57, 0xFD]}, "sync_value"),
        ({"payload_length": 0}, "payload_length"),
        ({"payload_length": 65}, "payload_length"),
        ({"preamble_size": 0}, "preamble_size"),
        ({"preamble_size": 23}, "preamble_size"),
    ],
)
def test_rejects_settings_the_backend_cannot_work_with(override, message):
    with pytest.raises(hub_mod.cv.Invalid, match=message):
        hub_mod.validate_sx127x_radio({**GOOD_RADIO, **override})


def test_rejects_a_block_without_dio0():
    radio = {key: value for key, value in GOOD_RADIO.items() if key != "dio0_pin"}
    with pytest.raises(hub_mod.cv.Invalid, match="dio0_pin"):
        hub_mod.validate_sx127x_radio(radio)


def test_finds_the_referenced_block_among_several():
    other = {**GOOD_RADIO, "id": "other_radio"}
    full_config = {"sx127x": [other, GOOD_RADIO]}
    assert hub_mod.find_sx127x_config(full_config, "sx1276_radio") is GOOD_RADIO
    assert hub_mod.find_sx127x_config(full_config, "missing") is None
    assert hub_mod.find_sx127x_config({}, "sx1276_radio") is None


def test_2w_cover_is_rejected_on_an_sx127x_hub():
    hub = {"id": "iohc_hub", "type": "iohc", "sx127x_id": "sx1276_radio"}
    with pytest.raises(cover_mod.cv.Invalid, match="1W only"):
        cover_mod.validate_iohc_radio({"mode": "2w"}, hub)


def test_1w_cover_is_accepted_on_an_sx127x_hub():
    hub = {"id": "iohc_hub", "type": "iohc", "sx127x_id": "sx1276_radio"}
    assert cover_mod.validate_iohc_radio({"mode": "1w"}, hub) == {"mode": "1w"}
    assert cover_mod.validate_iohc_radio({}, hub) == {}


def test_2w_cover_stays_available_on_a_cc1101_hub():
    hub = {"id": "iohc_hub", "type": "iohc", "cc1101_id": "cc1101_radio"}
    assert cover_mod.validate_iohc_radio({"mode": "2w"}, hub) == {"mode": "2w"}
    # An unresolved hub is skipped rather than guessed at.
    assert cover_mod.validate_iohc_radio({"mode": "2w"}, None) == {"mode": "2w"}


def test_each_build_compiles_only_its_own_radio_backend():
    assert 'cg.add_define("USE_SOMFY_IOHC_SX127X")' in HUB_PY
    assert 'cg.add_define("USE_SOMFY_IOHC_CC1101")' in HUB_PY
    assert "#ifdef USE_SOMFY_IOHC_SX127X" in RADIO_CPP
    assert "cv.has_exactly_one_key(CONF_CC1101_ID, CONF_SX127X_ID)" in HUB_PY


def test_hub_refuses_2w_on_a_radio_that_cannot_hop():
    command = HUB_CPP.split("void SomfyIohcHub::send_2w_command", 1)[1]
    command = command.split("// Initialize session", 1)[0]
    assert "supports_2w()" in command
    assert "callback(false, nullptr)" in command
    listen = HUB_CPP.split("void SomfyIohcHub::start_2w_listen()", 1)[1]
    listen = listen.split("void SomfyIohcHub::stop_2w_listen", 1)[0]
    assert "supports_2w()" in listen


class _Period:
    """Stand-in for ESPHome's TimePeriod as config validation hands it over."""

    def __init__(self, milliseconds):
        self.total_milliseconds = milliseconds


def test_wake_preamble_is_converted_to_bytes_at_38400_bit_per_second():
    # 254 ms is what a Situo io sends in front of the first copy of a burst.
    assert hub_mod.wake_preamble_bytes(254) == 1219
    assert hub_mod.wake_preamble_bytes(10) == 48


def test_wake_preamble_is_accepted_on_an_sx127x_hub():
    config = {"sx127x_id": "sx1276_radio", "wake_preamble": _Period(254)}
    assert hub_mod.validate_wake_preamble(config) is config
    without = {"sx127x_id": "sx1276_radio"}
    assert hub_mod.validate_wake_preamble(without) is without


def test_wake_preamble_is_rejected_on_a_cc1101_hub():
    config = {"cc1101_id": "cc1101_radio", "wake_preamble": _Period(254)}
    with pytest.raises(hub_mod.cv.Invalid, match="only available with 'sx127x_id'"):
        hub_mod.validate_wake_preamble(config)


@pytest.mark.parametrize("milliseconds", [0, 2001, 60000])
def test_wake_preamble_length_is_bounded(milliseconds):
    config = {"sx127x_id": "sx1276_radio", "wake_preamble": _Period(milliseconds)}
    with pytest.raises(hub_mod.cv.Invalid, match="at most"):
        hub_mod.validate_wake_preamble(config)
