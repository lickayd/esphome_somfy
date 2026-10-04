import esphome.codegen as cg
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.components import remote_receiver, remote_transmitter, sx127x
from esphome.const import CONF_ID, CONF_TYPE, PLATFORM_ESP32
from esphome.core import CORE

CODEOWNERS = ["@LeonardPitzu"]
DEPENDENCIES = ["esp32"]
# RX state synchronisation publishes a detected-remote sensor when configured,
# and the same RX implementation is also used to model a native MY recall. Load
# the lightweight text_sensor base unconditionally so every valid combination
# of those optional fields has its C++ headers available.
AUTO_LOAD = ["button", "text_sensor"]
MULTI_CONF = True

DOMAIN = "somfy"

somfy_ns = cg.esphome_ns.namespace("somfy")
SomfyRtsHub = somfy_ns.class_("SomfyRtsHub", cg.Component)
SomfyIohcHub = somfy_ns.class_("SomfyIohcHub", cg.Component)
IohcRadio = somfy_ns.class_("IohcRadio")
IohcRadioCC1101 = somfy_ns.class_("IohcRadioCC1101", IohcRadio)
IohcRadioSX127x = somfy_ns.class_("IohcRadioSX127x", IohcRadio)

CONF_REMOTE_TRANSMITTER = "remote_transmitter"
CONF_REMOTE_RECEIVER = "remote_receiver"
CONF_CC1101_ID = "cc1101_id"
CONF_SX127X_ID = "sx127x_id"
CONF_FREQUENCY_1W = "frequency_1w"
CONF_RADIO_ID = "radio_id"
CONF_WAKE_PREAMBLE = "wake_preamble"

TYPE_RTS = "rts"
TYPE_IOHC = "iohc"

RTS_HUB_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SomfyRtsHub),
        cv.Required(CONF_REMOTE_TRANSMITTER): cv.use_id(
            remote_transmitter.RemoteTransmitterComponent
        ),
        cv.Optional(CONF_REMOTE_RECEIVER): cv.use_id(
            remote_receiver.RemoteReceiverComponent
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

# A preamble byte is 8 bits at 38400 bit/s.
PREAMBLE_BYTES_PER_MS = 38400 / 8 / 1000
# Longest wake preamble accepted. A Situo io sends about 254 ms; the native
# sx127x component gives up on a transmit after 4 s.
MAX_WAKE_PREAMBLE_MS = 2000


def wake_preamble_bytes(milliseconds):
    """Preamble length in bytes for a wake preamble given in milliseconds."""
    return round(milliseconds * PREAMBLE_BYTES_PER_MS)


def validate_wake_preamble(config):
    """`wake_preamble` needs the sx127x backend and a sane length."""
    if CONF_WAKE_PREAMBLE not in config:
        return config
    if CONF_SX127X_ID not in config:
        raise cv.Invalid(
            f"'{CONF_WAKE_PREAMBLE}' is only available with '{CONF_SX127X_ID}': the "
            "CC1101 preamble is limited to 24 bytes"
        )
    milliseconds = config[CONF_WAKE_PREAMBLE].total_milliseconds
    if not 0 < milliseconds <= MAX_WAKE_PREAMBLE_MS:
        raise cv.Invalid(
            f"'{CONF_WAKE_PREAMBLE}' must be longer than 0 and at most "
            f"{MAX_WAKE_PREAMBLE_MS}ms"
        )
    return config


IOHC_HUB_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SomfyIohcHub),
            # The hub talks to its chip through a radio backend object; the ID
            # is generated, not something a config needs to set.
            cv.GenerateID(CONF_RADIO_ID): cv.declare_id(IohcRadio),
            # Exactly one radio: ESPHome's native cc1101 or sx127x component.
            cv.Optional(CONF_CC1101_ID): cv.use_id(cg.Component),
            cv.Optional(CONF_SX127X_ID): cv.use_id(sx127x.SX127x),
            cv.Optional(CONF_FREQUENCY_1W, default="868.95MHz"): cv.All(
                cv.frequency, cv.float_range(min=860.0e6, max=870.0e6)
            ),
            # sx127x only: preamble time in front of the first copy of a 1W
            # burst, as physical remotes send it. Unset = same as the repeats.
            cv.Optional(CONF_WAKE_PREAMBLE): cv.positive_time_period_milliseconds,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.has_exactly_one_key(CONF_CC1101_ID, CONF_SX127X_ID),
    validate_wake_preamble,
)

# What the SX127x backend needs from the referenced `sx127x:` block. The chip
# is used as a bit pipe behind a software UART codec, so these are not tuning
# choices: with any other value nothing is received or the motor hears nothing.
SX127X_BITRATE = 38400
SX127X_DEVIATION = 19200
# Preamble tail + UART-framed 0xFF + the first two bits of UART-framed 0x33;
# more preamble (0x55) bytes in front are fine and reduce false triggers.
SX127X_SYNC_TAIL = [0x57, 0xFD]
SX127X_PREAMBLE_POLARITY = 0x55
# Shortest transmit preamble accepted: the value the CC1101 backend is
# hardware-validated with against real motors.
SX127X_MIN_PREAMBLE_SIZE = 24
# The FIFO depth of the chip in FSK packet mode.
SX127X_MAX_RX_WINDOW = 64


def validate_sx127x_radio(radio):
    """Check an `sx127x:` block against what the io-homecontrol backend needs."""

    def require(key, expected, shown=None):
        if radio.get(key) != expected:
            raise cv.Invalid(
                f"the sx127x used for io-homecontrol needs '{key}: "
                f"{expected if shown is None else shown}'"
            )

    require("modulation", "FSK")
    require("packet_mode", True, "true")
    require("bitsync", True, "true")
    require("crc_enable", False, "false")
    require("rx_start", True, "true")
    require("bitrate", SX127X_BITRATE)
    require("deviation", SX127X_DEVIATION, "19.2kHz")
    require("preamble_polarity", SX127X_PREAMBLE_POLARITY, "0x55")
    if "dio0_pin" not in radio:
        raise cv.Invalid("the sx127x used for io-homecontrol needs a 'dio0_pin'")
    sync = list(radio.get("sync_value") or [])
    head, tail = sync[:-2], sync[-2:]
    if tail != SX127X_SYNC_TAIL or any(byte != 0x55 for byte in head):
        raise cv.Invalid(
            "the sx127x used for io-homecontrol needs 'sync_value: [0x57, 0xFD]' "
            "(optionally with 0x55 bytes in front)"
        )
    window = radio.get("payload_length", 0)
    if not 1 <= window <= SX127X_MAX_RX_WINDOW:
        raise cv.Invalid(
            "the sx127x used for io-homecontrol needs a fixed 'payload_length' "
            f"of 1..{SX127X_MAX_RX_WINDOW} bytes (the RX capture window; 60 is the "
            "hardware-validated value)"
        )
    if radio.get("preamble_size", 0) < SX127X_MIN_PREAMBLE_SIZE:
        raise cv.Invalid(
            "the sx127x used for io-homecontrol needs 'preamble_size' of at least "
            f"{SX127X_MIN_PREAMBLE_SIZE} bytes, or motors will not hear it"
        )
    return radio


def find_sx127x_config(full_config, radio_id):
    """Return the sx127x block with the given id, or None if absent."""
    for radio in full_config.get("sx127x") or []:
        if str(radio.get(CONF_ID)) == str(radio_id):
            return radio
    return None

CONFIG_SCHEMA = cv.All(
    cv.typed_schema(
        {
            TYPE_RTS: RTS_HUB_SCHEMA,
            TYPE_IOHC: IOHC_HUB_SCHEMA,
        },
    ),
    cv.only_on([PLATFORM_ESP32]),
)


def _final_validate(config):
    if config[CONF_TYPE] == TYPE_IOHC and CONF_SX127X_ID in config:
        radio = find_sx127x_config(fv.full_config.get(), config[CONF_SX127X_ID])
        if radio is not None:
            validate_sx127x_radio(radio)
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    typ = config[CONF_TYPE]

    if typ == TYPE_RTS:
        var = cg.new_Pvariable(config[CONF_ID])
        await cg.register_component(var, config)
        cg.add_define("USE_SOMFY_RTS")

        tx = await cg.get_variable(config[CONF_REMOTE_TRANSMITTER])
        cg.add(var.set_remote_transmitter(tx))

        if CONF_REMOTE_RECEIVER in config:
            rx = await cg.get_variable(config[CONF_REMOTE_RECEIVER])
            cg.add(var.set_remote_receiver(rx))

    elif typ == TYPE_IOHC:
        var = cg.new_Pvariable(config[CONF_ID])
        await cg.register_component(var, config)

        # One backend per build: the define keeps the other chip's driver
        # headers out of it.
        if CONF_SX127X_ID in config:
            chip = await cg.get_variable(config[CONF_SX127X_ID])
            chip_config = find_sx127x_config(CORE.config, config[CONF_SX127X_ID])
            wake_bytes = 0
            if CONF_WAKE_PREAMBLE in config:
                wake_bytes = wake_preamble_bytes(
                    config[CONF_WAKE_PREAMBLE].total_milliseconds
                )
            rhs = IohcRadioSX127x.new(
                chip,
                chip_config["payload_length"],
                chip_config["preamble_size"],
                wake_bytes,
            )
            cg.add_define("USE_SOMFY_IOHC_SX127X")
        else:
            chip = await cg.get_variable(config[CONF_CC1101_ID])
            rhs = IohcRadioCC1101.new(chip)
            cg.add_define("USE_SOMFY_IOHC_CC1101")
        radio = cg.Pvariable(config[CONF_RADIO_ID], rhs)
        cg.add(var.set_radio(radio))
        cg.add(var.set_frequency_1w(config[CONF_FREQUENCY_1W]))
        cg.add_define("USE_SOMFY_IOHC")
