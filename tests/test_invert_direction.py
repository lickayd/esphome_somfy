"""Invert direction: config surface and the invariants the C++ relies on.

Behaviour itself is covered by the host suite (tests/cpp/test_rts_cover.cpp);
the io cover has no host harness, so its mapping is pinned here by source.
"""

from pathlib import Path

from somfy import cover as cover_mod

ROOT = Path(__file__).parent.parent
SOMFY = ROOT / "components" / "somfy"
COVER_PY = (SOMFY / "cover.py").read_text()
SWITCH_PY = (SOMFY / "switch.py").read_text()
BASE_H = (SOMFY / "somfy_time_based_cover.h").read_text()
BASE_CPP = (SOMFY / "somfy_time_based_cover.cpp").read_text()
RTS_CPP = (SOMFY / "somfy_rts.cpp").read_text()
IOHC_CPP = (SOMFY / "somfy_iohc.cpp").read_text()
SWITCH_H = (SOMFY / "somfy_invert_switch.h").read_text()
SWITCH_CPP = (SOMFY / "somfy_invert_switch.cpp").read_text()


def _function(source: str, signature: str) -> str:
    body = source.split(signature, 1)[1]
    return body.split("\n}\n", 1)[0]


def test_option_is_shared_by_rts_and_iohc_covers_and_defaults_off():
    assert cover_mod.CONF_INVERT_DIRECTION == "invert_direction"
    assert cover_mod.CONF_INVERT_DIRECTION in {
        str(key) for key in cover_mod.COMMON_COVER_FIELDS
    } or "CONF_INVERT_DIRECTION, default=False" in COVER_PY
    assert COVER_PY.count("var.set_invert_direction(config[CONF_INVERT_DIRECTION])") == 2


def test_rts_swaps_both_the_sent_command_and_the_remote_mapping():
    assert "this->invert_direction_ ? RtsCommand::Down : RtsCommand::Up" in RTS_CPP
    assert "this->invert_direction_ ? RtsCommand::Up : RtsCommand::Down" in RTS_CPP
    on_frame = _function(RTS_CPP, "void SomfyCover::on_rts_frame_")
    assert "this->invert_direction_ ? cover::COVER_OPERATION_CLOSING : cover::COVER_OPERATION_OPENING" in on_frame


def test_iohc_swaps_the_sent_main_parameter_in_1w_and_2w():
    opened = _function(IOHC_CPP, "void SomfyIohcCover::open()")
    closed = _function(IOHC_CPP, "void SomfyIohcCover::close()")
    assert "this->invert_direction_ ? iohc_cmd::MP_CLOSE : iohc_cmd::MP_OPEN" in opened
    assert "this->invert_direction_ ? iohc_cmd::MP_OPEN : iohc_cmd::MP_CLOSE" in closed
    for body in (opened, closed):
        assert "this->send_2w_command(main_param)" in body
        assert "this->send_1w_command(main_param)" in body
        # The lift tilt follows the physical command, not HA's direction.
        assert "this->set_lift_tilt_(main_param == iohc_cmd::MP_OPEN)" in body


def test_iohc_stop_and_my_are_not_inverted():
    stop = _function(IOHC_CPP, "void SomfyIohcCover::stop()")
    assert "invert_direction_" not in stop
    assert "iohc_cmd::MP_STOP" in stop


def test_iohc_mirrors_the_remote_mapping_and_my_position():
    handler = _function(IOHC_CPP, "void SomfyIohcCover::handle_rx_command_")
    assert handler.count("this->invert_direction_ ?") == 2
    changed = _function(IOHC_CPP, "void SomfyIohcCover::on_invert_direction_changed_()")
    assert "this->my_position_ = 1.0f - this->my_position_;" in changed


def test_runtime_toggle_mirrors_position_but_boot_restore_does_not():
    apply = _function(BASE_CPP, "void SomfyTimeBasedCover::apply_invert_direction")
    restore = _function(BASE_CPP, "void SomfyTimeBasedCover::restore_invert_direction")
    assert "1.0f - this->position" in apply
    assert "this->stop_remote_animation_();" in apply
    assert "this->position" not in restore
    assert "this->on_invert_direction_changed_();" in restore


def test_switch_prefers_the_stored_choice_and_falls_back_to_yaml():
    setup = _function(SWITCH_CPP, "void SomfyInvertSwitch::setup()")
    assert "this->get_initial_state()" in setup
    assert "this->cover_->get_invert_direction()" in setup
    assert "this->cover_->restore_invert_direction(invert)" in setup
    write = _function(SWITCH_CPP, "void SomfyInvertSwitch::write_state")
    assert "this->cover_->apply_invert_direction(state)" in write


def test_switch_sources_do_not_break_builds_without_the_switch_component():
    assert "#ifdef USE_SWITCH" in SWITCH_H
    assert "#ifdef USE_SWITCH" in SWITCH_CPP
    assert "cv.use_id(SomfyTimeBasedCover)" in SWITCH_PY
    assert "virtual void stop_remote_animation_() {}" in BASE_H
