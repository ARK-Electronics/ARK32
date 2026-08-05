"""Tests for strict rig-config validation and profile (de)serialization."""
import pytest

from hwci.config import (RigConfig, apply_target_preset, load_profile, load_rig,
                         profile_from_dict, profile_to_dict)
from hwci.debugger.openocd import (APP_LOAD_ADDR, DEFAULT_CONFIGS,
                                   G4_APP_LOAD_ADDR, G4_CONFIGS)

VALID_RIG = """\
target: ARK_4IN1_F051
debugger_backend: openocd
telem_backend: serial
throttle_backend: flightstand
stand_backend: grpc
pole_pairs: 11
"""


def _write(tmp_path, text):
    p = tmp_path / "rig.yaml"
    p.write_text(text)
    return str(p)


def test_valid_rig_loads(tmp_path):
    rig = load_rig(_write(tmp_path, VALID_RIG))
    assert rig.debugger_backend == "openocd"
    assert rig.pole_pairs == 11
    # F051 preset fills SWD settings when omitted from YAML.
    assert rig.app_load_addr == APP_LOAD_ADDR
    assert list(rig.openocd_configs) == list(DEFAULT_CONFIGS)


def test_unknown_key_rejected(tmp_path):
    with pytest.raises(ValueError, match="unknown key"):
        load_rig(_write(tmp_path, VALID_RIG + "debugger_bakend: openocd\n"))


def test_unknown_backend_value_rejected(tmp_path):
    bad = VALID_RIG.replace("stand_backend: grpc", "stand_backend: gprc")
    with pytest.raises(ValueError, match="stand_backend"):
        load_rig(_write(tmp_path, bad))


def test_sim_backend_rejected_in_rig_file(tmp_path):
    bad = VALID_RIG.replace("stand_backend: grpc", "stand_backend: sim")
    with pytest.raises(ValueError, match="not allowed in a rig file"):
        load_rig(_write(tmp_path, bad))


def test_omitted_backend_rejected_in_rig_file(tmp_path):
    # An omitted backend would default to "sim" - a rig file must be explicit.
    partial = VALID_RIG.replace("telem_backend: serial\n", "")
    with pytest.raises(ValueError, match="telem_backend"):
        load_rig(_write(tmp_path, partial))


def test_flightstand_throttle_needs_a_stand(tmp_path):
    bad = VALID_RIG.replace("stand_backend: grpc", "stand_backend: none")
    with pytest.raises(ValueError, match="flightstand"):
        load_rig(_write(tmp_path, bad))


def test_none_backends_allowed(tmp_path):
    text = VALID_RIG.replace("stand_backend: grpc", "stand_backend: none")
    text = text.replace("throttle_backend: flightstand",
                        "throttle_backend: external")
    rig = load_rig(_write(tmp_path, text))
    assert rig.stand_backend == "none"


def test_no_config_gives_sim_defaults():
    rig = load_rig(None)
    assert rig.stand_backend == "sim"
    rig.validate()  # sim allowed for the built-in default


def test_g431_preset_fills_swd_settings(tmp_path):
    text = """\
target: ARK_G431_CAN
debugger_backend: openocd
telem_backend: none
debug_uart_backend: serial
debug_uart_port: /dev/esc-debug-uart
throttle_backend: flightstand
stand_backend: grpc
stand_host: 127.0.0.1
pole_pairs: 7
"""
    rig = load_rig(_write(tmp_path, text))
    assert rig.app_load_addr == G4_APP_LOAD_ADDR
    assert list(rig.openocd_configs) == list(G4_CONFIGS)
    assert rig.adapter_speed_khz == 8000
    assert rig.debug_uart_backend == "serial"
    assert rig.debug_uart_baud == 115200


def test_g431_explicit_app_load_not_overwritten(tmp_path):
    text = """\
target: ARK_G431_CAN
debugger_backend: openocd
telem_backend: none
throttle_backend: external
stand_backend: none
app_load_addr: 0x08005000
"""
    rig = load_rig(_write(tmp_path, text))
    assert rig.app_load_addr == 0x08005000
    # openocd still comes from preset
    assert list(rig.openocd_configs) == list(G4_CONFIGS)


def test_apply_target_preset_manual():
    cfg = RigConfig(target="ARK_G431_CAN")
    apply_target_preset(cfg, explicit_keys=set())
    assert cfg.app_load_addr == G4_APP_LOAD_ADDR
    assert list(cfg.openocd_configs) == list(G4_CONFIGS)


def test_profile_roundtrips_through_dict():
    p = load_profile("demag_step_stress")
    q = profile_from_dict(profile_to_dict(p))
    assert q == p
