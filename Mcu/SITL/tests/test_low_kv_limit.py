"""Low-kV protection policy exercised through the native firmware runtime.

The motor stays stopped: these tests verify the configured envelope and the
ceiling it actually produces, without making claims about loaded startup or
sine-to-six-step handoff performance.
"""

import pytest

import sitl_dshot as sd
from sitl_gui_backend import EepromClient
from sitl_harness import Sender
from test_state_protocol import Watch, control_socket


VARIABLES = [
    (2, 'motor_kv'),
    (1, 'low_rpm_throttle_limit'),
    (2, 'low_rpm_level'),
    (2, 'high_rpm_level'),
    (2, 'duty_cycle_maximum'),
    (2, 'advance_erpm_scale_q12'),
    (2, 'k_erpm'),
    (1, 'running'),
]


@pytest.fixture
def stopped_esc(sitl_factory):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    # Zero-throttle frames keep signal-loss resets from hiding a settings
    # reload bug by restoring the process's initial global values.
    sender = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, VARIABLES)
            watch.until(lambda: watch.resolved is not None
                        and len(watch.values) == len(VARIABLES))
            assert watch.resolved == [1] * len(VARIABLES)
            yield sitl, EepromClient(port=sitl.state_port), watch
    finally:
        sender.stop()


def _load_and_expect(stopped_esc, raw_kv, poles, low, high, ceiling):
    sitl, editor, watch = stopped_esc
    # MOTOR_KV and MOTOR_POLES are adjacent bytes in the EEPROM wire layout.
    ok, message = editor.set(26, [raw_kv, poles])
    assert ok, message
    expected = {
        0: 20 + 40 * raw_kv,
        1: int(raw_kv != 0),
        2: low,
        3: high,
        4: ceiling,
        6: 0,
        7: 0,
    }
    # Low-kV throttle protection must not also opt into the RPM-based
    # timing schedule; that still starts at 300 kV.
    watch.until(lambda: all(watch.values.get(index) == value
                            for index, value in expected.items())
                and ((watch.values.get(5) == 0) if raw_kv < 7
                     else watch.values.get(5, 0) > 0))
    assert sitl.proc.poll() is None, sitl.log_tail()
    assert 'reset (' not in sitl.log_tail(100), sitl.log_tail(100)


@pytest.mark.parametrize('raw_kv,poles,low,high,ceiling', [
    # Every representable low-kV bin newly protected by this change.
    (1, 42, 0, 4, 400),    # 60 kV
    (2, 42, 1, 7, 400),    # 100 kV
    (3, 42, 1, 10, 400),   # 140 kV
    (4, 42, 2, 13, 400),   # 180 kV (160-kV motor rounded for EEPROM)
    (5, 42, 2, 16, 400),   # 220 kV
    (6, 42, 3, 20, 400),   # 260 kV
    # Raw zero is the explicit 20-kV bypass, including high pole counts.
    (0, 14, 0, 0, 2000),
    (0, 42, 0, 1, 2000),
    (0, 128, 0, 4, 2000),
    # Existing envelope arithmetic must still retain fractional pole ratios.
    (4, 14, 0, 4, 400),
    (4, 128, 7, 42, 400),
    (7, 14, 1, 7, 400),    # 300-kV timing boundary
    (7, 42, 3, 23, 400),
    (7, 128, 12, 70, 400),
    (22, 14, 3, 23, 400),  # Normal 900-kV motor
    # Integer resolution can still collapse a tiny envelope to zero;
    # map() must preserve its existing unrestricted, divide-safe fallback.
    (1, 2, 0, 0, 2000),
])
def test_low_kv_runtime_ceiling(stopped_esc, raw_kv, poles, low, high, ceiling):
    _load_and_expect(stopped_esc, raw_kv, poles, low, high, ceiling)


def test_low_kv_bypass_reload_reenables_protection(stopped_esc):
    # One process and subscription throughout: a one-way "disable" in the
    # settings loader would leave each later protected setting unrestricted.
    for raw_kv, low, high, ceiling in [
        (0, 0, 1, 2000),
        (4, 2, 13, 400),
        (0, 0, 1, 2000),
        (7, 3, 23, 400),
        (4, 2, 13, 400),
    ]:
        _load_and_expect(stopped_esc, raw_kv, 42, low, high, ceiling)
