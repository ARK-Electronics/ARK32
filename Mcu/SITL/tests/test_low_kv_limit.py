"""Low-kV protection policy exercised through the native firmware runtime.

The motor stays stopped: these tests verify the configured envelope and the
ceiling it actually produces, without making claims about loaded startup or
sine-to-six-step handoff performance.
"""

import pytest

import sitl_dshot as sd
from sitl_gui_backend import EepromClient
from sitl_harness import Sender
from sitl_params import PARAMS_BY_NAME, byte_to_kv
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
    (2, 'throttle_max_at_low_rpm'),
    (2, 'min_startup_duty'),
    (1, 'dead_time_override'),
    (1, 'use_current_limit'),
    (1, 'beep_volume'),
]
INDEX = {name: index for index, (_size, name) in enumerate(VARIABLES)}

# Power-on values from Inc/motor_runtime.h.
LOW_RPM_LEVEL_DEFAULT = 20
HIGH_RPM_LEVEL_DEFAULT = 70
THROTTLE_MAX_AT_LOW_RPM_DEFAULT = 400


class StoppedEsc:
    def __init__(self, sitl, editor, watch):
        self.sitl = sitl
        self.editor = editor
        self.watch = watch

    def value(self, name):
        return self.watch.values.get(INDEX[name])

    def set(self, name, *raw):
        # Every write makes the firmware re-run loadEEpromSettings().
        ok, message = self.editor.set(PARAMS_BY_NAME[name][0], list(raw))
        assert ok, message

    def expect(self, predicate=lambda esc: True, **expected):
        self.watch.until(lambda: predicate(self) and all(
            self.value(name) == value for name, value in expected.items()))
        assert self.sitl.proc.poll() is None, self.sitl.log_tail()
        assert 'reset (' not in self.sitl.log_tail(100), self.sitl.log_tail(100)


@pytest.fixture
def stopped_esc(sitl_factory):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    # Zero-throttle frames keep signal-loss resets from hiding a settings
    # reload bug by restoring the process's initial global values.
    sender = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, VARIABLES)
            # Check resolution first so a renamed global fails by name
            # rather than as a timeout waiting for its value.
            watch.until(lambda: watch.resolved is not None)
            assert watch.resolved == [1] * len(VARIABLES), [
                name for ok, (_size, name) in zip(watch.resolved, VARIABLES) if not ok]
            watch.until(lambda: len(watch.values) == len(VARIABLES))
            esc = StoppedEsc(sitl, EepromClient(port=sitl.state_port), watch)
            # Pin the settings the low-speed ceiling depends on, whatever the
            # boot image holds: min_startup_duty = 1 * 10 + 100, no dead-time
            # compensation.
            esc.set('MIN_DUTY_CYCLE', 1)
            esc.set('STARTUP_POWER', 100)
            esc.set('DRIVING_BRAKE_STRENGTH', 10)
            esc.expect(min_startup_duty=110, throttle_max_at_low_rpm=THROTTLE_MAX_AT_LOW_RPM_DEFAULT)
            yield esc
    finally:
        sender.stop()


def _load_and_expect(esc, raw_kv, poles, low, high, ceiling):
    # MOTOR_KV and MOTOR_POLES are adjacent bytes in the EEPROM layout.
    assert PARAMS_BY_NAME['MOTOR_POLES'][0] == PARAMS_BY_NAME['MOTOR_KV'][0] + 1
    esc.set('MOTOR_KV', raw_kv, poles)
    # Low-kV throttle protection must not also opt into the RPM-based
    # timing schedule; that still starts at 300 kV.
    if byte_to_kv(raw_kv) < 300:
        def timing(esc):
            return esc.value('advance_erpm_scale_q12') == 0
    else:
        def timing(esc):
            return (esc.value('advance_erpm_scale_q12') or 0) > 0
    esc.expect(timing, motor_kv=byte_to_kv(raw_kv), low_rpm_throttle_limit=int(raw_kv != 0),
               low_rpm_level=low, high_rpm_level=high, duty_cycle_maximum=ceiling,
               k_erpm=0, running=0)


@pytest.mark.parametrize('raw_kv,poles,low,high,ceiling', [
    # Every representable low-kV bin newly protected by this change.
    (1, 42, 0, 4, 400),    # 60 kV
    (2, 42, 1, 7, 400),    # 100 kV
    (3, 42, 1, 10, 400),   # 140 kV
    (4, 42, 2, 13, 400),   # 180 kV (160-kV motor rounded for EEPROM)
    (5, 42, 2, 16, 400),   # 220 kV
    (6, 42, 3, 20, 400),   # 260 kV
    # Raw zero is the explicit 20-kV opt-out, including high pole counts.
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


def test_low_kv_opt_out_reload_reenables_protection(stopped_esc):
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


def test_running_brake_reload_does_not_raise_ceiling(stopped_esc):
    esc = stopped_esc
    esc.set('MOTOR_KV', 4, 42)
    esc.set('DRIVING_BRAKE_STRENGTH', 5)
    # SITL DEAD_TIME 80 + (150 - 5 * 10): compensation the loader adds once.
    esc.expect(dead_time_override=180)
    raised = THROTTLE_MAX_AT_LOW_RPM_DEFAULT + 180
    # Beep volume changes only mark each reload; the ceiling must not grow
    # by the dead time again on any of them.
    for volume in (6, 7, 6, 7):
        esc.set('BEEP_VOLUME', volume)
        esc.expect(beep_volume=3 * volume, throttle_max_at_low_rpm=raised, duty_cycle_maximum=raised)
    esc.set('DRIVING_BRAKE_STRENGTH', 10)
    esc.expect(throttle_max_at_low_rpm=THROTTLE_MAX_AT_LOW_RPM_DEFAULT,
               duty_cycle_maximum=THROTTLE_MAX_AT_LOW_RPM_DEFAULT, min_startup_duty=110)


@pytest.mark.parametrize('raw_kv', [4, 22])
@pytest.mark.parametrize('min_duty,floor', [(30, 450), (50, 650)])
def test_low_rpm_ceiling_respects_startup_floor(stopped_esc, raw_kv, min_duty, floor):
    # min_startup_duty = min_duty * 10 + startup_power. A ceiling below it
    # would override the configured minimum duty and startup power.
    esc = stopped_esc
    esc.set('MOTOR_KV', raw_kv, 42)
    esc.set('STARTUP_POWER', 150)
    esc.set('MIN_DUTY_CYCLE', min_duty)
    esc.expect(min_startup_duty=floor, throttle_max_at_low_rpm=floor, duty_cycle_maximum=floor)
    esc.set('MIN_DUTY_CYCLE', 1)
    esc.expect(min_startup_duty=160, throttle_max_at_low_rpm=THROTTLE_MAX_AT_LOW_RPM_DEFAULT,
               duty_cycle_maximum=THROTTLE_MAX_AT_LOW_RPM_DEFAULT)


def test_current_limit_disable_takes_effect_on_reload(stopped_esc):
    esc = stopped_esc
    esc.set('CURRENT_LIMIT', 20)  # 40 A
    esc.expect(use_current_limit=1)
    esc.set('CURRENT_LIMIT', 102)  # off
    esc.expect(use_current_limit=0)


def test_rc_car_limiter_override_survives_reload(stopped_esc):
    esc = stopped_esc
    esc.set('MOTOR_KV', 0, 42)
    esc.expect(low_rpm_throttle_limit=0, duty_cycle_maximum=2000)
    # RC-car mode forces the limiter on with its higher low-speed ceiling.
    esc.set('RC_CAR_REVERSE', 1)
    esc.expect(low_rpm_throttle_limit=1, throttle_max_at_low_rpm=1000, duty_cycle_maximum=1000)
    esc.set('BEEP_VOLUME', 6)
    esc.expect(beep_volume=18, low_rpm_throttle_limit=1, throttle_max_at_low_rpm=1000)
    esc.set('RC_CAR_REVERSE', 0)
    esc.expect(low_rpm_throttle_limit=0, throttle_max_at_low_rpm=THROTTLE_MAX_AT_LOW_RPM_DEFAULT,
               duty_cycle_maximum=2000)


def test_version_zero_reload_restores_power_on_limiter(stopped_esc):
    # Layout version 0 skips every setting introduced later, the kV and
    # pole terms included, so a reload must fall back to power-on values
    # rather than keep the previous load's opt-out.
    esc = stopped_esc
    esc.set('MOTOR_KV', 0, 42)
    esc.expect(low_rpm_throttle_limit=0, low_rpm_level=0, high_rpm_level=1)
    esc.set('EEPROM_VERSION', 0)
    esc.expect(low_rpm_throttle_limit=1, low_rpm_level=LOW_RPM_LEVEL_DEFAULT,
               high_rpm_level=HIGH_RPM_LEVEL_DEFAULT)


def test_dronecan_motor_kv_reaches_opt_out_only_explicitly(sitl_can_factory, mcast_uri):
    dronecan = pytest.importorskip('dronecan')
    from test_eeprom_schema import _fetch, _wait_for_esc, zero_throttle_can
    from test_params import _request_wait, _set_param

    kv_offset = PARAMS_BY_NAME['MOTOR_KV'][0]
    with zero_throttle_can(mcast_uri):
        sitl = sitl_can_factory(extra_args=['--node-id', '10', '--eeprom', 'low-kv-can.bin'],
                                can_uri=mcast_uri)
        client = EepromClient('127.0.0.1', sitl.state_port)
        node, found = _wait_for_esc(mcast_uri, our_id=121)
        try:
            assert 10 in found, sitl.log_tail()
            # Wire stores truncate, except that 21-59 kV must not land on
            # raw 0, the low-RPM protection opt-out.
            for kv, raw in [(20, 0), (21, 1), (29, 1), (59, 1), (60, 1), (100, 2),
                            (1021, 25), (10220, 255)]:
                assert _set_param(node, 10, 'MOTOR_KV', kv) is not None, kv
                assert _fetch(client)[kv_offset] == raw, kv
            # Out-of-range requests leave the setting unchanged instead of
            # wrapping through the byte (10260 kV used to store raw 0).
            for kv in (10260, 10299, 19, 0):
                request = dronecan.uavcan.protocol.param.GetSet.Request()
                request.name = 'MOTOR_KV'
                request.value = dronecan.uavcan.protocol.param.Value(integer_value=kv)
                response = _request_wait(node, 10, request)
                assert response is not None and int(response.value.integer_value) == 10220, kv
                assert _fetch(client)[kv_offset] == 255, kv
        finally:
            node.close()
