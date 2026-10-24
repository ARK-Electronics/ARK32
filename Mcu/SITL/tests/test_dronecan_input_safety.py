"""DroneCAN throttle mapping and filtering must never drive the wrong way.

Each case watches the simulated rotor (state stream samples are
(t, omega, theta, theta_e, iu, iv, iw, ...); positive omega is the
configured forward direction).
"""

import pytest

from sitl_harness import wait_for_state
from test_params import _request_wait
from test_safety import _can_drive, _need_dronecan


def _set(dronecan, node, name, **value):
    request = dronecan.uavcan.protocol.param.GetSet.Request()
    request.name = name
    request.value = dronecan.uavcan.protocol.param.Value(**value)
    for _ in range(5):
        if _request_wait(node, 10, request) is not None:
            return
    pytest.fail('no GetSet reply for %s' % name)


def _omega(sim, seconds):
    window = sim.window(seconds)
    assert window, 'no state samples'
    return [s[1] for s in window]


@pytest.fixture
def can_esc(sitl_can_factory, state_stream, mcast_uri):
    dronecan = _need_dronecan()
    sitl = sitl_can_factory(extra_args=['--node-id', '10'], can_uri=mcast_uri, wait_s=1.0)
    sim = state_stream(sitl)
    assert wait_for_state(sim, timeout=5.0), sitl.log_tail()
    node = dronecan.make_node(mcast_uri, node_id=121, bitrate=1000000)
    try:
        yield dronecan, node, sim, sitl
    finally:
        _can_drive(dronecan, node, thr=0.0, armed=False, duration=0.3)
        node.close()


def test_direction_change_while_running_stops_driving(can_esc):
    dronecan, node, sim, sitl = can_esc
    _can_drive(dronecan, node, thr=0.0, armed=True, duration=2.5)
    _can_drive(dronecan, node, thr=0.35, armed=True, duration=3.0)
    spinning = _omega(sim, 0.5)
    assert sum(spinning) / len(spinning) > 200, sitl.log_tail()
    # The change disarms; REQUIRE_ZERO_THROTTLE (default 1) keeps it
    # disarmed while nonzero commands continue. It used to clear running
    # without allOff(): commutation stopped with one phase pair still
    # driven at the old duty, about 20 A peak into a stalled rotor here.
    _set(dronecan, node, 'DIR_REVERSED', boolean_value=1)
    _can_drive(dronecan, node, thr=0.35, armed=True, duration=2.0)
    window = sim.window(0.5)
    peak = max(max(abs(s[4]), abs(s[5]), abs(s[6])) for s in window)
    assert peak < 1.0, (peak, sitl.log_tail())
    # Zero throttle re-arms it, and it then runs the new way.
    _can_drive(dronecan, node, thr=0.0, armed=True, duration=3.0)
    _can_drive(dronecan, node, thr=0.35, armed=True, duration=3.0)
    reversed_run = _omega(sim, 0.5)
    assert sum(reversed_run) / len(reversed_run) < -200, sitl.log_tail()


@pytest.mark.parametrize('raw', [1, 4, 8])
def test_tiny_forward_bidirectional_command_is_not_full_reverse(can_esc, raw):
    dronecan, node, sim, sitl = can_esc
    _set(dronecan, node, 'BI_DIRECTIONAL', boolean_value=1)
    _can_drive(dronecan, node, thr=0.0, armed=True, duration=2.5)
    # +1..+8 of 8191 used to encode as 1047, the top of the reverse band.
    _can_drive(dronecan, node, thr=(raw + 0.5) / 8191.0, armed=True, duration=1.5)
    assert min(_omega(sim, 1.5)) > -60, sitl.log_tail()


def test_filtered_bidirectional_throttle_keeps_its_direction(can_esc):
    dronecan, node, sim, sitl = can_esc
    _set(dronecan, node, 'BI_DIRECTIONAL', boolean_value=1)
    _set(dronecan, node, 'INPUT_FILTER_HZ', integer_value=2)
    _can_drive(dronecan, node, thr=0.0, armed=True, duration=2.5)
    # From rest the filter used to carry the encoded command through the
    # reverse band: the rotor spun up backwards first (about -420 rad/s).
    # The start kick alone is about -30.
    _can_drive(dronecan, node, thr=0.5, armed=True, duration=2.0)
    start = _omega(sim, 2.0)
    assert min(start) > -100, sitl.log_tail()
    steady = sum(start[-200:]) / len(start[-200:])
    assert steady > 300, sitl.log_tail()
    # Falling back to zero crossed the reverse band again, and the rotor
    # sped up to about 1.7x steady before stopping.
    _can_drive(dronecan, node, thr=0.0, armed=True, duration=3.0)
    stop = _omega(sim, 3.0)
    assert max(stop) < steady * 1.1 and min(stop) > -60, (steady, max(stop), min(stop))
