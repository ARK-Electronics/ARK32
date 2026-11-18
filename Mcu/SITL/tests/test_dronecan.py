'''DroneCAN RawCommand / telemetry path over multicast UDP.'''

from __future__ import annotations

import time

import pytest

from sitl_harness import rpm_from_state, wait_for_state

dronecan = pytest.importorskip('dronecan')


def _multicast_usable(sitl, sim, timeout=5.0):
    '''GitHub macOS (and some locked-down VMs) have no multicast route.
    Skip rather than fail when the SITL never starts streaming with CAN on.'''
    if wait_for_state(sim, timeout=timeout):
        return True
    pytest.skip(
        'SITL state stream never started with CAN enabled; '
        'multicast is probably unavailable.\n' + sitl.log_tail())


def test_dronecan_throttle_and_esc_status(sitl_can_factory, state_stream, mcast_uri):
    sitl = sitl_can_factory(
        extra_args=['--node-id', '10'],
        can_uri=mcast_uri,
        wait_s=1.0)
    sim = state_stream(sitl)
    _multicast_usable(sitl, sim)

    node = dronecan.make_node(mcast_uri, node_id=100, bitrate=1000000)
    status = {}

    def on_esc(e):
        status['rpm'] = e.message.rpm
        status['voltage'] = e.message.voltage
        status['errors'] = e.message.error_count
        status['count'] = status.get('count', 0) + 1

    node.add_handler(dronecan.uavcan.equipment.esc.Status, on_esc)
    try:
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < 10:
            node.spin(0)
            now = time.time()
            if now >= nxt:
                nxt += 0.02
                thr = 0.35 if now - t0 > 2.5 else 0.0
                node.broadcast(
                    dronecan.uavcan.equipment.safety.ArmingStatus(status=255))
                node.broadcast(
                    dronecan.uavcan.equipment.esc.RawCommand(
                        cmd=[int(8191 * thr)]))
            time.sleep(0.001)

        rpm = rpm_from_state(sim)
        assert 3500 <= rpm <= 6500, 'state rpm=%.0f' % rpm
        assert 3500 <= status.get('rpm', -1) <= 6500, status
        assert 11 < status.get('voltage', 0) < 18, status
        assert status.get('count', 0) >= 3, 'too few esc.Status: %s' % status
        # error_count aggregates hard-error events (jump desync + stall rail).
        # An honest spool-up to a steady 4-6k rpm must report none: the stall
        # rail is gated on zero_crosses > 100 precisely so the kicks a normal
        # start needs are not counted as errors.
        assert status.get('errors', -1) == 0, \
            'healthy run reported error_count=%s' % status.get('errors')
    finally:
        # stop motor
        for _ in range(10):
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
            node.spin(0.02)
        node.close()


def test_dronecan_requires_arming(sitl_can_factory, state_stream, mcast_uri):
    '''default REQUIRE_ARMING=1: RawCommand alone must not spin the motor'''
    sitl = sitl_can_factory(
        extra_args=['--node-id', '10'],
        can_uri=mcast_uri,
        wait_s=1.0)
    sim = state_stream(sitl)
    _multicast_usable(sitl, sim)

    node = dronecan.make_node(mcast_uri, node_id=101, bitrate=1000000)
    try:
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < 5.0:
            node.spin(0)
            now = time.time()
            if now >= nxt:
                nxt += 0.02
                # no ArmingStatus — only RawCommand at mid throttle
                node.broadcast(
                    dronecan.uavcan.equipment.esc.RawCommand(cmd=[4000]))
            time.sleep(0.001)
        rpm = rpm_from_state(sim, 0.5)
        assert rpm < 500, 'spun without arming: rpm=%.0f' % rpm
    finally:
        for _ in range(5):
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
            node.spin(0.02)
        node.close()


def _send_for(node, watch, seconds, raw, until=None):
    """Stream ArmingStatus(armed) + RawCommand(raw) at 50 Hz while polling the watch."""
    end = time.monotonic() + seconds
    nxt = 0.0
    while time.monotonic() < end:
        node.spin(0)
        now = time.monotonic()
        if now >= nxt:
            nxt = now + 0.02
            node.broadcast(dronecan.uavcan.equipment.safety.ArmingStatus(status=255))
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[raw]))
        watch.poll()
        if until is not None and until():
            return True
    return until is None


def test_small_rawcommand_runs_at_minimum_throttle(sitl_can_factory, state_stream, mcast_uri):
    """RawCommand 1-4 is the slowest throttle (input 48), not input 47.

    DSDL esc.RawCommand: a nonzero setpoint below minimum is the minimum valid
    setpoint. PX4 sends UAVCAN_EC_MIN (default 1) at armed zero thrust and
    means it as the slowest reliable spin. Input 47 ran at minimum duty in the
    start branch but counted as a commanded stop in the stall rail (a
    coast/stop/re-kick loop on the 12S bench), and under brake_on_stop 2 it
    braked. Now it is input 48: the run carries on under brake_on_stop 2, and
    only RawCommand 0 stops it.
    """
    from sitl_gui_backend import EepromClient
    from sitl_params import PARAMS_BY_NAME
    from test_state_protocol import Watch, control_socket

    sitl = sitl_can_factory(extra_args=['--node-id', '10'], can_uri=mcast_uri, wait_s=1.0)
    sim = state_stream(sitl)
    _multicast_usable(sitl, sim)
    ee = EepromClient(port=sitl.state_port)
    ok, message = ee.set(PARAMS_BY_NAME['BRAKE_ON_STOP'][0], [2])
    assert ok, message
    node = dronecan.make_node(mcast_uri, node_id=100, bitrate=1000000)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, [(2, 'input'), (1, 'running')])
            send = lambda *a, **k: _send_for(node, watch, *a, **k)

            send(2.5, 0)  # arm at zero throttle
            assert send(5.0, int(8191 * 0.35), until=lambda: watch.values.get(1) == 1), 'never started'
            send(1.0, int(8191 * 0.35))
            first = len(watch.events)
            assert send(1.5, 2, until=lambda: watch.values.get(0) == 48), ('input 48 not reached', watch.values)
            send(0.5, 2)
            events = watch.events[first:]
            assert not any(i == 0 and v == 47 for _, i, v in events), 'RawCommand 2 reached input 47'
            assert not any(i == 1 and v == 0 for _, i, v in events), (
                'the run ended at RawCommand 2 (minimum throttle) under brake_on_stop 2', watch.values)
            assert send(1.0, 0, until=lambda: watch.values.get(1) == 0), ('RawCommand 0 did not stop', watch.values)
    finally:
        for _ in range(10):
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
            node.spin(0.02)
        node.close()


def test_bidirectional_small_rawcommand_is_the_slowest_throttle(sitl_can_factory, state_stream, mcast_uri):
    """Bidirectional: RawCommand +1 is the slowest forward, -1 the slowest reverse.

    +1..+8 used to map to 1047, the top of the reverse range (full reverse
    throttle), and -1..-8 to 47 (a stop). Now +1 is 1048 and -1 is 48.
    """
    from sitl_gui_backend import EepromClient
    from sitl_params import PARAMS_BY_NAME
    from test_state_protocol import Watch, control_socket

    sitl = sitl_can_factory(extra_args=['--node-id', '10'], can_uri=mcast_uri, wait_s=1.0)
    sim = state_stream(sitl)
    _multicast_usable(sitl, sim)
    ee = EepromClient(port=sitl.state_port)
    ok, message = ee.set(PARAMS_BY_NAME['BI_DIRECTIONAL'][0], [1])
    assert ok, message
    node = dronecan.make_node(mcast_uri, node_id=100, bitrate=1000000)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, [(2, 'newinput'), (2, 'adjusted_input')])
            send = lambda *a, **k: _send_for(node, watch, *a, **k)

            send(2.5, 0)  # arm at zero throttle
            first = len(watch.events)
            assert send(1.5, 1, until=lambda: watch.values.get(0) == 1048), ('+1 did not map to 1048', watch.values)
            send(0.5, 1)
            events = watch.events[first:]
            assert not any(i == 0 and v == 1047 for _, i, v in events), 'RawCommand +1 reached 1047 (full reverse)'
            top = max((v for _, i, v in events if i == 1), default=0)
            assert top < 200, 'RawCommand +1 drove adjusted_input to %d' % top
            send(1.0, 0)
            assert send(1.5, -1, until=lambda: watch.values.get(0) == 48), ('-1 did not map to 48', watch.values)
    finally:
        for _ in range(10):
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
            node.spin(0.02)
        node.close()


def test_rawcommand_timeout_does_not_clear_a_stuck_latch(sitl_can_factory, state_stream, mcast_uri):
    """The RawCommand timeout zeroes the input; that is not a pilot zero.

    After 250 ms without RawCommand the failsafe forces the input to zero.
    That zero used to clear a stuck-rotor latch, so when the flight controller
    resumed a nonzero command the motor restarted with no zero from the pilot.
    The latch must hold through the gap and the resumed throttle, and clear
    only on a received zero. ArmingStatus keeps flowing during the gap so the
    signal watchdog stays fed and the ESC does not reset.
    """
    from sitl_gui_backend import EepromClient
    from sitl_params import PARAMS_BY_NAME
    from test_state_protocol import Watch, control_socket

    esc_fault_stuck = 7  # Inc/esc_state.h
    sitl = sitl_can_factory(extra_args=['--node-id', '10'], can_uri=mcast_uri, wait_s=1.0)
    sim = state_stream(sitl)
    _multicast_usable(sitl, sim)
    ee = EepromClient(port=sitl.state_port)
    ok, message = ee.set(PARAMS_BY_NAME['STUCK_ROTOR_PROTECTION'][0], [1])
    assert ok, message
    node = dronecan.make_node(mcast_uri, node_id=100, bitrate=1000000)
    run = int(8191 * 0.35)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, [(1, 'esc_state'), (1, 'running'), (1, 'bemf_timeout_happened'),
                                 (2, 'adjusted_input')])
            state = lambda: watch.values.get(0)

            def send_for(seconds, raw, until=None, check=None):
                end = time.monotonic() + seconds
                nxt = 0.0
                while time.monotonic() < end:
                    node.spin(0)
                    now = time.monotonic()
                    if now >= nxt:
                        nxt = now + 0.02
                        node.broadcast(dronecan.uavcan.equipment.safety.ArmingStatus(status=255))
                        if raw is not None:
                            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[raw]))
                    watch.poll()
                    if check is not None:
                        check()
                    if until is not None and until():
                        return True
                return until is None

            def latched(what):
                def check():
                    v = watch.values
                    assert v.get(0) == esc_fault_stuck and v.get(2) == 102 and v.get(1) == 0, (
                        '%s: stuck latch lost (esc_state=%r bemf_timeout_happened=%r running=%r '
                        'adjusted_input=%r)' % (what, v.get(0), v.get(2), v.get(1), v.get(3)))
                return check

            send_for(2.5, 0)
            assert send_for(5.0, run, until=lambda: watch.values.get(1) == 1), 'never started'
            send_for(1.0, run)
            sim.set_stuck(1.0)
            assert send_for(10.0, run, until=lambda: state() == esc_fault_stuck), (
                'locked rotor never latched', watch.values)
            sim.set_stuck(0.0)
            send_for(0.5, run, check=latched('throttle after the jam'))
            # RawCommand gap: the failsafe zeroes the input after 250 ms.
            assert send_for(1.0, None, until=lambda: watch.values.get(3) == 0), (
                'RawCommand timeout never zeroed the input', watch.values)
            send_for(0.5, None, check=latched('RawCommand timeout'))
            send_for(2.0, run, check=latched('throttle resumed after the timeout'))
            # A received zero is the pilot's: it clears, and throttle starts again.
            assert send_for(2.0, 0, until=lambda: state() != esc_fault_stuck), (
                'a received zero did not clear the latch', watch.values)
            assert send_for(5.0, run, until=lambda: watch.values.get(1) == 1), 'no start after the clear'
    finally:
        for _ in range(10):
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
            node.spin(0.02)
        node.close()
