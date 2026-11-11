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


def test_active_brake_at_input_47_ends_the_run(sitl_can_factory, state_stream, mcast_uri):
    """RawCommand 1-4 maps to input 47, which tenKhzRoutine treats as a brake.

    With brake_on_stop 2 and no sine start, input 47 used to fall between the
    stop branch (input < 47) and the start branch (input >= 47), so six-step
    kept running - or restarted - underneath the active brake.
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

            def send_for(seconds, raw, until=None):
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

            send_for(2.5, 0)  # arm at zero throttle, as test_dronecan_throttle_and_esc_status does
            assert send_for(5.0, int(8191 * 0.35), until=lambda: watch.values.get(1) == 1), 'never started'
            send_for(1.0, int(8191 * 0.35))
            assert watch.values.get(1) == 1, ('run did not hold', watch.values)
            first = len(watch.events)
            assert send_for(1.0, 2, until=lambda: watch.values.get(0) == 47), ('input 47 not reached', watch.values)
            send_for(0.5, 2)
            events = watch.events[first:]
            t_47 = next(ts for ts, i, v in events if i == 0 and v == 47)
            t_stop = next((ts for ts, i, v in events if i == 1 and v == 0), None)
            assert t_stop is not None, ('input 47 never ended the run under the active brake', watch.values)
            # Stamps are SITL ns. The stop acts in the same setInput as the frame.
            assert t_stop - t_47 < 5_000_000, 'running=1 %.1f ms into input 47' % ((t_stop - t_47) / 1e6)
            restarted = [e for e in events if e[0] > t_stop and e[1] == 1 and e[2] != 0]
            assert not restarted, 'six-step restarted under the active brake at input 47'
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
