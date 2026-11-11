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
            assert send_for(1.0, 2, until=lambda: watch.values.get(0) == 47), ('input 47 not reached', watch.values)
            t0 = time.monotonic()
            assert send_for(0.15, 2, until=lambda: watch.values.get(1) == 0), (
                'running=1 %.2f s into input 47 under the active brake' % (time.monotonic() - t0))
            first = len(watch.events)
            send_for(0.5, 2)
            restarted = [e for e in watch.events[first:] if e[1] == 1 and e[2] != 0]
            assert not restarted, 'six-step restarted under the active brake at input 47'
    finally:
        for _ in range(10):
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
            node.spin(0.02)
        node.close()
