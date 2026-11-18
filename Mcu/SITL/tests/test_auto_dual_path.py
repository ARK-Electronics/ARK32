'''AUTO input with RawCommand and a wire signal together (PR #147 review).

1. A wire stop is not the pilot while RawCommand owns throttle. The servo
   path wrote fault_pilot_zero before its RawCommand early-out, and the
   RawCommand timeout's set_input(0) re-decoded the last wire capture, so a
   standing PWM stop on the wire cleared a stuck latch that formed under CAN,
   and the motor restarted when RawCommand resumed. The latch must hold
   through the gap and the resumed throttle, and clear on a RawCommand zero.
2. RawCommand first: set_input() marks the input set before any wire protocol
   is detected, and the capture used to stop after the first wire frame, so
   the wire could never take over after a RawCommand timeout.
'''

from __future__ import annotations

import time

import pytest

import sitl_dshot as sd
from sitl_harness import Sender, wait_for_state

dronecan = pytest.importorskip('dronecan')

ESC_FAULT_STUCK = 7    # Inc/esc_state.h
ESC_STUCK_LATCH = 102  # Inc/esc_state.h


def _multicast_usable(sitl, sim, timeout=5.0):
    if wait_for_state(sim, timeout=timeout):
        return True
    pytest.skip('SITL state stream never started with CAN enabled; '
                'multicast is probably unavailable.\n' + sitl.log_tail())


def _sender(node, watch):
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
    return send_for


def test_wire_stop_does_not_clear_a_latch_set_under_rawcommand(sitl_can_factory, state_stream, mcast_uri):
    from sitl_gui_backend import EepromClient
    from sitl_params import PARAMS_BY_NAME
    from test_state_protocol import Watch, control_socket

    sitl = sitl_can_factory(extra_args=['--node-id', '10'], can_uri=mcast_uri, wait_s=1.0)
    sim = state_stream(sitl)
    _multicast_usable(sitl, sim)
    ee = EepromClient(port=sitl.state_port)
    ok, message = ee.set(PARAMS_BY_NAME['STUCK_ROTOR_PROTECTION'][0], [1])
    assert ok, message
    wire = Sender('127.0.0.1', sitl.input_port, sd.TYPE_PWM)
    wire.value = 1000  # a standing PWM stop on the wire throughout
    node = dronecan.make_node(mcast_uri, node_id=100, bitrate=1000000)
    run = int(8191 * 0.35)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, [(1, 'esc_state'), (1, 'running'), (1, 'bemf_timeout_happened'),
                                 (2, 'adjusted_input'), (1, 'servoPwm')])
            send_for = _sender(node, watch)

            def latched(what):
                def check():
                    v = watch.values
                    assert v.get(0) == ESC_FAULT_STUCK and v.get(2) == ESC_STUCK_LATCH and v.get(1) == 0, (
                        '%s: stuck latch lost (esc_state=%r bemf_timeout_happened=%r running=%r '
                        'adjusted_input=%r)' % (what, v.get(0), v.get(2), v.get(1), v.get(3)))
                return check

            assert send_for(3.0, 0, until=lambda: watch.values.get(4) == 1), (
                'wire PWM never detected', watch.values)
            send_for(1.0, 0)
            assert send_for(5.0, run, until=lambda: watch.values.get(1) == 1), 'never started'
            send_for(1.0, run)
            sim.set_stuck(1.0)
            assert send_for(10.0, run, until=lambda: watch.values.get(0) == ESC_FAULT_STUCK), (
                'locked rotor never latched', watch.values)
            sim.set_stuck(0.0)
            send_for(0.5, run, check=latched('throttle after the jam'))
            assert send_for(1.0, None, until=lambda: watch.values.get(3) == 0), (
                'RawCommand timeout never zeroed the input', watch.values)
            send_for(1.0, None, check=latched('RawCommand gap with a PWM stop on the wire'))
            send_for(2.0, run, check=latched('throttle resumed after the gap'))
            assert send_for(2.0, 0, until=lambda: watch.values.get(0) != ESC_FAULT_STUCK), (
                'a RawCommand zero did not clear the latch', watch.values)
    finally:
        for _ in range(10):
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
            node.spin(0.02)
        node.close()
        wire.stop()


def test_wire_takes_over_after_rawcommand_came_first(sitl_can_factory, state_stream, mcast_uri):
    from test_state_protocol import Watch, control_socket

    sitl = sitl_can_factory(extra_args=['--node-id', '10'], can_uri=mcast_uri, wait_s=1.0)
    sim = state_stream(sitl)
    _multicast_usable(sitl, sim)
    node = dronecan.make_node(mcast_uri, node_id=100, bitrate=1000000)
    wire = None
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, [(1, 'dshot'), (1, 'running'), (1, 'armed'), (2, 'adjusted_input')])
            send_for = _sender(node, watch)
            # RawCommand first, with no wire signal at all: the ESC arms on CAN.
            assert send_for(4.0, 0, until=lambda: watch.values.get(2) == 1), ('never armed on CAN', watch.values)
            send_for(0.5, 0)
            # RawCommand stops (ArmingStatus keeps the bus alive); then the wire starts.
            send_for(1.0, None)
            wire = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
            wire.value = 0
            assert send_for(3.0, None, until=lambda: watch.values.get(0) == 1), (
                'DShot on the wire never detected after a RawCommand-first boot', watch.values)
            send_for(1.0, None)
            wire.value = 500
            assert send_for(6.0, None, until=lambda: watch.values.get(1) == 1), (
                'wire throttle never started the motor', watch.values)
    finally:
        if wire is not None:
            wire.value = 0
            time.sleep(0.3)
            wire.stop()
        node.close()
