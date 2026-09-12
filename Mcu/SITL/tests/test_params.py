'''DroneCAN parameter GetSet / save path.'''

from __future__ import annotations

import time

import pytest

from sitl_harness import wait_for_state

dronecan = pytest.importorskip('dronecan')


def _wait_for_node(uri, timeout=8.0, our_id=110):
    node = dronecan.make_node(uri, node_id=our_id, bitrate=1000000)
    found = {}

    def on_status(e):
        found[e.transfer.source_node_id] = True

    node.add_handler(dronecan.uavcan.protocol.NodeStatus, on_status)
    deadline = time.time() + timeout
    while time.time() < deadline and not found:
        node.spin(0.1)
    return node, found


def _request_wait(node, target, req, timeout=2.5):
    '''send a service request; pydronecan delivers timeout as e=None'''
    result = {}

    def cb(e):
        result['response'] = e.response if e is not None else None
        result['done'] = True

    node.request(req, target, cb)
    deadline = time.time() + timeout
    while 'done' not in result and time.time() < deadline:
        node.spin(0.05)
    return result.get('response')


def _get_param(node, target, name, attempts=5):
    '''GetSet with retries — idle SITL reboots every ~2s on signal timeout,
    which can swallow a single in-flight service transfer.'''
    for _ in range(attempts):
        req = dronecan.uavcan.protocol.param.GetSet.Request()
        req.name = name
        rsp = _request_wait(node, target, req)
        if rsp is not None and str(rsp.name):
            return rsp
        time.sleep(0.3)
    return None


def _set_param(node, target, name, value, attempts=5):
    for _ in range(attempts):
        req = dronecan.uavcan.protocol.param.GetSet.Request()
        req.name = name
        req.value = dronecan.uavcan.protocol.param.Value(integer_value=int(value))
        rsp = _request_wait(node, target, req)
        if rsp is not None and str(rsp.name):
            if int(rsp.value.integer_value) == int(value):
                return rsp
        time.sleep(0.3)
    return None


def _require_mcast_node(sitl, sim, mcast_uri, our_id=110):
    if not wait_for_state(sim, timeout=5.0):
        pytest.skip('multicast/state unavailable\n' + sitl.log_tail())
    node, found = _wait_for_node(mcast_uri, our_id=our_id)
    if 10 not in found:
        node.close()
        pytest.skip('ESC node 10 not seen on %s (mcast likely broken)\n%s'
                    % (mcast_uri, sitl.log_tail()))
    return node, found


def test_param_get_defaults(sitl_can_factory, state_stream, mcast_uri):
    sitl = sitl_can_factory(
        extra_args=['--node-id', '10'],
        can_uri=mcast_uri,
        wait_s=1.0)
    sim = state_stream(sitl)
    node, found = _require_mcast_node(sitl, sim, mcast_uri)
    try:
        rsp = _get_param(node, 10, 'MOTOR_POLES')
        assert rsp is not None, 'GetSet failed for MOTOR_POLES\n' + sitl.log_tail()
        assert int(rsp.value.integer_value) == 14, rsp.value
        rsp = _get_param(node, 10, 'INPUT_SIGNAL_TYPE')
        assert rsp is not None
        assert int(rsp.value.integer_value) in (0, 1, 2, 5), rsp.value
        rsp = _get_param(node, 10, 'TELEM_RATE')
        assert rsp is not None
        assert 0 <= int(rsp.value.integer_value) <= 200
    finally:
        node.close()


def test_param_set_telem_rate(sitl_can_factory, state_stream, mcast_uri):
    sitl = sitl_can_factory(
        extra_args=['--node-id', '10'],
        can_uri=mcast_uri,
        wait_s=1.0)
    sim = state_stream(sitl)
    node, found = _require_mcast_node(sitl, sim, mcast_uri, our_id=111)
    try:
        new_rate = 50
        rsp = _set_param(node, 10, 'TELEM_RATE', new_rate)
        assert rsp is not None, 'set TELEM_RATE failed\n' + sitl.log_tail()
        assert int(rsp.value.integer_value) == new_rate, rsp.value
        rsp = _get_param(node, 10, 'TELEM_RATE')
        assert rsp is not None
        assert int(rsp.value.integer_value) == new_rate
    finally:
        node.close()


def test_param_save_opcode(sitl_can_factory, state_stream, mcast_uri):
    sitl = sitl_can_factory(
        extra_args=['--node-id', '10'],
        can_uri=mcast_uri,
        wait_s=1.0)
    sim = state_stream(sitl)
    node, found = _require_mcast_node(sitl, sim, mcast_uri, our_id=112)
    try:
        assert _set_param(node, 10, 'BEEP_VOLUME', 7) is not None
        ok = False
        for _ in range(5):
            req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
            req.opcode = req.OPCODE_SAVE
            rsp = _request_wait(node, 10, req, timeout=3.0)
            if rsp is not None and rsp.ok:
                ok = True
                break
            time.sleep(0.3)
        assert ok, 'OPCODE_SAVE failed\n' + sitl.log_tail()
    finally:
        node.close()


def test_param_erase_restores_armed_protections(sitl_can_factory, state_stream,
                                                mcast_uri):
    '''a "restore defaults" must not disarm the protections

    OPCODE_ERASE memcpy's default_settings[] over the whole eeprom page, and
    that array began life as the AM32 configurator skeleton whose bytes 43/44
    (141/102) sit just OUTSIDE the ranges settings.c arms. So a field param
    erase used to silently turn the thermal foldback and the current limiter
    off on a shipped ESC — the one config change nobody re-checks afterwards.

    Drives the real opcode and reads the parameters back, so it covers the
    array, the post-skeleton band byte written by
    apply_post_skeleton_defaults(), and the settings.c coercion together.
    scripts/check-erase-defaults.py gates the same values against the
    product's factory JSON at build time.
    '''
    sitl = sitl_can_factory(
        extra_args=['--node-id', '10'],
        can_uri=mcast_uri,
        wait_s=1.0)
    sim = state_stream(sitl)
    node, _found = _require_mcast_node(sitl, sim, mcast_uri, our_id=113)
    try:
        # Move all three away from the shipped values first, so a no-op erase
        # cannot pass this test.
        for name, value in (('TEMPERATURE_LIMIT', 255),
                            ('CURRENT_LIMIT', 0),
                            ('TEMP_DERATE_BAND', 40)):
            assert _set_param(node, 10, name, value) is not None, \
                'setup: could not clear %s\n%s' % (name, sitl.log_tail())

        ok = False
        for _ in range(5):
            req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
            req.opcode = req.OPCODE_ERASE
            rsp = _request_wait(node, 10, req, timeout=3.0)
            if rsp is not None and rsp.ok:
                ok = True
                break
            time.sleep(0.3)
        assert ok, 'OPCODE_ERASE failed\n' + sitl.log_tail()

        temp = _get_param(node, 10, 'TEMPERATURE_LIMIT')
        current = _get_param(node, 10, 'CURRENT_LIMIT')
        band = _get_param(node, 10, 'TEMP_DERATE_BAND')
        for name, rsp in (('TEMPERATURE_LIMIT', temp), ('CURRENT_LIMIT', current),
                          ('TEMP_DERATE_BAND', band)):
            assert rsp is not None, 'read-back of %s failed\n%s' % (name, sitl.log_tail())

        t = int(temp.value.integer_value)
        c = int(current.value.integer_value)   # reported in amps (2x the byte)
        b = int(band.value.integer_value)
        # Src/settings.c arms only inside these ranges; outside is "off".
        assert 70 <= t <= 140, \
            'erase left the thermal foldback disabled: TEMPERATURE_LIMIT=%d' % t
        assert 0 < c <= 200, \
            'erase left the current limiter disabled: CURRENT_LIMIT=%d' % c
        assert 5 <= b <= 40, 'erase left a nonsense foldback band: %d' % b
        # Shipped ARK_G431_CAN envelope (TARGET_DEFAULT_* in targets.h).
        assert (t, c, b) == (105, 200, 15), \
            'erase restored %s, expected (105, 200, 15)' % ((t, c, b),)
    finally:
        node.close()
