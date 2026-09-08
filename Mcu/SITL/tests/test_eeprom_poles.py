"""Pole-count invariants at flash load and live protocol boundaries."""

import time

import pytest
import sitl_dshot as sd
import sitl_params
from sitl_gui_backend import EepromClient
from sitl_harness import Sender


@pytest.mark.parametrize('poles', [0, 1, 2, 14, 46, 128, 129, 255])
def test_flash_pole_count_is_sanitized(sitl_factory, workdir, poles):
    from pathlib import Path
    image = bytearray([255] * 192)
    for offset, _name, _lo, _hi, default, _help in sitl_params.PARAMS:
        image[offset] = default
    image[27] = poles
    path = Path(workdir) / 'pole-settings.bin'
    path.write_bytes(image)
    sitl = sitl_factory(extra_args=['--input-type', '1', '--eeprom', str(path)], can_uri='none')
    result, _ = EepromClient(port=sitl.state_port).fetch()
    assert result is not None, sitl.log_tail()
    assert result[27] == (poles if 2 <= poles <= 128 else 14)
    assert sitl.proc.poll() is None, sitl.log_tail()


@pytest.mark.parametrize('poles', [0, 1, 128, 255])
def test_dshot_cannot_publish_invalid_poles(sitl_factory, poles):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    time.sleep(2.2)
    tx.stop()
    port = sd.InputPort('127.0.0.1', sitl.input_port)
    try:
        for word in [36] * 6 + [27, poles, 37]:
            port.send_dshot(word, ptype=sd.TYPE_DSHOT600, telem=word != 0)
            time.sleep(0.004)
        result, _ = EepromClient(port=sitl.state_port).fetch()
        assert result is not None, sitl.log_tail()
        assert result[27] == (poles if 2 <= poles <= 128 else 14)
    finally:
        port.close()


@pytest.mark.parametrize('poles', [-1, 0, 1, 128, 129, 256, 65536])
def test_can_checks_poles_before_narrowing(sitl_can_factory, mcast_uri, poles):
    import dronecan
    from test_params import _request_wait
    sitl = sitl_can_factory(extra_args=['--node-id', '10'], can_uri=mcast_uri, wait_s=1.0)
    node = dronecan.make_node(mcast_uri, node_id=115, bitrate=1000000)
    try:
        req = dronecan.uavcan.protocol.param.GetSet.Request()
        req.name = 'MOTOR_POLES'
        req.value = dronecan.uavcan.protocol.param.Value(integer_value=poles)
        rsp = None
        for _ in range(5):
            rsp = _request_wait(node, 10, req)
            if rsp is not None:
                break
        assert rsp is not None, sitl.log_tail()
        assert int(rsp.value.integer_value) == (poles if 2 <= poles <= 128 else 14)
    finally:
        node.close()
