"""Pole-count invariants at flash load and live protocol boundaries."""

import pytest
import sitl_params
from sitl_gui_backend import EepromClient
from test_dshot_programming import programmer


def _advance_erpm_scale_q12(kv, poles):
    # loadEEpromSettings(): the timing schedule's eRPM scale, 0 below 300 kV.
    scale = (kv * poles * 256 // 12500) & 0xffff
    return scale - (scale >> 4)


@pytest.mark.parametrize('version', [3, 0])
@pytest.mark.parametrize('poles', [0, 1, 2, 14, 46, 128, 129, 255])
def test_flash_pole_count_is_sanitized(sitl_factory, workdir, poles, version):
    from pathlib import Path
    from test_state_protocol import Watch, control_socket
    image = bytearray([255] * 192)
    for offset, _name, _lo, _hi, default, _help in sitl_params.PARAMS:
        image[offset] = default
    image[27] = poles
    image[1] = version  # 0 skips the version-gated load policies
    path = Path(workdir) / 'pole-settings.bin'
    path.write_bytes(image)
    sitl = sitl_factory(extra_args=['--input-type', '1', '--eeprom', str(path)], can_uri='none')
    expected = poles if 2 <= poles <= 128 else 14
    result, _ = EepromClient(port=sitl.state_port).fetch()
    assert result is not None, sitl.log_tail()
    assert result[27] == expected
    # The SITL CAN build's DroneCAN startup also range-checks the byte, after
    # the loader has used it, so the byte alone cannot show where it was
    # fixed. The loader's own output does: unsanitized poles leave the
    # timing scale at 0 (the duty proxy) on both layout versions.
    kv = image[26] * 40 + 20
    with control_socket(sitl) as sock:
        watch = Watch(sock, [(2, 'advance_erpm_scale_q12')])
        watch.until(lambda: 0 in watch.values)
    assert watch.values[0] == _advance_erpm_scale_q12(kv, expected), sitl.log_tail()
    assert sitl.proc.poll() is None, sitl.log_tail()


@pytest.mark.parametrize('telem', [False, True])
@pytest.mark.parametrize('poles', [0, 1, 128, 255, 256, 257, 384, 2047])
def test_dshot_cannot_publish_invalid_poles(programmer, poles, telem):
    send, enter, ee, sitl, _ = programmer
    # Seed a distinct value so sanitization cannot pass on a dropped write.
    ok, message = ee.set(27, [22])
    assert ok, message
    before, _ = ee.fetch()
    assert before is not None, sitl.log_tail()
    enter(telem)
    for word in (27, poles, 37):
        send(word, telem)
    result, _ = ee.fetch()
    # Legacy DShot stores the low byte; validate that byte before publishing it.
    stored_poles = poles & 0xff
    expected = bytearray(before)
    expected[27] = stored_poles if 2 <= stored_poles <= 128 else 14
    assert result == bytes(expected), sitl.log_tail()


@pytest.mark.parametrize('poles', [-1, 0, 1, 128, 129, 256, 65536])
def test_can_checks_poles_before_narrowing(sitl_can_factory, mcast_uri, poles):
    import dronecan
    from test_params import _request_wait
    sitl = sitl_can_factory(extra_args=['--node-id', '10'], can_uri=mcast_uri, wait_s=1.0)
    node = dronecan.make_node(mcast_uri, node_id=115, bitrate=1000000)
    try:
        def set_poles(value):
            req = dronecan.uavcan.protocol.param.GetSet.Request()
            req.name = 'MOTOR_POLES'
            req.value = dronecan.uavcan.protocol.param.Value(integer_value=value)
            for _ in range(5):
                rsp = _request_wait(node, 10, req)
                if rsp is not None:
                    return int(rsp.value.integer_value)
            assert False, sitl.log_tail()

        # A distinct valid value first, so a rejected write is told apart from a reset to the default.
        assert set_poles(22) == 22
        assert set_poles(poles) == (poles if 2 <= poles <= 128 else 22)
    finally:
        node.close()
