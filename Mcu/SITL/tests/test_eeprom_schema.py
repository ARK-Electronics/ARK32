"""Generated layout/defaults and EEPROM behavior in the real firmware process."""

from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import subprocess
import sys
import threading
import time

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO_ROOT / 'scripts' / 'eeprom'))

from schema import default_bytes, load_schema, resolve_schema, snake_case  # noqa: E402
from sitl_gui_backend import EepromClient  # noqa: E402
from sitl_harness import Sender  # noqa: E402
import sitl_dshot as sd  # noqa: E402


@pytest.fixture(scope='module')
def document():
    return load_schema()


@pytest.fixture(scope='module')
def current_schema(document):
    return resolve_schema(document, layout=3, firmware='32.0')


def _fetch(client):
    image, _model = client.fetch()
    assert image is not None, 'SITL EEPROM fetch timed out'
    assert len(image) == 192
    return image


def _write(client, offset, data):
    ok, message = client.set(offset, data)
    assert ok, message


def _wait_for_esc(uri, our_id):
    from test_params import _wait_for_node

    node, found = _wait_for_node(uri, our_id=our_id)
    deadline = time.monotonic() + 8
    # The existing helper returns on any NodeStatus, including our separate
    # keepalive node. Wait specifically for the ESC before issuing requests.
    while 10 not in found and time.monotonic() < deadline:
        node.spin(0.05)
    return node, found


@contextmanager
def zero_throttle_can(uri, node_id=124):
    """Keep firmware alive while the test blocks on UDP or a codec subprocess.

    Use a separate node owned entirely by this thread. pydronecan nodes are
    not shared with the foreground GetSet client.
    """
    import dronecan

    stopped = threading.Event()
    errors = []

    def run():
        node = None
        try:
            node = dronecan.make_node(uri, node_id=node_id, bitrate=1000000)
            while not stopped.is_set():
                node.broadcast(dronecan.uavcan.equipment.safety.ArmingStatus(status=255))
                node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0] * 20))
                node.spin(0.01)
                stopped.wait(0.01)
        except Exception as error:
            errors.append(error)
        finally:
            if node is not None:
                node.close()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    try:
        yield
        assert not errors, f'CAN keepalive failed: {errors}'
    finally:
        stopped.set()
        thread.join(timeout=5)
        assert not thread.is_alive(), 'CAN keepalive did not stop'


def test_generated_packed_offsets_and_defaults(document, current_schema, tmp_path):
    generated = REPO_ROOT / 'obj' / 'generated'
    header = (generated / 'eeprom_layout.h').read_text()
    for required in ('#include <stdint.h>', 'motor_poles', 'can_esc_index',
                     'sine_mode_range', '#define MOTOR_POLES_MAX 128', 'buffer[192]'):
        assert required in header
    assertions = ['_Static_assert(sizeof(EEprom_t) == 192, "image size");']
    for key, field in current_schema['fields'].items():
        member = snake_case(key)
        assertions.append(
            f'_Static_assert(offsetof(EEprom_t, {member}) == {field["offset"]}, "{key}");')
        assertions.append(
            f'_Static_assert(sizeof(((EEprom_t *)0)->{member}) == {field["size"]}, "{key} width");')
    source = tmp_path / 'offsets.c'
    source.write_text('\n'.join([
        '#include <stddef.h>', '#include <stdio.h>',
        '#include "eeprom_layout.h"', '#include "eeprom_defaults.h"',
        *assertions,
        'int main(void) { return fwrite(default_settings, 1, sizeof(default_settings), stdout) == 48 ? 0 : 1; }',
    ]))
    binary = tmp_path / 'offsets'
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Werror', '-I', str(generated),
                    str(source), '-o', str(binary)], check=True, capture_output=True, text=True)
    compiled_defaults = subprocess.run([str(binary)], check=True, capture_output=True).stdout
    golden = bytes.fromhex((REPO_ROOT / 'schema' / 'eeprom-defaults.hex').read_text())
    assert len(golden) == 48
    assert compiled_defaults == golden == default_bytes(document, layout=3, firmware='32.0')


def test_eeprom_boot_version_and_surviving_raw_values(sitl_factory, current_schema):
    sitl = sitl_factory(extra_args=['--input-type', '1', '--eeprom', 'schema-sweep.bin'], can_uri='none')
    client = EepromClient('127.0.0.1', sitl.state_port)
    sender = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        base = _fetch(client)
        assert base[1] == 3
        assert base[3:5] == bytes([32, 0])
        checked = set()
        for key, field in current_schema['fields'].items():
            bounds = field.get('raw', {})
            if (field['size'] != 1 or field.get('preserveOnDefaults')
                    or field.get('hidden') or 'min' not in bounds or 'max' not in bounds):
                continue
            lo, hi = int(bounds['min']), int(bounds['max'])
            for value in sorted({lo, (lo + hi) // 2, hi}):
                # These are legacy translations / target restrictions, covered
                # separately below. Every other field tests its real load path.
                if key == 'timingAdvance' and value < 4:
                    continue
                if key == 'hallSensors' and value != 0:
                    continue
                image = bytearray(base)
                image[field['offset']] = value
                _write(client, 0, image)
                actual = _fetch(client)
                assert actual[field['offset']] == value, (key, value, actual[field['offset']])
                checked.add(key)
        assert {'motorKv', 'motorPoles', 'temperatureLimit', 'currentLimit', 'inputType'} <= checked
    finally:
        sender.stop()


@pytest.mark.parametrize('key,raw,expected', [
    ('timingAdvance', 3, 34),
    ('temperatureLimit', 69, 255), ('temperatureLimit', 141, 255),
    ('temperatureLimit', 255, 255),
    ('sineModeRange', 4, 5), ('sineModeRange', 26, 5),
    ('dragBrakeStrength', 0, 10), ('dragBrakeStrength', 11, 10),
    ('runningBrakeLevel', 0, 10), ('runningBrakeLevel', 10, 10),
    ('runningBrakeLevel', 11, 10),
    ('sineModePower', 0, 5), ('sineModePower', 11, 5),
    ('hallSensors', 1, 0),
    ('currentLimit', 0, 0), ('currentLimit', 102, 102),
])
def test_eeprom_load_policies(sitl_factory, current_schema, key, raw, expected):
    sitl = sitl_factory(extra_args=['--input-type', '1', '--eeprom', 'schema-load.bin'], can_uri='none')
    client = EepromClient('127.0.0.1', sitl.state_port)
    offset = current_schema['fields'][key]['offset']
    _write(client, offset, bytes([raw]))
    assert _fetch(client)[offset] == expected


def test_eeprom_can_save_survives_restart(sitl_can_factory, mcast_uri):
    dronecan = pytest.importorskip('dronecan')
    from test_params import _get_param, _request_wait, _set_param

    with zero_throttle_can(mcast_uri):
        sitl = sitl_can_factory(extra_args=['--node-id', '10', '--eeprom', 'schema-persist.bin'],
                                can_uri=mcast_uri)
        client = EepromClient('127.0.0.1', sitl.state_port)
        # A different persisted starting value makes SAVE observable: cmd 6
        # itself persists, while GetSet alone only changes runtime state.
        _write(client, 26, bytes([35]))
        node, found = _wait_for_esc(mcast_uri, our_id=121)
        try:
            assert 10 in found, 'ESC missing from software CAN bus\n' + sitl.log_tail()
            assert _set_param(node, 10, 'MOTOR_KV', 1020) is not None
            req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
            req.opcode = req.OPCODE_SAVE
            response = _request_wait(node, 10, req)
            assert response is not None and response.ok
        finally:
            node.close()
        sitl.close()
        restarted = sitl_can_factory(extra_args=['--node-id', '10', '--eeprom', 'schema-persist.bin'],
                                    can_uri=mcast_uri)
        node, found = _wait_for_esc(mcast_uri, our_id=122)
        try:
            assert 10 in found, 'Restarted ESC missing\n' + restarted.log_tail()
            response = _get_param(node, 10, 'MOTOR_KV')
            assert response is not None and int(response.value.integer_value) == 1020
            assert _fetch(EepromClient('127.0.0.1', restarted.state_port))[26] == 25
        finally:
            node.close()


def test_dronecan_schema_ranges_and_integer_stores(sitl_can_factory, mcast_uri):
    dronecan = pytest.importorskip('dronecan')
    from test_params import _get_param, _request_wait, _set_param

    with zero_throttle_can(mcast_uri):
        sitl = sitl_can_factory(extra_args=['--node-id', '10', '--eeprom', 'schema-wire.bin'],
                                can_uri=mcast_uri)
        client = EepromClient('127.0.0.1', sitl.state_port)
        node, found = _wait_for_esc(mcast_uri, our_id=121)
        try:
            assert 10 in found, sitl.log_tail()
            for name in ('BRAKE_ON_STOP', 'LOW_VOLTAGE_CUTOFF'):
                response = _get_param(node, 10, name)
                assert response is not None
                assert int(response.max_value.integer_value) == 2
                assert _set_param(node, 10, name, 2) is not None
            assert int(_get_param(node, 10, 'ESC_INDEX').max_value.integer_value) == 31
            assert int(_get_param(node, 10, 'MOTOR_POLES').max_value.integer_value) == 128
            assert int(_get_param(node, 10, 'STARTUP_POWER').default_value.integer_value) == 100

            # Wire stores truncate, whereas UI conversions round. An odd
            # current limit must therefore come back as the preceding even A.
            request = dronecan.uavcan.protocol.param.GetSet.Request()
            request.name = 'CURRENT_LIMIT'
            request.value = dronecan.uavcan.protocol.param.Value(integer_value=101)
            response = _request_wait(node, 10, request)
            assert response is not None and int(response.value.integer_value) == 100
            assert _fetch(client)[44] == 50
            assert _set_param(node, 10, 'ADVANCE_LEVEL', 17) is not None
            assert _fetch(client)[23] == 27
            assert _set_param(node, 10, 'MOTOR_KV', 1021) is not None
            assert _fetch(client)[26] == 25
            assert _set_param(node, 10, 'CELL_VOLTAGE_THRESHOLD', 301) is not None
            assert _fetch(client)[37] == 51
        finally:
            node.close()
