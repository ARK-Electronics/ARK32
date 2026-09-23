"""Legacy DShot writes, including commit-time bounds and sender compatibility.

Run with SITL_SANITIZE=address as well as the normal build: the invalid-address
cases must fail on an unguarded decoder even when adjacent RAM corruption does
not immediately affect the EEPROM readback.
"""

import time

import pytest
import sitl_dshot as sd
from sitl_gui_backend import EepromClient
from sitl_harness import Sender
from test_state_protocol import Watch, control_socket


@pytest.fixture
def programmer(sitl_factory):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    port = sd.InputPort('127.0.0.1', sitl.input_port)
    with control_socket(sitl) as sock:
        watch = Watch(sock, [(1, 'armed'), (1, 'sitl_tone_active'),
                             (2, 'newinput'), (1, 'running')])
        try:
            watch.until(lambda: watch.values.get(0) == 1
                        and watch.values.get(1) == 0)
            tx.stop()
            ee = EepromClient(port=sitl.state_port)

            def send(value, telem):
                port.send_dshot(value, ptype=sd.TYPE_DSHOT600, telem=telem)
                time.sleep(0.004)

            def enter(telem):
                for _ in range(6):
                    send(36, telem)

            yield send, enter, ee, sitl, watch
        finally:
            tx.stop()
            port.close()


@pytest.mark.parametrize('telem', [False, True])
@pytest.mark.parametrize('address,value', [(0, 0), (30, 7), (191, 255)])
def test_program_valid_boundaries(programmer, address, value, telem):
    send, enter, ee, sitl, _ = programmer
    before, _ = ee.fetch()
    assert before is not None, sitl.log_tail()
    enter(telem)
    for word in (address, value, 37):
        send(word, telem)
    after, _ = ee.fetch()
    expected = bytearray(before)
    expected[address] = value
    assert after == bytes(expected), sitl.log_tail()


@pytest.mark.parametrize('telem', [False, True])
@pytest.mark.parametrize('address', [192, 2047])
def test_invalid_address_is_consumed_and_rejected_at_commit(programmer, address, telem):
    send, enter, ee, sitl, watch = programmer
    before, _ = ee.fetch()
    assert before is not None, sitl.log_tail()
    enter(telem)
    send(address, telem)
    send(255, telem)
    # Give any accidental throttle enough time to become motor drive before
    # commit; the address guard must not abort into the normal throttle path.
    deadline = time.monotonic() + 0.15
    while time.monotonic() < deadline:
        watch.poll()
    assert not any(index in (2, 3) and value for _, index, value in watch.events)
    send(37, telem)
    assert sitl.proc.poll() is None, sitl.log_tail()
    after, _ = ee.fetch()
    assert after == before, sitl.log_tail()
    # Rejection must finish the transaction so the next valid write succeeds.
    enter(telem)
    for word in (30, 7, 37):
        send(word, telem)
    after, _ = ee.fetch()
    assert after is not None and after[30] == 7, sitl.log_tail()
