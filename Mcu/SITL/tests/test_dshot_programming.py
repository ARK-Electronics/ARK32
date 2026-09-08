"""Exercise legacy PX4 programming frames through the real DShot decoder."""

import time

import pytest
import sitl_dshot as sd
from sitl_gui_backend import EepromClient
from sitl_harness import Sender


@pytest.fixture
def programmer(sitl_factory):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    time.sleep(2.2)
    tx.stop()
    port = sd.InputPort('127.0.0.1', sitl.input_port)
    ee = EepromClient(port=sitl.state_port)

    def send(value, telem=None, corrupt=False):
        port.send_dshot(value, ptype=sd.TYPE_DSHOT600,
                        telem=(value != 0) if telem is None else telem,
                        corrupt=corrupt)
        time.sleep(0.004)

    def enter():
        for _ in range(6):
            send(36)

    try:
        yield send, enter, ee
    finally:
        port.close()


@pytest.mark.parametrize('address,value', [(0, 0), (30, 7), (191, 255)])
def test_program_valid_boundaries(programmer, address, value):
    send, enter, ee = programmer
    before, _ = ee.fetch()
    assert before is not None
    enter()
    for word in (address, value, 37):
        send(word)
    after, _ = ee.fetch()
    expected = bytearray(before)
    expected[address] = value
    assert after == bytes(expected)


@pytest.mark.parametrize('words', [(192, 7, 37), (2047, 7, 37),
                                  (30, 256, 37), (30, 2047, 37),
                                  (30, 7, 0, 37)])
def test_invalid_transaction_does_not_write(programmer, words):
    send, enter, ee = programmer
    before, _ = ee.fetch()
    assert before is not None
    enter()
    for word in words:
        send(word)
    after, _ = ee.fetch()
    assert after == before


@pytest.mark.parametrize('stage', [0, 1, 2])
def test_expired_transaction_does_not_commit_and_can_restart(programmer, stage):
    send, enter, ee = programmer
    before, _ = ee.fetch()
    assert before is not None
    enter()
    for word in (30, 7)[:stage]:
        send(word)
    # Keep valid traffic present: signal timeout alone cannot end programming.
    for _ in range(40):
        send(0)
    send(37)
    after, _ = ee.fetch()
    assert after == before
    enter()
    for word in (30, 7, 37):
        send(word)
    after, _ = ee.fetch()
    assert after is not None and after[30] == 7


@pytest.mark.parametrize('corrupt', [False, True])
def test_unmarked_or_bad_crc_data_aborts(programmer, corrupt):
    send, enter, ee = programmer
    before, _ = ee.fetch()
    assert before is not None
    enter()
    send(30, telem=corrupt, corrupt=corrupt)
    send(7)
    send(37)
    after, _ = ee.fetch()
    assert after == before
