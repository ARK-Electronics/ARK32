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
                             (2, 'newinput'), (1, 'running'),
                             (1, 'programming_mode'), (1, 'command_count')])
        try:
            watch.until(lambda: watch.values.get(0) == 1
                        and watch.values.get(1) == 0)
            # Morse arming tones contain silent gaps. Keep sending stop until
            # the tone has stayed quiet longer than a gap, with IRQs live.
            quiet_since = time.monotonic()
            deadline = quiet_since + 5
            while time.monotonic() - quiet_since < 0.15:
                assert time.monotonic() < deadline, sitl.log_tail()
                watch.poll()
                if watch.values.get(1):
                    quiet_since = time.monotonic()
            tx.stop()
            ee = EepromClient(port=sitl.state_port)
            # The last byte defaults to 255; seed it so writing 255 proves
            # that the upper valid boundary actually reached the decoder.
            ok, message = ee.set(191, [0])
            assert ok, message
            watch.until(lambda: 4 in watch.values and 5 in watch.values)

            # Deliver each frame exactly once, and read back only after the
            # commit. After a short host stall the SITL catches up and reads
            # queued UDP frames back to back in simulated time: a frame that
            # lands before the capture is re-armed is lost as on the wire,
            # and a fixed 4 ms gap let the EEPROM read beat the commit. So each frame waits
            # for the decoder state it must produce (command_count for the
            # enter command, then programming_mode 1 -> 2 -> 3 -> 0). Only a
            # frame whose state never shows is resent, inside the armed
            # ESC's 0.5 s signal-loss reset (faults.c), and a resend must
            # leave the state exactly there: a late original would push it
            # one step further.
            def show(what):
                return '%s; armed=%r programming_mode=%r command_count=%r\n%s' % (
                    what, watch.values.get(0), watch.values.get(4),
                    watch.values.get(5), sitl.log_tail())

            def frame(value, telem, reached, what):
                for attempt in range(2):
                    port.send_dshot(value, ptype=sd.TYPE_DSHOT600, telem=telem)
                    deadline = time.monotonic() + 0.3
                    while time.monotonic() < deadline:
                        watch.poll()
                        if reached():
                            if attempt:
                                settle = time.monotonic() + 0.05
                                while time.monotonic() < settle:
                                    watch.poll()
                                assert reached(), show('%s: the resent frame was a duplicate' % what)
                            return
                pytest.fail(show('%s: frame %d never decoded' % (what, value)))

            def enter(telem):
                assert watch.values.get(4) == 0, show('enter outside idle')
                for k in range(1, 7):
                    if k < 6:
                        reached = (lambda k=k: watch.values.get(5) == k)
                    else:
                        reached = (lambda: watch.values.get(4) == 1)
                    frame(36, telem, reached, 'enter %d/6' % k)

            def send(value, telem):
                mode = watch.values.get(4)
                assert mode in (1, 2, 3), show('word %d outside programming mode' % value)
                assert mode != 3 or value == 37, 'only the commit (37) leaves WAIT_COMMIT'
                want = {1: 2, 2: 3, 3: 0}[mode]
                frame(value, telem, lambda: watch.values.get(4) == want,
                      'word %d in mode %d' % (value, mode))

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


@pytest.mark.parametrize('words', [(), (30,), (30, 7)])
def test_interrupted_programming_expires_despite_stop_traffic(programmer, words):
    send, enter, ee, sitl, watch = programmer
    before, _ = ee.fetch()
    assert before is not None, sitl.log_tail()
    enter(False)
    for word in words:
        send(word, False)
    # Keep the ordinary signal-loss watchdog alive. Without a separate
    # transaction deadline this stream leaves WAIT_COMMIT stuck forever.
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        watch.until(lambda: watch.values.get(4) == 0, timeout=1.0)
        assert watch.values.get(0) == 1, sitl.log_tail()
        assert not any(i in (2, 3) and v for _, i, v in watch.events)
        after, _ = ee.fetch()
        assert after == before, 'expired transaction wrote EEPROM'
    finally:
        tx.stop()
    enter(False)
    for word in (30, 7, 37):
        send(word, False)
    after, _ = ee.fetch()
    assert after is not None and after[30] == 7, sitl.log_tail()
