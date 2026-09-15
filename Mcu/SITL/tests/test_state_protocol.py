"""Exercise upstream state-port additions against the native firmware.

These tests use wire packets directly so the ARK GUI cannot mask a protocol
regression. The low upstream command numbers must coexist with ARK's fault
injection and control commands in the 0x80 range.
"""

import math
import socket
import struct
import time

import sitl_dshot as sd
from sitl_gui_backend import EepromClient
from sitl_harness import Sender
from sitl_state_protocol import STATE_MAGIC_CMD


STATE_SAMPLE = struct.Struct('<Q11f3sBB3x')
SCOPE_EXTRA = struct.Struct('<7f3bxfI')
WATCH_EVENT = struct.Struct('<QIQ')


def control_socket(sitl):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    sock.connect(('127.0.0.1', sitl.state_port))
    sock.settimeout(0.05)
    return sock


class Watch:
    def __init__(self, sock, variables):
        self.sock = sock
        self.request = struct.pack('<HBBI', STATE_MAGIC_CMD, 8,
                                   len(variables), 1_000_000)
        for size, name in variables:
            self.request += bytes([size]) + name.encode('ascii') + b'\0'
        self.resolved = None
        self.events = []
        self.values = {}
        self.next_refresh = 0

    def poll(self, refresh=True):
        if refresh and time.monotonic() >= self.next_refresh:
            self.sock.send(self.request)
            self.next_refresh = time.monotonic() + 0.4
        try:
            data = self.sock.recv(4096)
        except socket.timeout:
            return
        assert len(data) >= 4
        magic, version, count = struct.unpack_from('<HBB', data)
        assert version == 1
        if magic == 0x5359:
            assert len(data) == 4 + count
            self.resolved = list(data[4:])
        else:
            assert magic == 0x535a
            assert len(data) == 4 + count * WATCH_EVENT.size
            for offset in range(4, len(data), WATCH_EVENT.size):
                event = WATCH_EVENT.unpack_from(data, offset)
                self.events.append(event)
                self.values[event[1]] = event[2]

    def until(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, (
                'watch timed out: resolved=%r values=%r events=%r'
                % (self.resolved, self.values, self.events[-10:]))
            self.poll()


def test_watch_initial_values_changes_and_identical_refresh(sitl_factory):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        editor = EepromClient(port=sitl.state_port)
        ok, message = editor.set(30, [7])  # BEEP_VOLUME -> beep_volume = 3 * 7
        assert ok, message
        with control_socket(sitl) as sock:
            watch = Watch(sock, [(1, 'beep_volume'), (1, 'no_such_sitl_global'),
                                 (3, 'beep_volume'), (1, 'beep_volume')])
            watch.until(lambda: watch.resolved is not None and 3 in watch.values)
            assert watch.resolved == [1, 0, 0, 1]
            assert watch.values == {0: 21, 3: 21}
            initial = list(watch.events)

            # Refresh beyond the 2-second lease. A refresh must not replay
            # unchanged values, and must keep the subscription alive.
            deadline = time.monotonic() + 3.2
            while time.monotonic() < deadline:
                watch.poll()
            assert watch.events == initial

            ok, message = editor.set(30, [6])
            assert ok, message
            # Do not refresh after changing the value: this must use the
            # existing lease, not start a replacement subscription.
            deadline = time.monotonic() + 2
            while watch.values.get(3) != 18 and time.monotonic() < deadline:
                watch.poll(refresh=False)
            assert watch.values == {0: 18, 3: 18}
            assert len(watch.events) == len(initial) + 2
            assert watch.events[-1][0] > initial[-1][0]
    finally:
        tx.stop()


def test_reset_during_beacon_preserves_eeprom_and_resumes(sitl_factory):
    sitl = sitl_factory(extra_args=['--input-type', '1', '--speedup', '0.5'],
                        can_uri='none', wait_s=0.1)
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        editor = EepromClient(port=sitl.state_port)
        ok, message = editor.set(30, [7])
        assert ok, message
        with control_socket(sitl) as sock:
            watch = Watch(sock, [(1, 'armed'), (1, 'sitl_tone_active')])
            watch.until(lambda: watch.values.get(0) == 1
                        and watch.values.get(1) == 0, timeout=25)
            tx.cmds = [sd.DSHOT_CMD_BEACON1]
            watch.until(lambda: watch.values.get(1) == 1)
            before_reset = watch.events[-1][0]
            sock.send(struct.pack('<HBB', STATE_MAGIC_CMD, 9, 0))

            deadline = time.monotonic() + 5
            while 'reset (state port)' not in sitl.log_tail(100):
                assert time.monotonic() < deadline, sitl.log_tail(100)
                assert sitl.proc.poll() is None, sitl.log_tail(100)
                time.sleep(0.02)

            # Reset re-execs with the same PID. UDP service must recover,
            # keep the flash image, and report a fresh simulation clock.
            image, _ = editor.fetch()
            assert image is not None, sitl.log_tail(100)
            assert image[30] == 7
            restarted = Watch(sock, [(1, 'beep_volume'), (1, 'sitl_tone_active')])
            restarted.until(lambda: restarted.resolved is not None
                            and restarted.values.get(0) == 21)
            assert restarted.resolved == [1, 1]
            assert any(event[0] < before_reset for event in restarted.events)
            assert sitl.proc.poll() is None, sitl.log_tail(100)
            assert sitl.log_tail(100).count('reset (state port)') == 1
    finally:
        tx.stop()


def test_scope_opt_in_preserves_legacy_state_wire_layout(sitl_factory):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        with control_socket(sitl) as sock:
            for flags, version, size in ((0, 2, 60), (2, 3, 100),
                                         (3, 3, 100), (1, 2, 60)):
                request = struct.pack('<HBBI', STATE_MAGIC_CMD, 0, flags, 50000)
                deadline = time.monotonic() + 3
                matched = None
                next_send = 0
                while time.monotonic() < deadline:
                    if time.monotonic() >= next_send:
                        sock.send(request)
                        next_send = time.monotonic() + 0.4
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        continue
                    magic, received_version, count = struct.unpack_from('<HBB', data)
                    assert magic == 0x5354
                    if received_version == version:
                        matched = data
                        break
                assert matched is not None, 'no state version %d' % version
                assert 1 <= count <= 12
                assert len(matched) == 4 + count * size
                previous_time = -1
                for offset in range(4, len(matched), size):
                    sample = STATE_SAMPLE.unpack_from(matched, offset)
                    assert sample[0] > previous_time
                    previous_time = sample[0]
                    assert all(math.isfinite(value) for value in sample[1:12])
                    assert 10 < sample[10] < 30  # default model bus voltage
                    if version == 3:
                        extra = SCOPE_EXTRA.unpack_from(matched, offset + 60)
                        assert all(math.isfinite(value) for value in extra[:7])
                        assert all(value in (-1, 0, 1) for value in extra[7:10])
                        assert 0 <= extra[10] <= 1  # active duty
    finally:
        tx.stop()
