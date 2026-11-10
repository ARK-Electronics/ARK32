'''A missed window costs its own length in blind steps, not a cascade.

When the blind-step deadline fires, PeriodElapsedCallback commutates on dead
reckoning. It used to take the whole elapsed time (~1.5 CI) as the interval
and restart INTERVAL_TIMER at the blind commutation. That grew the estimate
12.5% per step and put the timer origin ~CI/2 late, so the next real
crossing arrived inside the comparator's average_interval/2 blanking and was
dropped: on this plant one missed window became 7 blind steps with the
estimate doubled, and more than half the dropouts ended in a desync restart.
On the ARK 4IN1 bench the same cascade spiked the bus current to 13-41 A.

The blind step now places the missed crossing where it was expected (one
estimated interval after the last crossing), restarts the timer at the grace
already spent, and arms the next deadline from that virtual crossing.

These tests hold steady speed, blank the comparator (ZC_FAULT mode 1) for one
and two commutation windows, and watch every change of zc_blind_steps,
commutation_interval, desync_happened and zero_crosses. zc_blind_steps is
the consecutive count, reset by every accepted crossing, so the total cost
of a dropout is the sum of its increments: a blind step / false accept /
blind step loop would show a peak of 1 but a growing total. The estimate is
bounded both ways: a cascade grows it, a false accept ~CI/2 early shrinks it.
Only dropouts that produced a blind step are judged, so a host stall that
delays the fault cannot make a case pass vacuously. The chop case injects
the dropout while the rotor is decelerating, where dead reckoning at
constant speed is the wrong model and the next real crossing must still be
accepted.

Mutation check: the old blind step (elapsed as the interval, timer restarted
at 0) fails every case here on the first dropout.
'''

from __future__ import annotations

import socket
import struct
import time

import pytest
from sitl_state_protocol import STATE_MAGIC_CMD, STATE_CMD_ZC_FAULT
from test_blind_step import _spin_up
from test_state_protocol import WATCH_EVENT

VARS = [(1, 'zc_blind_steps'), (4, 'commutation_interval'), (4, 'desync_happened'),
        (4, 'zero_crosses'), (1, 'old_routine'), (1, 'running')]
BLIND, CI, DESYNC, ZC, OLD, RUNNING = range(6)

COUNTED = 5          # dropouts judged per case
MAX_ATTEMPTS = 15    # dropouts tried per case before giving up
THROTTLE = 1000      # DShot300; closed loop on the default model
CHOP_TO = 250


class _AllChangesWatch:
    '''Watch (cmd 8) with a zero minimum period: every change is reported,
    so the peak of a 1 ms blind-step burst is not coalesced away.'''

    def __init__(self, sitl):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.connect(('127.0.0.1', sitl.state_port))
        self.sock.settimeout(0.02)
        self.req = struct.pack('<HBBI', STATE_MAGIC_CMD, 8, len(VARS), 0)
        for size, name in VARS:
            self.req += bytes([size]) + name.encode('ascii') + b'\0'
        self.values = {}
        self.next_refresh = 0.0

    def poll(self):
        if time.monotonic() >= self.next_refresh:
            self.sock.send(self.req)
            self.next_refresh = time.monotonic() + 0.4
        try:
            data = self.sock.recv(65536)
        except socket.timeout:
            return []
        magic, _version, _count = struct.unpack_from('<HBB', data)
        if magic != 0x535a:
            return []
        events = []
        for off in range(4, len(data), WATCH_EVENT.size):
            ev = WATCH_EVENT.unpack_from(data, off)
            self.values[ev[1]] = ev[2]
            events.append(ev)
        return events

    def run_for(self, seconds):
        events = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            events += self.poll()
        return events

    def close(self):
        self.sock.close()


def _closed_loop(v):
    return v.get(RUNNING) == 1 and v.get(OLD) == 0 and v.get(ZC, 0) > 200


def _dropouts(sitl, sim, tx, windows, chop):
    ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctl.connect(('127.0.0.1', sitl.state_port))
    w = _AllChangesWatch(sitl)
    judged = []
    attempts = 0
    try:
        w.run_for(0.5)
        while len(judged) < COUNTED and attempts < MAX_ATTEMPTS:
            attempts += 1
            tx.value = THROTTLE
            w.run_for(0.6)
            if chop:
                tx.value = CHOP_TO
                w.run_for(0.03)   # dropout lands mid-deceleration
            v = dict(w.values)
            if not _closed_loop(v):
                continue
            ci0, desync0, zc0 = v[CI], v[DESYNC], v[ZC]
            dur_us = int(ci0 / 2 * windows) + 20   # CI is in 0.5 us ticks
            ctl.send(struct.pack('<HBBI', STATE_MAGIC_CMD, STATE_CMD_ZC_FAULT, 1, dur_us))
            blind0 = v.get(BLIND, 0)
            events = w.run_for(0.3)
            total_blind, last = 0, blind0
            for e in events:
                if e[1] == BLIND:
                    total_blind += max(0, e[2] - last)
                    last = e[2]
            cis = [e[2] for e in events if e[1] == CI] or [ci0]
            peak_ci, min_ci = max(cis), min(cis)
            restarted = (any(e[1] == ZC and e[2] < zc0 for e in events)
                         or any(e[1] == OLD and e[2] for e in events))
            desyncs = w.values.get(DESYNC, desync0) - desync0
            if total_blind == 0 and not restarted and not desyncs:
                continue   # the fault never reached the loop (host stall): not judged
            judged.append(dict(blind=total_blind, ci_ratio=peak_ci / float(ci0),
                               ci_min_ratio=min_ci / float(ci0),
                               restarted=restarted, desyncs=desyncs, ci0=ci0))
    finally:
        w.close()
        ctl.close()
    assert len(judged) >= 3, (
        'only %d of %d dropouts reached the loop\n%s' % (len(judged), attempts, sitl.log_tail()))
    return judged


@pytest.mark.parametrize('windows', [1, 2])
def test_steady_dropout_costs_its_own_length(sitl_factory, state_stream, windows):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim, tx, _ = _spin_up(sitl, state_stream, value=THROTTLE)
    try:
        judged = _dropouts(sitl, sim, tx, windows, chop=False)
    finally:
        tx.stop()
    for d in judged:
        assert not d['restarted'] and d['desyncs'] == 0, (
            'a %d-window dropout at steady speed ended in a restart or desync: %r\n%s'
            % (windows, judged, sitl.log_tail()))
        assert d['blind'] <= windows + 2, (
            'a %d-window dropout cost %d blind steps (cascade): %r' % (windows, d['blind'], judged))
        assert d['ci_ratio'] < 1.2, (
            'a %d-window dropout grew the interval estimate %.2fx at steady speed: %r'
            % (windows, d['ci_ratio'], judged))
        assert d['ci_min_ratio'] > 0.85, (
            'a %d-window dropout shrank the interval estimate to %.2fx at steady speed '
            '(an early false crossing accepted after a blind step?): %r'
            % (windows, d['ci_min_ratio'], judged))


def test_dropout_during_throttle_chop_is_bridged(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim, tx, _ = _spin_up(sitl, state_stream, value=THROTTLE)
    try:
        judged = _dropouts(sitl, sim, tx, 1, chop=True)
    finally:
        tx.stop()
    for d in judged:
        assert not d['restarted'] and d['desyncs'] == 0, (
            'a one-window dropout during a throttle chop ended in a restart or desync: %r\n%s'
            % (judged, sitl.log_tail()))
        assert d['blind'] <= 3, (
            'a one-window dropout during a throttle chop cost %d blind steps: %r' % (d['blind'], judged))
        # The rotor is slowing, so the estimate only grows; an early false
        # crossing accepted after a blind step would pull it down.
        assert d['ci_min_ratio'] > 0.9, (
            'a dropout during a throttle chop shrank the interval estimate to %.2fx: %r'
            % (d['ci_min_ratio'], judged))
