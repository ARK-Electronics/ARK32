'''PWM/DShot input path tests against the SITL motor model.'''

from __future__ import annotations

import time

import pytest

import sitl_dshot as sd
from sitl_harness import Sender, rpm_from_state, wait_for_state


def _run_throttle(sitl, state_stream, ptype, value, rpm_lo, rpm_hi,
                  bidir=False, edt=False, arm_s=2.2, run_s=4.0, stop_s=3.0):
    sim = state_stream(sitl)
    assert wait_for_state(sim), 'state stream dead\n' + sitl.log_tail()
    tx = Sender('127.0.0.1', sitl.input_port, ptype, bidir=bidir)
    try:
        time.sleep(arm_s)
        if edt:
            # firmware latches a DShot command only after six consecutive
            # frames while armed and stopped; frames can drop on a loaded
            # runner, so keep topping up the queue and confirm via replies
            for _ in range(6):
                tx.port.get_replies()
                tx.cmds = [sd.DSHOT_CMD_EDT_ENABLE] * 20
                time.sleep(0.5)
                got = [sd.decode_reply(r[3], edt_expected=True)
                       for r in tx.port.get_replies()]
                if any(k not in ('erpm', 'badcrc') for k, _v in got):
                    break
        tx.value = value
        time.sleep(run_s)
        rpm = rpm_from_state(sim)
        assert rpm_lo <= rpm <= rpm_hi, (
            'rpm=%.0f expected %d..%d\n%s' % (rpm, rpm_lo, rpm_hi, sitl.log_tail()))

        if bidir:
            replies = tx.port.reply_count
            assert replies > 500, 'too few BDShot replies: %d' % replies
            erpm = [sd.decode_reply(r[3], edt_expected=edt)
                    for r in tx.port.get_replies()]
            rpms = [sd.erpm_period_to_rpm(v) for k, v in erpm if k == 'erpm']
            if rpms:
                assert abs(rpms[-1] - rpm) < max(200, rpm * 0.05), (
                    'bdshot=%.0f state=%.0f' % (rpms[-1], rpm))
            if edt:
                edt_vals = {k: v for k, v in erpm
                            if k in ('temp', 'volt', 'current')}
                # SITL reports fixed plant telemetry (see sitl ADC/state);
                # temp is encoded as C, voltage as the simulated bus.
                assert edt_vals.get('temp') == 38, edt_vals
                assert 11 < edt_vals.get('volt', 0) < 13, edt_vals

        # must stop again at zero throttle. Poll with a deadline instead of
        # asserting after a fixed sleep: the coast-down is wall-clock
        # physics, and on a loaded CI runner a fixed window races it
        # (observed 570 rpm at the old 3 s check - a pass locally 8/8).
        tx.value = 0
        deadline = time.time() + stop_s + 4.0
        rpm = float('inf')
        while time.time() < deadline:
            rpm = rpm_from_state(sim, 0.3)
            if rpm < 500:
                break
        assert rpm < 500, 'motor did not stop: rpm=%.0f' % rpm
    finally:
        tx.stop()


def test_dshot600_bidir_edt(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    # floors leave ~10% margin under the v9 plant steady points
    # (throttle 800 ~3850 rpm with the default 7-inch model)
    _run_throttle(sitl, state_stream, sd.TYPE_DSHOT600, value=800,
                  rpm_lo=3500, rpm_hi=7000, bidir=True, edt=True)


def test_dshot300(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    _run_throttle(sitl, state_stream, sd.TYPE_DSHOT300, value=600,
                  rpm_lo=2500, rpm_hi=6000)


def test_zero_throttle_stays_stopped(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600, bidir=True)
    try:
        tx.value = 0
        time.sleep(3.0)
        rpm = rpm_from_state(sim, 0.5)
        assert rpm < 200, 'spun at zero throttle: rpm=%.0f' % rpm
    finally:
        tx.stop()


def test_bad_crc_frames_do_not_arm_or_spin(sitl_factory, state_stream):
    '''all-corrupted frames must never produce throttle'''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    port = sd.InputPort('127.0.0.1', sitl.input_port)
    try:
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < 4.0:
            now = time.time()
            while now >= nxt:
                nxt += 1.0 / 500.0
                port.send_dshot(800, ptype=sd.TYPE_DSHOT600, corrupt=True)
            time.sleep(0.0005)
        rpm = rpm_from_state(sim, 0.5)
        assert rpm < 200, 'motor spun on bad-CRC frames: rpm=%.0f' % rpm
    finally:
        port.close()


def test_single_full_throttle_frame_from_idle_is_ignored(sitl_factory, state_stream):
    '''one isolated full-scale frame at armed idle must not start the motor.

    This is the fault the idle-exit holdoff exists for, and it is NOT a CRC
    problem: an all-ones capture (0xFFFF, every pulse read long) is a
    CRC-VALID DShot frame carrying throttle 2047, and its frame span lands
    only +1.63% off nominal - inside the +-6.25% span gate. So one lost edge
    on the signal line turns a commanded stop into commanded full throttle
    and both integrity checks pass it. Observed on an ARK G431 CAN bench as a
    30-90 ms spin-up kick at armed idle, roughly once a minute.

    The isolated frame is only half of it: newinput LATCHES, because it is
    only ever overwritten by the next ACCEPTED frame. On the bench the same
    marginal link that corrupted one frame dropped the next several, so the
    bogus demand survived for the length of the dropout - that latch is what
    turns a 2 us decode error into a 30-90 ms burst of drive. A lone bogus
    frame inside an otherwise healthy stream is harmless (the next zero frame
    clears it ~2 ms later, before the plant moves), so the sequence
    reproduced here is bogus-frame-then-silence, which is what was actually
    measured.
    '''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), 'state stream dead\n' + sitl.log_tail()
    port = sd.InputPort('127.0.0.1', sitl.input_port)
    try:
        # arm on sustained zero, as an FC does while disarmed
        t0 = time.time()
        while time.time() - t0 < 2.5:
            port.send_dshot(0, ptype=sd.TYPE_DSHOT300)
            time.sleep(1.0 / 500.0)
        # one full-scale frame, then a dropout that lets it latch, then the
        # zero stream resumes. 120 ms stays well inside the 500 ms armed
        # signal-loss failsafe, so the ESC holds the stale demand rather
        # than resetting - exactly the bench case.
        t0 = time.time()
        injected = 0
        while time.time() - t0 < 5.0:
            port.send_dshot(2047, ptype=sd.TYPE_DSHOT300)
            injected += 1
            time.sleep(0.120)
            tq = time.time()
            while time.time() - tq < 0.25:
                port.send_dshot(0, ptype=sd.TYPE_DSHOT300)
                time.sleep(1.0 / 500.0)
        assert injected >= 5, 'injector did not run: %d' % injected
        rpm = rpm_from_state(sim, 0.5)
        assert rpm < 200, (
            'latched single-frame 2047 spun the motor: rpm=%.0f after %d '
            'injections\n%s' % (rpm, injected, sitl.log_tail()))
    finally:
        port.close()


def test_holdoff_delays_but_does_not_block_spool_up(sitl_factory, state_stream):
    '''a real, repeated demand must still spin up - the holdoff only delays.

    Guards the other side of the idle-exit holdoff: it discards
    DSHOT_IDLE_EXIT_HOLDOFF_FRAMES frames, so a genuine spool-up pays two
    frame periods (2.5 ms at 800 Hz) and nothing more. The 2 s deadline is
    far looser than that on purpose - it is here to catch a holdoff that
    blocks outright or never clears its counter, not to measure latency.
    '''
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), 'state stream dead\n' + sitl.log_tail()
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT300)
    try:
        tx.value = 0
        time.sleep(2.5)
        tx.value = 1000
        deadline = time.time() + 2.0
        rpm = 0.0
        while time.time() < deadline:
            rpm = rpm_from_state(sim, 0.3)
            if rpm > 1500:
                break
        assert rpm > 1500, (
            'holdoff blocked a sustained demand: rpm=%.0f\n%s' % (rpm, sitl.log_tail()))
    finally:
        tx.stop()
