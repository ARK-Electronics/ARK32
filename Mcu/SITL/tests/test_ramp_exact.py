'''The configured ramp runs exactly, on every pack, and never faster.

max_ramp_speed R is 0.1 %/ms of full duty: R/10 duty counts per control tick.
settings.c gives each regime whole counts per tick plus a remainder in tenths
that the 20 kHz slew limiter carries tick to tick, with no pack-voltage
scaling. EEPROM layout 3 scaled the ramp by 14.8 V / Vbat and truncated it, so
the factory 2.0 %/ms flew 1 count per tick on every pack above 14.8 V.

test_ramp_runs_at_setting drives the unloaded motor on 3S, 4S and 6S packs,
drops the throttle, and compares the applied duty fall with R/10 counts per
tick of the firmware's own tenkhzcounter. The 6S cases spin up on 4S and
switch packs while running (see SPIN_UP_MAX_V); the fall and the re-punch
are always measured on the pack under test. A ramp-down is used because nothing
but the slew limiter bounds a falling duty; the throttle ceilings only cap a
rising one, so the re-punch that follows is checked as an upper bound only.
The pack gets a regen sink so the bus does not pump on the drop. The fall
must not be faster than the setting, and not more than a few percent slower.
2.0 and 1.5 %/ms on 4S/6S catch a down-scaled or truncated step (layout 3:
0.5x and 0.67x); 0.5 %/ms on 3S catches one scaled UP (layout 3: 0.6 %/ms);
0.1 %/ms checks the sub-count path. Watch events are change-driven at ~1 ms,
so a chunked step with the right average would also pass; the layout test
only checks the loader published the remainder. Per-tick behaviour (base or
base + 1 counts) is by construction and is checked on hardware with a scope.

test_layout_does_not_rewrite_ramp writes a layout-3 and a layout-4 page and
checks the regime steps the loader derives. The firmware runs the stored ramp
as written on either layout; migrating a layout-3 ramp is the configurator's
job (it reads the page before flashing, since boot stamps layout 4).
'''

from __future__ import annotations

import bisect
import json
import os
import time

import pytest
import sitl_dshot as sd
from sitl_gui_backend import EepromClient
from sitl_harness import SITL_DIR, Sender, rpm_from_state, wait_for_state
from test_ramp_settings_fixed import _open_ctl, _zc_stats
from test_state_protocol import Watch, control_socket

MODELS = os.path.join(SITL_DIR, 'models')

EEPROM_VERSION_OFFSET = 1
EEPROM_MAX_RAMP = 5  # maxRampSpeed offset, 0.1 %/ms per count


LOOP_HZ = 20000  # main.c resets tenkhzcounter once it passes this
SPIN_UP_RAMP = 10  # 1.0 %/ms: the factory ramp, so spin-up is not under test
# Spin up on at most a 4S pack, then switch to the pack under test. Starting
# this model from rest straight onto 6S at 80 % throttle trips the firmware's
# interval-jump desync check about every 40 ms near 2,000 rpm (main before
# the exact ramp did the same). It escapes or latches on the episode rail
# depending on host timing, which made the 6S cases flaky. Only the fall and
# re-punch are under test, and both run on the pack under test.
SPIN_UP_MAX_V = 16.8


def _model(tmp_path, pack_v):
    with open(os.path.join(MODELS, 'unloaded.json')) as f:
        model = json.load(f)
    model['battery'].update(voltage=pack_v, capacitance=1e-3, sink_resistance=0.012)
    path = tmp_path / ('unloaded_%.1fv_regen_sink.json' % pack_v)
    path.write_text(json.dumps(model))
    return str(path)


def _tick_clock(events):
    '''(times, cumulative control ticks) from tenkhzcounter samples.

    Counts the firmware's own ticks across the window instead of scaling a
    median rate: the 1 s reset (main.c) loses at most a few ticks once.'''
    assert len(events) >= 10, 'too few tenkhzcounter samples: %r' % events[:5]
    times, ticks, total = [], [], 0
    for i, (t, v) in enumerate(events):
        if i:
            prev = events[i - 1][1]
            total += v - prev if v >= prev else max(0, LOOP_HZ + 1 - prev) + v
        times.append(t)
        ticks.append(total)
    return times, ticks


def _ticks_between(clock, ta, tb):
    times, ticks = clock

    def at(t):
        i = min(max(bisect.bisect_left(times, t), 1), len(times) - 1)
        t0, t1 = times[i - 1], times[i]
        f = 0.0 if t1 == t0 else (t - t0) / float(t1 - t0)
        return ticks[i - 1] + f * (ticks[i] - ticks[i - 1])

    return at(tb) - at(ta)


@pytest.mark.parametrize('ramp, pack_v', [
    (20, 16.8), (20, 22.2), (15, 16.8), (15, 22.2), (5, 11.1), (1, 22.2)])
def test_ramp_runs_at_setting(sitl_factory, state_stream, tmp_path, ramp, pack_v):
    spin_v = min(pack_v, SPIN_UP_MAX_V)
    spin_model = _model(tmp_path, spin_v)
    test_model = _model(tmp_path, pack_v)

    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    sim.load_model(spin_model)

    client = EepromClient(port=sitl.state_port)
    ok, message = client.set(EEPROM_MAX_RAMP, [SPIN_UP_RAMP])
    assert ok, message

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        with control_socket(sitl) as sock:
            # Spin-up phase: only the firmware's pack reading is watched, on
            # its own socket. The state port holds one watch, so the
            # measurement watch below replaces it, and a late keepalive reply
            # for this one cannot be read as the measurement watch's.
            with control_socket(sitl) as vsock:
                volts = Watch(vsock, [(2, 'battery_voltage')])
                volts.until(lambda: volts.resolved is not None)
                assert volts.resolved == [1], volts.resolved
                time.sleep(2.2)

                tx.value = 1600
                deadline = time.monotonic() + 60  # SITL runs slower than real time
                spun = 8000 * spin_v / 16.8  # free-run rpm scales with the pack
                while rpm_from_state(sim, 0.2) < spun:
                    assert time.monotonic() < deadline, sitl.log_tail()
                    volts.poll()
                if pack_v != spin_v:
                    # The model reload keeps the rotor turning (only the motor
                    # constants and the pack are re-read). Measure once the
                    # firmware's own pack reading has followed it.
                    sim.load_model(test_model)
                    want_cv = 0.97 * pack_v * 100
                    volts.until(lambda: volts.values.get(0, 0) >= want_cv, timeout=20)
                    assert sim.model_status.startswith('loaded'), sim.model_status

            watch = Watch(sock, [(2, 'last_duty_cycle'), (2, 'tenkhzcounter'), (1, 'running')])
            watch.until(lambda: watch.resolved is not None)
            assert watch.resolved[:2] == [1, 1], watch.resolved
            # Switch to the ramp under test only once spinning (SITL reloads
            # settings on an EEPROM write), then let duty settle.
            ok, message = client.set(EEPROM_MAX_RAMP, [ramp])
            assert ok, message
            settle = time.monotonic() + 0.5
            while time.monotonic() < settle:
                watch.poll()

            start = len(watch.events)
            tx.value = 200
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                watch.poll()
            events = watch.events[start:]

            # Re-punch while still spinning: a rise may be held below the
            # setting by the throttle ceilings, but never above it.
            start = len(watch.events)
            tx.value = 1600
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                watch.poll()
            rise_events = watch.events[start:]
    finally:
        tx.stop()

    duty = [(t, v) for t, i, v in events if i == 0]
    clock = _tick_clock([(t, v) for t, i, v in events if i == 1])

    # The falling segment: from the first decrease until duty stops moving.
    falling = []
    for (t0, v0), (t1, v1) in zip(duty, duty[1:]):
        if v1 < v0:
            if not falling:
                falling.append((t0, v0))
            falling.append((t1, v1))
        elif falling:
            break
    assert len(falling) >= 12, 'no ramp-limited fall observed: %r' % duty[:20]
    assert falling[0][1] - falling[-1][1] > 400, 'fall too short to measure: %r' % falling

    # Trim the ends (first/last sample straddle the step and the floor).
    core = falling[2:-2]
    measured = (core[0][1] - core[-1][1]) / _ticks_between(clock, core[0][0], core[-1][0])
    expected = ramp / 10.0
    assert measured <= 1.03 * expected, (
        'duty fell %.3f counts/tick on a %.1f V pack: FASTER than the %.1f %%/ms '
        'setting (%.2f counts/tick)' % (measured, pack_v, ramp / 10.0, expected))
    assert measured >= 0.92 * expected, (
        'duty fell %.3f counts/tick on a %.1f V pack where the %.1f %%/ms setting '
        'is %.2f counts/tick; a truncated or voltage-scaled step runs slow'
        % (measured, pack_v, ramp / 10.0, expected))

    rise = [(t, v) for t, i, v in rise_events if i == 0]
    rising = []
    for (t0, v0), (t1, v1) in zip(rise, rise[1:]):
        if v1 > v0:
            if not rising:
                rising.append((t0, v0))
            rising.append((t1, v1))
        elif rising:
            break
    if len(rising) >= 12 and rising[-1][1] - rising[0][1] > 200:
        core = rising[2:-2]
        rise_clock = _tick_clock([(t, v) for t, i, v in rise_events if i == 1])
        up = (core[-1][1] - core[0][1]) / _ticks_between(rise_clock, core[0][0], core[-1][0])
        assert up <= 1.03 * expected, (
            'duty ROSE %.3f counts/tick on a %.1f V pack: faster than the %.1f %%/ms '
            'setting (%.2f counts/tick)' % (up, pack_v, ramp / 10.0, expected))


@pytest.mark.parametrize('version, ramp, steps, rem_low', [
    (3, 20, (2, 2, 2), 0),    # layout 3: run as written, not migrated here
    (3, 160, (2, 6, 16), 0),
    (4, 25, (2, 2, 2), 5),    # 2.5 %/ms: startup capped at 2, low/high 2 + 0.5 carried
    (4, 15, (1, 1, 1), 5),    # 1.5 %/ms: 1 count/tick + 0.5 carried, every regime
    (4, 5, (0, 0, 0), 5),     # 0.5 %/ms: sub-count, all carried
])
def test_layout_does_not_rewrite_ramp(sitl_factory, version, ramp, steps, rem_low):
    sitl = sitl_factory(can_uri='none')
    client = EepromClient(port=sitl.state_port)
    ok, message = client.set(EEPROM_VERSION_OFFSET, [version])
    assert ok, message
    ok, message = client.set(EEPROM_MAX_RAMP, [ramp])
    assert ok, message
    ctl = _open_ctl(sitl)
    try:
        s = _zc_stats(ctl)
    finally:
        ctl.close()
    image, _model = client.fetch()
    assert image is not None, 'SITL EEPROM fetch timed out'
    assert image[EEPROM_MAX_RAMP] == ramp, (
        'layout %d page: stored max_ramp %d came back as %d; the firmware must not '
        'rewrite it' % (version, ramp, image[EEPROM_MAX_RAMP]))
    got = (int(s['max_ramp_startup']), int(s['max_ramp_low']), int(s['max_ramp_high']))
    assert got == steps, (
        'layout %d page with max_ramp %d loaded as regime steps %r, want %r'
        % (version, ramp, got, steps))
    assert int(s['ramp_rem_low']) == rem_low, (
        'max_ramp %d: low-rpm remainder %d tenths, want %d (whole counts per tick plus '
        'a carried remainder, not a chunk every 10th tick)'
        % (ramp, int(s['ramp_rem_low']), rem_low))
