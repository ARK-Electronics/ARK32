'''The voltage-compensated ramp must slew at the scaled rate, not a truncation of it.

runtimeTransientGovernorTick scales the configured duty step by
GOV_RAMP_VREF_CV / Vbat (14.8 V reference, clamped 0.5x-1.5x). It used to
truncate the result to whole duty counts, so on the factory ramp (2.0 %/ms,
all three regimes clamped to 2 counts per tick) every pack above 14.8 V flew
1 count per tick: half the configured ramp on a full 4S and on all of 6S,
instead of the intended 14.8/Vbat.

This drives the unloaded motor on a 16.8 V pack (scale 225/256) at the
factory ramp, drops the throttle, and compares the applied duty fall with
the drop the compensated ramp allows, integrated over the firmware's own
battery_voltage, last_duty_cycle and tenkhzcounter. A ramp-down is used
because nothing but the slew limiter bounds a falling duty; the throttle
ceilings only cap a rising one. The pack is given a regen sink: the stiff
legacy source in unloaded.json pumps the simulated bus to ~70 V on a fast
throttle drop, and the 1 kHz compensation would follow it.

Mutation check: with the truncating computation the duty falls 1.0
count/tick against an expected ~1.76, well outside the tolerance.
'''

from __future__ import annotations

import json
import os
import statistics
import time

import sitl_dshot as sd
from sitl_gui_backend import EepromClient
from sitl_harness import SITL_DIR, Sender, rpm_from_state, wait_for_state
from test_state_protocol import Watch, control_socket

MODELS = os.path.join(SITL_DIR, 'models')

EEPROM_MAX_RAMP = 5   # maxRampSpeed offset, 0.1 %/ms per count
FACTORY_RAMP = 20     # 2.0 %/ms -> 2 duty counts per tick in every regime
GOV_RAMP_VREF_CV = 1480


def _expected_counts_per_tick(vbat_cv):
    scale_q8 = 256
    if vbat_cv > 600:
        scale_q8 = min(max((GOV_RAMP_VREF_CV << 8) // vbat_cv, 128), 384)
    return 2 * scale_q8 / 256.0


def _ticks_per_ns(events):
    '''Control-loop rate from tenkhzcounter change events (skips its 1 s reset).'''
    rates = []
    for (t0, v0), (t1, v1) in zip(events, events[1:]):
        if v1 > v0 and t1 > t0:
            rates.append((v1 - v0) / float(t1 - t0))
    assert len(rates) >= 10, 'too few tenkhzcounter samples: %r' % events[:5]
    return statistics.median(rates)


def _vbat_at(series, t):
    v = series[0][1]
    for ts, vs in series:
        if ts > t:
            break
        v = vs
    return v


def test_factory_ramp_slews_at_compensated_rate(sitl_factory, state_stream, tmp_path):
    with open(os.path.join(MODELS, 'unloaded.json')) as f:
        model = json.load(f)
    model['battery'].update(capacitance=1e-3, sink_resistance=0.012)
    model_path = tmp_path / 'unloaded_regen_sink.json'
    model_path.write_text(json.dumps(model))

    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    sim.load_model(str(model_path))

    ok, message = EepromClient(port=sitl.state_port).set(EEPROM_MAX_RAMP, [FACTORY_RAMP])
    assert ok, message

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, [(2, 'last_duty_cycle'), (2, 'tenkhzcounter'),
                                 (2, 'battery_voltage'), (1, 'running')])
            watch.until(lambda: watch.resolved is not None)
            assert watch.resolved[:3] == [1, 1, 1], watch.resolved
            time.sleep(2.2)

            tx.value = 1600
            deadline = time.monotonic() + 20
            while rpm_from_state(sim, 0.2) < 8000:
                assert time.monotonic() < deadline, sitl.log_tail()
                watch.poll()
            # Let duty settle at the setpoint before the step.
            settle = time.monotonic() + 0.5
            while time.monotonic() < settle:
                watch.poll()

            start = len(watch.events)
            tx.value = 200
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                watch.poll()
            events = watch.events[start:]
    finally:
        tx.stop()

    duty = [(t, v) for t, i, v in events if i == 0]
    ticks = [(t, v) for t, i, v in events if i == 1]
    vbat = [(t, v) for t, i, v in watch.events if i == 2]
    assert vbat, 'no battery_voltage samples'

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
    tpn = _ticks_per_ns(ticks)
    expected = sum((tb - ta) * tpn * _expected_counts_per_tick(_vbat_at(vbat, ta))
                   for (ta, _), (tb, _) in zip(core, core[1:]))
    measured = core[0][1] - core[-1][1]
    n_ticks = (core[-1][0] - core[0][0]) * tpn
    vbat_cv = _vbat_at(vbat, core[len(core) // 2][0])
    assert abs(measured - expected) <= 0.08 * expected, (
        'duty fell %d counts (%.2f/tick) where the compensated ramp allows %.0f '
        '(%.2f/tick at ~%d cV: 2 counts x %d/%d). A truncated scale gives a '
        'whole number per tick.'
        % (measured, measured / n_ticks, expected, expected / n_ticks, vbat_cv,
           GOV_RAMP_VREF_CV, vbat_cv))
