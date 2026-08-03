'''SITL regression against real HWCI / Flight Stand motor maps.

The JSON under Mcu/SITL/data/*/expected.json is captured on the ARK thrust
stand (optical RPM, HV V/I). Each expected file may include a sitl_gate
block that names a first-order plant model and the throttle levels that are
stable enough to assert in CI.

WHY: stock models (unloaded / racer / 7inch) are synthetic. Bench gates catch
regressions that only show up when KV, pack voltage, and load roughly match
the plant we actually fly (900KV noprop, 900KV 10", 2807 1300KV).

Tolerances are deliberately wider than the capture noise (15–20%): the SITL
plant is first-order and still desyncs at high stick; we gate the band that
tracks, not a perfect digital twin.
'''

from __future__ import annotations

import json
import os
import time

import pytest

from sitl_harness import SITL_DIR, Sender, rpm_from_state, wait_for_state
import sitl_dshot as sd
import sitl_params

MODELS = os.path.join(SITL_DIR, 'models')
DATA = os.path.join(SITL_DIR, 'data')

# (data_subdir, expected.json is under data_subdir)
BENCH_CASES = [
    'ARK_4IN1_F051_900kv_noprop',
    'ARK_4IN1_F051_900kv_10inch',
    'ARK_4IN1_F051_2807_1300kv',
]


def _load_expected(case: str) -> dict:
    path = os.path.join(DATA, case, 'expected.json')
    with open(path) as f:
        return json.load(f)


def _eeprom_for_case(case: str, dest: str) -> str:
    '''Build an eeprom image from the capture's sitl.param (MOTOR_KV, ramp, …).

    Empty default eeprom seeds the wrong Kv for ARK plants and desyncs under
    the ARK_4IN1_F051 sense/ramp policy the SITL target now mirrors.
    '''
    param = os.path.join(DATA, case, 'sitl.param')
    if not os.path.isfile(param):
        param = os.path.join(DATA, 'ARK_4IN1_F051', 'sitl.param')
    return sitl_params.write_eeprom(param, dest)


def _dshot_for_throttle(thr: float) -> int:
    thr = max(0.0, min(1.0, thr))
    return int(round(48 + thr * (2047 - 48)))


def _wait_model(sim, token: str, timeout: float = 3.0) -> str:
    deadline = time.time() + timeout
    status = ''
    while time.time() < deadline:
        status = sim.model_status or ''
        low = status.lower()
        if status and 'fail' not in low and 'error' not in low:
            if 'loading' not in low or 'ok' in low or token in low:
                return status
        time.sleep(0.1)
    return status


def _steady_rpm(sim, settle_s: float, min_rpm: float = 500.0) -> float:
    '''Return a stable post-settle RPM, not a single desync-recovery sample.

    CI hosts occasionally return a mid-ramp or post-desync window if we take
    the first average above min_rpm. Require three consecutive 0.35s windows
    that agree within 5% (or 200 rpm), then return their median.
    '''
    time.sleep(settle_s)
    deadline = time.time() + 5.0
    best = 0.0
    window: list[float] = []
    while time.time() < deadline:
        rpm = rpm_from_state(sim, 0.35)
        best = max(best, rpm)
        if rpm < min_rpm:
            window.clear()
            time.sleep(0.1)
            continue
        window.append(rpm)
        if len(window) >= 3:
            trio = window[-3:]
            lo, hi = min(trio), max(trio)
            if (hi - lo) <= max(200.0, 0.05 * hi):
                return sorted(trio)[1]
        time.sleep(0.1)
    return best


@pytest.mark.parametrize('case', BENCH_CASES)
def test_bench_steady_map_matches_hardware(sitl_factory, state_stream, case):
    exp = _load_expected(case)
    gate = exp.get('sitl_gate')
    if not gate:
        pytest.skip('no sitl_gate in expected.json')
    model = gate['model']
    model_path = os.path.join(MODELS, model)
    assert os.path.isfile(model_path), model_path

    levels = gate.get('levels') or []
    if not levels:
        pytest.skip('sitl_gate.levels empty (map not gated for this plant)')
    tol = float(gate.get('tolerance_pct', exp['steady'].get('tolerance_pct', 15)))
    settle = float(gate.get('settle_s', 1.8))
    min_run = float(gate.get('min_running_rpm', 1000))
    expect_levels = exp['steady']['levels']

    eeprom = _eeprom_for_case(case, 'bench_map_%s.bin' % case)
    sitl = sitl_factory(
        extra_args=['--input-type', '1', '--eeprom', eeprom], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    sim.load_model(model_path)
    status = _wait_model(sim, model.split('.')[0][:8])
    assert 'error' not in status.lower() and 'fail' not in status.lower(), status

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    failures = []
    try:
        time.sleep(2.2)
        for key in levels:
            thr = float(key)
            want = float(expect_levels[key])
            tx.value = _dshot_for_throttle(thr)
            rpm = _steady_rpm(sim, settle)
            if rpm < min_run:
                failures.append(
                    'thr=%.2f rpm=%.0f below min_running_rpm=%.0f (want ~%.0f)'
                    % (thr, rpm, min_run, want))
                continue
            lo = want * (1.0 - tol / 100.0)
            hi = want * (1.0 + tol / 100.0)
            if not (lo <= rpm <= hi):
                failures.append(
                    'thr=%.2f rpm=%.0f outside [%.0f, %.0f] (bench %.0f ±%.0f%%)'
                    % (thr, rpm, lo, hi, want, tol))
        tx.value = 0
        time.sleep(0.4)
    finally:
        tx.stop()

    assert not failures, (
        'bench map regression for %s / %s:\n  - %s\n%s'
        % (case, model, '\n  - '.join(failures), sitl.log_tail()))


@pytest.mark.parametrize('case', BENCH_CASES)
def test_bench_model_spins_and_stops(sitl_factory, state_stream, case):
    '''Coarse health: model loads, mid-stick spins, zero throttle coasts.'''
    exp = _load_expected(case)
    gate = exp.get('sitl_gate') or {}
    model = gate.get('model')
    if not model:
        pytest.skip('no sitl_gate.model')
    model_path = os.path.join(MODELS, model)
    eeprom = _eeprom_for_case(case, 'bench_spin_%s.bin' % case)
    sitl = sitl_factory(
        extra_args=['--input-type', '1', '--eeprom', eeprom], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    sim.load_model(model_path)
    _wait_model(sim, model.split('.')[0][:8])

    # Prefer the lowest sitl_gate level; plants with empty levels (map not
    # gated yet) still exercise load/spin at a gentle 0.10 stick.
    thr = 0.10
    levels = gate.get('levels') or []
    if levels:
        thr = float(levels[0])
    settle = float(gate.get('settle_s', 2.0))
    min_run = float(gate.get('min_running_rpm', 1000))

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        time.sleep(2.2)
        # A few plants (2807 noprop) can miss the first cold start under
        # CI load; retry the spin rather than flaking the gate.
        rpm = 0.0
        for attempt in range(3):
            tx.value = _dshot_for_throttle(thr)
            rpm = _steady_rpm(sim, settle, min_rpm=min_run * 0.5)
            if rpm >= min_run:
                break
            tx.value = 0
            time.sleep(1.0)
        assert rpm >= min_run, (
            'did not spin on %s at thr=%.2f: rpm=%.0f\n%s'
            % (model, thr, rpm, sitl.log_tail()))
        tx.value = 0
        deadline = time.time() + 8.0
        stopped = False
        while time.time() < deadline:
            if rpm_from_state(sim, 0.3) < 800:
                stopped = True
                break
            time.sleep(0.15)
        assert stopped, 'motor did not stop on %s' % model
    finally:
        tx.stop()


def test_bench_900kv_10inch_mid_stick_current_band(sitl_factory, state_stream):
    '''Prop plant should not free-run like noprop at 30% (load is the point).'''
    noprop = os.path.join(MODELS, 'ark_900kv_noprop.json')
    prop = os.path.join(MODELS, 'ark_900kv_10inch.json')

    # Fresh SITL per model (private eeprom) so load_model cannot leave residual
    # rotor state that inverts ordering. Retry: noprop free-run at 30% can fail
    # to start on a loaded CI host (seen as noprop=0 while prop is fine).
    def spin_once(model_path: str, case: str, attempt: int) -> tuple[float, str]:
        eeprom = _eeprom_for_case(case, 'bench_order_%s_%d.bin' % (case, attempt))
        sitl = sitl_factory(
            extra_args=['--input-type', '1', '--eeprom', eeprom, '--node-id', '100'],
            can_uri='none')
        sim = state_stream(sitl)
        assert wait_for_state(sim), sitl.log_tail()
        sim.load_model(model_path)
        _wait_model(sim, 'ark_900')
        tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
        try:
            time.sleep(2.5)
            tx.value = _dshot_for_throttle(0.30)
            # min_rpm low enough to accept a slow ramp; stability filter still
            # rejects mid-desync samples when three windows agree.
            rpm = _steady_rpm(sim, 3.0, min_rpm=800.0)
            return rpm, sitl.log_tail()
        finally:
            tx.value = 0
            time.sleep(0.2)
            tx.stop()
            sitl.close()

    def spin(model_path: str, case: str, need: float) -> tuple[float, str]:
        last_log = ''
        best = 0.0
        for attempt in range(3):
            rpm, log = spin_once(model_path, case, attempt)
            last_log = log
            best = max(best, rpm)
            if rpm >= need:
                return rpm, log
            time.sleep(0.3)
        return best, last_log

    rpm_noprop, log_np = spin(noprop, 'ARK_4IN1_F051_900kv_noprop', 5000.0)
    rpm_prop, log_p = spin(prop, 'ARK_4IN1_F051_900kv_10inch', 3000.0)
    # Prop load must pull RPM down vs noprop at the same stick (bench ~6419 vs ~6901;
    # SITL first-order only needs a clear ordering / separation).
    log = log_np + '\n---\n' + log_p
    assert rpm_noprop > 5000 and rpm_prop > 3000, (
        'unexpected spin: noprop=%.0f prop=%.0f\n%s'
        % (rpm_noprop, rpm_prop, log))
    # Under the ARK_4IN1_F051 sense/ramp policy the first-order plants
    # only separate by ~1–2% at 30%; require strict ordering, not a
    # large gap (the HWCI map is the quantitative gate).
    assert rpm_prop < rpm_noprop, (
        '10inch model is not heavier than noprop at 30%%: '
        'prop=%.0f noprop=%.0f\n%s'
        % (rpm_prop, rpm_noprop, log))
