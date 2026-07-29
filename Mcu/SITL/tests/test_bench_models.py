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


def _steady_rpm(sim, settle_s: float) -> float:
    time.sleep(settle_s)
    # Prefer a short average once spun; retry if a brief desync dips the window.
    deadline = time.time() + 3.0
    best = 0.0
    while time.time() < deadline:
        rpm = rpm_from_state(sim, 0.35)
        best = max(best, rpm)
        if rpm > 500:
            return rpm
        time.sleep(0.15)
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

    levels = gate['levels']
    tol = float(gate.get('tolerance_pct', exp['steady'].get('tolerance_pct', 15)))
    settle = float(gate.get('settle_s', 1.8))
    min_run = float(gate.get('min_running_rpm', 1000))
    expect_levels = exp['steady']['levels']

    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
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
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    sim.load_model(model_path)
    _wait_model(sim, model.split('.')[0][:8])

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        time.sleep(2.2)
        tx.value = _dshot_for_throttle(0.20)
        rpm = _steady_rpm(sim, 2.0)
        assert rpm >= float(gate.get('min_running_rpm', 1000)), (
            'did not spin on %s: rpm=%.0f\n%s' % (model, rpm, sitl.log_tail()))
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
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()

    def spin(model_path: str) -> float:
        sim.load_model(model_path)
        _wait_model(sim, 'ark_900')
        tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
        try:
            time.sleep(2.2)
            tx.value = _dshot_for_throttle(0.30)
            return _steady_rpm(sim, 2.2)
        finally:
            tx.value = 0
            time.sleep(0.5)
            tx.stop()

    rpm_noprop = spin(noprop)
    rpm_prop = spin(prop)
    # Prop load must pull RPM down vs noprop at the same stick (bench ~6419 vs ~6901;
    # SITL first-order only needs a clear ordering / separation).
    assert rpm_noprop > 5000 and rpm_prop > 3000, (
        'unexpected spin: noprop=%.0f prop=%.0f\n%s'
        % (rpm_noprop, rpm_prop, sitl.log_tail()))
    assert rpm_prop < rpm_noprop * 0.98, (
        '10inch model is not heavier than noprop at 30%%: '
        'prop=%.0f noprop=%.0f\n%s'
        % (rpm_prop, rpm_noprop, sitl.log_tail()))
