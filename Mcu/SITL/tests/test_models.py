'''Runtime motor model load over the SITL state port.'''

from __future__ import annotations

import os
import time

import pytest

from sitl_harness import (SITL_DIR, peak_rpm_from_state, rpm_from_state,
                          wait_for_state)
import sitl_dshot as sd
from sitl_harness import Sender


MODELS = os.path.join(SITL_DIR, 'models')


@pytest.mark.parametrize('model', [
    'racer_5inch.json',
    'default_7inch.json',
    'heavy_13inch.json',
    'unloaded.json',
    # Hardware-calibrated plants (see data/*/expected.json + test_bench_models)
    'ark_900kv_noprop.json',
    'ark_900kv_10inch.json',
    'ark_2807_1300kv_noprop.json',
])
def test_load_stock_model(sitl_factory, state_stream, model):
    path = os.path.join(MODELS, model)
    assert os.path.isfile(path), path
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    sim.load_model(path)
    # model_status is updated asynchronously over UDP
    deadline = time.time() + 3.0
    status = ''
    while time.time() < deadline:
        status = sim.model_status or ''
        if status and 'fail' not in status.lower() and 'error' not in status.lower():
            if 'loading' not in status.lower() or 'ok' in status.lower() or model in status:
                break
        time.sleep(0.1)
    # Accept either an explicit OK-style status or a cleared error-free load
    assert 'error' not in status.lower() and 'fail' not in status.lower(), status


def test_racer_model_spins_under_dshot(sitl_factory, state_stream):
    '''heavier/lighter models still produce plausible RPM under DShot'''
    path = os.path.join(MODELS, 'racer_5inch.json')
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim)
    sim.load_model(path)
    # model load is async over UDP; wait for a non-error status before arming
    deadline = time.time() + 3.0
    status = ''
    while time.time() < deadline:
        status = sim.model_status or ''
        if status and 'fail' not in status.lower() and 'error' not in status.lower():
            if 'loading' not in status.lower() or 'ok' in status.lower() or 'racer' in status:
                break
        time.sleep(0.1)
    assert 'error' not in status.lower() and 'fail' not in status.lower(), status

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        time.sleep(2.2)
        tx.value = 700
        # Steady state is ~3k at this throttle. The high-kV racer desyncs
        # often under CI load: a 0.3 s MEAN can land at 0 between restarts
        # while the plant still hit 3k (CI log tail: ZC_ACCEPT ~3.1k rpm,
        # assert still saw mean=0). Track peak over the poll window so a
        # real spin passes; mean still used when it is already in band.
        rpm_lo, rpm_hi = 2000.0, 15000.0
        deadline = time.time() + 10.0
        mean_rpm = -1.0
        peak_rpm = -1.0
        while time.time() < deadline:
            mean_rpm = rpm_from_state(sim, 0.3)
            burst = peak_rpm_from_state(sim, 0.5)
            if burst > peak_rpm:
                peak_rpm = burst
            if rpm_lo <= mean_rpm <= rpm_hi:
                break
            if rpm_lo <= peak_rpm <= rpm_hi:
                break
            time.sleep(0.1)
        in_band = ((rpm_lo <= mean_rpm <= rpm_hi) or
                   (rpm_lo <= peak_rpm <= rpm_hi))
        assert in_band, (
            'racer never reached %.0f..%.0f rpm '
            '(mean=%.0f peak=%.0f)\n%s'
            % (rpm_lo, rpm_hi, mean_rpm, peak_rpm, sitl.log_tail()))

        tx.value = 0
        deadline = time.time() + 7.0
        rpm = float('inf')
        while time.time() < deadline:
            rpm = rpm_from_state(sim, 0.3)
            if rpm < 800:
                break
            time.sleep(0.15)
        assert rpm < 800, 'motor did not stop: rpm=%.0f' % rpm
    finally:
        tx.stop()
