'''Runtime motor model load over the SITL state port.'''

from __future__ import annotations

import os
import time

import pytest

from sitl_harness import SITL_DIR, rpm_from_state, wait_for_state
import sitl_dshot as sd
from sitl_harness import Sender


MODELS = os.path.join(SITL_DIR, 'models')


@pytest.mark.parametrize('model', [
    'racer_5inch.json',
    'default_7inch.json',
    'heavy_13inch.json',
    'unloaded.json',
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
        # Poll for a short-window RPM in band. Steady state is ~3k at this
        # throttle, but the high-kV racer can desync briefly and pull a fixed
        # 1 s average under the floor (CI saw ~1850). Same pattern as the
        # coast-down deadline in test_dshot.py.
        rpm_lo, rpm_hi = 2000.0, 15000.0
        deadline = time.time() + 8.0
        rpm = -1.0
        while time.time() < deadline:
            rpm = rpm_from_state(sim, 0.3)
            if rpm_lo <= rpm <= rpm_hi:
                break
            time.sleep(0.15)
        assert rpm_lo <= rpm <= rpm_hi, (
            'rpm=%.0f out of range on racer model (expected %.0f..%.0f)\n%s'
            % (rpm, rpm_lo, rpm_hi, sitl.log_tail()))

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
