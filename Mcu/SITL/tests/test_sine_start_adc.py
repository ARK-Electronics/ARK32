'''ADC sampling during sine start (upstream 9696d28).

With sinusoidal startup enabled the ESC sits in the stepper whenever it is
armed below the six-step handoff, idle included. The 1 kHz ADC request used
to live only on the six-step side of the control loop, so for that whole
time battery_voltage, actual_current and degrees_celsius froze at their
last six-step values: voltage cutoff could not trip and telemetry reported
a stale pack.

The plant's supply voltage and ESC temperature are changed by reloading the
model while the firmware is in the stepper; the firmware's readings must
follow. Remove the sine-start PROCESS_ADC_FLAG request in tenKhzRoutine and
both checks fail with the readings stuck at the first model's values.
'''

from __future__ import annotations

import json
import os
import time

import sitl_dshot as sd
from sitl_gui_backend import EepromClient
from sitl_harness import SITL_DIR, Sender, wait_for_state
from test_state_protocol import Watch, control_socket

MODELS = os.path.join(SITL_DIR, 'models')
SINE_STARTUP_OFFSET = 19  # schema/eeprom.json sineStartup
VARS = [(1, 'armed'), (1, 'stepper_sine'), (2, 'battery_voltage'),
        (2, 'degrees_celsius')]
ARMED, STEPPER_SINE, BATTERY_VOLTAGE, DEGREES = range(len(VARS))


def _write_model(dirname, name, volts, celsius):
    with open(os.path.join(MODELS, 'unloaded.json')) as f:
        model = json.load(f)
    model['battery']['voltage'] = volts
    model.setdefault('esc', {})['temperature_c'] = celsius
    path = os.path.join(dirname, name)
    with open(path, 'w') as f:
        json.dump(model, f)
    return path


def _load(sim, path):
    sim.load_model(path)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if (sim.model_status or '').startswith('loaded'):
            return
        time.sleep(0.05)
    raise AssertionError('model load failed: %r' % sim.model_status)


def _assert_follows(watch, volts, celsius, what):
    # battery_voltage is a 1/8 IIR on 1 kHz samples: settled well inside 1 s
    def settled():
        v = watch.values.get(BATTERY_VOLTAGE)
        return (v is not None and abs(v - volts * 100) < 50
                and watch.values.get(DEGREES) == celsius)
    try:
        watch.until(settled, timeout=3)
    except AssertionError:
        raise AssertionError('%s: ADC frozen in sine start, want %.1f V %d C, '
                             'got battery_voltage=%r degrees=%r'
                             % (what, volts, celsius,
                                watch.values.get(BATTERY_VOLTAGE),
                                watch.values.get(DEGREES)))
    assert watch.values.get(STEPPER_SINE) == 1, what


def test_adc_runs_in_sine_start(sitl_factory, state_stream, workdir):
    low = _write_model(workdir, 'sine_low.json', 12.4, 38)
    high = _write_model(workdir, 'sine_high.json', 16.0, 60)
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    _load(sim, low)
    ok, message = EepromClient(port=sitl.state_port).set(SINE_STARTUP_OFFSET, [1])
    assert ok, message

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, VARS)
            watch.until(lambda: watch.values.get(ARMED) == 1, timeout=25)
            watch.until(lambda: watch.values.get(STEPPER_SINE) == 1)
            _assert_follows(watch, 12.4, 38, 'armed idle (baseline)')

            # Armed idle: the stepper holds, nothing is driven.
            _load(sim, high)
            _assert_follows(watch, 16.0, 60, 'armed idle')

            # Stepping below the six-step handoff (input 48..136).
            tx.value = 100
            time.sleep(0.5)
            _load(sim, low)
            _assert_follows(watch, 12.4, 38, 'stepping')
    finally:
        tx.stop()
