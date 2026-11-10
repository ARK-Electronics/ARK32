'''A tripped low-voltage cutoff stays latched while the input keeps coming.

The cutoff is meant to hold until a power cycle (esc_state.c: "FAULT_LVC:
latched until power cycle"). With the absolute cutoff it did not: tripping
cleared `armed`, and because cell_count stays 0 in absolute mode, the 20 kHz
arming block ran again. Its one-second timer had already expired and the trip
had zeroed zero_input_count, so it took the "arming failed" branch and
cleared inputSet. Frames stopped being decoded, the signal-loss watchdog reset
the MCU 2 s later, and the reset wiped the latch: the ESC re-armed and was
drivable again for ~10 s, every ~13 s (ARK 4IN1 bench, 2026-11-10).

Sine startup is on so the cutoff trips in ~100 ms at armed idle instead of
after the 10 s six-step count; the latch path after the trip is the same.
The model's 12.4 V pack is below a 15.0 V absolute cutoff. After the trip the
input keeps arriving, first at zero and then at a throttle that would start
the motor; the ESC must stay latched, keep decoding frames, and never drive.
Without the !LOW_VOLTAGE_CUTOFF guard on the arming block this fails within
about 2 s: inputSet drops to 0 and the firmware resets.
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
ABSOLUTE_VOLTAGE_CUTOFF_OFFSET = 8   # schema/eeprom.json absoluteVoltageCutoff, 0.5 V steps
SINE_STARTUP_OFFSET = 19             # sineStartup
LOW_VOLTAGE_CUTOFF_OFFSET = 36       # lowVoltageCutoff: 2 = absolute
ESC_FAULT_LVC = 9                    # Inc/esc_state.h
VARS = [(1, 'esc_state'), (1, 'armed'), (1, 'inputSet'), (1, 'LOW_VOLTAGE_CUTOFF'),
        (1, 'running'), (1, 'stepper_sine'), (2, 'signaltimeout')]
STATE, ARMED, INPUT_SET, LVC, RUNNING, STEPPER, SIGNALTIMEOUT = range(len(VARS))


def _low_pack_model(dirname):
    with open(os.path.join(MODELS, 'unloaded.json')) as f:
        model = json.load(f)
    model['battery']['voltage'] = 12.4
    path = os.path.join(dirname, 'lvc_low.json')
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


def _hold_latched(watch, seconds, what, sitl):
    end = time.time() + seconds
    while time.time() < end:
        watch.poll()
        v = watch.values
        assert v.get(LVC) == 1 and v.get(STATE) == ESC_FAULT_LVC, (
            '%s: cutoff no longer latched (esc_state=%r LOW_VOLTAGE_CUTOFF=%r)\n%s'
            % (what, v.get(STATE), v.get(LVC), sitl.log_tail()))
        assert v.get(ARMED) == 0, '%s: re-armed while latched' % what
        assert v.get(RUNNING) == 0 and v.get(STEPPER) == 0, '%s: drove the motor while latched' % what
        assert v.get(INPUT_SET) == 1, (
            '%s: inputSet cleared while latched, frames stop decoding and the '
            'signal-loss watchdog resets the MCU' % what)
        assert (v.get(SIGNALTIMEOUT) or 0) < 2000, (
            '%s: signaltimeout %r, frames are not being decoded' % (what, v.get(SIGNALTIMEOUT)))


def test_absolute_cutoff_stays_latched(sitl_factory, state_stream, workdir):
    low = _low_pack_model(workdir)
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim), sitl.log_tail()
    _load(sim, low)
    ee = EepromClient(port=sitl.state_port)
    for offset, value in ((ABSOLUTE_VOLTAGE_CUTOFF_OFFSET, 30), (LOW_VOLTAGE_CUTOFF_OFFSET, 2),
                          (SINE_STARTUP_OFFSET, 1)):
        ok, message = ee.set(offset, [value])
        assert ok, message

    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT600)
    try:
        with control_socket(sitl) as sock:
            watch = Watch(sock, VARS)
            watch.until(lambda: watch.values.get(LVC) == 1, timeout=25)
            watch.until(lambda: watch.values.get(STATE) == ESC_FAULT_LVC, timeout=2)
            # Zero throttle: the re-arm and signal-loss reset used to follow
            # within ~2 s.
            _hold_latched(watch, 4.0, 'zero throttle after the trip', sitl)
            # A throttle that would start the motor must not.
            tx.value = 400
            _hold_latched(watch, 2.0, 'throttle after the trip', sitl)
            tx.value = 0
            _hold_latched(watch, 1.0, 'back at zero', sitl)
    finally:
        tx.stop()
