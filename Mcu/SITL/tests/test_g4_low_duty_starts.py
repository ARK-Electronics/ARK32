'''ARK 12S CAN (G431) low-duty starts on the G4 SITL target.

The 12S board's comparator reads the crossing on time only in the settled
PWM on-window. In the off-window it keeps the pre-crossing level ~28 deg
past a rising crossing and ~9 deg past a falling one. The turn-off transient
reads HIGH. With the G4 confirm loop (up to 48 reads) outlasting a short
low-duty on-window, rising-BEMF crossings are accepted ~0.47 CI late. Bench
2026-11-12 (360 kV + 13x10x3 on 6S, PX4-equivalent starts: slew to input
346, hold, ramp down to the PX4 idle level 156, park, stop):

  #79 e8353dc8                  24 restarts with throttle held per 20 starts,
                                runaways (interval collapse, no thrust, no
                                fault, survives zero throttle)
  #79 with #137 reverted        0 restarts, 0 dead starts
  #79 October head 6c0f22ef     0 restarts, 0 dead starts

ARK_G4_12S_SITL_CAN runs the G431 zero-crossing front end and the G431
branches of the shared motor code (ZC_FRONTEND_G431, Inc/zc_frontend.h). The ark_12s_360kv_13x10_6s model sets the
sim.comparator_offwin_* front end that reproduces the board's window
dependence. This test drives the same start sequence and fails on any
restart with throttle held, any runaway (firmware interval far shorter than
the rotor's), or a start that does not reach speed.

Build: make ARK_G4_12S_SITL_CAN. Run: G4_SITL=1 pytest test_g4_low_duty_starts.py
(the test finds obj/ARK32_ARK_G4_12S_SITL_CAN_*.elf itself), or
python3 test_g4_low_duty_starts.py ELF [starts] for a per-start table.

STATUS 2026-11-12: calibration pending, so the test is opt-in (G4_SITL=1).
- Reproduced: the rising/falling asymmetry at idle (1.32/0.68; bench
  1.51/0.53) and through the ramps (1.29/0.56).
- Not reproduced: the asymmetry at the 0.10 hold (0.98/1.01; bench
  1.45/0.55), and so not the restarts or the runaway.
- Model artifacts: the first start after the settings write never spins
  up, and some starts run backwards.
The confirm-read time, the lag angles and the on-window settle need the
bench's calibration data (confirm-loop duration, maps at several duties)
before this can gate anything. It must fail #79 e8353dc8 and pass with
#137 reverted, with no retuning between the two.
'''

from __future__ import annotations

import glob
import json
import math
import os
import socket
import struct
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SITL_DIR = os.path.normpath(os.path.join(HERE, '..'))
REPO = os.path.normpath(os.path.join(SITL_DIR, '..', '..'))
sys.path.insert(0, SITL_DIR)
sys.path.insert(0, os.path.join(REPO, 'scripts'))

import sitl_dshot as sd  # noqa: E402
from sitl_gui_backend import EepromClient  # noqa: E402
from sitl_harness import Sender, Sitl, open_state, wait_for_state  # noqa: E402
from sitl_state_protocol import STATE_MAGIC_CMD  # noqa: E402

MODEL = os.path.join(SITL_DIR, 'models', 'ark_12s_360kv_13x10_6s.json')
POLE_PAIRS = 7
IDLE, HOLD = 156, 346          # PX4 DShot at idle and at actuator 0.10
STARTS = int(os.environ.get('G4_STARTS', '6'))

VARS = [(1, 'running'), (1, 'old_routine'), (4, 'zero_crosses'), (4, 'commutation_interval'),
        (1, 'zc_blind_steps'), (4, 'desync_happened'), (2, 'input'), (1, 'armed')]
RUNNING, OLD, ZC, CI, BLIND, DESYNC, INPUT, ARMED = range(len(VARS))
WATCH_EVENT = struct.Struct('<QIQ')  # as test_state_protocol.WATCH_EVENT: t, index, value


def g4_binary():
    hits = glob.glob(os.path.join(REPO, 'obj', 'ARK32_ARK_G4_12S_SITL_CAN_*.elf'))
    return max(hits, key=os.path.getmtime) if hits else None


def bench_page():
    '''The bench page: ARK_G431_CAN factory defaults, 340 kV / 14 poles, max_ramp 10.'''
    from build_factory_image import build_eeprom_page
    d = json.load(open(os.path.join(REPO, 'factory', 'ARK_G431_CAN_eeprom_defaults.json')))
    d['settings'].update(motor_kv=340, motor_poles=14)
    page = bytearray(build_eeprom_page(d, 32, 0, 4)[:192])
    page[5] = 10
    return page


class Watch:
    '''every change of VARS (state command 8, zero minimum period), host-timestamped'''

    def __init__(self, state_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.connect(('127.0.0.1', state_port))
        self.sock.settimeout(0.005)
        self.req = struct.pack('<HBBI', STATE_MAGIC_CMD, 8, len(VARS), 0)
        for size, name in VARS:
            self.req += bytes([size]) + name.encode('ascii') + b'\0'
        self.values, self.events, self.next_refresh = {}, [], 0.0

    def poll(self):
        if time.monotonic() >= self.next_refresh:
            self.sock.send(self.req)
            self.next_refresh = time.monotonic() + 0.4
        try:
            data = self.sock.recv(65536)
        except socket.timeout:
            return
        if len(data) < 4:
            return
        magic, _v, _n = struct.unpack_from('<HBB', data)
        if magic != 0x535a or (len(data) - 4) % WATCH_EVENT.size:
            return  # not an event batch, or truncated
        now = time.monotonic()
        for off in range(4, len(data), WATCH_EVENT.size):
            ev = WATCH_EVENT.unpack_from(data, off)
            self.values[ev[1]] = ev[2]
            self.events.append((now, ev[1], ev[2]))

    def close(self):
        self.sock.close()


def run_starts(elf, starts=STARTS, model=MODEL, workdir=None, log=None, sim_overrides=None):
    '''Drive the bench start sequence; return per-start records.'''
    if sim_overrides:
        cfg = json.load(open(model))
        cfg['sim'].update(sim_overrides)
        model = os.path.join(workdir, 'model.json')
        json.dump(cfg, open(model, 'w'))
    sitl = Sitl(elf, extra_args=['--config', model, '--input-type', '1'] + (['--verbose'] if os.environ.get('G4_VERBOSE') else []), workdir=workdir, can_uri='none')
    sim = open_state('127.0.0.1', sitl.state_port, period_us=500)
    tx = Sender('127.0.0.1', sitl.input_port, sd.TYPE_DSHOT300)
    w = Watch(sitl.state_port)
    out = []
    try:
        assert wait_for_state(sim), sitl.log_tail()
        page = bench_page()
        editor = EepromClient(port=sitl.state_port)
        for off in range(5, 192, 16):
            ok, msg = editor.set(off, list(page[off:min(off + 16, 192)]))
            assert ok, msg
        tx.value = 0

        def hold(value, seconds, phase, rec):
            end = time.monotonic() + seconds
            tx.value = value
            while time.monotonic() < end:
                w.poll()
                s = sim.samples[-1] if sim.samples else None
                if s is not None:
                    rec['true_rpm'].append((time.monotonic(), phase, value, s[1] * 60.0 / (2 * math.pi)))
                    rec['fw'].append((time.monotonic(), phase, value, dict(w.values)))

        def ramp(a, b, seconds, phase, rec, steps=25):
            for i in range(1, steps + 1):
                hold(int(round(a + (b - a) * i / steps)), seconds / steps, phase, rec)

        # settings writes reload the page; arm on zero throttle afterwards
        arm = {'true_rpm': [], 'fw': []}
        deadline = time.monotonic() + 10
        while w.values.get(ARMED) != 1:
            assert time.monotonic() < deadline, 'never armed\n' + sitl.log_tail()
            hold(0, 0.2, 'arm', arm)
        hold(0, 1.0, 'arm', arm)
        for k in range(starts):
            rec = {'k': k, 'true_rpm': [], 'fw': [], 'ev0': len(w.events)}
            hold(IDLE, 0.05, 'min', rec)
            ramp(IDLE, HOLD, 1.0, 'slew', rec)
            hold(HOLD, 2.0, 'hold', rec)
            ramp(HOLD, IDLE, 0.5, 'rampdn', rec)
            hold(IDLE, 1.0, 'park', rec)
            hold(0, 2.5, 'stop', rec)
            rec['ev1'] = len(w.events)
            out.append(rec)
        return out, w.events, sitl.log_tail(60)
    finally:
        tx.stop()
        w.close()
        sim.close()
        sitl.close()


def judge(records, events):
    '''Per start: restarts with throttle held (running dropped, or an established
    closed loop fell back to poll), runaway samples, hold speed.'''
    res = []
    for r in records:
        ev = events[r['ev0']:r['ev1']]
        restarts, fallbacks = 0, 0
        last_in, last_run, last_old, zc = None, None, None, 0
        for t, idx, val in ev:
            if idx == INPUT:
                last_in = val
            elif idx == ZC:
                zc = val
            elif idx == RUNNING:
                if last_run == 1 and val == 0 and (last_in or 0) > 0:
                    restarts += 1
                last_run = val
            elif idx == OLD:
                # established closed loop dropping back to poll with throttle held
                # (escNoteStallOrDesync(1) keeps running == 1)
                if last_old == 0 and val == 1 and last_run == 1 and (last_in or 0) > 0 and zc > 100:
                    fallbacks += 1
                last_old = val
        # runaway: firmware CI (0.5 us ticks) under half the rotor's true interval while driving
        runaway = 0
        for (t, ph, val, rpm), (_t2, _p2, _v2, fw) in zip(r['true_rpm'], r['fw']):
            ci = fw.get(CI)
            if val > 0 and fw.get(RUNNING) == 1 and fw.get(OLD) == 0 and ci and abs(rpm) > 100:
                true_ci = 1.0 / (6.0 * abs(rpm) * POLE_PAIRS / 60.0) / 0.5e-6
                if ci < 0.5 * true_ci:
                    runaway += 1
        hold_rpm = [rpm for (t, ph, val, rpm) in r['true_rpm'] if ph == 'hold']
        tail = hold_rpm[len(hold_rpm) // 2:] or [0]
        blind = sum(1 for (t, idx, val) in ev if idx == BLIND and val > 0)
        res.append(dict(k=r['k'], restarts=restarts, fallbacks=fallbacks, runaway=runaway,
                        hold_rpm=sum(tail) / len(tail), blind_events=blind))
    return res


@pytest.mark.skipif(not os.environ.get('G4_SITL'), reason='G4 front end not yet calibrated to the bench; opt in with G4_SITL=1')
@pytest.mark.skipif(g4_binary() is None, reason='build the G4 SITL target: make ARK_G4_12S_SITL_CAN')
def test_px4_idle_starts_keep_sync_on_the_12s_g4_front_end(workdir):
    records, events, log_tail = run_starts(g4_binary(), workdir=workdir)
    res = judge(records, events)
    # Model artifacts (docstring): the first start after the settings write
    # never spins up, and some starts run backwards. Neither is the firmware
    # behaviour under test, so judge the forward-running starts after the first.
    judged = [r for r in res if r['k'] > 0 and r['hold_rpm'] > 0]
    assert len(judged) >= max(1, (len(res) - 1) // 2), ('too few forward starts to judge: %r\n%s' % (res, log_tail))
    speeds = sorted(r['hold_rpm'] for r in judged)
    med = speeds[len(speeds) // 2]
    bad = [r for r in judged if r['restarts'] or r['fallbacks'] or r['runaway'] > 5 or r['hold_rpm'] < 0.6 * med or med < 500]
    assert not bad, ('12S/G4 low-duty starts lost sync: %r\nall: %r\n%s' % (bad, res, log_tail))

if __name__ == '__main__':
    elf = sys.argv[1] if len(sys.argv) > 1 else g4_binary()
    n = int(sys.argv[2]) if len(sys.argv) > 2 else STARTS
    import tempfile
    ov = json.loads(os.environ['G4_SIM']) if os.environ.get('G4_SIM') else None
    records, events, log_tail = run_starts(elf, starts=n, workdir=tempfile.mkdtemp(prefix='g4sitl_'), sim_overrides=ov)
    for r in judge(records, events):
        print(r)
    if os.environ.get('G4_VERBOSE'):
        print(log_tail)
