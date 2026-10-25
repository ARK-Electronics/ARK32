#!/usr/bin/env python3
"""Run pinned ESCSim behavioral tests against ARK32, with ARK expectations.

Clone am32-firmware/ESCSim at escsim-revision.txt, then pass --escsim PATH.
The upstream runner and its clients are loaded from that checkout; ARK's
startup tune, input watchdog, stuck-rotor rails and schema-backed defaults
are adapted here.
"""

import argparse
import glob
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ARK_ROOT = HERE.parent.parent


def test_startup_tune(suite, sitl_path):
    '''The ARK startup signature must arrive on the tone event stream.'''
    # ARK's .- .-. -.- signature (Src/sounds.c), not upstream's three beeps.
    expected = [(1047, 0.05), (1047, 0.15), (1319, 0.05), (1319, 0.15),
                (1319, 0.05), (1568, 0.15), (1568, 0.05), (1568, 0.15)]
    tones = suite.ToneStream('127.0.0.1', suite.STATE_PORT)
    try:
        with suite.Sitl(sitl_path, ['--can-uri', 'none']):
            found, notes = suite.wait_for_notes(tones, expected)
            suite.check('startup tune', found is not None,
                  'notes=%s' % ['%.1fHz %.3fs' % n[:2] for n in notes])
    finally:
        tones.close()


def test_physics_audio(suite, sitl_path):
    '''the physics audio stream must carry the boot tune: some 21ms
    sim-time window has the first note frequency (1047Hz) dominant'''
    audio = suite.AudioStream('127.0.0.1', suite.STATE_PORT)
    try:
        with suite.Sitl(sitl_path, ['--can-uri', 'none']):
            deadline = time.time() + 12
            batches = []
            ok = False
            while time.time() < deadline and not ok:
                time.sleep(0.5)
                batches += audio.take_batches()
                runs = [[]]
                expected_t = None
                for t0, vals in batches:
                    # Never concatenate samples across dropped UDP batches:
                    # that changes their phase and creates false frequencies.
                    # The scheduler rounds each sample to its next tick;
                    # allow that drift, but split at half a missing batch.
                    if expected_t is not None and abs(t0 - expected_t) > len(vals) * 20833 // 2:
                        runs.append([])
                    runs[-1].extend(vals)
                    expected_t = t0 + len(vals) * 20833
                for run in runs:
                    # ARK dots last only 50 ms. Short overlapping windows also
                    # avoid mixing a dot with silence at fixed bucket edges.
                    for start in range(0, len(run) - 1023, 512):
                        vals = run[start:start + 1024]
                        g1 = suite.sitl_tones.goertzel(vals, 1047.0)
                        g2 = suite.sitl_tones.goertzel(vals, 700.0)
                        if g1 > 1e-3 and g1 > 5 * g2:
                            ok = True
                            break
                    if ok:
                        break
            suite.check('physics audio boot tune', ok,
                  '%d batches captured' % len(batches))
    finally:
        audio.close()


def can_state_stream(suite, name):
    """Require a live CAN-enabled simulator, with logs if multicast fails."""
    sim = suite.SimStream('127.0.0.1', suite.STATE_PORT, period_us=200)
    sim.enabled = True
    deadline = time.time() + 5
    while time.time() < deadline and not sim.samples:
        time.sleep(0.2)
    if not sim.samples:
        suite.check(name + ' prerequisite', False,
                    'SITL state stream never started with CAN enabled; check multicast routing')
        log = Path('sitl_ci.log')
        if log.exists():
            print('\n'.join(log.read_text(errors='replace').splitlines()[-5:]), flush=True)
        sim.close()
        return None
    return sim


def test_dronecan_params(suite, sitl_path):
    """DroneCAN parameter get/set round-trip through the firmware"""
    try:
        import dronecan
    except ImportError:
        suite.check('dronecan params prerequisite', False, 'package not installed')
        return
    with suite.Sitl(sitl_path, ['--can-uri', 'mcast:4', '--node-id', '40'],
              nosleep=False):
        sim = suite.can_state_stream('dronecan param set')
        if sim is None:
            return
        sim.close()
        node = dronecan.make_node('mcast:4', node_id=101)
        try:
            # ARK resets on input timeout. Keep the motor stopped with valid
            # CAN input so a reset cannot discard the in-RAM parameter write.
            def spin_stopped():
                node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=[0]))
                node.spin(0.05)

            ready_at = time.time() + 2.0
            while time.time() < ready_at:
                spin_stopped()
            got = {}

            def getset(req, key):
                done = {}
                def cb(e):
                    done['e'] = e
                node.request(req, 40, cb)
                t0 = time.time()
                while 'e' not in done and time.time() - t0 < 3:
                    try:
                        spin_stopped()
                    except Exception:
                        pass
                got[key] = done.get('e')

            # read TELEM_RATE, set it, read it back
            gp = dronecan.uavcan.protocol.param.GetSet.Request
            getset(gp(name=b'TELEM_RATE'), 'read')
            newval = dronecan.uavcan.protocol.param.Value(integer_value=77)
            getset(gp(name=b'TELEM_RATE', value=newval), 'set')
            getset(gp(name=b'TELEM_RATE'), 'reread')
            ok = (got['reread'] is not None and
                  got['reread'].response.value.integer_value == 77)
            suite.check('dronecan param set', ok,
                  'reread=%s' % (got['reread'].response.value.integer_value
                                 if got['reread'] else None))
        finally:
            node.close()


def test_stuck_rotor(suite, sitl_path, protection):
    """A brief jam can recover; repeated failed starts must eventually latch."""
    from run_test import WatchStream

    name = 'stuck rotor protection %s' % ('on' if protection else 'off')
    offset = suite.sitl_params.PARAMS_BY_NAME['STUCK_ROTOR_PROTECTION'][0]
    # Yield between simulator ticks so the input sender and variable-watch
    # threads keep running while the rotor is locked. A busy-waiting child
    # can starve them and turn this into a signal-timeout test instead.
    with suite.Sitl(sitl_path, ['--can-uri', 'none', '--input-type', '1'],
                    nosleep=False):
        ok, msg = suite.EepromClient('127.0.0.1', suite.STATE_PORT).set(
            offset, bytes([1 if protection else 0]))
        suite.check(name + ' eeprom set', ok, msg)
        sim = suite.SimStream('127.0.0.1', suite.STATE_PORT, period_us=200)
        sim.enabled = True
        watch = WatchStream('127.0.0.1', suite.STATE_PORT,
                            [('desync_episode_bucket', 1, False, None)])
        tx = suite.Sender(suite.sd.TYPE_DSHOT600)

        def check_rpm(label, spinning, window=0.5):
            rpm = suite.rpm_from_state(sim, window)
            valid = (2000 <= rpm <= 4000 if spinning
                     else rpm != -1 and abs(rpm) < 100)
            suite.check(name + ' ' + label, valid, 'rpm=%.0f' % rpm)

        try:
            resolved = watch.wait_resolved()
            suite.check(name + ' episode watch', resolved == [1],
                        'resolved=%r' % resolved)
            suite.sim_sleep(sim, 2.2)
            tx.value = 800
            suite.sim_sleep(sim, 3.0)
            check_rpm('spins', True)
            sim.set_stuck(1.0)
            # ARK's acquisition/desync rail is independent of the EEPROM
            # flag. Upstream's 3-second jam approaches its latch threshold,
            # so rotor phase can decide whether that test auto-restarts.
            suite.sim_sleep(sim, 3.0 if protection else 0.5)
            check_rpm('stalls', False, window=0.2)
            sim.set_stuck(0.0)
            suite.sim_sleep(sim, 4.0)
            if not protection:
                check_rpm('restarts after brief obstruction', True)
                # Keep the independent rail covered too: a sustained jam
                # must latch despite STUCK_ROTOR_PROTECTION being disabled.
                sim.set_stuck(1.0)
                suite.sim_sleep(sim, 8.0)
                events = watch.get(0)
                bucket = events[-1][1] if events else None
                suite.check(name + ' sustained obstruction latches episode rail',
                            bucket is not None and bucket >= 40,
                            'episode bucket=%s' % bucket)
                sim.set_stuck(0.0)
                suite.sim_sleep(sim, 4.0)
            check_rpm('stays off', False)
            tx.value = 0
            suite.sim_sleep(sim, 1.0)
            tx.value = 800
            suite.sim_sleep(sim, 3.0)
            check_rpm('recovers after throttle cycle', True)
        finally:
            tx.stop()
            watch.close()
            sim.close()


def test_ark_dataset_params(suite):
    """Keep ARK's calibration fixtures covered alongside ESCSim's datasets."""
    pairs = (
        ('ARK_4IN1_F051_900kv_noprop', 'ark_900kv_noprop'),
        ('ARK_4IN1_F051_2807_1300kv', 'ark_2807_1300kv_noprop'),
        ('ARK_4IN1_F051_900kv_10inch', 'ark_900kv_10inch'),
    )
    for dataset, model_name in pairs:
        param = HERE / 'data' / dataset / 'sitl.param'
        try:
            image = suite.sitl_params.image_from_param_file(str(param))
            model = json.loads((HERE / 'models' / (model_name + '.json')).read_text())['motor']
            kv = suite.sitl_params.byte_to_kv(image[suite.sitl_params.PARAMS_BY_NAME['MOTOR_KV'][0]])
            poles = image[suite.sitl_params.PARAMS_BY_NAME['MOTOR_POLES'][0]]
            valid = (model['kv'] * 0.8 <= kv <= model['kv'] * 1.25
                     and poles == model['poles']
                     and len(image) == suite.sitl_params.EEPROM_SIZE)
            suite.check('dataset params ' + dataset, valid,
                        '%d Kv, %d poles, %d bytes' % (kv, poles, len(image)))
        except (OSError, ValueError, KeyError) as ex:
            suite.check('dataset params ' + dataset, False, str(ex))


def load_suite(escsim):
    # sitl_params reads firmware version defaults at import time.
    os.environ['AM32_ROOT'] = str(ARK_ROOT)
    expected = (HERE / 'escsim-revision.txt').read_text().strip()
    actual = subprocess.check_output(
        ['git', '-C', str(escsim), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != expected:
        raise ValueError('ESCSim is at %s; check out pinned revision %s' % (actual, expected))
    runner = escsim / 'SITL' / 'run_ci_tests.py'
    # ESCSim modules must resolve together; ARK's older GUI clients use some
    # of the same names and intentionally remain available to the ARK suite.
    sys.path.insert(0, str(runner.parent))
    spec = importlib.util.spec_from_file_location('escsim_ci', runner)
    suite = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(suite)
    # Pinned ESCSim still parses a literal default_settings[] in DroneCAN.c.
    # ARK now generates that array from its schema. Supply only the defaults
    # callback so upstream parameter parsing and image construction stay tested.
    ark_spec = importlib.util.spec_from_file_location('ark_sitl_params', HERE / 'sitl_params.py')
    ark_params = importlib.util.module_from_spec(ark_spec)
    ark_spec.loader.exec_module(ark_params)
    suite.sitl_params._firmware_defaults = ark_params._firmware_defaults
    suite.INPUT_PORT = 17833
    suite.STATE_PORT = 17834
    suite.can_state_stream = lambda name: can_state_stream(suite, name)
    suite.test_startup_tune = lambda path: test_startup_tune(suite, path)
    suite.test_physics_audio = lambda path: test_physics_audio(suite, path)
    suite.test_dronecan_params = lambda path: test_dronecan_params(suite, path)
    suite.test_stuck_rotor = lambda path, protection: test_stuck_rotor(suite, path, protection)
    upstream_datasets = suite.test_dataset_params

    def datasets():
        upstream_datasets()
        test_ark_dataset_params(suite)

    suite.test_dataset_params = datasets
    return suite


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--escsim', type=Path,
                        default=Path(os.environ.get('ESCSIM_ROOT', ARK_ROOT / 'ESCSim')))
    parser.add_argument('--sitl', help='ARK32 SITL executable')
    parser.add_argument('--bootloader', help='optional bootloader SITL for serial/4-way tests')
    args = parser.parse_args()
    hits = sorted(glob.glob(str(ARK_ROOT / 'obj' / 'ARK32_AM32_SITL_CAN_*.elf')))
    binary = args.sitl or (hits[-1] if hits else None)
    if not binary or not Path(binary).is_file():
        parser.error('SITL binary not found: %s; run make AM32_SITL_CAN' % binary)
    # CAN is part of the behavioral suite, so a missing dependency must fail.
    try:
        import dronecan  # noqa: F401
        import serial  # noqa: F401
        suite = load_suite(args.escsim.resolve())
    except (ImportError, OSError, subprocess.CalledProcessError, ValueError) as ex:
        parser.error(str(ex))
    sys.argv = [str(args.escsim / 'SITL' / 'run_ci_tests.py'), '--sitl', str(Path(binary).resolve())]
    if args.bootloader:
        sys.argv += ['--bootloader', str(Path(args.bootloader).resolve())]
    result = 0
    try:
        suite.main()
    except SystemExit as ex:
        result = ex.code
    # Catch child sanitizer failures even if a later protocol test recovered.
    log_path = Path('sitl_ci.log')
    log = log_path.read_text(errors='replace') if log_path.exists() else ''
    suite.check('sanitizer log', not any(marker in log for marker in
                ('ERROR: AddressSanitizer', 'ERROR: LeakSanitizer', 'runtime error:')),
                'no sanitizer diagnostics')
    return result or int(bool(suite.failures))


if __name__ == '__main__':
    sys.exit(main())
