'''Speed telemetry conversions (Inc/telemetry_rpm.h), checked exactly on the host.

The first start after boot used to report e_rpm 37856 (600000 / 2 truncated to 16
bits) because e_com_time is 2 us until the first commutation interval is timed;
esc.Status wrapped it to 16512 rpm and DShot telemetry sent a 2 us period. The
SITL test test_rpm_telemetry_start.py checks the same start end to end, but the
spike lasts one loop pass, so it is caught only some of the time; this harness
pins the conversions themselves. Skipped when no host C compiler is installed.
'''

from __future__ import annotations

import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
HARNESS_C = os.path.join(HERE, 'telemetry_rpm_harness.c')


def test_telemetry_rpm_conversions(tmp_path):
    cc = shutil.which('cc') or shutil.which('gcc')
    if not cc:
        pytest.skip('no host C compiler')
    exe = str(tmp_path / 'telemetry_rpm_harness')
    subprocess.run([cc, '-std=c11', '-Wall', '-Wextra', '-Werror', '-O1',
                    '-I', os.path.join(ROOT, 'Inc'), HARNESS_C, '-o', exe], check=True)
    out = subprocess.run([exe], capture_output=True, text=True)
    assert out.returncode == 0 and out.stdout.startswith('OK'), out.stdout
