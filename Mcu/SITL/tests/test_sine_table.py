'''The quarter-wave sine table must reproduce the table it replaced.

Inc/sine_table.h stores a quarter period as bytes (91 B) and rebuilds the rest
from the table's two symmetries. Sine startup drives the phases straight from
these values, so the contract is bit-identical output for every angle in
[0, 360). The harness carries the previous half-period table and lookup
verbatim and compares all 360 angles.

Needs only a host C compiler (the same one that builds the SITL).
'''

from __future__ import annotations

import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..', '..'))


def test_quarter_table_matches_previous_table(tmp_path):
    cc = shutil.which(os.environ.get('CC', 'cc')) or shutil.which('gcc')
    if not cc:
        pytest.skip('no host C compiler')
    exe = str(tmp_path / 'sine_table_harness')
    subprocess.run([cc, '-Wall', '-Wextra', '-Werror', '-I', os.path.join(ROOT, 'Inc'),
                    os.path.join(HERE, 'sine_table_harness.c'), '-o', exe], check=True)
    run = subprocess.run([exe], capture_output=True, text=True, timeout=30)
    assert run.returncode == 0 and run.stdout.strip() == 'OK 360', run.stdout + run.stderr
