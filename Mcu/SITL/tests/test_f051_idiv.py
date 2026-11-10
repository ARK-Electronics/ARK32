'''The F051 signed divide (Mcu/f051/Src/idiv.s) must match libgcc's __divsi3.

idiv.s replaces libgcc's 460-byte __divsi3 with a 44-byte wrapper around the
unsigned divide. It is linked in place of the compiler's helper for every
signed `/` in the F051 firmware, so "same result for every input" is the whole
contract. This does not need the SITL binary: it cross-builds a Cortex-M0
harness with the pinned toolchain and runs it under qemu-arm, comparing the
wrapper with the libgcc routine it displaces (edge pairs incl. divide by zero
and INT32_MIN, plus random pairs at every operand magnitude).

Skipped when the pinned arm-none-eabi toolchain or qemu-arm is not installed.
'''

from __future__ import annotations

import glob
import os
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..', '..'))
IDIV_S = os.path.join(ROOT, 'Mcu', 'f051', 'Src', 'idiv.s')
HARNESS_C = os.path.join(HERE, 'f051_idiv_harness.c')


def _tool(name):
    hits = sorted(glob.glob(os.path.join(
        ROOT, 'tools', '*', 'xpack-arm-none-eabi-gcc-*', 'bin', 'arm-none-eabi-' + name)))
    return hits[-1] if hits else None


def _qemu():
    for name in ('qemu-arm', 'qemu-arm-static'):
        path = shutil.which(name)
        if path:
            return path
    return None


def test_idiv_matches_libgcc(tmp_path):
    gcc, objcopy, qemu = _tool('gcc'), _tool('objcopy'), _qemu()
    if not gcc or not objcopy:
        pytest.skip('pinned arm-none-eabi toolchain not installed (make arm_sdk_install)')
    if not qemu:
        pytest.skip('qemu-arm not installed')

    arch = ['-mcpu=cortex-m0', '-mthumb']
    obj = str(tmp_path / 'idiv.o')
    elf = str(tmp_path / 'harness.elf')
    subprocess.run([gcc, *arch, '-c', IDIV_S, '-o', obj], check=True)
    # Rename the wrapper so the harness links it next to libgcc's __divsi3
    # instead of in place of it.
    subprocess.run([objcopy,
                    '--redefine-sym', '__aeabi_idiv=ark_idiv',
                    '--redefine-sym', '__divsi3=ark_divsi3', obj], check=True)
    subprocess.run([gcc, *arch, '-O1', '--specs=rdimon.specs',
                    HARNESS_C, obj, '-o', elf], check=True)

    run = subprocess.run([qemu, '-cpu', 'cortex-m3', elf],
                         capture_output=True, text=True, timeout=120)
    out = run.stdout + run.stderr
    assert run.returncode == 0 and out.startswith('OK '), out
    # 40x40 edge pairs + 32x32x64 random pairs
    assert int(out.split()[1]) == 40 * 40 + 32 * 32 * 64, out
