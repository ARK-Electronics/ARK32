"""Pinned ESCSim parameter clients can read ARK's schema-backed defaults."""

from pathlib import Path
import os
import subprocess
import sys
import textwrap

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]


def test_upstream_dataset_params_use_schema_defaults(tmp_path):
    escsim = Path(os.environ.get('ESCSIM_ROOT', REPO_ROOT / 'ESCSim')).expanduser().resolve()
    if not (escsim / 'SITL' / 'run_ci_tests.py').is_file():
        pytest.skip('Pinned ESCSim checkout unavailable; set ESCSIM_ROOT to run adapter coverage')

    # Other SITL tests import ARK's identically named clients. Start an isolated
    # interpreter so their cached modules cannot replace the upstream clients.
    script = textwrap.dedent('''
        import importlib.util
        from pathlib import Path
        import re
        import sys

        root, escsim = map(Path, sys.argv[1:])
        spec = importlib.util.spec_from_file_location(
            'ark_upstream_adapter_under_test', root / 'Mcu/SITL/run_upstream_tests.py')
        adapter = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(adapter)
        suite = adapter.load_suite(escsim)
        params = suite.sitl_params
        upstream_source = escsim / 'SITL/sitl_params.py'
        assert Path(params.__file__).resolve() == upstream_source
        for function in (params.base_image, params.build_image, params.parse_param_file):
            assert Path(function.__code__.co_filename).resolve() == upstream_source

        expected = bytearray(b'\\xff' * 192)
        golden = bytes.fromhex((root / 'schema/eeprom-defaults.hex').read_text())
        assert len(golden) == 48
        expected[:len(golden)] = golden
        versions = dict(re.findall(
            r'^\\s*#define\\s+(EEPROM_VERSION|VERSION_MAJOR|VERSION_MINOR)\\s+(\\d+)',
            (root / 'Inc/version.h').read_text(), re.MULTILINE))
        for offset, name in ((1, 'EEPROM_VERSION'), (3, 'VERSION_MAJOR'), (4, 'VERSION_MINOR')):
            expected[offset] = int(versions[name])
        actual = params.base_image()
        assert len(actual) == 192
        assert actual == expected

        checks = []
        original_check = suite.check

        def record_check(name, condition, detail):
            checks.append(name)
            original_check(name, condition, detail)

        suite.check = record_check
        suite.test_dataset_params()
        assert not suite.failures, suite.failures
        assert sorted(checks) == sorted('dataset params ' + name for name in (
            'gt2215', 'nano2216', 'vim1404',
            'ARK_4IN1_F051_900kv_noprop', 'ARK_4IN1_F051_2807_1300kv',
            'ARK_4IN1_F051_900kv_10inch'))
    ''')
    result = subprocess.run(
        [sys.executable, '-I', '-c', script, str(REPO_ROOT), str(escsim)],
        cwd=tmp_path, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
