"""Explicit CAN availability policy for local runs versus dedicated CI."""

import importlib
import os
import pytest


def unavailable_can(reason):
    if os.environ.get('SITL_REQUIRE_CAN') == '1':
        pytest.fail(reason, pytrace=False)
    pytest.skip(reason, allow_module_level=True)


def require_dronecan():
    try:
        return importlib.import_module('dronecan')
    except ModuleNotFoundError as exc:
        if exc.name != 'dronecan':
            raise
        unavailable_can('dronecan is required for CAN protocol tests')
