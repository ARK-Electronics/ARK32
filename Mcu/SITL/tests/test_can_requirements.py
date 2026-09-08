"""CI must fail rather than skip when its CAN prerequisite disappears."""

import pytest
import sitl_test_requirements as policy


@pytest.mark.parametrize('strict', [False, True])
def test_unavailable_can_policy(monkeypatch, strict):
    monkeypatch.setenv('SITL_REQUIRE_CAN', '1' if strict else '0')
    outcome = pytest.fail.Exception if strict else pytest.skip.Exception
    with pytest.raises(outcome, match='missing multicast'):
        policy.unavailable_can('missing multicast')


@pytest.mark.parametrize('strict', [False, True])
def test_missing_dronecan_policy(monkeypatch, strict):
    monkeypatch.setenv('SITL_REQUIRE_CAN', '1' if strict else '0')
    def missing(name):
        raise ModuleNotFoundError(name=name)
    monkeypatch.setattr(policy.importlib, 'import_module', missing)
    outcome = pytest.fail.Exception if strict else pytest.skip.Exception
    with pytest.raises(outcome, match='dronecan is required'):
        policy.require_dronecan()
