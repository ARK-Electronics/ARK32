"""12S motor profiles must arm the ESC-estimate runaway guard (esc_max_rpm).

safety.max_rpm watches only the stand's speed sensor. A false commutation lock
reports a large ESC speed at low stand speed and current, and only
esc_max_rpm catches that (LiveDesyncWatch skips the guard when it is unset).
PR #147 review.
"""
from __future__ import annotations

import re
from pathlib import Path

from hwci.config import load_profile

PROFILES = Path(__file__).resolve().parents[1] / "hwci" / "profiles"
TWELVE_S = re.compile(r"(160kv|345kv|_12s)")


def test_12s_motor_profiles_set_esc_max_rpm():
    names = sorted(p for p in PROFILES.glob("*.yaml") if TWELVE_S.search(p.stem))
    assert names, "no 12S motor profiles found"
    missing = [p.name for p in names if not load_profile(str(p)).esc_max_rpm]
    assert not missing, f"12S profiles without esc_max_rpm: {missing}"
