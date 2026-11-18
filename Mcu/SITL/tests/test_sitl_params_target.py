'''SITL images and the parameter editor follow the selected target (PR #147 review).

The image builder used to resolve the post-skeleton defaults from the
AM32_SITL_CAN block only, so an image for ARK_G4_12S_SITL_CAN got the generic
MAX_RAMP 160 instead of that target's 5. The editor's "All defaults" used the
schema skeleton (current limit 102 = limiter off, input DShot) instead of the
bytes the firmware seeds (100, AUTO).
'''

from __future__ import annotations

import sitl_params as p

CORRECTED = ('MAX_RAMP', 'TEMPERATURE_LIMIT', 'CURRENT_LIMIT', 'INPUT_SIGNAL_TYPE', 'TEMP_DERATE_BAND')


def test_image_defaults_follow_the_target():
    ramp = p.PARAMS_BY_NAME['MAX_RAMP'][0]
    assert p.build_image({}, target='AM32_SITL_CAN')[ramp] == 160
    assert p.build_image({}, target='ARK_G4_12S_SITL_CAN')[ramp] == 5


def test_editor_defaults_are_the_seeded_bytes():
    saved = p.TARGET
    try:
        for target, ramp in (('AM32_SITL_CAN', 160), ('ARK_G4_12S_SITL_CAN', 5)):
            p.set_target(target)
            image = p.base_image()
            for name in CORRECTED:
                off, _name, _lo, _hi, dflt, _help = p.PARAMS_BY_NAME[name]
                assert dflt == image[off], (target, name, dflt, image[off])
            assert p.PARAMS_BY_NAME['MAX_RAMP'][4] == ramp
            assert p.PARAMS_BY_NAME['CURRENT_LIMIT'][4] == 100
            assert p.PARAMS_BY_NAME['INPUT_SIGNAL_TYPE'][4] == 0
            # version bytes still come from Inc/version.h, not the image
            for name in ('EEPROM_VERSION', 'VERSION_MAJOR', 'VERSION_MINOR'):
                assert p.PARAMS_BY_NAME[name][4] == p._VER[name]
    finally:
        p.set_target(saved)
