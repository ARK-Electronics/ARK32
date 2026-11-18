'''SITL process boot and basic I/O surface.'''

from __future__ import annotations

import os
import time

import pytest

from sitl_harness import wait_for_state


def test_boot_binds_ports_and_streams_state(sitl_factory, state_stream):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none')
    sim = state_stream(sitl)
    assert wait_for_state(sim, timeout=5.0), (
        'state stream never started\n' + sitl.log_tail())
    latest = sim.latest()
    assert latest is not None
    # omega ~ 0 at idle, bus voltage from default model (~16.8 V)
    omega, vbus = latest[1], latest[10]
    assert abs(omega) < 50.0, 'motor spinning at idle: omega=%g' % omega
    assert 10.0 < vbus < 30.0, 'unexpected bus voltage: %g' % vbus


def test_boot_verbose_log_mentions_ports(sitl_factory):
    sitl = sitl_factory(extra_args=['--input-type', '1', '--verbose'],
                        can_uri='none', wait_s=1.0)
    # give the verbose 1 Hz line a chance; port banners print at startup
    time.sleep(0.5)
    log = sitl.log_tail(40)
    assert 'PWM/DShot input on udp port' in log or str(sitl.input_port) in log, log
    assert 'state/model port' in log or str(sitl.state_port) in log, log


def test_eeprom_file_created(sitl_factory, workdir):
    sitl = sitl_factory(extra_args=['--input-type', '1'], can_uri='none', wait_s=0.8)
    eeprom = os.path.join(workdir, 'am32_eeprom.bin')
    assert os.path.exists(eeprom), 'eeprom not created in %s: %s' % (
        workdir, os.listdir(workdir))
    assert os.path.getsize(eeprom) > 0


def test_boot_save_keeps_the_stored_protection_bytes(sitl_factory, workdir):
    '''Firmware never rewrites stored settings; migrations live in the configurator.

    The ARK 4IN1 factory page carries the disabled sentinels 141 (temperature,
    byte 43) and 102 (current, byte 44), and never wrote byte 184 (derate band,
    0xFF). The firmware resolves those at use. A boot that stamps a new
    firmware version saves the RAM page, so a coercion written into RAM would
    land in flash: 141 -> 255 and 0xFF -> 15 did on this branch before.
    '''
    import sitl_params

    img = bytearray(sitl_params.build_image({'TEMPERATURE_LIMIT': 141, 'CURRENT_LIMIT': 102}))
    img[184] = 0xFF
    minor = img[4]
    img[4] = (minor + 1) & 0xFF  # an older stamp: the boot re-stamps and saves
    path = os.path.join(workdir, 'am32_eeprom.bin')
    with open(path, 'wb') as f:
        f.write(bytes(img))
    sitl = sitl_factory(can_uri='none')
    deadline = time.time() + 5
    page = b''
    while time.time() < deadline:
        with open(path, 'rb') as f:
            page = f.read()
        if page[4] == minor:
            break
        time.sleep(0.1)
    assert page[4] == minor, ('boot never re-stamped the page, so this tested nothing', sitl.log_tail())
    assert (page[43], page[44], page[184]) == (141, 102, 0xFF), (
        'boot save rewrote stored protection bytes: temperature %d, current %d, band 0x%02x'
        % (page[43], page[44], page[184]))
