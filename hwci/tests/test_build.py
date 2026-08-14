"""Firmware artifact picker: never flash EEPROM/factory sidecars as the app."""
import time

from hwci.build import find_artifact


def test_find_artifact_skips_eeprom_and_factory_sidecars(tmp_path):
    target = "ARK_G431_CAN"
    eeprom = tmp_path / f"{target}_3.0.3-ark.eeprom.bin"
    factory_hex = tmp_path / f"{target}_3.0.3-ark.factory.hex"
    uavcan = tmp_path / f"{target}_3.0.3-ark.uavcan.bin"
    app_bin = tmp_path / f"{target}_3.0.3-ark.bin"
    app_hex = tmp_path / f"{target}_3.0.3-ark.hex"
    older = tmp_path / f"{target}_3.0.2-ark.bin"
    for p, n in ((eeprom, 2048), (factory_hex, 64), (uavcan, 64),
                 (older, 64), (app_bin, 128), (app_hex, 64)):
        p.write_bytes(b"\x00" * n)
        time.sleep(0.01)
    older.write_bytes(b"\x00" * 64)  # older mtime if the loop order varies
    # Ensure the 3.0.3 app is newest among the real images.
    time.sleep(0.01)
    app_bin.write_bytes(b"\x11" * 128)
    app_hex.write_bytes(b"\x11" * 64)

    assert find_artifact(tmp_path, target, "bin") == app_bin
    assert find_artifact(tmp_path, target, "hex") == app_hex
    assert find_artifact(tmp_path, target, "elf") is None
