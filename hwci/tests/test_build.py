"""Firmware artifact picker: never flash EEPROM/factory sidecars as the app."""
import time

from hwci.build import find_artifact


def test_find_artifact_skips_eeprom_and_factory_sidecars(tmp_path):
    target = "ARK_G431_CAN"
    # Makefile IDENTIFIER := ARK32 → obj/ARK32_<target>_<ver>.{bin,hex,elf}
    prefix = f"ARK32_{target}"
    eeprom = tmp_path / f"{prefix}_3.0.3-ark.eeprom.bin"
    factory_hex = tmp_path / f"{prefix}_3.0.3-ark.factory.hex"
    uavcan = tmp_path / f"{prefix}_3.0.3-ark.uavcan.bin"
    app_bin = tmp_path / f"{prefix}_3.0.3-ark.bin"
    app_hex = tmp_path / f"{prefix}_3.0.3-ark.hex"
    older = tmp_path / f"{prefix}_3.0.2-ark.bin"
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


def test_find_artifact_pairs_images_with_the_newest_elf(tmp_path):
    """A .bin/.hex comes from the same build as the ELF, never by its own mtime."""
    import os
    target = "ARK_G431_CAN"
    old = {e: tmp_path / f"ARK32_{target}_31.0.{e}" for e in ("elf", "bin", "hex")}
    new = {e: tmp_path / f"ARK32_{target}_32.0.{e}" for e in ("elf", "bin", "hex")}
    for f in list(old.values()) + list(new.values()):
        f.write_bytes(b"x")
    for f in old.values():
        os.utime(f, (1000, 1000))
    for f in new.values():
        os.utime(f, (2000, 2000))
    os.utime(old["bin"], (3000, 3000))  # an older build's bin touched last
    assert find_artifact(tmp_path, target, "elf") == new["elf"]
    assert find_artifact(tmp_path, target, "bin") == new["bin"]
    assert find_artifact(tmp_path, target, "hex") == new["hex"]


def test_find_artifact_missing_sibling_is_none(tmp_path):
    """The newest ELF without its .bin must not borrow another build's .bin."""
    import os
    target = "ARK_G431_CAN"
    (tmp_path / f"ARK32_{target}_31.0.bin").write_bytes(b"x")
    os.utime(tmp_path / f"ARK32_{target}_31.0.bin", (3000, 3000))
    (tmp_path / f"ARK32_{target}_32.0.elf").write_bytes(b"x")
    assert find_artifact(tmp_path, target, "bin") is None
