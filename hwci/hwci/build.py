"""Firmware build helpers (wrap the AM32 Makefile)."""
from __future__ import annotations

import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass
class BuildArtifacts:
    elf: Path
    bin: Path
    hex: Path


# Sidecar images share the ``ARK32_<target>_<ver>`` prefix. A glob of ``*.bin``
# must not pick ``.eeprom.bin`` / ``.uavcan.bin`` (or ``.factory.hex`` for
# ``*.hex``) — flashing those at the app base overwrites vectors. Observed
# 2026-08-14: hwci ci programmed the 2 KiB EEPROM dump at 0x08004000.
_SIDECAR_MARKERS = (".eeprom.", ".factory.", ".uavcan.")


def _app_images(obj_dir: Path, target: str, ext: str) -> list[Path]:
    """``obj/ARK32_<target>_*.<ext>`` firmware images, oldest first by mtime."""
    ext = ext.lstrip(".")
    hits = [
        p for p in Path(obj_dir).glob(f"ARK32_{target}_*.{ext}")
        if p.is_file() and not any(m in p.name for m in _SIDECAR_MARKERS)
    ]
    hits.sort(key=lambda p: p.stat().st_mtime)
    return hits


def find_artifact(obj_dir: Path, target: str, ext: str) -> Path | None:
    """The ``obj/ARK32_<target>_*.<ext>`` firmware image to use, or None.

    Ignores factory/EEPROM/UAVCAN sidecars. The ELF is the newest by mtime, so
    a rebuild wins over an older version sitting in ``obj/``. A ``.bin`` or
    ``.hex`` is that ELF's sibling (same stem), never chosen by its own mtime:
    a touched or copied older image would otherwise pair one build's binary
    with another build's symbols (PR #147 review). With no ELF at all, the
    newest image of the asked kind. RigConfig ELF resolution uses this too.
    """
    ext = ext.lstrip(".")
    elves = _app_images(obj_dir, target, "elf")
    if ext == "elf":
        return elves[-1] if elves else None
    if elves:
        sibling = elves[-1].with_suffix("." + ext)
        return sibling if sibling.is_file() else None
    images = _app_images(obj_dir, target, ext)
    return images[-1] if images else None


def build_firmware(repo_root: str | Path, target: str, *,
                   hwci_perf: bool = True, jobs: int = 4,
                   arm_sdk_prefix: str | None = None,
                   extra_make_args: list[str] | None = None) -> BuildArtifacts:
    """Run ``make <target>`` (with HWCI_PERF=1 by default) and return artifacts."""
    repo_root = Path(repo_root)
    cmd = ["make", target, f"-j{jobs}"]
    if hwci_perf:
        cmd.append("HWCI_PERF=1")
    if arm_sdk_prefix:
        cmd.append(f"ARM_SDK_PREFIX={arm_sdk_prefix}")
    if extra_make_args:
        cmd += extra_make_args
    proc = subprocess.run(cmd, cwd=repo_root, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"firmware build failed (rc={proc.returncode}):\n"
            f"{proc.stdout[-2000:]}\n{proc.stderr[-2000:]}")
    obj = repo_root / "obj"
    elf, binf, hexf = (find_artifact(obj, target, e) for e in ("elf", "bin", "hex"))
    if elf is None or binf is None:
        raise RuntimeError(f"build produced no artifacts for {target} in {obj}")
    if hwci_perf:
        # The Makefile does not encode HWCI_PERF in object paths, so a prior
        # non-instrumented build leaves up-to-date objects and this "build"
        # silently packages firmware WITHOUT the perf struct (caught on the
        # bench: flashed an ELF with no hwci_perf symbol). Verify, don't hope.
        try:
            from . import elf as elfmod
            elfmod.find_symbol(str(elf), "hwci_perf")
        except ImportError:
            pass  # no pyelftools: PerfReader will catch it at run time
        except Exception as e:
            raise RuntimeError(
                f"{elf} lacks the hwci_perf symbol - stale non-instrumented "
                f"objects in obj/ (run 'make clean' and rebuild): {e}") from e
    return BuildArtifacts(elf=elf, bin=binf, hex=hexf)
