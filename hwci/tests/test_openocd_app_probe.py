"""OpenOcdDebugger.ensure_app_running: never force-boot a running app.

It used to probe ``eeprom_address`` at hard-coded RAM addresses
(0x20000FB4 / 0x20000EB4). The global has since moved (0x200010a8 on the
ARK_4IN1_F051 HWCI_PERF build), so every attach saw "not a settings page"
and restarted the running app from its vectors, wiping its RAM. The probe
address now comes from the flashed ELF, and without one nothing is done.
"""
from __future__ import annotations

import struct

import pytest

from hwci import elf
from hwci.debugger import openocd
from hwci.debugger.base import DebuggerError
from hwci.settings import EEPROM_ADDRESS_SYMBOL

PROBE = 0x200010A8  # eeprom_address in the ARK_4IN1_F051 HWCI_PERF build


class FakeOcd(openocd.OpenOcdDebugger):
    """No openocd process: RAM is a dict, RPC answers a fixed PC reply."""

    def __init__(self, ram: dict[int, int], *, pc_reply: str | Exception,
                 elf_path: str | None = "fw.elf"):
        super().__init__(elf_path=elf_path)
        self.ram = ram
        self.pc_reply = pc_reply
        self.boots = 0
        self.reads: list[int] = []

    def _rpc(self, command: str) -> str:
        assert command == "reg pc"
        if isinstance(self.pc_reply, Exception):
            raise self.pc_reply
        return self.pc_reply

    def read_memory(self, addr: int, length: int) -> bytes:
        self.reads.append(addr)
        if addr not in self.ram:
            raise DebuggerError(f"unmapped 0x{addr:08x}")
        return struct.pack("<I", self.ram[addr])[:length]

    def _boot_app_from_vectors(self) -> None:
        self.boots += 1


@pytest.fixture(autouse=True)
def _openocd_on_path(monkeypatch):
    monkeypatch.setattr(openocd.shutil, "which", lambda _: "/usr/bin/openocd")


@pytest.fixture
def symtab(monkeypatch):
    def find_symbol(path, name):
        assert (path, name) == ("fw.elf", EEPROM_ADDRESS_SYMBOL)
        return elf.Symbol(name=name, address=PROBE, size=4)
    monkeypatch.setattr(elf, "find_symbol", find_symbol)


# A running core does not answer "reg pc" with a value.
RUNNING = DebuggerError("target not halted")


def test_running_app_is_left_alone(symtab):
    # The old hard-coded slots hold unrelated data; only the ELF address counts.
    dbg = FakeOcd({PROBE: 0x08007C00, 0x20000FB4: 0x1234, 0x20000EB4: 0},
                  pc_reply=RUNNING)
    dbg.ensure_app_running()
    assert dbg.boots == 0
    assert dbg.reads == [PROBE]


def test_stuck_bootloader_by_ram_probe_is_booted(symtab):
    dbg = FakeOcd({PROBE: 0}, pc_reply=RUNNING)
    dbg.ensure_app_running()
    assert dbg.boots == 1


def test_halted_in_bootloader_is_booted_without_reading_ram(symtab):
    dbg = FakeOcd({}, pc_reply="pc (/32): 0x08000b42")
    dbg.ensure_app_running()
    assert dbg.boots == 1
    assert dbg.reads == []


def test_no_elf_never_force_boots():
    dbg = FakeOcd({0x20000FB4: 0x1234, 0x20000EB4: 0}, pc_reply=RUNNING,
                  elf_path=None)
    dbg.ensure_app_running()
    assert dbg.boots == 0
    assert dbg.reads == []


def test_missing_symbol_never_force_boots(monkeypatch):
    def find_symbol(path, name):
        raise elf.ElfError("symbol not found")
    monkeypatch.setattr(elf, "find_symbol", find_symbol)
    dbg = FakeOcd({}, pc_reply=RUNNING)
    dbg.ensure_app_running()
    assert dbg.boots == 0


def test_failed_swd_read_never_force_boots(symtab):
    dbg = FakeOcd({}, pc_reply=RUNNING)  # PROBE unmapped -> DebuggerError
    dbg.ensure_app_running()
    assert dbg.boots == 0
