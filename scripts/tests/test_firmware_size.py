#!/usr/bin/env python3
"""Exercise flash/RAM accounting against ELFs linked like the F051 app.

Needs arm-none-eabi binutils on PATH.
Run with: python3 -m unittest discover -s scripts/tests -p test_firmware_size.py
"""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "firmware_size.py"
sys.path.insert(0, str(SCRIPT.parent))

from firmware_size import format_change, format_usage, memory_usage, summarize

# .file_name sits at a fixed address past the end of FLASH, as on the F051.
LINKER_SCRIPT = """
MEMORY
{
    FLASH (rx) : ORIGIN = 0x08001000, LENGTH = 1K
    FILE_NAME (rx) : ORIGIN = 0x08001400, LENGTH = 32
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 1K
}
SECTIONS
{
    .text : { *(.text) } > FLASH
    .data : { *(.data) } > RAM AT > FLASH
    .bss (NOLOAD) : { *(.bss) } > RAM
    .file_name : { *(.file_name) } > FILE_NAME
    .debug_info 0 : { *(.debug_info) }
}
"""


class FirmwareSizeTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.linker = self.root / "firmware.ld"
        self.linker.write_text(LINKER_SCRIPT)

    def build(self, name="firmware", *, text=64, data=16, bss=32, debug=16):
        source = self.root / f"{name}.s"
        obj = source.with_suffix(".o")
        elf = source.with_suffix(".elf")
        source.write_text(f"""
.section .text,"ax"
.space {text}, 1
.section .data,"aw"
.space {data}, 2
.section .bss,"aw",%nobits
.space {bss}
.section .file_name,"a"
.space 32, 3
.section .debug_info,"",%progbits
.space {debug}, 4
""")
        subprocess.run(["arm-none-eabi-as", str(source), "-o", str(obj)], check=True)
        subprocess.run(["arm-none-eabi-ld", "-T", str(self.linker), str(obj), "-o", str(elf)],
                       check=True)
        return elf

    def test_flash_excludes_gap_before_file_name(self):
        self.assertEqual(memory_usage(self.build()), {"flash": 64 + 16 + 32, "ram": 16 + 32})

    def test_sections_count_where_they_live(self):
        before = memory_usage(self.build())
        for change, expected in (
            ({"text": 96}, {"flash": 32, "ram": 0}),
            ({"data": 48}, {"flash": 32, "ram": 32}),
            ({"bss": 64}, {"flash": 0, "ram": 32}),
            ({"debug": 4096}, {"flash": 0, "ram": 0}),
        ):
            with self.subTest(change=change):
                after = memory_usage(self.build(**change))
                self.assertEqual({key: after[key] - before[key] for key in after}, expected)

    def test_summary(self):
        capacity = {"flash": 2048, "ram": 4096}
        result = summarize({"flash": 1024, "ram": 1024}, {"flash": 1088, "ram": 960}, capacity)
        self.assertEqual(result, {
            "flash": "1,088 / 2,048 B (53.12%)", "flash_change": "🟡 +64 B (+6.25%)",
            "ram": "960 / 4,096 B (23.44%)", "ram_change": "🟢 -64 B (-6.25%)",
            "changed": True,
        })
        unchanged = {"flash": 8, "ram": 8}
        self.assertFalse(summarize(unchanged, unchanged, capacity)["changed"])
        self.assertEqual(format_usage(26592, 27648), "26,592 / 27,648 B (96.18%)")
        self.assertEqual(format_change(1000, 1000), "+0 B (+0.00%)")
        self.assertEqual(format_change(1000, 1256), "🟡 +256 B (+25.60%)")
        self.assertEqual(format_change(1000, 1257), "🔴 +257 B (+25.70%)")

    def test_cli_from_outside_checkout(self):
        before = self.build("before")
        after = self.build("after", bss=96)
        output = subprocess.check_output([
            sys.executable, str(SCRIPT), "--before", str(before), "--after", str(after),
            "--flash-capacity", "1024", "--ram-capacity", "1024",
        ], cwd=self.root, text=True)
        self.assertEqual(json.loads(output), {
            "flash": "112 / 1,024 B (10.94%)", "flash_change": "+0 B (+0.00%)",
            "ram": "112 / 1,024 B (10.94%)", "ram_change": "🟡 +64 B (+133.33%)",
            "changed": True,
        })


if __name__ == "__main__":
    unittest.main()
