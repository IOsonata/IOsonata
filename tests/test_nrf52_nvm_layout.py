#!/usr/bin/env python3
"""Static nRF52 NVM partition regression checks for normal and MBR+DFU layouts."""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
DIR = ROOT / "ARM" / "Nordic" / "nRF52" / "ldscript"
PAGE = 0x1000

# (part, DFU enabled, flash origin, nvm0 origin, dfu bootloader origin, flash end)
LAYOUTS = (
    ("nrf52832_xxaa", False, 0x00000, 0x7D000, None, 0x80000),
    ("nrf52832_xxaa", True,  0x01000, 0x74000, 0x77000, 0x80000),
    ("nrf52840_xxaa", False, 0x00000, 0xFD000, None, 0x100000),
    ("nrf52840_xxaa", True,  0x01000, 0xDD000, 0xE0000, 0x100000),
)


def regions(script):
    content = script.read_text(encoding="utf-8")
    entries = {}
    pattern = re.compile(
        r"(?m)^\s*(FLASH|NVM0|NVM1)\s*\([^)]*\)\s*:\s*"
        r"ORIGIN\s*=\s*(0x[0-9A-Fa-f]+)\s*,\s*"
        r"LENGTH\s*=\s*(0x[0-9A-Fa-f]+)"
    )
    for match in pattern.finditer(content):
        name, origin, length = match.groups()
        if name in entries:
            raise AssertionError("duplicate region " + name)
        entries[name] = (int(origin, 16), int(length, 16))
    return content, entries


class Nrf52NvmLayoutTests(unittest.TestCase):
    def test_nvm_geometry_and_bootloader_bounds(self):
        for part, dfu, app_start, nvm_start, boot_start, flash_end in LAYOUTS:
            filename = "gcc_" + part + ("_mbr" if dfu else "") + ".ld"
            content, layout = regions(DIR / filename)
            with self.subTest(filename=filename):
                self.assertEqual(set(layout), {"FLASH", "NVM0", "NVM1"})
                flash, bond, app = (layout[key] for key in ("FLASH", "NVM0", "NVM1"))
                self.assertEqual(flash, (app_start, nvm_start - app_start))
                self.assertEqual(bond, (nvm_start, 2 * PAGE))
                self.assertEqual(app, (nvm_start + 2 * PAGE, PAGE))
                storage_end = app[0] + app[1]
                self.assertEqual(storage_end, boot_start if dfu else flash_end)

                for origin, size in layout.values():
                    self.assertEqual(origin % PAGE, 0)
                    self.assertEqual(size % PAGE, 0)

                for index in (0, 1):
                    lower = index
                    self.assertIn("__start_nvm%d = .;" % lower, content)
                    self.assertIn("__stop_nvm%d = .;" % lower, content)
                    self.assertIn(
                        ". = ORIGIN(NVM%d) + LENGTH(NVM%d);" % (lower, lower),
                        content,
                    )
                    self.assertRegex(
                        content,
                        r"(?s)nvm%d \(NOLOAD\).*?\}\s*> NVM%d" % (lower, lower)
                    )

                if dfu:
                    self.assertIn("MBR + Nordic DFU", content)
                    self.assertEqual(boot_start,
                                     0x77000 if part.startswith("nrf52832") else 0xE0000)
                    # Nordic reserves the two highest 4 KB pages for metadata.
                    self.assertEqual(flash_end - 2 * PAGE,
                                     0x7E000 if part.startswith("nrf52832") else 0xFE000)

    def test_special_no_dfu_variants_preserve_two_nvm_regions(self):
        cases = (
            ("gcc_nrf52832_xxaa_cli.ld", 0x7D000),
            ("gcc_nrf52840_xxaa_bossa.ld", 0xFD000),
            ("gcc_nrf52840_xxaa_cli.ld", 0xFD000),
            ("gcc_nrf52840_xxaa_ram40k.ld", 0xFD000),
        )
        for filename, start in cases:
            content, layout = regions(DIR / filename)
            with self.subTest(filename=filename):
                self.assertEqual(layout["NVM0"], (start, 0x2000))
                self.assertEqual(layout["NVM1"], (start + 0x2000, 0x1000))
                flash_start, flash_size = layout["FLASH"]
                self.assertEqual(flash_start + flash_size, start)
                self.assertIn("__start_nvm0 = .;", content)
                self.assertIn("__stop_nvm0 = .;", content)
                self.assertIn("__start_nvm1 = .;", content)
                self.assertIn("__stop_nvm1 = .;", content)

    def test_small_s112_dfu_releases_reserve_pds_and_user_data(self):
        for part in ("nrf52805_xxaa", "nrf52810_xxaa"):
            layout = (DIR / ("gcc_" + part + "_s112_dfu_layout.ld")).read_text(
                encoding="utf-8"
            )
            app = (DIR / ("gcc_" + part + "_s112_dfu.ld")).read_text(
                encoding="utf-8"
            )
            symbols = {
                name: int(value, 16)
                for name, value in re.findall(
                    r"(?m)^(__dfu_\\w+)\\s*=\\s*(0x[0-9a-fA-F]+);", layout
                )
            }
            with self.subTest(part=part):
                self.assertEqual(symbols["__dfu_slot0_start"], 0x19000)
                self.assertEqual(symbols["__dfu_slot0_end"], 0x23000)
                self.assertEqual(symbols["__dfu_nvm0_start"], 0x23000)
                self.assertEqual(symbols["__dfu_nvm0_size"], 0x2000)
                self.assertEqual(symbols["__dfu_nvm1_start"], 0x25000)
                self.assertEqual(symbols["__dfu_nvm1_size"], 0x1000)
                self.assertEqual(symbols["__dfu_rec_start"], 0x26000)
                self.assertEqual(symbols["__dfu_boot_start"], 0x27000)
                for name in ("nvm0", "nvm1"):
                    self.assertIn("__start_" + name + " = .;", app)
                    self.assertIn("__stop_" + name + " = .;", app)

    def test_no_dfu_does_not_reserve_mbr(self):
        for part in ("nrf52832_xxaa", "nrf52840_xxaa"):
            content, layout = regions(DIR / ("gcc_" + part + ".ld"))
            with self.subTest(part=part):
                self.assertEqual(layout["FLASH"][0], 0)
                self.assertIn("No MBR/DFU", content)


if __name__ == "__main__":
    unittest.main()
