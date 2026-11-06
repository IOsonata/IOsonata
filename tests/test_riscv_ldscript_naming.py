#!/usr/bin/env python3
"""Check standardized RISC-V linker filenames, project references, and DFU layouts."""

from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
RISC = ROOT / "RISCV"
ESP = RISC / "Espressif" / "ESP32C"
REN = RISC / "Renesas" / "R9A02"
SCRIPTS = REN / "ldscript"
EXAMPLES = REN / "R9A02G021" / "exemples"

EXPECTED = {
    "gcc_r9a02g021.ld",
    "gcc_r9a02g021_dfu.ld",
    "gcc_r9a02g021_dfu_boot.ld",
    "gcc_r9a02g021_dfu_layout.ld",
    "llvm_r9a02g021.ld",
    "llvm_r9a02g021_dfu.ld",
    "llvm_r9a02g021_dfu_boot.ld",
    "r9a02g021_head.ld",
}
OBSOLETE = (
    "gcc_R9A02G021.ld",
    "gcc_R9A02G021_dfu.ld",
    "dfu_boot_r9a02g021.ld",
    "dfu_layout_r9a02g021.ld",
    "llvm_R9A02G021.ld",
    "llvm_dfu_boot_r9a02g021.ld",
)


def script_selections(project):
    xml = ET.parse(project)
    values = []
    for config in xml.findall(".//cconfiguration"):
        settings = config.find(".//configuration")
        if settings is None:
            continue
        selected = []
        for option in config.iter("option"):
            identity = option.get("superClass", "") + option.get("id", "")
            if "linker.scriptfile" not in identity:
                continue
            value = option.get("value")
            if value:
                selected.append(value.strip('"'))
            selected.extend(
                elem.get("value", "").strip('"')
                for elem in option.findall("listOptionValue")
            )
        if selected:
            values.append((settings.get("name"), selected))
    return values


class RiscvLinkerNamingTests(unittest.TestCase):
    def test_renesas_script_inventory(self):
        actual = {p.name for p in SCRIPTS.glob("*.ld")}
        self.assertEqual(actual, EXPECTED)
        for name in OBSOLETE:
            self.assertNotIn(name, actual)

    def test_renesas_managed_projects(self):
        projects = sorted(EXAMPLES.glob("*/ioc/.cproject"))
        self.assertEqual(len(projects), 17)
        for project in projects:
            expected = ("gcc_r9a02g021_dfu_boot.ld" if
                        project.parents[1].name == "DfuBoot" else
                        "gcc_r9a02g021.ld")
            selected = script_selections(project)
            with self.subTest(project=project.parents[1].name):
                self.assertEqual(len(selected), 2)
                for config, references in selected:
                    self.assertIn(config, ("Debug", "Release"))
                    self.assertEqual(len(references), 2)
                    for reference in references:
                        self.assertEqual(Path(reference).name, expected)
                        resolved = (project.parent / config / reference).resolve()
                        self.assertEqual(resolved, (SCRIPTS / expected).resolve())

    def test_espressif_managed_projects(self):
        for part in ("ESP32C3", "ESP32C6"):
            projects = sorted((ESP / part / "exemples").glob("*/ioc/.cproject"))
            self.assertEqual(len(projects), 3)
            expected = "gcc_" + part.lower() + ".ld"
            self.assertTrue((ESP / "ldscript" / expected).is_file())
            for project in projects:
                selected = script_selections(project)
                with self.subTest(part=part, project=project.parents[1].name):
                    if part == "ESP32C6" and project.parents[1].name == "qemutest":
                        # Deliberately incomplete skeleton; not a linkable example.
                        self.assertEqual(selected, [])
                        continue
                    self.assertEqual(len(selected), 2)
                    for config, references in selected:
                        self.assertEqual(len(references), 2)
                        for reference in references:
                            self.assertEqual(Path(reference).name, expected)
                            self.assertEqual(
                                (project.parent / config / reference).resolve(),
                                (ESP / "ldscript" / expected).resolve(),
                            )

    def test_all_renesas_internal_includes_resolve(self):
        for script in SCRIPTS.glob("*.ld"):
            content = script.read_text(encoding="utf-8")
            for name in OBSOLETE:
                self.assertNotIn(name, content)
            for include in re.findall(r'^\s*INCLUDE\s+"([^"]+)"', content, re.M):
                with self.subTest(script=script.name, include=include):
                    self.assertTrue(
                        (SCRIPTS / include).is_file()
                        or (RISC / "ldscript" / include).is_file()
                    )

    def test_dfu_layout_preserved(self):
        layout = (SCRIPTS / "gcc_r9a02g021_dfu_layout.ld").read_text()
        values = {
            name: int(number, 0)
            for name, number in re.findall(
                r"(__dfu_\w+)\s*=\s*(0x[0-9A-Fa-f]+);", layout
            )
        }
        expected = {
            "__dfu_flag": 0x20006FF0,
            "__dfu_boot_start": 0,
            "__dfu_boot_end": 0x9000,
            "__dfu_rec_start": 0x9000,
            "__dfu_rec_end": 0xA000,
            "__dfu_slot0_start": 0xA000,
            "__dfu_slot0_end": 0x20000,
        }
        self.assertEqual(values, expected)
        for compiler in ("gcc", "llvm"):
            boot = (SCRIPTS / (compiler + "_r9a02g021_dfu_boot.ld")).read_text()
            app = (SCRIPTS / (compiler + "_r9a02g021_dfu.ld")).read_text()
            for content in (boot, app):
                self.assertIn('INCLUDE "gcc_r9a02g021_dfu_layout.ld"', content)
                self.assertIn('INCLUDE "' + compiler + '_riscv_flash.ld"', content)
                self.assertIn("__dfu_flag - 0x20004000", content)
            self.assertIn('INCLUDE "r9a02g021_head.ld"', boot)
            self.assertNotIn('INCLUDE "r9a02g021_head.ld"', app)
            self.assertIn("RESET_VECTOR = __dfu_slot0_start", app)

    def test_nordic_riscv_target_is_archive_only(self):
        project = RISC / "Nordic/nRF54/nRF54H20/lib/ioc/.cproject"
        self.assertTrue(project.is_file())
        self.assertEqual(script_selections(project), [])


if __name__ == "__main__":
    unittest.main()
