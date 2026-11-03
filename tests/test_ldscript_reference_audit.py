#!/usr/bin/env python3
"""Offline regression tests for Eclipse linker-script reference validation."""

import importlib.util
import re
from pathlib import Path
import tempfile
import unittest


AUDIT = Path(__file__).with_name("ldscript_reference_audit.py")
SPEC = importlib.util.spec_from_file_location("ldscript_reference_audit", AUDIT)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


class LinkerReferenceAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.project = self.root / "ARM" / "Vendor" / "Family" / "Part"
        self.ioc = self.project / "exemples" / "Blinky" / "ioc"
        self.ioc.mkdir(parents=True)
        self.linkers = self.project.parent / "ldscript"
        self.linkers.mkdir()
        self.script = self.linkers / "gcc_part.ld"
        self.script.write_text("SECTIONS { .text : { *(.text*) } }\n", encoding="utf-8")
        self.reference = "../../../../../ldscript/gcc_part.ld"

    def write_project(self, reference=None):
        if reference is None:
            reference = self.reference
        path = self.ioc / ".cproject"
        path.write_text(
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<cproject><cconfiguration>'
            '<storageModule><configuration name="Debug">'
            '<toolChain><tool><option superClass="cross.linker.scriptfile" '
            'valueType="stringList"><listOptionValue builtIn="false" '
            'value="' + reference + '"/></option></tool></toolChain>'
            '</configuration></storageModule>'
            '</cconfiguration></cproject>\n',
            encoding="utf-8",
        )
        return path

    def test_existing_script_from_generated_build_directory(self):
        path = self.write_project()
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped, errors), (1, 0, []))

    def test_missing_script_is_reported(self):
        path = self.write_project("../../../../../ldscript/not_found.ld")
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped), (1, 0))
        self.assertEqual(len(errors), 1)
        self.assertIn("not_found.ld", errors[0])

    def test_dynamic_script_is_not_assumed_missing(self):
        path = self.write_project("${LINKER_SCRIPT}")
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped, errors), (0, 1, []))

    def test_absolute_linker_script_is_rejected(self):
        path = self.write_project("/opt/vendor/part.ld")
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped), (1, 0))
        self.assertIn("nonportable absolute", errors[0])

    def test_windows_absolute_linker_script_is_rejected(self):
        path = self.write_project("C:/vendor/part.ld")
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped), (1, 0))
        self.assertIn("nonportable absolute", errors[0])

    def test_windows_backslash_absolute_linker_script_is_rejected(self):
        path = self.write_project("C:\\\\vendor\\\\part.ld")
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped), (1, 0))
        self.assertIn("nonportable absolute", errors[0])

    def test_windows_unc_linker_script_is_rejected(self):
        path = self.write_project("\\\\\\\\server\\\\share\\\\part.ld")
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped), (1, 0))
        self.assertIn("nonportable absolute", errors[0])


    def write_iar_project(self, reference):
        iar = self.project / "exemples" / "Blinky" / "IAR"
        iar.mkdir()
        path = iar / "Blinky.ewp"
        from xml.sax.saxutils import escape
        path.write_text(
            "<project><configuration><settings><name>ILINK</name><data><option>"
            "<name>IlinkIcfFile</name><state>" + escape(reference) +
            "</state></option></data></settings></configuration></project>",
            encoding="utf-8",
        )
        return path

    def test_iar_linker_reference(self):
        iar = self.project / "exemples" / "Blinky" / "IAR"
        reference = "../../../../ldscript/gcc_part.icf"
        (self.linkers / "gcc_part.icf").write_text("define memory mem with size = 4G;")
        path = self.write_iar_project(reference.replace("/", chr(92)))
        self.assertEqual(audit.check_iar_project(path), (1, 0, []))

    def test_iar_missing_linker_reference(self):
        path = self.write_iar_project("../../../../ldscript/missing.icf")
        count, skipped, errors = audit.check_iar_project(path)
        self.assertEqual((count, skipped), (1, 0))
        self.assertIn("missing IAR linker script", errors[0])

    def test_inventory_identifies_referenced_script(self):
        self.write_project()
        referenced = audit.referenced_scripts(self.root)
        self.assertIn(self.script.resolve(), referenced)

    def test_inventory_keeps_unselected_script_as_candidate(self):
        self.write_project()
        unselected = self.linkers / "gcc_unused.ld"
        unselected.write_text("SECTIONS {}")
        referenced = audit.referenced_scripts(self.root)
        self.assertNotIn(unselected.resolve(), referenced)

    def test_gnu_ld_include_is_retained(self):
        child = self.linkers / "shared_memory.ld"
        child.write_text("MEMORY {}\n", encoding="utf-8")
        self.script.write_text('INCLUDE "shared_memory.ld"\n', encoding="utf-8")
        self.assertIn(child.resolve(), audit.included_scripts([self.script, child]))

    def test_architecture_shared_ld_include_is_retained(self):
        architecture = self.root / "ARM" / "ldscript"
        architecture.mkdir()
        shared = architecture / "gcc_arm_flash.ld"
        shared.write_text("SECTIONS {}\n", encoding="utf-8")
        self.script.write_text('INCLUDE "gcc_arm_flash.ld"\n', encoding="utf-8")
        self.assertIn(
            shared.resolve(),
            audit.included_scripts([self.script, shared]),
        )

    def test_iar_include_is_retained(self):
        primary = self.linkers / "iar_part.icf"
        child = self.linkers / "iar_memory.icf"
        primary.write_text('include "iar_memory.icf";\n', encoding="utf-8")
        child.write_text("define memory mem with size = 4G;\n", encoding="utf-8")
        self.assertIn(child.resolve(), audit.included_scripts([primary, child]))

    def test_ncs_metadata_without_linker_settings(self):
        path = self.ioc / ".cproject"
        path.write_text(
            '<cproject><cconfiguration id="org.eclipse.cdt.core.default.config"/>'
            '</cproject>',
            encoding="utf-8",
        )
        self.assertEqual(audit.check_project(path), (0, 0, []))

    def test_iar_disabled_linker_override_is_ignored(self):
        iar = self.project / "exemples" / "Blinky" / "IAR"
        iar.mkdir()
        path = iar / "Library.ewp"
        path.write_text(
            '<project><configuration><settings><name>ILINK</name><data>'
            '<option><name>IlinkIcfOverride</name><state>0</state></option>'
            '<option><name>IlinkIcfFile</name><state>lnk0t.icf</state></option>'
            '</data></settings></configuration></project>',
            encoding="utf-8",
        )
        self.assertEqual(audit.check_iar_project(path), (0, 0, []))

    def test_nrf5340_application_dfu_preserves_nvm_and_ipc(self):
        family = Path(__file__).resolve().parents[1] / "ARM" / "Nordic" / "nRF53" / "ldscript"
        layout = (family / "gcc_nrf5340_xxaa_application_dfu_layout.ld").read_text(encoding="utf-8")
        script = (family / "gcc_nrf5340_xxaa_application_dfu.ld").read_text(encoding="utf-8")
        symbols = {}
        for line in layout.splitlines():
            line = line.split("/*", 1)[0].strip()
            if "=" not in line or not line.endswith(";"):
                continue
            name, value = line[:-1].split("=", 1)
            name = name.strip()
            value = value.strip()
            if value.startswith("0x"):
                symbols[name] = int(value, 16)

        self.assertEqual(symbols["__dfu_slot0_end"], 0xFD000)
        self.assertEqual(symbols["__dfu_nvm0_start"], 0xFD000)
        self.assertEqual(symbols["__dfu_nvm0_end"], 0x100000)
        self.assertEqual(symbols["__dfu_flag"], 0x2007EFF0)
        self.assertIn("NVM0 (rw)", script)
        self.assertIn("__start_nvm0 = .;", script)
        self.assertIn("__stop_nvm0 = .;", script)
        self.assertIn("RAM1 (rwx)\t: ORIGIN = 0x20040000, LENGTH = 0x30000", script)
        self.assertIn("IPC0 (rw)\t: ORIGIN = 0x20070000, LENGTH = __dfu_flag - 0x20070000", script)
        self.assertIn("__start_ipc0 = .;", script)
        self.assertIn("__stop_ipc0 = .;", script)

    def test_nrf5340_dfu_ipc_block_fits_both_core_examples(self):
        root = Path(__file__).resolve().parents[1]
        ipc_start = 0x20070000
        recovery_flag = 0x2007EFF0
        available = recovery_flag - ipc_start
        for core in ("nRF5340_App", "nRF5340_Net"):
            path = (
                root / "ARM" / "Nordic" / "nRF53" / core
                / "exemples" / "IpcIntrfTest" / "src" / "board.h"
            )
            board = path.read_text(encoding="utf-8")
            addr = re.search(r"^#define\s+IPCTEST_MEM_ADDR\s+(0x[0-9A-Fa-f]+)UL", board, re.MULTILINE)
            size = re.search(r"^#define\s+IPCTEST_MEM_SIZE\s+(0x[0-9A-Fa-f]+)U", board, re.MULTILINE)
            with self.subTest(core=core):
                self.assertIsNotNone(addr)
                self.assertIsNotNone(size)
                self.assertEqual(int(addr.group(1), 16), ipc_start)
                self.assertLessEqual(int(size.group(1), 16), available)

    def test_nrf52840_s140c_flash_does_not_overlap_storage(self):
        family = Path(__file__).resolve().parents[1] / "ARM" / "Nordic" / "nRF52" / "ldscript"
        content = (family / "gcc_nrf52840_xxaa_s140c.ld").read_text(encoding="utf-8")
        regions = {}
        pattern = re.compile(
            r"^\s*(FLASH|NVM0|BOOTLOADER_SETTINGS)\s*\([^)]*\)\s*:"
            r"\s*ORIGIN\s*=\s*(0x[0-9a-fA-F]+)\s*,\s*"
            r"LENGTH\s*=\s*(0x[0-9a-fA-F]+)",
            re.MULTILINE,
        )
        for name, origin, length in pattern.findall(content):
            regions[name] = (int(origin, 16), int(length, 16))
        self.assertEqual(regions["FLASH"], (0x27000, 0xD6000))
        self.assertEqual(regions["NVM0"], (0xFD000, 0x2000))
        self.assertEqual(regions["BOOTLOADER_SETTINGS"], (0xFF000, 0x1000))
        self.assertEqual(regions["FLASH"][0] + regions["FLASH"][1], regions["NVM0"][0])
        self.assertEqual(regions["NVM0"][0] + regions["NVM0"][1], regions["BOOTLOADER_SETTINGS"][0])
        self.assertIn("__start_nvm0 = .;", content)
        self.assertIn("__stop_nvm0 = .;", content)

    def test_nrf52832_iar_nvm_reservation_matches_gcc(self):
        family = Path(__file__).resolve().parents[1] / "ARM" / "Nordic" / "nRF52" / "ldscript"
        expected = (
            ("iar_nrf52832_xxaa.icf", "__ICFEDIT_region_ROM_end__"),
            ("iar_nrf52832_xxaa_irom.icf", "__ICFEDIT_region_IROM1_end__"),
        )
        for filename, end_symbol in expected:
            with self.subTest(filename=filename):
                content = (family / filename).read_text(encoding="utf-8")
                self.assertRegex(
                    content,
                    end_symbol + r"\s*=\s*0x0*7CFFF\s*;",
                )
                self.assertIn("define symbol __start_nvm0 = 0x0007D000;", content)
                self.assertIn("define symbol __stop_nvm0 = 0x0007F000;", content)
                self.assertIn("export symbol __start_nvm0;", content)
                self.assertIn("export symbol __stop_nvm0;", content)

    def test_malformed_xml_is_reported(self):
        path = self.ioc / ".cproject"
        path.write_text("<cproject>", encoding="utf-8")
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped), (0, 0))
        self.assertIn("invalid XML", errors[0])


if __name__ == "__main__":
    unittest.main()
