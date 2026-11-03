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

    def test_projdirpath_existing_script(self):
        reference = "${ProjDirPath}/../../../../ldscript/gcc_part.ld"
        path = self.write_project(reference)
        self.assertEqual(audit.check_project(path), (1, 0, []))
        self.assertIn(self.script.resolve(), audit.referenced_scripts(self.root))

    def test_projdirpath_wrong_directory_depth_is_reported(self):
        reference = "${ProjDirPath}/../../../../../ldscript/gcc_part.ld"
        path = self.write_project(reference)
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped), (1, 0))
        self.assertEqual(len(errors), 1)
        self.assertIn("gcc_part.ld", errors[0])

    def test_unsupported_eclipse_macro_is_skipped(self):
        path = self.write_project("${UnknownMacro}/gcc_part.ld")
        self.assertEqual(audit.check_project(path), (0, 1, []))

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

    def test_iar_projdirpath_existing_icf(self):
        reference = "$PROJ_DIR$\\..\\..\\..\\..\\ldscript\\gcc_part.icf"
        script = self.linkers / "gcc_part.icf"
        script.write_text("define memory mem with size = 4G;\\n", encoding="utf-8")
        path = self.write_iar_project(reference)
        self.assertEqual(audit.check_iar_project(path), (1, 0, []))
        self.assertIn(script.resolve(), audit.referenced_scripts(self.root))

    def test_iar_projdirpath_missing_icf(self):
        reference = "$PROJ_DIR$\\\\..\\\\..\\\\..\\\\..\\\\ldscript\\\\missing.icf"
        path = self.write_iar_project(reference)
        checked, skipped, errors = audit.check_iar_project(path)
        self.assertEqual((checked, skipped), (1, 0))
        self.assertEqual(len(errors), 1)
        self.assertIn("missing.icf", errors[0])

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

    def test_all_nordic_linker_scripts_are_classified(self):
        nordic = Path(__file__).resolve().parents[1] / "ARM" / "Nordic"
        scripts = sorted(nordic.rglob("*.ld"))
        self.assertTrue(scripts)
        for script in scripts:
            with self.subTest(script=str(script.relative_to(nordic))):
                content = script.read_text(encoding="utf-8")
                sdk_include = re.search(
                    r'(?m)^\s*INCLUDE\s+["\']?nrf_common\.ld\b', content
                )
                if sdk_include:
                    # Nordic SDK linker scripts belong only to isolated nrfx
                    # reference examples, never to IOsonata's family directory.
                    self.assertNotEqual(script.parent.name, "ldscript")
                    self.assertIn("nrfx", str(script).lower())
                    continue
                self.assertEqual(script.parent.name, "ldscript")
                self.assertTrue(script.name.startswith("gcc_"), script.name)
                if re.search(r"(?m)^\s*MEMORY\s*$", content):
                    self.assertIn('INCLUDE "gcc_arm_flash.ld"', content)

    def test_sam4e16_dfu_memory_and_project_selections(self):
        root = Path(__file__).resolve().parents[1]
        family = root / "ARM" / "Microchip" / "SAM4E" / "ldscript"
        base = (family / "gcc_sam4e16.ld").read_text(encoding="utf-8")
        layout = (family / "gcc_sam4e16_dfu_layout.ld").read_text(encoding="utf-8")
        app = (family / "gcc_sam4e16_dfu.ld").read_text(encoding="utf-8")
        boot = (family / "gcc_sam4e16_dfu_boot.ld").read_text(encoding="utf-8")
        symbols = {
            name: int(value, 0)
            for name, value in re.findall(
                r"(?m)^(__dfu_\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+);", layout
            )
        }
        self.assertIn("FLASH (rx)  : ORIGIN = 0x00400000, LENGTH = 0x00100000", base)
        self.assertIn("RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00020000", base)
        self.assertEqual(symbols["__dfu_boot_start"], 0x00400000)
        self.assertEqual(symbols["__dfu_boot_end"], symbols["__dfu_rec_start"])
        self.assertEqual(symbols["__dfu_rec_end"], symbols["__dfu_slot0_start"])
        self.assertEqual(symbols["__dfu_slot0_end"], 0x00500000)
        self.assertEqual(symbols["__dfu_flag"], 0x2001FFF0)
        self.assertEqual(symbols["__dfu_state_unit"], 512)
        self.assertEqual(symbols["__dfu_rec_end"] - symbols["__dfu_rec_start"], 0x2000)
        for key in ("__dfu_boot_start", "__dfu_boot_end", "__dfu_rec_start",
                    "__dfu_rec_end", "__dfu_slot0_start", "__dfu_slot0_end"):
            self.assertEqual(symbols[key] % 0x2000, 0)
        for script in (app, boot):
            self.assertIn('INCLUDE "gcc_sam4e16_dfu_layout.ld"', script)
            self.assertIn('INCLUDE "gcc_arm_flash.ld"', script)
            self.assertIn("LENGTH = __dfu_flag - 0x20000000", script)
        self.assertIn("ORIGIN = __dfu_slot0_start", app)
        self.assertIn("ORIGIN = __dfu_boot_start", boot)
        projects = root / "ARM" / "Microchip" / "SAM4E" / "SAM4E16E" / "exemples"
        for name in ("Blinky", "CryptoTest", "UartPrbsTxTest", "UartRetargetDemo", "DfuBoot"):
            with self.subTest(project=name):
                content = (projects / name / "ioc" / ".cproject").read_text(encoding="utf-8")
                selected = "gcc_sam4e16_dfu_boot.ld" if name == "DfuBoot" else "gcc_sam4e16.ld"
                self.assertEqual(content.count("ldscript/" + selected), 4)

    def test_nrf52832_nrfx_prbs_uses_local_nordic_linker(self):
        nordic = Path(__file__).resolve().parents[1] / "ARM" / "Nordic"
        example = nordic / "nRF52" / "nRF52832" / "exemples" / "UartPrbsTxnrfx"
        project = (example / "ioc" / ".cproject").read_text(encoding="utf-8")
        script = example / "src" / "nrf52_xxaa_nrfx_prbs.ld"
        self.assertTrue(script.exists())
        self.assertIn('INCLUDE "nrf_common.ld"', script.read_text(encoding="utf-8"))
        self.assertNotIn("gcc_nrf52832_xxaa_nrfx_prbs.ld", project)
        self.assertEqual(project.count("../../src/nrf52_xxaa_nrfx_prbs.ld"), 4)

    def test_nrf54_iosonata_gcc_scripts_use_generic_linker(self):
        family = Path(__file__).resolve().parents[1] / "ARM" / "Nordic" / "nRF54" / "ldscript"
        scripts = list(family.glob("gcc_*.ld"))
        self.assertTrue(scripts)
        for script in scripts:
            with self.subTest(script=script.name):
                content = script.read_text(encoding="utf-8")
                self.assertNotIn('INCLUDE "nrf_common.ld"', content)
                if re.search(r"^\s*MEMORY\s*$", content, re.MULTILINE):
                    if "_dfu_layout" not in script.name:
                        self.assertIn('INCLUDE "gcc_arm_flash.ld"', content)

    def test_nrf54h20_dfu_respects_core_mram_and_recovery_ram(self):
        family = Path(__file__).resolve().parents[1] / "ARM" / "Nordic" / "nRF54" / "ldscript"
        cases = (
            ("application", 0x0E0A0000, 0x40000, 0x22000000, 0x8000, False),
            ("radiocore", 0x0E120000, 0x80000, 0x23000000, 0x10000, True),
        )
        primary_ranges = []
        for core, mram_start, mram_size, ram_start, ram_size, has_slot1 in cases:
            with self.subTest(core=core):
                stem = "gcc_nrf54h20_xxaa_" + core
                bare = (family / (stem + ".ld")).read_text(encoding="utf-8")
                boot = (family / (stem + "_dfu_boot.ld")).read_text(encoding="utf-8")
                app = (family / (stem + "_dfu.ld")).read_text(encoding="utf-8")
                layout = (family / (stem + "_dfu_layout.ld")).read_text(encoding="utf-8")
                symbols = {
                    name: int(value, 16)
                    for name, value in re.findall(
                        r"^(__dfu_\w+)\s*=\s*(0x[0-9A-Fa-f]+);", layout, re.MULTILINE
                    )
                }
                self.assertIn(
                    "FLASH (rx) : ORIGIN = 0x{:X}, LENGTH = 0x{:X}".format(mram_start, mram_size),
                    bare,
                )
                self.assertEqual(symbols["__dfu_boot_start"], mram_start)
                self.assertEqual(symbols["__dfu_boot_end"], symbols["__dfu_rec_start"])
                self.assertEqual(symbols["__dfu_rec_end"], symbols["__dfu_slot0_start"])
                self.assertLess(symbols["__dfu_boot_start"], symbols["__dfu_boot_end"])
                self.assertLess(symbols["__dfu_rec_start"], symbols["__dfu_rec_end"])
                self.assertLess(symbols["__dfu_slot0_start"], symbols["__dfu_slot0_end"])
                self.assertEqual(symbols["__dfu_flag"], ram_start + ram_size - 16)
                for value in symbols.values():
                    if mram_start <= value <= mram_start + mram_size:
                        self.assertEqual(value % 0x1000, 0)
                if has_slot1:
                    self.assertEqual(symbols["__dfu_slot0_end"], symbols["__dfu_slot1_start"])
                    self.assertEqual(symbols["__dfu_slot1_end"], mram_start + mram_size)
                    self.assertGreater(
                        symbols["__dfu_slot1_end"] - symbols["__dfu_slot1_start"],
                        symbols["__dfu_slot0_end"] - symbols["__dfu_slot0_start"],
                    )
                else:
                    self.assertNotIn("__dfu_slot1_start", symbols)
                    self.assertNotIn("__dfu_slot1_end", symbols)
                    self.assertEqual(symbols["__dfu_slot0_end"], mram_start + mram_size)
                for script in (app, boot):
                    self.assertIn('INCLUDE "' + stem + '_dfu_layout.ld"', script)
                    self.assertIn("LENGTH = __dfu_flag - 0x{:08X}".format(ram_start), script)
                    self.assertIn('INCLUDE "gcc_arm_flash.ld"', script)
                self.assertIn("ORIGIN = __dfu_slot0_start", app)
                self.assertIn("ORIGIN = __dfu_boot_start", boot)
                primary_ranges.append((mram_start, mram_start + mram_size))
        self.assertLessEqual(primary_ranges[0][1], primary_ranges[1][0])

    def test_nrf54_s145_dfu_slot_and_storage_boundaries(self):
        family = Path(__file__).resolve().parents[1] / "ARM" / "Nordic" / "nRF54" / "ldscript"
        cases = (
            ("nrf54l15_xxaa", 0x158800, 0x20004900, 0x2003FFF0),
            ("nrf54lm20a_xxaa", 0x1D8800, 0x20004780, 0x2007FCB0),
            ("nrf54lm20b_xxaa", 0x0D8800, 0x20004780, 0x2007FCB0),
        )
        for target, storage, ram_start, recovery_flag in cases:
            with self.subTest(target=target):
                layout = (family / ("gcc_" + target + "_s145_dfu_layout.ld")).read_text(encoding="utf-8")
                app = (family / ("gcc_" + target + "_s145_dfu.ld")).read_text(encoding="utf-8")
                boot = (family / ("gcc_" + target + "_s145_dfu_boot.ld")).read_text(encoding="utf-8")
                normal = (family / ("gcc_" + target + "_s145.ld")).read_text(encoding="utf-8")
                symbols = {}
                for name, value in re.findall(
                    r"^(__dfu_\w+)\s*=\s*(0x[0-9a-fA-F]+);", layout, re.MULTILINE
                ):
                    symbols[name] = int(value, 16)

                self.assertEqual(symbols["__dfu_boot_start"], 0)
                self.assertEqual(symbols["__dfu_boot_end"], symbols["__dfu_rec_start"])
                self.assertEqual(symbols["__dfu_rec_end"], symbols["__dfu_slot0_start"])
                self.assertEqual(symbols["__dfu_slot0_end"], symbols["__dfu_slot1_start"])
                self.assertEqual(symbols["__dfu_slot1_end"], symbols["__dfu_nvm0_start"])
                self.assertEqual(symbols["__dfu_nvm0_start"], storage)
                self.assertEqual(symbols["__dfu_nvm0_start"] + symbols["__dfu_nvm_size"], symbols["__dfu_nvm1_start"])
                self.assertEqual(symbols["__dfu_nvm_size"], 0x1000)
                self.assertEqual(symbols["__dfu_flag"], recovery_flag)
                self.assertEqual(symbols["__dfu_flag"] % 16, 0)
                self.assertGreater(symbols["__dfu_slot1_end"] - symbols["__dfu_slot1_start"],
                                   symbols["__dfu_slot0_end"] - symbols["__dfu_slot0_start"])
                for script in (app, boot):
                    self.assertIn('INCLUDE "gcc_' + target + '_s145_dfu_layout.ld"', script)
                    self.assertIn("LENGTH = __dfu_flag - ", script)
                self.assertIn("ORIGIN = __dfu_slot0_start", app)
                self.assertIn("ORIGIN = __dfu_boot_start", boot)
                self.assertIn("ORIGIN = __dfu_nvm0_start", app)
                self.assertIn("ORIGIN = __dfu_nvm1_start", app)
                self.assertIn("ORIGIN = 0x{:08X}".format(ram_start), app)
                self.assertIn("NVM0  (rw)  : ORIGIN = 0x{:06X}".format(storage), normal)
                self.assertIn("NVM1  (rw)  : ORIGIN = 0x{:06X}".format(storage + 0x1000), normal)

    def test_nrf54_bare_metal_reserved_nvm_is_declared(self):
        family = Path(__file__).resolve().parents[1] / "ARM" / "Nordic" / "nRF54" / "ldscript"
        cases = (
            ("gcc_nrf54l15_xxaa.ld", 0x17D000),
            ("gcc_nrf54lm20a_xxaa.ld", 0x1FD000),
            ("gcc_nrf54l05_xxaa.ld", 0x7D000),
            ("gcc_nrf54l10_xxaa.ld", 0xFD000),
            ("gcc_nrf54lm20a_enga_xxaa.ld", 0x1FD000),
            ("gcc_nrf54ls05b_enga_xxaa.ld", 0x7F000),
            ("gcc_nrf54lv10a_enga_xxaa.ld", 0xFD000),
        )
        for filename, flash_end in cases:
            with self.subTest(filename=filename):
                content = (family / filename).read_text(encoding="utf-8")
                flash = re.search(
                    r"^\s*FLASH\s*\([^)]*\)\s*:\s*ORIGIN\s*=\s*(0x[0-9a-fA-F]+),"
                    r"\s*LENGTH\s*=\s*(0x[0-9a-fA-F]+)",
                    content, re.MULTILINE,
                )
                nvm = re.search(
                    r"^\s*NVM0\s*\([^)]*\)\s*:\s*ORIGIN\s*=\s*(0x[0-9a-fA-F]+),"
                    r"\s*LENGTH\s*=\s*(0x[0-9a-fA-F]+)",
                    content, re.MULTILINE,
                )
                self.assertIsNotNone(flash)
                self.assertIsNotNone(nvm)
                flash_start, flash_length = (int(value, 16) for value in flash.groups())
                nvm_start, nvm_length = (int(value, 16) for value in nvm.groups())
                self.assertEqual(flash_start + flash_length, nvm_start)
                self.assertEqual(nvm_start + nvm_length, flash_end)
                expected_size = 0x1000 if filename == "gcc_nrf54ls05b_enga_xxaa.ld" else 0x3000
                self.assertEqual(nvm_length, expected_size)
                self.assertIn("__start_nvm0 = .;", content)
                self.assertIn("__stop_nvm0 = .;", content)

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
                self.assertIn("define symbol __start_nvm1 = 0x0007F000;", content)
                self.assertIn("define symbol __stop_nvm1 = 0x00080000;", content)
                self.assertIn("export symbol __start_nvm1;", content)
                self.assertIn("export symbol __stop_nvm1;", content)

    def test_malformed_xml_is_reported(self):
        path = self.ioc / ".cproject"
        path.write_text("<cproject>", encoding="utf-8")
        checked, skipped, errors = audit.check_project(path)
        self.assertEqual((checked, skipped), (0, 0))
        self.assertIn("invalid XML", errors[0])


if __name__ == "__main__":
    unittest.main()
