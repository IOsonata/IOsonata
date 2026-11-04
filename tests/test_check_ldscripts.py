#!/usr/bin/env python3
"""Check Installer/check_ldscripts.py on small temporary trees.

Covers references into vendor packages in external/: listed without a finding
when external/ is absent, checked when it is present, and Eclipse linked folder
references listed without a check. Repository references keep their existing
missing and duplicate findings.
"""

import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
CHECK = ROOT / "Installer" / "check_ldscripts.py"
SPEC = importlib.util.spec_from_file_location("check_ldscripts", CHECK)
check_ldscripts = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check_ldscripts)

NRFX_SCRIPT = 'MEMORY\n{\n}\nINCLUDE "nrf_common.ld"\n'
SDK_PROJECT = ('<option value="&quot;../../../../external/nRF5_SDK/modules/'
               'nrfx/mdk/nrf52_xxaa.ld&quot;"/>\n')
CUBE_PROJECT = ('<option value="&quot;${workspace_loc:/${ProjName}/CubeRoot}'
                '/Projects/BOARD/STM32CubeIDE/PART_FLASH.ld&quot;"/>\n')


class CheckLdscriptsTests(unittest.TestCase):

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.home = Path(self.temp.name)
        self.root = self.home / "IOsonata"
        self.write("ARM/ldscript/gcc_arm_flash.ld", "SECTIONS\n{\n}\n")
        self.write("ARM/Nordic/exemples/Prbs/src/nrf52_xxaa_prbs.ld",
                   NRFX_SCRIPT)
        self.write("ARM/Nordic/exemples/Sdk/ioc/.cproject", SDK_PROJECT)
        self.write("ARM/ST/exemples/Cube/ioc/.cproject", CUBE_PROJECT)

    def tearDown(self):
        self.temp.cleanup()

    def write(self, rel, text, base=None):
        path = (base or self.root) / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    def run_check(self, external):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            result = check_ldscripts.check(self.root, external)
        return result, out.getvalue()

    def test_vendor_references_without_external_are_listed(self):
        result, out = self.run_check(None)
        self.assertEqual(result, 0, out)
        self.assertIn("0 findings, 3 external references", out)
        self.assertIn("not checked, no external/ folder", out)
        self.assertIn("not checked, Eclipse linked folder", out)

    def test_vendor_references_are_checked_in_external(self):
        external = self.home / "external"
        self.write("nrfx/bsp/stable/mdk/nrf_common.ld", "", external)
        self.write("nRF5_SDK/modules/nrfx/mdk/nrf52_xxaa.ld", "", external)
        result, out = self.run_check(external)
        self.assertEqual(result, 0, out)
        self.assertIn("0 findings, 3 external references", out)

    def test_vendor_file_missing_from_external_is_a_finding(self):
        external = self.home / "external"
        self.write("nRF5_SDK/modules/nrfx/mdk/nrf52_xxaa.ld", "", external)
        result, out = self.run_check(external)
        self.assertEqual(result, 1, out)
        self.assertIn("INCLUDE nrf_common.ld: missing in", out)

    def test_external_found_next_to_iosonata(self):
        (self.home / "external").mkdir()
        self.assertEqual(check_ldscripts.find_external_root(self.root),
                         self.home / "external")

    def test_repository_references_keep_their_findings(self):
        self.write("ARM/Nordic/ldscript/gcc_part.ld",
                   'INCLUDE "gcc_unknown.ld"\n')
        self.write("ARM/Nordic/other/gcc_arm_flash.ld", "")
        result, out = self.run_check(None)
        self.assertEqual(result, 1, out)
        self.assertIn("INCLUDE gcc_unknown.ld: missing", out)
        self.assertIn("DUPLICATE: gcc_arm_flash.ld", out)


if __name__ == "__main__":
    unittest.main()
