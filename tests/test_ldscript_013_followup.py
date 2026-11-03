#!/usr/bin/env python3
"""Regression checks for the 0.13 linker follow-up corrections."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
LD = ROOT / "ARM" / "Nordic" / "nRF54" / "ldscript"
NRF91 = ROOT / "ARM" / "Nordic" / "nRF91"


class LinkerFollowupTests(unittest.TestCase):
    def test_re01_dfu_uses_vli_api_and_renamed_boot_script(self):
        build = (ROOT / "tests" / "re01" / "build_dfu.py").read_text()
        self.assertIn("-DuECC_ENABLE_VLI_API=1", build)
        self.assertIn("gcc_re01_1500kb_dfu_boot.ld", build)
        self.assertIn("gcc_re01_1500kb_dfu.ld", build)
        self.assertNotIn("'dfu_boot_re01_1500kb.ld'", build)

    def test_lm20b_s145_matches_lm20a_memory_map(self):
        for suffix in ("_s145.ld", "_s145_dfu_layout.ld"):
            a = (LD / ("gcc_nrf54lm20a_xxaa" + suffix)).read_text()
            b = (LD / ("gcc_nrf54lm20b_xxaa" + suffix)).read_text()
            # Compare layout constants, ignoring target names and descriptions.
            if suffix == "_s145.ld":
                for value in ("0x1D8800", "0x1D9800", "0x1DA800", "0x1FCBFF"):
                    self.assertIn(value, b)
                    self.assertIn(value, a)
                self.assertNotIn("0x0D8800", b)
                self.assertNotIn("0x0DA800", b)
            else:
                for symbol in ("__dfu_slot0_end", "__dfu_slot1_start",
                               "__dfu_slot1_end", "__dfu_nvm0_start",
                               "__dfu_nvm1_start"):
                    pattern = rf"(?m)^\s*{symbol}\s*=\s*(0x[0-9A-Fa-f]+);"
                    a_value = re.search(pattern, a)
                    b_value = re.search(pattern, b)
                    self.assertIsNotNone(a_value, symbol)
                    self.assertIsNotNone(b_value, symbol)
                    self.assertEqual(a_value.group(1), b_value.group(1), symbol)
                self.assertNotIn("0x000D8800", b)
                self.assertIn("gcc_nrf54lm20b_xxaa_s145_dfu_boot.ld", b)

    def test_nrf5340_readmes_name_shared_family_linker(self):
        script = ROOT / "ARM" / "Nordic" / "nRF53" / "ldscript" / "gcc_nrf5340_xxaa_application.ld"
        self.assertTrue(script.is_file())
        for readme_path in (
            ROOT / "ARM" / "Nordic" / "nRF53" / "nRF5340_App"
                / "exemples" / "BleSecurityDemo" / "README.md",
            ROOT / "exemples" / "ipc" / "README.md",
        ):
            with self.subTest(readme=readme_path):
                readme = readme_path.read_text()
                self.assertIn("`gcc_nrf5340_xxaa_application.ld`", readme)
                self.assertNotIn("`nrf5340_xxaa_application.ld`", readme)

    def test_nrf91_example_readmes_point_to_existing_scripts(self):
        examples = (
            ("nRF9160", "CoapDemo"), ("nRF9160", "MqttDemo"),
            ("nRF9160", "MqttTlsDemo"), ("nRF91x1", "CoapDemo"),
            ("nRF91x1", "LteCoapDemo"), ("nRF91x1", "LteMqttDemo"),
            ("nRF91x1", "MqttDemo"), ("nRF91x1", "MqttTlsDemo"),
        )
        for target, name in examples:
            with self.subTest(target=target, example=name):
                readme = (NRF91 / target / "exemples" / name / "README.md").read_text()
                script = "gcc_nrf9160_xxaa.ld" if target == "nRF9160" else "gcc_nrf9120_xxaa.ld"
                self.assertIn("ARM/Nordic/nRF91/ldscript/" + script, readme)
                self.assertTrue((NRF91 / "ldscript" / script).is_file())


if __name__ == "__main__":
    unittest.main()
