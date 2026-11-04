"""Keep size-conscious Debug flags on the nRF5340 network-core OTA example."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "ARM/Nordic/nRF53/nRF5340_Net/exemples/BleOta/ioc/.cproject"


class Nrf5340NetBleOtaSizeTest(unittest.TestCase):
    def test_debug_optimizes_for_size(self):
        root = ET.parse(PROJECT).getroot()
        configs = root.findall(".//configuration")
        self.assertEqual({cfg.get("name") for cfg in configs},
                         {"Debug", "Release"})
        debug = next(cfg for cfg in configs if cfg.get("name") == "Debug")
        options = [
            opt.get("value", "")
            for opt in debug.findall(".//option")
            if opt.get("name") == "Optimization Level"
        ]
        self.assertEqual(options, [
            "ilg.gnuarmeclipse.managedbuild.cross.option.optimization.level.size"
        ])

    def test_debug_does_not_force_semihosting(self):
        root = ET.parse(PROJECT).getroot()
        debug = next(
            cfg for cfg in root.findall(".//configuration")
            if cfg.get("name") == "Debug"
        )
        linker_flags = [
            opt.get("value", "")
            for opt in debug.findall(".//option")
            if opt.get("name") == "Other linker flags"
        ]
        self.assertEqual(len(linker_flags), 2)
        for flags in linker_flags:
            self.assertIn("--specs=nosys.specs", flags)
            self.assertNotIn("rdimon", flags)
            self.assertNotIn("-lrdimon", flags)

    def test_release_keeps_workaround_for_gcc_lto_failure(self):
        root = ET.parse(PROJECT).getroot()
        release = next(
            cfg for cfg in root.findall(".//configuration")
            if cfg.get("name") == "Release"
        )
        linker_flags = [
            opt.get("value", "")
            for opt in release.findall(".//option")
            if opt.get("name") == "Other linker flags"
        ]
        self.assertEqual(len(linker_flags), 2)
        for flags in linker_flags:
            self.assertIn("-fno-lto", flags.split())
            self.assertIn("-fno-use-linker-plugin", flags.split())


if __name__ == "__main__":
    unittest.main()
