"""DFU boot images must retain their flash partitions in Debug builds."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PROJECTS = (
    "ARM/Nordic/nRF52/nRF52840/exemples/DfuBoot/ioc/.cproject",
    "ARM/Nordic/nRF53/nRF5340_Net/exemples/DfuBoot/ioc/.cproject",
    "ARM/Renesas/RE01/RE01_1500KB/exemples/DfuBoot/ioc/.cproject",
)
OPTIMIZE_SIZE = "ilg.gnuarmeclipse.managedbuild.cross.option.optimization.level.size"


class DfuBootDebugSizeTest(unittest.TestCase):
    def test_debug_lto_for_overflowing_boot_images(self):
        for project in PROJECTS[1:]:
            with self.subTest(project=project):
                root = ET.parse(ROOT / project).getroot()
                configs = root.findall(".//cconfiguration")
                self.assertEqual(len(configs), 2)
                debug = next(
                    cfg for cfg in configs
                    if cfg.find(".//configuration").get("name") == "Debug"
                )
                for suffix in ("C Compiler", "C++ Compiler", "C Linker", "C++ Linker"):
                    matches = [
                        tool for tool in debug.findall(".//tool")
                        if tool.get("name", "").endswith(suffix)
                        and "Assembler" not in tool.get("name", "")
                    ]
                    self.assertEqual(len(matches), 1)
                    name = ("Other compiler flags" if "Compiler" in suffix
                            else "Other linker flags")
                    values = [
                        opt.get("value", "")
                        for opt in matches[0].findall("option")
                        if opt.get("name") == name
                    ]
                    self.assertEqual(len(values), 1)
                    self.assertIn("-flto", values[0].split())

    def test_boot_debug_uses_size_optimization(self):
        for project in PROJECTS:
            with self.subTest(project=project):
                root = ET.parse(ROOT / project).getroot()
                configs = root.findall(".//configuration")
                self.assertEqual({cfg.get("name") for cfg in configs},
                                 {"Debug", "Release"})
                for cfg in configs:
                    options = [
                        option.get("value", "")
                        for option in cfg.findall(".//option")
                        if option.get("name") == "Optimization Level"
                    ]
                    self.assertEqual(options, [OPTIMIZE_SIZE])


if __name__ == "__main__":
    unittest.main()
