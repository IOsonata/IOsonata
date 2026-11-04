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
