#!/usr/bin/env python3
"""Guard Debug flash-demo linkage against semihosting syscall collisions."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class FlashMemoryDemoSpecsTests(unittest.TestCase):
    def test_debug_does_not_link_rdimon(self):
        for chip in ("STM32L476", "STM32L496"):
            with self.subTest(chip=chip):
                path = ROOT / "ARM" / "ST" / "STM32L4xx" / chip / "exemples" / "FlashMemoryDemo" / "ioc" / ".cproject"
                configs = ET.parse(path).findall(".//cconfiguration")
                self.assertEqual(len(configs), 2)
                for cfg in configs:
                    name = cfg.find(".//configuration").get("name")
                    linkers = [
                        tool for tool in cfg.findall(".//tool")
                        if tool.get("name") == "GNU ARM Cross C++ Linker"
                    ]
                    self.assertEqual(len(linkers), 1)
                    options = [
                        opt.get("value", "")
                        for opt in linkers[0].findall("option")
                        if opt.get("name") == "Other linker flags"
                    ]
                    self.assertEqual(len(options), 1)
                    if name == "Debug":
                        self.assertIn("--specs=nosys.specs", options[0])
                    self.assertNotIn("rdimon", options[0])


if __name__ == "__main__":
    unittest.main()
