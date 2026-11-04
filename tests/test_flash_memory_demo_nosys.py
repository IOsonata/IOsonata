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
                    options = [
                        opt.get("value", "")
                        for opt in cfg.findall(".//option")
                        if opt.get("name") == "Other linker flags"
                    ]
                    if name == "Debug":
                        self.assertEqual(len(options), 1)
                        self.assertIn("--specs=nosys.specs", options[0])
                        self.assertNotIn("rdimon", options[0])
                    else:
                        self.assertFalse(any("rdimon" in v for v in options))


if __name__ == "__main__":
    unittest.main()
