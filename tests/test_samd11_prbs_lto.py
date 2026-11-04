"""SAMD11 PRBS examples use link-time optimization without enlarging flash."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "ARM/Microchip/SAMD/SAMD11/exemples"
PROJECTS = ("UartPrbsRxTest", "UartPrbsTxTestTaktOS", "UartSlipPrbsTxTest")


class Samd11PrbsLtoTest(unittest.TestCase):
    def test_prbs_examples_compile_and_link_with_lto(self):
        for project in PROJECTS:
            with self.subTest(project=project):
                root = ET.parse(EXAMPLES / project / "ioc/.cproject").getroot()
                configs = root.findall(".//cconfiguration")
                self.assertEqual(len(configs), 2)
                for cfg in configs:
                    configuration = cfg.find(".//configuration")
                    self.assertIn(configuration.get("name"), ("Debug", "Release"))
                    for suffix in ("C Compiler", "C++ Compiler", "C Linker", "C++ Linker"):
                        tool_name = "GNU ARM Cross " + suffix
                        tools = [
                            tool for tool in cfg.findall(".//tool")
                            if tool.get("name") == tool_name
                        ]
                        self.assertEqual(len(tools), 1)
                        flag_name = ("Other compiler flags" if "Compiler" in suffix
                                     else "Other linker flags")
                        options = [
                            option.get("value", "")
                            for option in tools[0].findall("option")
                            if option.get("name") == flag_name
                        ]
                        self.assertEqual(len(options), 1)
                        self.assertIn("-flto", options[0].split())


if __name__ == "__main__":
    unittest.main()
