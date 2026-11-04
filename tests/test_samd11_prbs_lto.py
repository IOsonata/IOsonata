"""SAMD11 PRBS examples use link-time optimization without enlarging flash."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "ARM/Microchip/SAMD/SAMD11/exemples"
PROJECTS = ("UartPrbsRxTest", "UartPrbsTxTestTaktOS", "UartSlipPrbsTxTest")


class Samd11PrbsLtoTest(unittest.TestCase):
    def test_slip_release_omits_progress_printf(self):
        project = EXAMPLES / "UartSlipPrbsTxTest/ioc/.cproject"
        root = ET.parse(project).getroot()
        configs = root.findall(".//cconfiguration")
        self.assertEqual(len(configs), 2)
        for cfg in configs:
            name = cfg.find(".//configuration").get("name")
            values = [
                value.get("value", "")
                for tool in cfg.findall(".//tool")
                if tool.get("name", "").endswith(("C Compiler", "C++ Compiler"))
                for option in tool.findall("option")
                if option.get("name") == "Defined symbols (-D)"
                for value in option.findall("listOptionValue")
            ]
            count = values.count("SLIP_PRBS_NO_PROGRESS")
            self.assertEqual(count, 2 if name == "Release" else 0)
        source = (ROOT / "exemples/uart/uart_slip_prbs_tx.cpp").read_text()
        self.assertEqual(source.count("#ifndef SLIP_PRBS_NO_PROGRESS"), 2)
        self.assertIn("SlipTx(&g_SlipDev, buff, SLIPTEST_BUFSIZE);", source)

    def test_mcu_library_debug_optimizes_for_size(self):
        project = ROOT / "ARM/Microchip/SAMD/SAMD11/lib/ioc/.cproject"
        root = ET.parse(project).getroot()
        configs = root.findall(".//configuration")
        self.assertEqual({cfg.get("name") for cfg in configs},
                         {"Debug", "Release"})
        for cfg in configs:
            with self.subTest(config=cfg.get("name")):
                levels = [
                    option.get("value", "")
                    for option in cfg.findall(".//option")
                    if option.get("name") == "Optimization Level"
                ]
                self.assertEqual(levels, [
                    "ilg.gnuarmeclipse.managedbuild.cross.option.optimization.level.size"
                ])

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
