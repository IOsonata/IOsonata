#!/usr/bin/env python3
"""Check the Thingy52 SDK reference example's existing board linker selection."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "ARM" / "Nordic" / "nRF52" / "nRF52832" / "exemples"
PROJECT = EXAMPLES / "Thingy52" / "ioc" / ".cproject"
SCRIPT = ROOT / "ARM" / "Nordic" / "nRF52" / "ldscript" / "gcc_nrf52832_xxaa_s132_thingy.ld"


class Thingy52ProjectTests(unittest.TestCase):
    def test_sdk_button_include_path(self):
        tree = ET.parse(PROJECT).getroot()
        configs = tree.findall(".//cconfiguration")
        self.assertEqual(len(configs), 2)
        for config in configs:
            name = config.find(".//configuration").get("name")
            with self.subTest(configuration=name):
                compilers = [
                    tool for tool in config.findall(".//tool")
                    if tool.get("name", "").endswith(("C Compiler", "C++ Compiler"))
                ]
                self.assertEqual(len(compilers), 2)
                for tool in compilers:
                    paths = [
                        value.get("value", "")
                        for option in tool.findall("option")
                        if option.get("name") == "Include paths (-I)"
                        for value in option.findall("listOptionValue")
                    ]
                    self.assertTrue(any(
                        path.endswith("external/nRF5_SDK/components/libraries/button&quot;")
                        for path in paths
                    ))

    def test_debug_release_linker_script_exists(self):
        self.assertTrue(SCRIPT.is_file())
        tree = ET.parse(PROJECT).getroot()
        configs = tree.findall(".//cconfiguration")
        self.assertEqual(len(configs), 2)
        names = set()
        for config in configs:
            name = config.find(".//configuration").get("name")
            names.add(name)
            values = [
                node.get("value", "").strip('"')
                for option in config.iter("option")
                if "linker.scriptfile" in option.get("superClass", "")
                for node in option.findall("listOptionValue")
            ]
            with self.subTest(configuration=name):
                self.assertEqual(values, [
                    "../../../../../ldscript/gcc_nrf52832_xxaa_s132_thingy.ld"
                ] * 2)
                self.assertEqual((PROJECT.parent / name / values[0]).resolve(),
                                 SCRIPT.resolve())
        self.assertEqual(names, {"Debug", "Release"})


if __name__ == "__main__":
    unittest.main()
