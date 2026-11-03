#!/usr/bin/env python3
"""Static checks for the ESP32C6 Eclipse managed example configurations."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
FAMILY = ROOT / "RISCV" / "Espressif" / "ESP32C"
LINKER = FAMILY / "ldscript" / "gcc_esp32c6.ld"


class Esp32C6ProjectTests(unittest.TestCase):
    def test_blinky_and_crypto_target_settings(self):
        self.assertTrue(LINKER.is_file())
        for name in ("Blinky", "CryptoTest"):
            project = (FAMILY / "ESP32C6" / "exemples" / name
                       / "ioc" / ".cproject")
            tree = ET.parse(project).getroot()
            configs = tree.findall(".//cconfiguration")
            self.assertEqual(len(configs), 2)
            for config in configs:
                with self.subTest(project=name, configuration=config.get("id")):
                    toolchain = config.find(".//toolChain")
                    self.assertIsNotNone(toolchain)
                    isa = [opt.get("value") for opt in toolchain.findall("option")
                           if opt.get("superClass", "").endswith("target.isa.extensions")]
                    self.assertEqual(isa, ["zicsr"])
                    for suffix in ("c.compiler", "cpp.compiler"):
                        compiler = next((tool for tool in toolchain.findall("tool")
                            if tool.get("superClass", "").endswith(".tool." + suffix)), None)
                        self.assertIsNotNone(compiler)
                        defines = [item.get("value") for option in compiler.findall("option")
                            if option.get("valueType") == "definedSymbols"
                            for item in option.findall("listOptionValue")]
                        self.assertIn("ESP32C6", defines)
                    selected = [item.get("value", "").strip('"')
                        for option in config.iter("option")
                        if option.get("superClass", "").endswith("linker.scriptfile")
                        for item in option.findall("listOptionValue")]
                    self.assertEqual(selected,
                        ["../../../../../ldscript/gcc_esp32c6.ld"] * 2)
                    build = project.parent / ("Debug" if "debug" in config.get("id", "") else "Release")
                    self.assertEqual((build / selected[0]).resolve(), LINKER.resolve())


if __name__ == "__main__":
    unittest.main()
