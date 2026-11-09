#!/usr/bin/env python3
"""Static checks for the ESP32C6 Eclipse managed example configurations."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
FAMILY = ROOT / "RISCV" / "Espressif" / "ESP32C"
LINKER = FAMILY / "ldscript" / "gcc_esp32c6.ld"


class Esp32C6ProjectTests(unittest.TestCase):
    def test_complete_isa_and_abi_in_every_configuration(self):
        prefix = "ilg.gnumcueclipse.managedbuild.cross.riscv.option."
        expected = {
            "target.isa.base": prefix + "target.arch.rv32i",
            "target.isa.multiply": "true",
            "target.isa.compressed": "true",
            "target.isa.atomic": "false",
            "target.isa.extensions": "zicsr",
            "target.abi.integer": prefix + "abi.integer.ilp32",
        }
        for name in ("lib", "exemples/LedDemo", "exemples/CryptoTest"):
            project = FAMILY / "ESP32C6" / name / "ioc" / ".cproject"
            configs = ET.parse(project).findall(".//cconfiguration")
            self.assertEqual(len(configs), 2)
            for config in configs:
                with self.subTest(project=str(project), config=config.get("id")):
                    toolchain = config.find(".//toolChain")
                    options = toolchain.findall("option")
                    for suffix, value in expected.items():
                        found = [o.get("value") for o in options
                                 if o.get("superClass") == prefix + suffix]
                        self.assertEqual(found, [value], suffix)

    def test_led_sources_and_matching_library(self):
        project = FAMILY / "ESP32C6/exemples/LedDemo/ioc"
        links = {link.findtext("name"): link.findtext("locationURI")
                 for link in ET.parse(project / ".project").findall(".//link")}
        self.assertEqual(links, {
            "src": "virtual:/virtual",
            "src/led_demo.cpp": "PARENT-7-PROJECT_LOC/exemples/misc/led_demo.cpp",
            "src/board.h": "PARENT-1-PROJECT_LOC/src/board.h",
        })
        self.assertTrue((ROOT / "exemples/misc/led_demo.cpp").is_file())
        for cfg in ET.parse(project / ".cproject").findall(".//cconfiguration"):
            config = cfg.find(".//configuration")
            name = config.get("name")
            build = project / name
            with self.subTest(configuration=name):
                self.assertEqual(config.get("artifactExtension"), "elf")
                self.assertIn('-O binary', config.get("postbuildStep"))
                entry = config.find("sourceEntries/entry")
                self.assertEqual(entry.get("name"), "src")
                self.assertIsNone(entry.get("excluding"))
                tc = cfg.find(".//toolChain")
                self.assertEqual(tc.find("builder").get("buildPath"),
                                 "${workspace_loc:/LedDemo}/" + name)
                for tool in tc.findall("tool"):
                    options = tool.findall("option")
                    for option in options:
                        if option.get("valueType") == "includePath":
                            for item in option.findall("listOptionValue"):
                                self.assertTrue((build / item.get("value").strip('"')).resolve().is_dir())
                    if not tool.get("superClass", "").endswith("linker"):
                        continue
                    for suffix, expected in (
                        ("linker.libs", "IOsonata_ESP32C6"),
                        ("linker.paths", '"../../../../lib/ioc/' + name + '"'),
                    ):
                        found = [item.get("value") for opt in options
                                 if opt.get("superClass", "").endswith(suffix)
                                 for item in opt.findall("listOptionValue")]
                        self.assertEqual(found, [expected])
                    for spec in ("usenewlibnano", "usenewlibnosys"):
                        found = [opt.get("value") for opt in options
                                 if opt.get("superClass", "").endswith("linker." + spec)]
                        self.assertEqual(found, ["true"])

    def test_led_and_crypto_target_settings(self):
        self.assertTrue(LINKER.is_file())
        for name in ("LedDemo", "CryptoTest"):
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
