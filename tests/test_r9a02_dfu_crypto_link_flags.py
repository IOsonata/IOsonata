"""Guard R9A02 GNU linker options without expanding the DFU partition."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "RISCV/Renesas/R9A02/R9A02G021/exemples"


class R9A02LinkFlagsTest(unittest.TestCase):
    def linker_flags(self, example):
        root = ET.parse(EXAMPLES / example / "ioc/.cproject").getroot()
        results = {}
        for config in root.findall(".//configuration"):
            flags = [
                option.get("value", "")
                for option in config.findall(".//option")
                if option.get("name") == "Other linker flags"
            ]
            self.assertEqual(len(flags), 2)
            results[config.get("name")] = flags
        self.assertEqual(set(results), {"Debug", "Release"})
        return results

    def test_constrained_debug_builds_optimize_for_size(self):
        for example in ("DfuBoot", "CryptoTest"):
            with self.subTest(example=example):
                root = ET.parse(EXAMPLES / example / "ioc/.cproject").getroot()
                configs = root.findall(".//configuration")
                self.assertEqual({cfg.get("name") for cfg in configs},
                                 {"Debug", "Release"})
                for cfg in configs:
                    options = [
                        option.get("value", "")
                        for option in cfg.findall(".//option")
                        if option.get("name") == "Optimization Level"
                    ]
                    self.assertEqual(options, [
                        "ilg.gnumcueclipse.managedbuild.cross.riscv.option.optimization.level.size"
                    ])

    def test_r9a02_debug_lto_is_enabled_in_compilers_and_linkers(self):
        for example in ("DfuBoot", "CryptoTest"):
            with self.subTest(example=example):
                root = ET.parse(EXAMPLES / example / "ioc/.cproject").getroot()
                configs = root.findall(".//cconfiguration")
                debug = next(
                    cfg for cfg in configs
                    if cfg.find(".//configuration").get("name") == "Debug"
                )
                tools = debug.findall(".//tool")
                for kind in ("C Compiler", "C++ Compiler", "C Linker", "C++ Linker"):
                    matches = [
                        tool for tool in tools
                        if tool.get("name") == "GNU RISC-V Cross " + kind
                    ]
                    self.assertEqual(len(matches), 1)
                    flags_name = ("Other compiler flags"
                                  if "Compiler" in kind else "Other linker flags")
                    options = [
                        opt.get("value", "")
                        for opt in matches[0].findall("option")
                        if opt.get("name") == flags_name
                    ]
                    self.assertEqual(len(options), 1)
                    self.assertIn("-flto", options[0].split())

    def test_release_dfu_boot_uses_shared_register_save_helpers(self):
        root = ET.parse(EXAMPLES / "DfuBoot/ioc/.cproject").getroot()
        release = next(
            cfg for cfg in root.findall(".//cconfiguration")
            if cfg.find(".//configuration").get("name") == "Release"
        )
        for suffix in ("C Compiler", "C++ Compiler"):
            tools = [
                tool for tool in release.findall(".//tool")
                if tool.get("name") == "GNU RISC-V Cross " + suffix
            ]
            self.assertEqual(len(tools), 1)
            options = [
                option.get("value", "")
                for option in tools[0].findall("option")
                if option.get("name") == "Other compiler flags"
            ]
            self.assertEqual(len(options), 1)
            self.assertIn("-msave-restore", options[0].split())

    def test_dfu_boot_omits_unwind_tables(self):
        root = ET.parse(EXAMPLES / "DfuBoot/ioc/.cproject").getroot()
        for cfg in root.findall(".//cconfiguration"):
            name = cfg.find(".//configuration").get("name")
            with self.subTest(config=name):
                for suffix in ("C Compiler", "C++ Compiler"):
                    tools = [
                        tool for tool in cfg.findall(".//tool")
                        if tool.get("name") == "GNU RISC-V Cross " + suffix
                    ]
                    self.assertEqual(len(tools), 1)
                    flags = [
                        option.get("value", "")
                        for option in tools[0].findall("option")
                        if option.get("name") == "Other compiler flags"
                    ]
                    self.assertEqual(len(flags), 1)
                    self.assertIn("-fno-unwind-tables", flags[0].split())
                    self.assertIn("-fno-asynchronous-unwind-tables", flags[0].split())

    def test_dfu_boot_never_forces_semihost_archive(self):
        flags = self.linker_flags("DfuBoot")
        for config, options in flags.items():
            with self.subTest(config=config):
                for value in options:
                    self.assertNotIn("semihost", value)
                    self.assertNotIn("--whole-archive", value)

    def test_crypto_debug_keeps_semihost_without_whole_archive(self):
        flags = self.linker_flags("CryptoTest")
        for value in flags["Debug"]:
            self.assertIn("--specs=semihost.specs", value)
            self.assertIn("-lsemihost", value)
            self.assertNotIn("--whole-archive", value)
        for value in flags["Release"]:
            self.assertNotIn("semihost", value)


if __name__ == "__main__":
    unittest.main()
