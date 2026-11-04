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
