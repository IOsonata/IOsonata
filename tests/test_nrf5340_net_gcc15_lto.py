#!/usr/bin/env python3
"""Work around GCC 15.2.1 lto1 ICE only for three affected NetCore releases."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
NET = ROOT / "ARM" / "Nordic" / "nRF53" / "nRF5340_Net"


class NetworkCoreGcc15Tests(unittest.TestCase):
    def test_release_links_native_code_from_fat_lto_archive(self):
        lib = (NET / "lib" / "ioc" / ".cproject").read_text()
        self.assertIn("-flto -ffat-lto-objects", lib)
        for example in ("BleAdvertiser", "BleIntrfPrbsTx", "BleOta"):
            with self.subTest(example=example):
                path = NET / "exemples" / example / "ioc" / ".cproject"
                for cfg in ET.parse(path).findall(".//cconfiguration"):
                    configuration = cfg.find(".//configuration")
                    name = configuration.get("name")
                    flags = [
                        opt.get("value", "")
                        for opt in cfg.findall(".//option")
                        if opt.get("name") == "Other linker flags"
                    ]
                    self.assertEqual(len(flags), 2)
                    for value in flags:
                        with self.subTest(configuration=name):
                            if name == "Release":
                                self.assertIn("-fno-lto", value)
                                self.assertIn("-fno-use-linker-plugin", value)
                            else:
                                self.assertNotIn("-fno-lto", value)
                                self.assertNotIn("-fno-use-linker-plugin", value)


if __name__ == "__main__":
    unittest.main()
