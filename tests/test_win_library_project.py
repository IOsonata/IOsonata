#!/usr/bin/env python3
"""Check Win MinGW managed-project include paths."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "Win" / "lib" / "ioc" / ".cproject"


class WinLibraryIncludeTests(unittest.TestCase):
    def test_win_sources_do_not_require_proprietary_invensense_sdk(self):
        linked = {
            link.findtext("name", "")
            for link in ET.parse(PROJECT.with_name(".project")).findall(".//linkedResources/link")
        }
        for source in (
            "src/motion/mot_invn_icm20948.cpp",
            "src/sensors/agm_invn_icm20948.cpp",
        ):
            self.assertNotIn(source, linked)

    def test_win_hid_header_uses_case_sensitive_mingw_include(self):
        header = ROOT / "Win" / "include" / "usb_hidhost_impl.h"
        content = header.read_text()
        self.assertIn("#include <windows.h>", content)
        self.assertNotIn('#include "Windows.h"', content)

    def test_debug_and_release_include_paths(self):
        root = ET.parse(PROJECT).getroot()
        configs = {}
        for config in root.findall(".//cconfiguration"):
            storage = config.find("./storageModule")
            name = storage.get("name") if storage is not None else None
            if name not in ("Debug", "Release"):
                continue
            paths = []
            for option in config.iter("option"):
                if option.get("valueType") != "includePath":
                    continue
                paths.extend(value.get("value", "").strip('"')
                             for value in option.findall("listOptionValue"))
            configs[name] = paths

        self.assertEqual(set(configs), {"Debug", "Release"})
        for name, paths in configs.items():
            with self.subTest(configuration=name):
                self.assertEqual(paths, [
                    "../../../include",
                    "../../../../include",
                    "../../../include",
                    "../../../../include",
                ])
                build = PROJECT.parent / name
                for offset, expected in (
                    ("../../../include", ROOT / "Win" / "include"),
                    ("../../../../include", ROOT / "include"),
                ):
                    self.assertEqual((build / offset).resolve(), expected.resolve())
                    self.assertTrue(expected.is_dir(), str(expected))


if __name__ == "__main__":
    unittest.main()
