#!/usr/bin/env python3
"""Validate Linux Eclipse managed-library source selection."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "Linux" / "lib" / ".project"
CONFIG = ROOT / "Linux" / "lib" / ".cproject"


class LinuxLibraryProjectTests(unittest.TestCase):
    def test_newlib_runtime_override_not_built_for_glibc(self):
        source = ROOT / "src" / "CppRuntimeOverload.cpp"
        text = source.read_text()
        self.assertIn("#include <reent.h>", text)
        self.assertIn("_malloc_r(", text)
        linked = {
            link.findtext("name", "")
            for link in ET.parse(PROJECT).findall(".//linkedResources/link")
        }
        self.assertNotIn("src/CppRuntimeOverload.cpp", linked)
        for path in (
            "src/device.cpp",
            "src/device_intrf.cpp",
            "src/uart.cpp",
            "src/usb/usb_hidhost.cpp",
            "src/stddev.c",
        ):
            with self.subTest(path=path):
                self.assertIn(path, linked)

    def test_all_four_linux_configurations_retained(self):
        cfgs = {
            item.find(".//configuration").get("name")
            for item in ET.parse(CONFIG).findall(".//cconfiguration")
        }
        self.assertEqual(cfgs, {"Debug", "Release", "Debug_RPI", "Release_RPI"})


if __name__ == "__main__":
    unittest.main()
