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

    def test_source_folder_exclusions_resolve_to_linked_resources(self):
        # Eclipse sourceEntries paths are relative to their source folder.
        linked = {
            link.findtext("name", "").removeprefix("src/")
            for link in ET.parse(PROJECT).findall(".//linkedResources/link")
            if link.findtext("name", "").startswith("src/")
        }
        excluded_sources = {
            "storage/diskio_flash.cpp",
            "coredev/timer.cpp",
            "storage/seep_impl.cpp",
            "storage/sdcard_impl.cpp",
            "storage/fatfs_vdisk.cpp",
            "storage/fatfs.cpp",
            "storage/diskio_impl.cpp",
        }
        self.assertTrue(excluded_sources.issubset(linked))

        for cfg in ET.parse(CONFIG).findall(".//cconfiguration"):
            config = cfg.find(".//configuration")
            name = config.get("name")
            entries = [
                entry for entry in cfg.findall(".//sourceEntries/entry")
                if entry.get("kind") == "sourcePath" and entry.get("name") == "src"
            ]
            with self.subTest(configuration=name):
                self.assertEqual(len(entries), 1)
                excludes = set(entries[0].get("excluding", "").split("|"))
                self.assertTrue(excluded_sources.issubset(excludes))
                self.assertNotIn("fatfs.cpp", excludes)
                self.assertNotIn("seep_impl.cpp", excludes)
                self.assertNotIn("fats_vdisk.c", excludes)

    def test_all_four_linux_configurations_retained(self):
        cfgs = {
            item.find(".//configuration").get("name")
            for item in ET.parse(CONFIG).findall(".//cconfiguration")
        }
        self.assertEqual(cfgs, {"Debug", "Release", "Debug_RPI", "Release_RPI"})


if __name__ == "__main__":
    unittest.main()
