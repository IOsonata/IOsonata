#!/usr/bin/env python3
"""Check nRF52832 UART CLI SDK source ownership."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
FAMILY = ROOT / "ARM" / "Nordic" / "nRF52" / "nRF52832"
LIBRARY = FAMILY / "lib" / "ioc" / ".project"
PROJECT = FAMILY / "exemples" / "UartCliDemo" / "ioc" / ".project"


def linked_source_paths(file):
    tree = ET.parse(file)
    sources = {}
    for link in tree.findall(".//linkedResources/link"):
        name = link.findtext("name", "")
        location = link.findtext("locationURI", "")
        if not name.endswith((".c", ".cpp")):
            continue
        key = ("external/" + location.split("external/", 1)[1]
               if "external/" in location else location)
        sources[key] = name
    return sources


class Nrf52832UartCliTests(unittest.TestCase):
    def test_sdk_sources_not_compiled_twice(self):
        library = linked_source_paths(LIBRARY)
        project = linked_source_paths(PROJECT)
        self.assertIn("src/main.c", project.values())
        overlap = sorted(set(library) & set(project))
        self.assertEqual(overlap, [], overlap)
        self.assertIn("src/nRF5_SDK/libraries/mpu/nrf_mpu_lib.c",
                      project.values())
        self.assertIn("src/nRF5_SDK/libraries/stack_guard/nrf_stack_guard.c",
                      project.values())


if __name__ == "__main__":
    unittest.main()
