#!/usr/bin/env python3
"""Check Nordic SDK example sources do not duplicate IOsonata library objects."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "ARM" / "Nordic" / "nRF52" / "nRF52840"
EXAMPLES = ("UsbCdcAcmDemo", "UsbHidGeneric", "usbd_hid_composite")


def linked_source_names(path):
    tree = ET.parse(path)
    return {
        element.findtext("name").split("/")[-1]
        for element in tree.findall(".//linkedResources/link")
        if element.findtext("name") and
        element.findtext("name").endswith((".c", ".cpp"))
    }


class Nrf52840SdkSourceTests(unittest.TestCase):
    def test_no_source_objects_duplicated_from_iosonata_library(self):
        library = linked_source_names(TARGET / "lib" / "ioc" / ".project")
        for name in EXAMPLES:
            project = TARGET / "exemples" / name / "ioc" / ".project"
            linked = linked_source_names(project)
            with self.subTest(example=name):
                self.assertIn("main.cpp" if name != "usbd_hid_composite"
                              else "main.c", linked)
                self.assertFalse(linked & library,
                                 sorted(linked & library))

    def test_nordic_usb_class_sources_remain_linked(self):
        composite = linked_source_names(
            TARGET / "exemples" / "usbd_hid_composite" / "ioc" / ".project")
        for filename in ("app_usbd.c", "app_usbd_core.c",
                         "app_usbd_hid_kbd.c", "app_usbd_hid_mouse.c"):
            self.assertIn(filename, composite)


if __name__ == "__main__":
    unittest.main()
