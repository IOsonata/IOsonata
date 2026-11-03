#!/usr/bin/env python3
"""Check that SDK example projects do not compile sources already owned by IOsonata."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "ARM" / "Nordic" / "nRF52" / "nRF52840"
EXAMPLES = ("UsbCdcAcmDemo", "UsbHidGeneric", "UsbHidKeyboard",
            "UartCliDemo", "usbd_hid_composite", "cli",
            "usb_cdc_acm", "usbd_hid_generic")


def linked_sources(path):
    tree = ET.parse(path)
    result = {}
    for entry in tree.findall(".//linkedResources/link"):
        name = entry.findtext("name", "")
        location = entry.findtext("locationURI", "")
        if not name.endswith((".c", ".cpp")):
            continue
        # Compare the canonical external SDK path, not just a basename:
        # different SDK sources may legitimately have the same basename.
        if "external/" in location:
            key = "external/" + location.split("external/", 1)[1]
        else:
            key = location
        result[key] = name.split("/")[-1]
    return result


class Nrf52840SdkSourceTests(unittest.TestCase):
    def test_no_source_objects_duplicated_from_iosonata_library(self):
        library = linked_sources(TARGET / "lib" / "ioc" / ".project")
        for name in EXAMPLES:
            project = TARGET / "exemples" / name / "ioc" / ".project"
            linked = linked_sources(project)
            with self.subTest(example=name):
                self.assertIn("main.cpp" if name in ("UsbCdcAcmDemo", "UsbHidGeneric", "UsbHidKeyboard")
                              else "main.c", linked.values())
                overlap = sorted(set(linked) & set(library))
                self.assertFalse(overlap, overlap)

    def test_nordic_usb_class_sources_remain_linked(self):
        for example, required in (
            ("usbd_hid_composite", ("app_usbd.c", "app_usbd_core.c",
                                     "app_usbd_hid_kbd.c", "app_usbd_hid_mouse.c")),
            ("usbd_hid_generic", ("app_usbd.c", "app_usbd_hid_generic.c")),
            ("usb_cdc_acm", ("app_usbd.c", "app_usbd_cdc_acm.c")),
        ):
            linked = linked_sources(TARGET / "exemples" / example / "ioc" / ".project")
            with self.subTest(example=example):
                self.assertTrue(set(required).issubset(linked.values()))


if __name__ == "__main__":
    unittest.main()
