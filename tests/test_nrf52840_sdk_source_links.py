#!/usr/bin/env python3
"""Check that SDK example projects do not compile sources already owned by IOsonata."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "ARM" / "Nordic" / "nRF52" / "nRF52840"
EXAMPLES = ("UsbCdcAcmDemo", "UsbHidGeneric",
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
                self.assertIn("main.cpp" if name in ("UsbCdcAcmDemo", "UsbHidGeneric")
                              else "main.c", linked.values())
                overlap = sorted(set(linked) & set(library))
                self.assertFalse(overlap, overlap)

    def test_usb_cpp_demos_compile_required_sdk_sources(self):
        expected = {
            "UsbCdcAcmDemo": ("app_usbd.c", "app_usbd_core.c",
                              "app_usbd_cdc_acm.c", "nrfx_usbd.c"),
            "UsbHidGeneric": ("app_usbd.c", "app_usbd_core.c",
                              "app_usbd_hid_generic.c",
                              "app_usbd_hid_mouse.c", "app_usbd_hid_kbd.c"),
        }
        for name, required in expected.items():
            folder = TARGET / "exemples" / name / "ioc"
            linked = linked_sources(folder / ".project")
            config_root = ET.parse(folder / ".cproject").getroot()
            configurations = config_root.findall(".//cconfiguration")
            self.assertEqual(len(configurations), 2)
            for config in configurations:
                title = config.find(".//configuration").get("name")
                exclusions = [
                    entry.get("excluding", "").split("|")
                    for entry in config.findall(".//sourceEntries/entry")
                    if entry.get("kind") == "sourcePath"
                ]
                ignored = {item for group in exclusions for item in group if item}
                with self.subTest(example=name, configuration=title):
                    self.assertTrue(set(required).issubset(linked.values()))
                    for src in linked:
                        if not src.startswith("external/"):
                            continue
                        source_name = next(
                            (k for k, v in linked.items() if k == src), None)
                        self.assertIsNotNone(source_name)
                    # Resolve workspace source names through the linked
                    # Eclipse resources. An excluded parent folder disables
                    # every C source beneath it.
                    project_tree = ET.parse(folder / ".project").getroot()
                    for resource in project_tree.findall(".//linkedResources/link"):
                        resource_name = resource.findtext("name", "")
                        if not resource_name.endswith(".c"):
                            continue
                        relative = resource_name.removeprefix("src/")
                        self.assertFalse(
                            any(relative == e or relative.startswith(e + "/")
                                for e in ignored),
                            (name, title, relative),
                        )

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
