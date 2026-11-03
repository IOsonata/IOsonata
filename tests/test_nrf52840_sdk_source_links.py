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
                # These SDK sources intentionally need direct inclusion:
                # CLI commands register via linker sections, while the
                # precompiled library logging objects are disabled.
                cli = "external/nRF5_SDK/components/libraries/cli/cli_utils_cmds.c"
                logs = {
                    "external/nRF5_SDK/components/libraries/log/src/nrf_log_frontend.c",
                    "external/nRF5_SDK/components/libraries/log/src/nrf_log_backend_flash.c",
                    "external/nRF5_SDK/components/libraries/log/src/nrf_log_str_formatter.c",
                }
                allowed = ({cli} if name in ("UartCliDemo", "cli", "usb_cdc_acm") else set())
                if name == "cli":
                    allowed |= logs
                if name == "UartCliDemo":
                    allowed.add("external/nRF5_SDK/components/libraries/cli/nrf_cli.c")
                self.assertEqual(set(overlap), allowed, overlap)

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

    def test_uart_cli_builtin_commands_sources_are_linked(self):
        project = TARGET / "exemples" / "UartCliDemo" / "ioc" / ".project"
        linked = linked_sources(project)
        self.assertIn("nrf_cli.c", linked.values())
        self.assertIn("fnmatch.c", linked.values())
        self.assertIn("cli_utils_cmds.c", linked.values())

    def test_usb_cdc_demo_build_configuration_uses_supported_sdk_sources(self):
        folder = TARGET / "exemples" / "UsbCdcAcmDemo" / "ioc"
        project = ET.parse(folder / ".project")
        config = ET.parse(folder / ".cproject")
        links = {
            entry.findtext("name"): entry.findtext("locationURI", "")
            for entry in project.findall(".//linkedResources/link")
        }
        self.assertEqual(links["src/main.cpp"], "PARENT-1-PROJECT_LOC/src/main.cpp")
        disabled = {
            "nRF5_SDK/nrfx/soc/nrfx_atomic.c",
            "nRF5_SDK/nrfx/legacy/nrf_drv_uart.c",
            "nRF5_SDK/nrfx/drivers/src/nrfx_uart.c",
            "nRF5_SDK/nrfx/drivers/src/nrfx_uarte.c",
            "nRF5_SDK/nrfx/drivers/src/prs/nrfx_prs.c",
        }
        for cfg in config.findall(".//cconfiguration"):
            name = cfg.find(".//configuration").get("name")
            exclusions = {
                item
                for entry in cfg.findall(".//sourceEntries/entry")
                for item in entry.get("excluding", "").split("|")
            }
            definitions = [
                item.get("value", "")
                for option in cfg.findall(".//option")
                if option.get("name", "").startswith("Defined symbols")
                for item in option.findall("listOptionValue")
            ]
            with self.subTest(configuration=name):
                self.assertTrue(disabled.issubset(exclusions))
                self.assertEqual(definitions.count("BOARD_PCA10056"), 2)

    def test_usb_cdc_release_has_no_softdevice_or_dfu_definitions(self):
        folder = TARGET / "exemples" / "UsbCdcAcmDemo" / "ioc"
        configurations = ET.parse(folder / ".cproject")
        for cfg in configurations.findall(".//cconfiguration"):
            name = cfg.find(".//configuration").get("name")
            definitions = [
                item.get("value", "")
                for option in cfg.findall(".//option")
                if option.get("name", "").startswith("Defined symbols")
                for item in option.findall("listOptionValue")
            ]
            scripts = [
                item.get("value", "").strip('"')
                for option in cfg.findall(".//option")
                if option.get("name", "").startswith("Script files")
                for item in option.findall("listOptionValue")
            ]
            with self.subTest(configuration=name):
                for symbol in ("SOFTDEVICE_PRESENT", "S140",
                               "BLE_STACK_SUPPORT_REQD", "BL_SETTINGS_ACCESS_ONLY",
                               "NRF_DFU_SETTINGS_VERSION=1"):
                    self.assertNotIn(symbol, definitions)
                self.assertTrue(scripts)
                self.assertTrue(all(path.endswith("gcc_nrf52840_xxaa.ld")
                                    for path in scripts))

    def test_usb_cdc_cpp_backend_is_linked(self):
        project = TARGET / "exemples" / "UsbCdcAcmDemo" / "ioc" / ".project"
        linked = linked_sources(project)
        self.assertIn("usbd_cdc_intrf.cpp", linked.values())

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
