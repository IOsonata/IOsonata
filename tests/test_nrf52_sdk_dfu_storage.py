#!/usr/bin/env python3
"""Validate Nordic SDK DFU storage boundaries independently for BLE and USB."""

from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1] / "ARM" / "Nordic" / "nRF52"
LD = ROOT / "ldscript"
SIZE = 0x3000
CASES = (
    ("nRF52832", "dfu_ble", "gcc_nrf52832_xxaa_s132_sdk_dfu_boot.ld",
     "gcc_nrf52832_xxaa_s132_sdk_ble_dfu.ld", 0x77000),
    ("nRF52832", "dfu_uart", "gcc_nrf52832_xxaa_sdk_uart_dfu_boot.ld",
     "gcc_nrf52832_xxaa_s132_sdk_ble_dfu.ld", 0x77000),
    ("nRF52832", "dfu_uart_open", "gcc_nrf52832_xxaa_sdk_uart_open_dfu_boot.ld",
     "gcc_nrf52832_xxaa_s132_sdk_ble_dfu.ld", 0x77000),
    ("nRF52840", "dfu_ble", "gcc_nrf52840_xxaa_sdk_ble_dfu_boot.ld",
     "gcc_nrf52840_xxaa_s140_sdk_ble_dfu.ld", 0xF7000),
    ("nRF52840", "dfu_usb", "gcc_nrf52840_xxaa_sdk_usb_dfu_boot.ld",
     "gcc_nrf52840_xxaa_s140_sdk_usb_dfu.ld", 0xE0000),
    ("nRF52840", "dfu_usb_open", "gcc_nrf52840_xxaa_sdk_usb_open_dfu_boot.ld",
     "gcc_nrf52840_xxaa_s140_sdk_usb_dfu.ld", 0xE0000),
)


def memory(content, region):
    pat = (r"(?m)^\s*" + region +
           r"\s*\([^)]*\)\s*:\s*ORIGIN\s*=\s*(0x[0-9a-fA-F]+)" +
           r"\s*,\s*LENGTH\s*=\s*(0x[0-9a-fA-F]+)")
    match = re.search(pat, content)
    if match is None:
        raise AssertionError("Missing literal " + region + " region")
    return int(match.group(1), 16), int(match.group(2), 16)


class SdkDfuStorageTests(unittest.TestCase):
    def test_bootloader_protects_both_nvm_regions(self):
        for part, transport, boot_script, app_script, boot_start in CASES:
            with self.subTest(part=part, transport=transport):
                config = (ROOT / part / "exemples" / transport /
                          "src" / "sdk_config.h").read_text(encoding="utf-8")
                value = re.search(
                    r"(?m)^#define\s+NRF_DFU_APP_DATA_AREA_SIZE\s+(\d+)\s*$",
                    config,
                )
                self.assertIsNotNone(value)
                self.assertEqual(int(value.group(1)), SIZE)
                boot = (LD / boot_script).read_text(encoding="utf-8")
                application = (LD / app_script).read_text(encoding="utf-8")
                self.assertEqual(memory(boot, "FLASH")[0], boot_start)
                flash_start, flash_size = memory(application, "FLASH")
                nvm0, size0 = memory(application, "NVM0")
                nvm1, size1 = memory(application, "NVM1")
                self.assertEqual(size0, 0x2000)
                self.assertEqual(size1, 0x1000)
                self.assertEqual(flash_start + flash_size, nvm0)
                self.assertEqual(nvm0 + size0, nvm1)
                self.assertEqual(nvm1 + size1, boot_start)
                self.assertEqual(boot_start - SIZE, nvm0)
                for idx in (0, 1):
                    self.assertIn("__start_nvm%d = .;" % idx, application)
                    self.assertIn("__stop_nvm%d = .;" % idx, application)

    def test_representative_eclipse_release_dfu_projects(self):
        cases = (
            ("nRF52832", "Release_SDK_BLE_DFU",
             "gcc_nrf52832_xxaa_s132_sdk_ble_dfu.ld"),
            ("nRF52840", "Release_SDK_BLE_DFU",
             "gcc_nrf52840_xxaa_s140_sdk_ble_dfu.ld"),
            ("nRF52840", "Release_SDK_USB_DFU",
             "gcc_nrf52840_xxaa_s140_sdk_usb_dfu.ld"),
        )
        for part, name, script in cases:
            project = ROOT / part
            project /= "exemples/BleSecurityDemo/ioc/.cproject"
            xml = ET.parse(project)
            configs = xml.findall(".//cconfiguration")
            matches = [
                item for item in configs
                if item.find(".//storageModule[@name='%s']" % name)
                is not None
            ]
            with self.subTest(part=part, name=name):
                self.assertEqual(len(matches), 1)
                options = matches[0].findall(".//option")
                selected = [
                    value.get("value", "")
                    for option in options
                    if "linker.scriptfile" in option.get("superClass", "")
                    for value in option.findall("listOptionValue")
                ]
                self.assertEqual(len(selected), 2)
                self.assertTrue(all(script in item for item in selected))
                builder = matches[0].find(".//builder")
                self.assertIsNotNone(builder)
                self.assertTrue(builder.get("buildPath", "").endswith("/" + name))

    def test_ble_usb_maps_are_distinct(self):
        ble = next(item[4] for item in CASES
                   if item[0] == "nRF52840" and item[1] == "dfu_ble")
        usb = next(item[4] for item in CASES
                   if item[0] == "nRF52840" and item[1] == "dfu_usb")
        self.assertNotEqual(ble, usb)


if __name__ == "__main__":
    unittest.main()
