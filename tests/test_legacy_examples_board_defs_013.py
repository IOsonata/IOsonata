#!/usr/bin/env python3
"""Check board definitions used by the release 0.13 legacy examples."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


class LegacyExampleBoardTests(unittest.TestCase):
    def test_ra4m1_usb_keyboard_gpio_map(self):
        path = ROOT / "ARM" / "Renesas" / "RA4" / "RA4M1" / "exemples" / "UsbHidKeyboard" / "src" / "board.h"
        text = path.read_text()
        defines = dict(re.findall(r"(?m)^#define\s+(HID_\w+)\s+([^\s/]+)", text))
        self.assertEqual(defines, {
            "HID_BUTTON_PORT": "1",
            "HID_BUTTON_PIN": "0",
            "HID_BUTTON_PINOP": "IOPINOP_GPIO",
            "HID_CAPS_BUTTON_PORT": "1",
            "HID_CAPS_BUTTON_PIN": "1",
            "HID_CAPS_BUTTON_PINOP": "IOPINOP_GPIO",
            "HID_LED_PORT": "1",
            "HID_LED_PIN": "11",
            "HID_LED_PINOP": "IOPINOP_GPIO",
        })
        self.assertIn("#define USB_PINS", text)

    def test_l4_flash_demo_sector_geometry(self):
        common = (ROOT / "exemples" / "storage" / "flash_memory_demo.cpp").read_text()
        self.assertIn("buff[FLASH_SECTOR_SIZE]", common)
        header = (ROOT / "include" / "storage" / "flash.h").read_text()
        self.assertIn("#define FLASH_MX25R6435F_SECTSIZE", header)
        for part in ("STM32L476", "STM32L496"):
            with self.subTest(part=part):
                path = ROOT / "ARM" / "ST" / "STM32L4xx" / part / "exemples" / "FlashMemoryDemo" / "src" / "board.h"
                board = path.read_text()
                self.assertRegex(board, r"(?m)^#define\s+FLASH_SECTOR_SIZE\s+FLASH_MX25R6435F_SECTSIZE\s*$")
                self.assertIn("#define FLASH_CFG", board)


if __name__ == "__main__":
    unittest.main()
