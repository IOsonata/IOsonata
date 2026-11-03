#!/usr/bin/env python3
"""Verify that nRF52840 BlueIOThingy board aliases resolve locally."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
TARGET = ROOT / "ARM" / "Nordic" / "nRF52"
BOARD = TARGET / "nRF52840" / "exemples" / "BlueIOThingy" / "src" / "board.h"
REFERENCE = TARGET / "nRF52832" / "exemples" / "BlueIOThingy" / "src" / "board.h"
EVIM = TARGET / "include" / "board_blueio_tag_evim.h"

def macros(path, section=None):
    text = path.read_text()
    if section == "evim":
        text = text.split("#elif BOARD == BLUEIO_TAG_EVIM", 1)[1]
        text = text.split("#elif BOARD == BLYST_NANO_DK", 1)[0]
    return dict(re.findall(r"(?m)^#define\s+(\w+)\s+([^\n]+)", text))


class BlueIoThingyBoardTests(unittest.TestCase):
    def test_uart_aliases_are_defined_in_both_board_variants(self):
        board, reference = macros(BOARD), macros(REFERENCE)
        for role in ("RX", "TX", "CTS", "RTS"):
            for field in ("PORT", "PIN", "PINOP"):
                macro = "BLUEIO_UART_" + role + "_" + field
                with self.subTest(macro=macro):
                    self.assertIn(macro, board)
                    self.assertEqual(board[macro].strip(), reference[macro].strip())

    def test_application_board_aliases_match_evim_reference(self):
        board, reference = macros(BOARD), macros(REFERENCE, "evim")
        roles = ("CONNECT_LED_PORT", "CONNECT_LED_PIN", "CONNECT_LED_LOGIC",
                 "LED_RED_PORT", "LED_RED_PIN", "LED_RED_PINOP", "LED_RED_LOGIC",
                 "LED_GREEN_PORT", "LED_GREEN_PIN", "LED_GREEN_PINOP", "LED_GREEN_LOGIC",
                 "LED_BLUE_PORT", "LED_BLUE_PIN", "LED_BLUE_PINOP", "LED_BLUE_LOGIC",
                 "I2C_DEVNO", "I2C_SDA_PORT", "I2C_SDA_PIN", "I2C_SDA_PINOP",
                 "I2C_SCL_PORT", "I2C_SCL_PIN", "I2C_SCL_PINOP",
                 "IMU_INT_PORT", "IMU_INT_PIN", "IMU_INT_PINOP",
                 "IMU_CS_PORT", "IMU_CS_PIN", "IMU_CS_PINOP", "SPI_PINS")
        for name in roles:
            with self.subTest(alias=name):
                self.assertIn(name, board)
                self.assertIn(name, reference)
                if name != "I2C_DEVNO":
                    self.assertEqual(board[name].strip(), reference[name].strip())

    def test_led_alias_uses_defined_logic(self):
        board = macros(BOARD)
        evim = macros(EVIM)
        self.assertEqual(board["BLUEIOTHINGY_CONNECT_LED_LOGIC"].strip(),
                         "BLUEIO_TAG_EVIM_LED1_LOGIC")
        self.assertIn("BLUEIO_TAG_EVIM_LED1_LOGIC", evim)
        self.assertNotIn("BLUEIOTHINGY_CONNECT_LED_ACTIVE", board)


if __name__ == "__main__":
    unittest.main()
