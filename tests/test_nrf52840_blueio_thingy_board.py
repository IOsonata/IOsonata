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

def macros(path):
    return dict(re.findall(r"(?m)^#define\s+(\w+)\s+([^\n]+)", path.read_text()))


class BlueIoThingyBoardTests(unittest.TestCase):
    def test_uart_aliases_are_defined_in_both_board_variants(self):
        board, reference = macros(BOARD), macros(REFERENCE)
        for role in ("RX", "TX", "CTS", "RTS"):
            for field in ("PORT", "PIN", "PINOP"):
                macro = "BLUEIO_UART_" + role + "_" + field
                with self.subTest(macro=macro):
                    self.assertIn(macro, board)
                    self.assertEqual(board[macro].strip(), reference[macro].strip())

    def test_led_alias_uses_defined_logic(self):
        board = macros(BOARD)
        evim = macros(EVIM)
        self.assertEqual(board["BLUEIOTHINGY_CONNECT_LED_LOGIC"].strip(),
                         "BLUEIO_TAG_EVIM_LED1_LOGIC")
        self.assertIn("BLUEIO_TAG_EVIM_LED1_LOGIC", evim)
        self.assertNotIn("BLUEIOTHINGY_CONNECT_LED_ACTIVE", board)


if __name__ == "__main__":
    unittest.main()
