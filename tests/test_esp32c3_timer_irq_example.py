#!/usr/bin/env python3
"""-------------------------------------------------------------------------
@file   test_esp32c3_timer_irq_example.py

@brief  Check ESP32-C3 dual Timer Group test project configuration.

@author IOsonata contributors
@date   Nov. 8, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

-------------------------------------------------------------------------"""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "RISCV/Espressif/ESP32C/ESP32C3/exemples/TimerIrqTest"
SOURCE = ROOT / "exemples/timer/timer_dual_irq_test.cpp"


class Esp32c3DualTimerExample(unittest.TestCase):
    def test_managed_project(self):
        project = ET.parse(EXAMPLE / "ioc/.project").getroot()
        self.assertEqual(project.findtext("name"), "TimerIrqTest")
        links = {
            x.findtext("name"): x.findtext("locationURI")
            for x in project.findall(".//linkedResources/link")
        }
        self.assertEqual(links.get("src/timer_dual_irq_test.cpp"),
                         "PARENT-7-PROJECT_LOC/exemples/timer/timer_dual_irq_test.cpp")
        self.assertEqual(links.get("src/board.h"),
                         "PARENT-1-PROJECT_LOC/src/board.h")
        config = ET.parse(EXAMPLE / "ioc/.cproject").getroot()
        configs = config.findall(".//configuration")
        self.assertTrue(any(x.get("name") == "Debug" for x in configs))
        self.assertTrue(any(x.get("name") == "Release" for x in configs))
        self.assertNotIn("CryptoTest", (EXAMPLE / "ioc/.cproject").read_text())

    def test_native_dual_timer_and_uart(self):
        code = SOURCE.read_text()
        self.assertIn("static Timer s_Timer0;", code)
        self.assertIn("static TimerDev_t s_Timer1", code)
        self.assertIn("initialized = s_Timer0.Init(config)", code)
        self.assertIn("initialized = TimerInit(&s_Timer1, &config)", code)
        self.assertIn("TIMER_TRIG_TYPE_CONTINUOUS", code)
        self.assertIn("TIMER_TRIG_TYPE_SINGLE", code)
        self.assertIn("TIMER_EVT_TRIGGER(0)", code)
        self.assertIn("UARTRetargetEnable(s_Uart, STDOUT_FILENO)", code)
        self.assertIn("TimerDisableTrigger(timers[i], 0)", code)
        self.assertIn("g_TimerUnexpectedEvents", code)
        self.assertNotIn("WS2812", code.replace("the ESP32-C3 DevKitM uses a WS2812 RGB LED", ""))

    def test_board_console(self):
        board = (EXAMPLE / "src/board.h").read_text()
        for word in (
            "#define UART_DEVNO",
            "#define UART_TX_PIN         21",
            "#define UART_RX_PIN         20",
            "ESP32_PINOP_U0TXD",
            "ESP32_PINOP_U0RXD",
        ):
            self.assertIn(word, board)


if __name__ == "__main__":
    unittest.main()
