#!/usr/bin/env python3
"""-------------------------------------------------------------------------
@file   esp32_uart_flow_source_test.py

@brief  Check ESP32 UART FIFO transfer source and coding conventions.

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

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "RISCV/Espressif/src/uart_esp32.cpp"


def section(text, start, end):
    i = text.index(start)
    j = text.index(end, i)
    return text[i:j]


class Esp32UartFlowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text()
        cls.irq = section(cls.source, "static void Esp32UartTxFill(",
                          "/*---------------------------------------------------------------------------\n * IRQ entry points.")
        cls.transfer = section(cls.source, "static int Esp32UartRxData(",
                               "/*---------------------------------------------------------------------------\n * UARTInit")

    def test_interrupt_fifo_byte_helpers(self):
        self.assertIn("CFifoPutByte(udev->hRxFifo", self.irq)
        self.assertIn("CFifoGetByte(pDev->pUartDev->hTxFifo)", self.irq)
        self.assertNotIn("CFifoPut(udev->hRxFifo", self.irq)
        self.assertNotIn("CFifoGet(udev->hTxFifo", self.irq)
        self.assertIn("udev->RxDropCnt++", self.irq)
        self.assertIn("udev->RxOvrErrCnt++", self.irq)
        self.assertIn("udev->FramErrCnt++", self.irq)
        self.assertIn("udev->ParErrCnt++", self.irq)

    def test_blocking_rx_copies_before_releasing(self):
        rx = section(self.transfer, "static int Esp32UartRxData(",
                     "static int Esp32UartTxData(")
        self.assertLess(rx.index("CFifoPeekMultiple(fifo, &len)"),
                        rx.index("memcpy(pBuff + cnt, p, len)"))
        self.assertLess(rx.index("memcpy(pBuff + cnt, p, len)"),
                        rx.index("CFifoRelease(fifo, len)"))
        self.assertIn("if (CFifoIsBlocking(fifo))", rx)
        self.assertIn("CFifoGetMultiple(fifo, &len)", rx)
        self.assertIn("state = DisableInterrupt();", rx)

    def test_tx_writes_hw_fifo_before_software_queue(self):
        tx = self.transfer[self.transfer.index("static int Esp32UartTxData("):]
        self.assertLess(tx.index("ESP32_UART_FIFO_REG(dev->Base) = *pData;"),
                        tx.index("p = CFifoPutMultiple(fifo, &len);"))
        self.assertIn("Esp32UartTxFill(dev);", tx)
        self.assertIn("if (pending || (udev->EvtCallback != nullptr &&", tx)
        self.assertIn("ESP32_UART_INT_ENA_REG(dev->Base) &=", tx)
        self.assertNotIn("TxDropCnt +=", tx)

    def test_modified_cpp_uses_tabs_ascii_no_library_prints(self):
        for body in (self.irq, self.transfer):
            self.assertTrue(body.isascii())
            self.assertNotIn("printf(", body)
            self.assertNotIn("goto ", body)
            self.assertNotIn("for (int ", body)
            for number, line in enumerate(body.splitlines(), 1):
                with self.subTest(line=number):
                    self.assertFalse(line.startswith("    "),
                                     "indent with tabs, not spaces")

    def test_debug_globals_do_not_need_c_linkage(self):
        self.assertNotIn('extern "C" {\nvolatile uint32_t g_Esp32C3UartDbg',
                         self.source)
        # The interrupt counters are defined in C vectors and keep C ABI.
        self.assertIn('extern "C" volatile uint32_t g_Esp32C3Uart0IrqCount;',
                      self.source)


if __name__ == "__main__":
    unittest.main()
