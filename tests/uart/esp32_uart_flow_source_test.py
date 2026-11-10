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
        cls.irq = section(cls.source, "static bool Esp32UartTxFill(",
                          "/*---------------------------------------------------------------------------\n * IRQ entry points.")
        cls.transfer = section(cls.source, "static int Esp32UartRxData(",
                               "/*---------------------------------------------------------------------------\n * UARTInit")

    def test_interrupt_fifo_batch_reservation(self):
        self.assertIn("CFifoResvMultiple(udev->hRxFifo, &len)", self.irq)
        self.assertIn("CFifoPutMultiple(udev->hRxFifo, &len)", self.irq)
        self.assertIn("udev->RxDropCnt++", self.irq)
        self.assertIn("udev->RxOvrErrCnt++", self.irq)
        self.assertIn("udev->FramErrCnt++", self.irq)
        self.assertIn("udev->ParErrCnt++", self.irq)

        rx = section(self.irq, "if (st & (ESP32_UART_INT_RXFIFO_FULL",
                     "if (st & ESP32_UART_INT_TXFIFO_EMPTY)")
        self.assertLess(rx.index("CFifoResvMultiple(udev->hRxFifo, &len)"),
                        rx.index("p[i] = (uint8_t)ESP32_UART_FIFO_REG(base)"))
        self.assertLess(rx.index("p[i] = (uint8_t)ESP32_UART_FIFO_REG(base)"),
                        rx.index("CFifoPutMultiple(udev->hRxFifo, &len)"))

        # TX fill: queued bytes into the free hardware FIFO slots, true when
        # the CFIFO is empty
        tx = section(self.irq, "static bool Esp32UartTxFill(",
                     "static void Esp32UartIRQHandler(")
        self.assertLess(tx.index("c = CFifoGetByte(fifo);"),
                        tx.index("ESP32_UART_FIFO_REG(pDev->Base) = (uint8_t)c;"))
        self.assertIn("return true;", tx)
        self.assertIn("return CFifoUsed(fifo) == 0;", tx)

        # The interrupt turns TX empty off once the CFIFO is empty
        isr = section(self.irq, "if (st & ESP32_UART_INT_TXFIFO_EMPTY)",
                      "ESP32_UART_INT_CLR_REG(base) = ESP32_UART_INT_TXFIFO_EMPTY;")
        self.assertLess(isr.index("if (Esp32UartTxFill(pDev))"),
                        isr.index("ESP32_UART_INT_ENA_REG(base) &= ~ESP32_UART_INT_TXFIFO_EMPTY;"))
        self.assertIn("udev->bTxReady = true;", isr)

    def test_interrupt_latency_threshold_and_init_style(self):
        self.assertIn("(3U << ESP32_UART_CONF1_RXFIFO_FULL_THRHD_Pos)",
                      self.source)
        init = section(self.source, "bool UARTInit(", "void UARTSetCtrlLineState(")
        self.assertIn("ESP32_UARTDEV *dev;", init)
        self.assertIn("uint32_t cc;", init)
        self.assertIn("uint32_t conf0;", init)
        self.assertIn("int bn;", init)
        self.assertNotIn("uint32_t cc =", init)
        self.assertNotIn("uint32_t conf0 =", init)
        self.assertNotIn("int bn =", init)

    def test_rx_copies_with_interrupts_disabled(self):
        rx = section(self.transfer, "static int Esp32UartRxData(",
                     "static int Esp32UartPollTx(")
        self.assertLess(rx.index("state = DisableInterrupt();"),
                        rx.index("p = CFifoGetMultiple(fifo, &len);"))
        self.assertLess(rx.index("p = CFifoGetMultiple(fifo, &len);"),
                        rx.index("memcpy(pBuff + cnt, p, len);"))
        self.assertLess(rx.index("memcpy(pBuff + cnt, p, len);"),
                        rx.index("EnableInterrupt(state);"))

    def test_tx_queues_then_starts_arming_only_for_queued_bytes(self):
        put = section(self.transfer, "static int Esp32UartTxPut(",
                      "static int Esp32UartTxData(")
        self.assertLess(put.index("state = DisableInterrupt();"),
                        put.index("p = CFifoPutMultiple(udev->hTxFifo, &len);"))
        self.assertLess(put.index("memcpy(p, pData, len);"),
                        put.index("EnableInterrupt(state);"))
        self.assertIn("udev->TxDropCnt += udev->hTxFifo->DropCnt - dropped;", put)

        tx = self.transfer[self.transfer.index("static int Esp32UartTxData("):]
        self.assertIn("return Esp32UartPollTx(dev, pData, DataLen);", tx)
        self.assertIn("len = Esp32UartTxPut(dev, pData, DataLen);", tx)
        # CFIFO first, then start; the TX empty interrupt only for bytes the
        # hardware FIFO could not take
        self.assertLess(tx.index("CFifoPutByte(udev->hTxFifo, *pData)"),
                        tx.index("if (udev->bTxReady && !Esp32UartTxFill(dev))"))
        self.assertLess(tx.index("if (udev->bTxReady && !Esp32UartTxFill(dev))"),
                        tx.index("udev->bTxReady = false;"))
        self.assertLess(tx.index("udev->bTxReady = false;"),
                        tx.index("ESP32_UART_INT_ENA_REG(dev->Base) |= ESP32_UART_INT_TXFIFO_EMPTY;"))

    def test_entire_cpp_uses_tabs_ascii_no_library_prints(self):
        body = self.source.split("----------------------------------------------------------------------------*/", 1)[1]
        self.assertTrue(self.source.isascii())
        self.assertNotIn("printf(", body)
        self.assertNotIn("goto ", body)
        self.assertNotIn("for (int ", body)
        for number, line in enumerate(body.splitlines(), 1):
            with self.subTest(line=number):
                self.assertFalse(line.startswith("    "),
                                 "indent with tabs, not spaces")
                if not line.lstrip().startswith("//"):
                    self.assertNotRegex(line, r"\)\s+(return|break|continue)\b")
                    self.assertNotRegex(line, r"\b(if|while)\s*\(.*\)\s*\{\s*\}")

    def test_debug_globals_do_not_need_c_linkage(self):
        self.assertNotIn('extern "C" {\nvolatile uint32_t g_Esp32C3UartDbg',
                         self.source)
        # The interrupt counters are defined in C vectors and keep C ABI.
        self.assertIn('extern "C" volatile uint32_t g_Esp32C3Uart0IrqCount;',
                      self.source)


if __name__ == "__main__":
    unittest.main()
