#!/usr/bin/env python3
"""Source-level checks for nRF UART RX ownership. Does not execute hardware."""

from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
UART = ROOT / "ARM/Nordic/src/uart_nrfx.cpp"


class UartRxSourceChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = UART.read_text(encoding="utf-8")

    def test_uarte_error_path_does_not_restart_active_rx(self):
        handler = self.source.split("static void UartIrqHandler(", 1)[1]
        errors = handler.split("// Handle errors", 1)[1]
        errors = errors.split("if (reg->EVENTS_CTS)", 1)[0]
        self.assertIn("NRFX_UARTE_TASKS_STOPRX(dev->pDmaReg) = 1;", errors)
        self.assertNotIn("NRFX_UARTE_TASKS_STARTRX", errors)

    def test_dma_end_owns_rearm(self):
        complete = self.source.split("static void nRFUARTRxDmaEnd(", 1)[1]
        complete = complete.split("static void nRFUARTPollRx(", 1)[0]
        self.assertIn("NRFX_UARTE_EVENTS_ENDRX", complete)
        self.assertIn("NRFX_UARTE_TASKS_STARTRX", complete)

    def test_interrupt_dma_read_does_not_stop_partial_rx(self):
        rx = self.source.split("static int nRFUARTRxDataDma(", 1)[1]
        rx = rx.split("static void nRFUARTStopRx(", 1)[0]
        self.assertIn("if (pDev->bIntEn == false)", rx)
        self.assertIn("NRFX_UARTE_TASKS_STOPRX(dev->pDmaReg) = 1;", rx)
        self.assertNotIn("pDev->bDma == false", rx)

    def test_polling_rx_still_reads_hardware(self):
        rx = self.source.split("static int nRFUARTRxData(", 1)[1]
        rx = rx.split("static int nRFUARTRxDataDma(", 1)[0]
        self.assertIn("if (pDev->bIntEn == false)", rx)
        self.assertIn("pBuff[cnt] = (uint8_t)dev->pReg->RXD;", rx)
        self.assertNotIn("NRFX_UARTE_TASKS_STOPRX", rx)

    def test_rx_callback_selected_by_dma_mode(self):
        init = self.source.split("bool UARTInit(", 1)[1]
        self.assertIn("if (pDev->DevIntrf.bDma)", init)
        self.assertIn("pDev->DevIntrf.RxData = nRFUARTRxDataDma;", init)
        self.assertIn("pDev->DevIntrf.RxData = nRFUARTRxData;", init)


if __name__ == "__main__":
    unittest.main()
