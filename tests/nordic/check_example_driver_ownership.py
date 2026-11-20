#!/usr/bin/env python3
"""Check that generic sleep and clock demos use driver interfaces.

These checks detect source-level regressions. Hardware TX completion, sleep,
and wake behavior require separate target testing.
"""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
CLOCK = ROOT / "exemples/misc/clock_demo.cpp"
POWER = ROOT / "exemples/misc/power_off_demo.cpp"
I2C = ROOT / "exemples/i2c/i2c_slave_demo.cpp"


class DemoDriverOwnership(unittest.TestCase):
	def test_clock_demo(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertNotIn("hTxFifo", source)
		self.assertNotIn("hRxFifo", source)
		self.assertNotIn("CFifoUsed(", source)
		start = source.index("static void ClockDemoFlush(void)\n{")
		end = source.index("\n}", start)
		self.assertNotIn("UARTRx(", source[start:end])
		self.assertIn("msDelay(2);", source[start:end])

	def test_power_off_demo(self):
		source = POWER.read_text(encoding="utf-8")
		self.assertNotIn("hTxFifo", source)
		self.assertNotIn("hRxFifo", source)
		self.assertNotIn("SysLogGet()->hFifo", source)
		self.assertNotIn("s_Uart.Rx(&unused", source)
		self.assertNotIn("s_Uart.Rx(&dummy", source)
		self.assertNotIn("timer_sam4l.h", source)
		self.assertNotIn("SAM4L_PM->", source)
		self.assertNotIn("SAM4L_AST->", source)
		self.assertNotIn("SAM4L_BPM->", source)
		self.assertNotIn("Sam4lUartSleepDiagnostic", source)
		self.assertIn("SysLogFlush(SysLogGet())", source)

	def test_i2c_sleep_demo(self):
		source = I2C.read_text(encoding="utf-8")
		self.assertNotIn("hTxFifo", source)
		self.assertNotIn("bTxReady", source)
		self.assertNotIn("g_Uart.Disable();", source)
		self.assertNotIn("g_Uart.Enable();", source)
		self.assertIn("SystemSleep();", source)
		self.assertIn("fflush(stdout);", source)

	def test_no_private_uart_casts(self):
		for path in (CLOCK, POWER, I2C):
			with self.subTest(path=path.name):
				source = path.read_text(encoding="utf-8")
				self.assertNotRegex(source, r"UARTDev_t\s*\*\s*(?:pUart|uart)\s*=")


if __name__ == "__main__":
	unittest.main()
