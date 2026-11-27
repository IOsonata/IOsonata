#!/usr/bin/env python3
"""Check generic CPU power modes and nRF5340 application clock scaling."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "include/coredev/system_core_clock.h"
APP = ROOT / "ARM/Nordic/nRF53/nRF5340_App/lib/src/system_nrf5340_application.c"
SHARED = ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp"
DEMO = ROOT / "exemples/misc/core_clock_test.cpp"


class RunningPowerMode(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		cls.header = HEADER.read_text(encoding="utf-8")
		cls.app = APP.read_text(encoding="utf-8")
		cls.shared = SHARED.read_text(encoding="utf-8")
		cls.demo = DEMO.read_text(encoding="utf-8")

	def test_generic_interface(self):
		for symbol in (
			"SYSTEM_POWER_MODE_PERFORMANCE",
			"SYSTEM_POWER_MODE_LOW_POWER",
			"bool SystemPowerModeSet(SYSTEM_POWER_MODE Mode);",
			"SYSTEM_POWER_MODE SystemPowerModeGet(void);",
		):
			self.assertIn(symbol, self.header)

	def test_application_starts_at_maximum_frequency(self):
		self.assertIn("#define __SYSTEM_CLOCK_MAX      (128000000UL)", self.app)
		self.assertIn("#define __SYSTEM_CLOCK_DEFAULT  (128000000UL)", self.app)
		init = self.app[self.app.index("void SystemInit(void)"): ]
		self.assertIn("NRF_APP_CLOCK->HFCLKCTRL = NRF_APP_HCLK_PERFORMANCE;", init)
		self.assertIn("SystemCoreClockUpdate();", self.app)

	def test_runtime_switch_preserves_peripheral_clock_ownership(self):
		start = self.app.index("bool SystemPowerModeSet(SYSTEM_POWER_MODE Mode)")
		end = self.app.index("SYSTEM_POWER_MODE SystemPowerModeGet(void)", start)
		body = self.app[start:end]
		self.assertIn("(NRF_APP_CLOCK->HFCLKCTRL & ~CLOCK_HFCLKCTRL_HCLK_Msk) | requested;", body)
		self.assertIn("SystemCoreClockUpdate();", body)
		self.assertIn("NRF_APP_HCLK_LOW_POWER", body)
		self.assertIn("NRF_APP_HCLK_PERFORMANCE", body)
		self.assertIn("__get_IPSR()", body)
		for banned in ("SystemSleep(", "SystemPowerOff(", "TimerInit(",
			"SystemPeriphClockSet(", "UARTPowerOff(", "SystemHighFreqClockDisable("):
			self.assertNotIn(banned, body)
		self.assertIn("CLOCK_HFCLKCTRL_HCLK_Div1", self.app)
		self.assertIn("CLOCK_HFCLKCTRL_HCLK_Div2", self.app)

	def test_other_nordic_cores_explicitly_reject_reduction(self):
		self.assertIn("#if !defined(NRF5340_XXAA_APPLICATION)", self.shared)
		self.assertIn("return Mode == SYSTEM_POWER_MODE_PERFORMANCE;", self.shared)

	def test_core_test_remains_architecture_neutral(self):
		self.assertIn("CoreClockTestPowerMode();", self.demo)
		start = self.demo.index("static void CoreClockTestPowerMode(void)")
		end = self.demo.index("static void CoreClockTestCalibrationTest(void)", start)
		body = self.demo[start:end]
		self.assertIn("CoreClockTestDelayCalibrationRecord();", body)
		self.assertLess(body.index("CoreClockTestDelayCalibrationRecord();"),
			body.index("SystemPowerModeSet(SYSTEM_POWER_MODE_PERFORMANCE);", 
				body.index("CoreClockTestDelayCalibrationRecord();")))
		self.assertIn("SYSTEM_POWER_MODE_LOW_POWER", self.demo)
		self.assertIn("SYSTEM_POWER_MODE_PERFORMANCE", self.demo)
		for term in ("NRF_CLOCK", "HFCLKCTRL", "__arm__", "__riscv", "DWT->"):
			self.assertNotIn(term, self.demo)


if __name__ == "__main__":
	unittest.main()
