#!/usr/bin/env python3
"""Verify that ARM System Core owns calibration and runtime delays remain loops."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
DELAY = ROOT / "ARM/include/idelay.h"
CAL = ROOT / "ARM/include/system_core_clock_calibration.h"
APP = ROOT / "ARM/Nordic/nRF53/nRF5340_App/lib/src/system_nrf5340_application.c"


class DelayCalibrationDwt(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		cls.delay = DELAY.read_text(encoding="utf-8")
		cls.cal = CAL.read_text(encoding="utf-8")

	def test_runtime_delays_have_no_counter_dependency(self):
		src = self.delay
		start = src.index("static inline __attribute__((always_inline)) void usDelay(")
		end = src.index('"system_core_clock_calibration.h"', start)
		runtime = src[start:end]
		self.assertIn('" SUBS %[remaining], #1\\n"', runtime)
		self.assertIn('" BGT 1b\\n"', runtime)
		self.assertIn('" .p2align 5\\n"', runtime)
		for forbidden in ("DWT->", "CoreDebug->", "ArmDelayCalibrate(",
			"DelayCalibrationMeasure(", "NRF52_SERIES", "nrf52_errata"):
			self.assertNotIn(forbidden, src)

	def test_dwt_only_measures_factors_and_restores_state(self):
		cal = self.cal
		start = cal.index("static inline bool ArmDelayCalibrate(")
		end = cal.index("static inline bool DelayCalibrationMeasure(", start)
		src = cal[start:end]
		for term in (
			"#if ARM_DELAY_CAL_DWT_AVAILABLE",
			"prevMask = __get_PRIMASK();",
			"prevTrace = CoreDebug->DEMCR;",
			"prevCtrl = DWT->CTRL;",
			"valid = DWT->CYCCNT != start;",
			"usDelay(ARM_DELAY_CAL_LOOPS);",
			"nsDelay(ARM_DELAY_CAL_NS);",
			"SystemMicroSecLoopCnt = prevUsFactor;",
			"SystemnsDelayFactor = prevNsFactor;",
			"DWT->CTRL = prevCtrl;",
			"CoreDebug->DEMCR = prevTrace;",
			"__set_PRIMASK(prevMask);",
		):
			self.assertIn(term, src)
		for forbidden in ("TimerInit(", "RTC", "NRF52_SERIES",
			"ArmDelayCalibrationDwtAllowed"):
			self.assertNotIn(forbidden, src)

	def test_clock_changes_use_cached_factors(self):
		app = APP.read_text(encoding="utf-8")
		self.assertIn("ArmDelayClockUpdate(&s_DelayCalibration, cpuHz)", app)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration, cpuHz, 20U, 3U)", app)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration, cpuHz, 22U, 3U)", app)


if __name__ == "__main__":
	unittest.main()
