#!/usr/bin/env python3
"""Verify DWT is used only for temporary ARM delay-loop calibration."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
DELAY = ROOT / "ARM/include/idelay.h"
NRF52 = ROOT / "ARM/Nordic/nRF52/src/system_nrf52.c"
APP = ROOT / "ARM/Nordic/nRF53/nRF5340_App/lib/src/system_nrf5340_application.c"


class DelayCalibrationDwt(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		cls.source = DELAY.read_text(encoding="utf-8")

	def test_runtime_delay_never_uses_dwt(self):
		src = self.source
		start = src.index("static inline __attribute__((always_inline)) void usDelay(")
		end = src.index("#define ARM_DELAY_CAL_NS", start)
		runtime = src[start:end]
		self.assertIn("ArmDelayUsLoop(cnt);", runtime)
		self.assertIn("ArmDelayUsLoop(cnt / 1000U);", runtime)
		self.assertIn("ArmDelayNsLoop(cnt);", runtime)
		for forbidden in ("DWT->", "CoreDebug->", "ArmDelayDwtInit(",
			"ArmDelayDwtWait(", "ARM_DELAY_CAL_DWT_AVAILABLE"):
			self.assertNotIn(forbidden, runtime)

	def test_calibration_restores_trace_counter_interrupt_and_factors(self):
		src = self.source
		cal = src[src.index("static inline bool ArmDelayCalibrate("):
			src.index("static inline bool DelayCalibrationMeasure(")]
		for term in (
			"#if ARM_DELAY_CAL_DWT_AVAILABLE",
			"prevMask = __get_PRIMASK();",
			"prevTrace = CoreDebug->DEMCR;",
			"CoreDebug->DEMCR = prevTrace | CoreDebug_DEMCR_TRCENA_Msk;",
			"prevCtrl = DWT->CTRL;",
			"DWT->CTRL = prevCtrl | DWT_CTRL_CYCCNTENA_Msk;",
			"valid = DWT->CYCCNT != start;",
			"ArmDelayUsLoop(ARM_DELAY_CAL_LOOPS);",
			"ArmDelayNsLoop(ARM_DELAY_CAL_NS);",
			"SystemMicroSecLoopCnt = prevUsFactor;",
			"SystemnsDelayFactor = prevNsFactor;",
			"DWT->CTRL = prevCtrl;",
			"CoreDebug->DEMCR = prevTrace;",
			"__set_PRIMASK(prevMask);",
		):
			self.assertIn(term, cal)
		self.assertNotIn("TimerInit(", cal)
		self.assertNotIn("RTC", cal)
		self.assertNotIn("ArmDelayDwtWait(", src)

	def test_nrf52_erratum_guard_and_no_startup_dwt(self):
		src = NRF52.read_text(encoding="utf-8")
		self.assertIn("bool ArmDelayCalibrationDwtAllowed(void)", src)
		self.assertIn("return !nrf52_errata_32();", src)
		self.assertNotIn("ArmDelayDwtInit(", src)
		cal = self.source[self.source.index("static inline bool ArmDelayCalibrate("):]
		self.assertIn("if (!ArmDelayCalibrationDwtAllowed())", cal)

	def test_frequency_changes_keep_fallback_updated(self):
		app = APP.read_text(encoding="utf-8")
		self.assertIn("ArmDelayClockUpdate(&s_DelayCalibration, cpuHz)", app)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration, cpuHz, 28U, 3U)", app)

	def test_long_nanosecond_delay_uses_microsecond_loop(self):
		src = self.source
		start = src.index("static inline __attribute__((always_inline)) void nsDelay(")
		end = src.index("#define ARM_DELAY_CAL_NS", start)
		ns = src[start:end]
		self.assertIn("if (cnt >= 10000U)", ns)
		self.assertIn("cnt %= 1000U;", ns)
		for ns_count in (10000, 10001, 100000000, 250000000):
			us, remainder = divmod(ns_count, 1000)
			self.assertEqual(us * 1000 + remainder, ns_count)


if __name__ == "__main__":
	unittest.main()
