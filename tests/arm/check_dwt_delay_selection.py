#!/usr/bin/env python3
"""Source and arithmetic regression for compile-time ARM DWT delays."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
DELAY = ROOT / "ARM/include/idelay.h"
APP = ROOT / "ARM/Nordic/nRF53/nRF5340_App/lib/src/system_nrf5340_application.c"


class DwtDelaySelection(unittest.TestCase):
	def test_fixed_selection_and_counter_enable(self):
		src = DELAY.read_text(encoding="utf-8")
		for text in ("#define ARM_DELAY_USE_DWT 1",
			"#define ARM_DELAY_USE_DWT 0", "defined(DWT)",
			"(__DWT_PRESENT == 1U)",
			"CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;",
			"DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;"):
			self.assertIn(text, src)

	def test_wrap_and_frequency_updates(self):
		src = DELAY.read_text(encoding="utf-8")
		self.assertIn("(uint32_t)(DWT->CYCCNT - start) < chunk", src)
		self.assertIn("chunk = Cycles > 0x40000000ULL ?", src)
		self.assertIn("((uint64_t)cnt * SystemCoreClock + 999999ULL) / 1000000ULL", src)
		self.assertIn("((uint64_t)cnt * SystemCoreClock + 999999999ULL) / 1000000000ULL", src)
		for hz in (16000000, 64000000, 128000000, 480000000):
			for us in (0, 1, 250000, 4294967295):
				cycles = (hz * us + 999999) // 1000000
				self.assertLess(cycles * 1000000 - hz * us, 1000000)
				remaining = cycles
				while remaining:
					chunk = min(remaining, 0x40000000)
					self.assertEqual((((0xfffffff0 + chunk) & 0xffffffff) -
						0xfffffff0) & 0xffffffff, chunk)
					remaining -= chunk

	def test_long_ns_wait_uses_calibrated_us_loop(self):
		src = DELAY.read_text(encoding="utf-8")
		start = src.index("static inline __attribute__((always_inline)) void nsDelay(")
		end = src.index("#define ARM_DELAY_CAL_NS", start)
		ns = src[start:end]
		self.assertIn("if (cnt >= 10000U)", ns)
		self.assertIn("ArmDelayUsLoop(cnt / 1000U);", ns)
		self.assertIn("cnt %= 1000U;", ns)
		self.assertIn("ArmDelayNsLoop(cnt);", ns)
		for nanoseconds in (10000, 10001, 100000000, 250000000):
			microseconds, remainder = divmod(nanoseconds, 1000)
			self.assertEqual(1000 * microseconds + remainder, nanoseconds)


	def test_loop_fallback_kept_separate(self):
		src = DELAY.read_text(encoding="utf-8")
		self.assertIn("void ArmDelayUsLoop(uint32_t cnt)", src)
		self.assertIn("void ArmDelayNsLoop(uint32_t cnt)", src)
		self.assertIn("ArmDelayUsLoop(cnt);", src)
		self.assertIn("ArmDelayNsLoop(cnt);", src)
		self.assertIn("ArmDelayUsLoop(ARM_DELAY_CAL_LOOPS);", src)
		self.assertIn("ArmDelayNsLoop(ARM_DELAY_CAL_NS);", src)
		app = APP.read_text(encoding="utf-8")
		self.assertIn("static ArmDelayCalibration_t s_DelayCalibration;", app)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration, cpuHz, 28U, 3U)", app)
		self.assertIn("ArmDelayClockUpdate(&s_DelayCalibration, cpuHz)", app)
		self.assertNotIn("#if !ARM_DELAY_USE_DWT\\nstatic ArmDelayCalibration_t", app)
		self.assertNotIn("#if !ARM_DELAY_USE_DWT\\n\\tif (s_DelayCalibration.UsCycles", app)


if __name__ == "__main__":
	unittest.main()
