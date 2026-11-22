#!/usr/bin/env python3
"""Check nRF5340 network-core delay factors and explicit calibration separation."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SYSTEM = ROOT / "ARM/Nordic/nRF53/nRF5340_Net/lib/src/system_nrf5340_network.c"
DELAY = ROOT / "ARM/include/idelay.h"
TEST = ROOT / "exemples/misc/core_clock_test.cpp"


class Nrf5340NetworkDelay(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		cls.system = SYSTEM.read_text(encoding="utf-8")
		cls.delay = DELAY.read_text(encoding="utf-8")
		cls.test = TEST.read_text(encoding="utf-8")

	def test_original_network_lfclk_default(self):
		self.assertIn(".LowPwrOsc = { OSC_TYPE_XTAL, 32768, 20, 0 }", self.system)
		self.assertIn("__WEAK McuOsc_t g_McuOsc", self.system)
		self.assertNotIn(".LowPwrOsc = { OSC_TYPE_RC, 32768", self.system)

	def test_fixed_frequency_network_uses_port_defaults(self):
		self.assertIn("23U, 10U", self.system)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration, SystemCoreClock, 23U, 10U)", self.system)
		self.assertNotIn("ArmDelayCalibrate(", self.system)
		self.assertNotIn("ArmDelayClockUpdate(&s_DelayCalibration", self.system)


	def test_explicit_dwt_and_timer_calibration_remain_available(self):
		self.assertIn("ArmDelayCalibrate(&measured, cpuHz)", self.delay)
		self.assertIn("DelayCalibrationMeasure(pSample)", self.test)
		self.assertIn("DelayCalibrationFromTimer(", self.test)

	def test_both_runtime_delays_use_original_countdowns(self):
		us_start = self.delay.index("static inline __attribute__((always_inline)) void ArmDelayUsLoop(")
		ns_start = self.delay.index("static inline __attribute__((always_inline)) void ArmDelayNsLoop(")
		us_loop = self.delay[us_start:ns_start]
		ns_loop = self.delay[ns_start:self.delay.index("// Calibration probe", ns_start)]
		self.assertEqual(us_loop.count('" NOP\\n"'), 10)
		for loop in (us_loop, ns_loop):
			self.assertIn('" MOVS %[remaining], %[count]\\n"', loop)
			self.assertIn('" SUBS %[remaining], #1\\n"', loop)
			self.assertIn('" BGT 1b\\n"', loop)
			self.assertIn("chunk = loops > INT32_MAX ? INT32_MAX", loop)
			self.assertNotIn('" BNE ', loop)
			self.assertNotIn("NRF5340", loop)
		self.assertIn('CoreClockTestResult("nsDelay repeated HF Timer 100 ms",', self.test)

	def test_250ms_nominal_loop_scale(self):
		for frequency in (64000000,):
			us = frequency // 23
			ns = frequency // 10
			us_loops = (250000 * us + 999999) // 1000000
			ns_loops = (250000000 * ns + 999999999) // 1000000000
			self.assertLess(abs(us_loops * 23 - frequency // 4), 5000)
			self.assertLess(abs(ns_loops * 10 - frequency // 4), 5000)
		self.assertIn("usDelay(ms * 1000U)", self.delay)


if __name__ == "__main__":
	unittest.main()
