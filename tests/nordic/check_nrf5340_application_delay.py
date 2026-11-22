#!/usr/bin/env python3
"""Check nRF5340 application-core delay factors and explicit calibration separation."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SYSTEM = ROOT / "ARM/Nordic/nRF53/nRF5340_App/lib/src/system_nrf5340_application.c"
NETWORK = ROOT / "ARM/Nordic/nRF53/nRF5340_Net/lib/src/system_nrf5340_network.c"
DELAY = ROOT / "ARM/include/idelay.h"
CAL = ROOT / "ARM/include/system_core_clock_calibration.h"
TEST = ROOT / "exemples/misc/core_clock_test.cpp"


class Nrf5340ApplicationDelay(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		cls.system = SYSTEM.read_text(encoding="utf-8")
		cls.network = NETWORK.read_text(encoding="utf-8")
		cls.delay = DELAY.read_text(encoding="utf-8")
		cls.cal = CAL.read_text(encoding="utf-8")
		cls.test = TEST.read_text(encoding="utf-8")

	def test_default_factors_are_preserved_at_default_frequency(self):
		self.assertIn("20U, 3U", self.system)
		self.assertIn("22U, 3U", self.system)
		self.assertIn("s_DelayCalibration.CpuHz != cpuHz", self.system)
		self.assertIn("cpuHz == __SYSTEM_CLOCK_DEFAULT", self.system)
		self.assertNotIn("ArmDelayCalibrate(", self.system)
		self.assertIn("NRF_APP_CLOCK->HFCLKCTRL & ~CLOCK_HFCLKCTRL_HCLK_Msk", self.system)
		self.assertIn("ArmDelayClockUpdate(&s_DelayCalibration, cpuHz)", self.system)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration, cpuHz, 20U, 3U)", self.system)


	def test_hf_timer_calibration_without_dwt(self):
		self.assertIn("static inline bool ArmDelayCalibrate(", self.cal)
		self.assertIn("DWT->CYCCNT", self.cal)
		self.assertIn("static ArmDelayCalibration_t s_DelayCalibration;", self.system)
		self.assertIn("DelayCalibrationMeasure(pSample)", self.test)
		self.assertIn("DelayCalibrationFromTimer(", self.cal)

	def test_both_cores_are_dwt_free_at_runtime(self):
		for core in (self.system, self.network):
			self.assertNotIn("ArmDelayCalibrate(", core)
			self.assertNotIn("ArmDelayDwtInit(", core)
			self.assertNotIn("DWT->", core)
			self.assertIn("ArmDelayClockInit(", core)
		self.assertIn("ArmDelayClockUpdate(&s_DelayCalibration, cpuHz)", self.system)
		self.assertIn("23U, 3U", self.network)
		self.assertIn("22U, 3U", self.system)
		delay = self.delay
		start = delay.index("static inline __attribute__((always_inline)) void usDelay(")
		end = delay.index("// The high-frequency Timer reports recommended", start)
		for name in ("DWT->", "CoreDebug->", "ArmDelayCalibrate("):
			self.assertNotIn(name, delay[start:end])

	def test_nsdelay_owns_its_assembly(self):
		start = self.delay.index("static inline __attribute__((always_inline)) void nsDelay(")
		end = self.delay.index("// The high-frequency Timer reports recommended", start)
		runtime = self.delay[start:end]
		self.assertIn('" SUBS %[remaining], #1\n"', runtime)
		self.assertIn('" BGT 1b\n"', runtime)
		self.assertNotIn("ArmDelayNsLoop(", self.delay)
		self.assertNotIn("ArmDelayUsLoop(", self.delay)
		self.assertIn("nsDelay(100000000U);", self.test)
		self.assertIn("nsDelay(250000000U);", self.test)

	def test_both_runtime_delays_use_original_countdowns(self):
		us_start = self.delay.index("static inline __attribute__((always_inline)) void usDelay(")
		ns_start = self.delay.index("static inline __attribute__((always_inline)) void nsDelay(")
		us_loop = self.delay[us_start:ns_start]
		ns_loop = self.delay[ns_start:self.delay.index("// The high-frequency Timer reports recommended", ns_start)]
		self.assertEqual(us_loop.count('" NOP\n"'), 10)
		for loop in (us_loop, ns_loop):
			self.assertIn('" MOVS %[remaining], %[count]\n"', loop)
			self.assertIn('" SUBS %[remaining], #1\n"', loop)
			self.assertIn('" BGT 1b\n"', loop)
			self.assertIn("chunk = loops > INT32_MAX ? INT32_MAX", loop)
			self.assertNotIn('" BNE ', loop)
			self.assertNotIn("NRF5340", loop)
		self.assertIn('CoreClockTestResult("nsDelay repeated HF Timer 100 ms",', self.test)
		self.assertIn('" .p2align 5\\n"', ns_loop)

	def test_250ms_nominal_loop_scale(self):
		for frequency in (64000000, 128000000,):
			us_cycles = 20 if frequency == 128000000 else 22
			us = frequency // us_cycles
			ns = frequency // 3
			us_loops = (250000 * us + 999999) // 1000000
			ns_loops = (250000000 * ns + 999999999) // 1000000000
			self.assertLess(abs(us_loops * us_cycles - frequency // 4), 5000)
			self.assertLess(abs(ns_loops * 3 - frequency // 4), 5000)
		self.assertIn("usDelay(ms * 1000U)", self.delay)


if __name__ == "__main__":
	unittest.main()
