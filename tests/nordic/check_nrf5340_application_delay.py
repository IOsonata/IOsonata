#!/usr/bin/env python3
"""Check nRF5340 application-core delay factors and explicit calibration separation."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SYSTEM = ROOT / "ARM/Nordic/nRF53/nRF5340_App/lib/src/system_nrf5340_application.c"
NETWORK = ROOT / "ARM/Nordic/nRF53/nRF5340_Net/lib/src/system_nrf5340_network.c"
DELAY = ROOT / "ARM/include/idelay.h"
TEST = ROOT / "exemples/misc/core_clock_test.cpp"


class Nrf5340ApplicationDelay(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		cls.system = SYSTEM.read_text(encoding="utf-8")
		cls.network = NETWORK.read_text(encoding="utf-8")
		cls.delay = DELAY.read_text(encoding="utf-8")
		cls.test = TEST.read_text(encoding="utf-8")

	def test_default_factors_are_preserved_at_default_frequency(self):
		self.assertIn("25U, 3U", self.system)
		self.assertIn("28U, 3U", self.system)
		self.assertIn("s_DelayCalibration.CpuHz != cpuHz", self.system)
		self.assertIn("cpuHz == __SYSTEM_CLOCK_DEFAULT", self.system)
		self.assertNotIn("ArmDelayCalibrate(", self.system)
		self.assertIn("NRF_APP_CLOCK->HFCLKCTRL & ~CLOCK_HFCLKCTRL_HCLK_Msk", self.system)
		self.assertIn("ArmDelayClockUpdate(&s_DelayCalibration, cpuHz)", self.system)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration, cpuHz, 25U, 3U)", self.system)


	def test_explicit_dwt_and_timer_calibration_remain_available(self):
		self.assertIn("ArmDelayCalibrate(&measured, cpuHz)", self.delay)
		self.assertIn("static ArmDelayCalibration_t s_DelayCalibration;", self.system)
		self.assertNotIn("#if !ARM_DELAY_USE_DWT\\nstatic ArmDelayCalibration_t", self.system)
		self.assertIn("DelayCalibrationMeasure(pSample)", self.test)
		self.assertIn("DelayCalibrationFromTimer(", self.test)

	def test_both_cores_are_dwt_free_at_runtime(self):
		for core in (self.system, self.network):
			self.assertNotIn("ArmDelayCalibrate(", core)
			self.assertNotIn("ArmDelayDwtInit(", core)
			self.assertNotIn("DWT->", core)
			self.assertIn("ArmDelayClockInit(", core)
		self.assertIn("ArmDelayClockUpdate(&s_DelayCalibration, cpuHz)", self.system)
		self.assertIn("23U, 3U", self.network)
		self.assertIn("28U, 3U", self.system)
		delay = self.delay
		start = delay.index("static inline __attribute__((always_inline)) void usDelay(")
		end = delay.index("#define ARM_DELAY_CAL_NS", start)
		forbidden = ("DWT->", "CoreDebug->", "ArmDelayCalibrate(")
		for name in forbidden:
			self.assertNotIn(name, delay[start:end])
		self.assertIn("ArmDelayUsLoop(cnt);", delay[start:end])

	def test_250ms_nominal_loop_scale(self):
		for frequency in (64000000, 128000000,):
			us_cycles = 25 if frequency == 128000000 else 28
			us = frequency // us_cycles
			ns = frequency // 3
			us_loops = (250000 * us + 999999) // 1000000
			ns_loops = (250000000 * ns + 999999999) // 1000000000
			self.assertLess(abs(us_loops * us_cycles - frequency // 4), 5000)
			self.assertLess(abs(ns_loops * 3 - frequency // 4), 5000)
		self.assertIn("usDelay(ms * 1000U)", self.delay)


if __name__ == "__main__":
	unittest.main()
