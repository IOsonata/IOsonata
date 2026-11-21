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

	def test_default_factors_do_not_measure_at_startup(self):
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration,", self.system)
		self.assertIn("23U, 3U", self.system)
		self.assertNotIn("ArmDelayCalibrate(", self.system)
		self.assertIn("ArmDelayClockUpdate(&s_DelayCalibration,", self.system)

	def test_explicit_dwt_and_timer_calibration_remain_available(self):
		self.assertIn("ArmDelayCalibrate(&measured, cpuHz)", self.delay)
		self.assertIn("DelayCalibrationMeasure(pSample)", self.test)
		self.assertIn("DelayCalibrationFromTimer(", self.test)

	def test_250ms_nominal_loop_scale(self):
		for frequency in (64000000,):
			us = (frequency << 16) // (23 * 1000000)
			ns = (frequency << 16) // 3000000
			us_loops = (250000 * us + 65535) >> 16
			ns_loops = (250000000 * ns + 65535999) // 65536000
			self.assertLess(abs(us_loops * 23 - frequency // 4), 5000)
			self.assertLess(abs(ns_loops * 3 - frequency // 4), 5000)
		self.assertIn("usDelay(ms * 1000U)", self.delay)


if __name__ == "__main__":
	unittest.main()
