#!/usr/bin/env python3
"""Check nRF5340 delay factor measurement without replacing the delay loops."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
CAL = ROOT / "ARM/include/idelay_calibration.h"
CLOCK = ROOT / "ARM/include/idelay_clock.h"
DELAY = ROOT / "ARM/include/idelay.h"
NET = ROOT / "ARM/Nordic/nRF53/nRF5340_Net/lib/src/system_nrf5340_network.c"
APP = ROOT / "ARM/Nordic/nRF53/nRF5340_App/lib/src/system_nrf5340_application.c"


class Nrf53DelayCalibration(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		cls.cal = CAL.read_text(encoding="utf-8")
		cls.clock = CLOCK.read_text(encoding="utf-8")
		cls.delay = DELAY.read_text(encoding="utf-8")
		cls.net = NET.read_text(encoding="utf-8")
		cls.app = APP.read_text(encoding="utf-8")

	def test_uses_real_cpu_cycle_measurements(self):
		self.assertIn("DWT->CYCCNT - start", self.cal)
		self.assertIn("CoreDebug_DEMCR_TRCENA_Msk", self.cal)
		self.assertIn("DWT_CTRL_CYCCNTENA_Msk", self.cal)
		self.assertIn("usDelay(ARM_DELAY_CAL_LOOPS)", self.cal)
		self.assertIn("nsDelay(ARM_DELAY_CAL_NS)", self.cal)
		self.assertIn("((uint64_t)pCal->UsCycles * 1000000ULL)", self.clock)
		self.assertIn("((uint64_t)pCal->NsCycles * 1000000ULL)", self.clock)
		self.assertIn("SystemMicroSecLoopCnt = prevUsFactor;", self.cal)
		self.assertIn("SystemnsDelayFactor = prevNsFactor;", self.cal)
		self.assertIn("__set_PRIMASK(prevMask)", self.cal)

	def test_both_cores_calibrate_from_hardware(self):
		for source in (self.app, self.net):
			self.assertIn('#include "idelay_calibration.h"', source)
			self.assertIn("ArmDelayCalibrate(", source)
			self.assertNotIn(" / 23000000UL", source)
			self.assertNotIn(" / 28000000UL", source)
			self.assertNotIn(" / 3000000UL", source)
		self.assertIn("s_DelayCalibration.CpuHz != cpuHz", self.app)
		self.assertIn("ArmDelayClockUpdate(&s_DelayCalibration, cpuHz)", self.app)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration", self.app)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration", self.net)
		self.assertIn("ArmDelayClockUpdate(&s_DelayCalibration", self.net)
		self.assertEqual(self.app.count("ArmDelayCalibrate("), 1)
		self.assertEqual(self.net.count("ArmDelayCalibrate("), 1)

	def test_delay_implementation_is_not_replaced(self):
		self.assertIn("static inline __attribute__((always_inline)) void usDelay(", self.delay)
		self.assertIn("static inline __attribute__((always_inline)) void nsDelay(", self.delay)
		self.assertIn('" SUBS %[low], #1\\n"', self.delay)
		self.assertIn('" BNE 1b\\n"', self.delay)

	def test_fixed_point_factor_math(self):
		for cpu_hz in (64000000, 128000000):
			for us_loop_cycles, ns_loop_cycles in ((13, 3), (23, 3), (28, 3)):
				count = 8192
				us_cycles = count * us_loop_cycles
				ns_cycles = count * ns_loop_cycles
				scale = cpu_hz * count * 65536
				us_factor = scale // (us_cycles * 1000000)
				ns_factor = scale // (ns_cycles * 1000000)
				us_iterations = (250000 * us_factor + 65535) >> 16
				ns_iterations = (250000000 * ns_factor + 65535999) // 65536000

				self.assertLess(abs(us_iterations * us_loop_cycles -
					cpu_hz // 4), 1000)
				self.assertLess(abs(ns_iterations * ns_loop_cycles -
					cpu_hz // 4), 1000)


if __name__ == "__main__":
	unittest.main()
