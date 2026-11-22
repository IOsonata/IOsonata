#!/usr/bin/env python3
"""Check optional DWT calibration after nRF5340 app CPU frequency changes."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
CAL = ROOT / "ARM/include/system_core_clock_calibration.h"
CLOCK = ROOT / "ARM/include/system_core_clock_calibration.h"
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
		self.assertIn("#define ARM_DELAY_CAL_DWT_AVAILABLE 1", self.cal)
		self.assertNotIn("ArmDelayDwtWait(", self.delay)
		self.assertIn("CoreDebug->DEMCR = prevTrace;", self.cal)
		self.assertIn("DWT->CTRL = prevCtrl;", self.cal)
		self.assertIn("CoreDebug_DEMCR_TRCENA_Msk", self.cal)
		self.assertIn("DWT_CTRL_CYCCNTENA_Msk", self.cal)
		self.assertIn("usDelay(ARM_DELAY_CAL_LOOPS)", self.cal)
		self.assertIn("nsDelay(ARM_DELAY_CAL_NS)", self.cal)
		self.assertIn("((uint64_t)CpuHz * ARM_DELAY_CAL_LOOPS) / pCal->UsCycles", self.clock)
		self.assertIn("((uint64_t)CpuHz * ARM_DELAY_CAL_LOOPS) / pCal->NsCycles", self.clock)
		self.assertIn("SystemMicroSecLoopCnt = prevUsFactor;", self.cal)
		self.assertIn("SystemnsDelayFactor = prevNsFactor;", self.cal)
		self.assertIn("__set_PRIMASK(prevMask)", self.cal)

	def test_app_frequency_update_uses_calibrated_loop_factors(self):
		self.assertIn('#include "idelay.h"', self.app)
		self.assertIn('#include "idelay.h"', self.net)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration, cpuHz, 20U, 3U)", self.app)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration, cpuHz, 22U, 3U)", self.app)
		self.assertIn("ArmDelayClockUpdate(&s_DelayCalibration, cpuHz)", self.app)
		self.assertNotIn("ArmDelayCalibrate(", self.app)
		self.assertNotIn("ArmDelayCalibrate(", self.net)
		self.assertIn("ArmDelayClockInit(&s_DelayCalibration, SystemCoreClock, 23U, 3U)", self.net)
		self.assertNotIn("ArmDelayClockUpdate(&s_DelayCalibration", self.net)


	def test_measurement_is_only_explicit_in_delay_interface(self):
		self.assertIn("static inline bool DelayCalibrationMeasure(", self.cal)
		self.assertIn("ArmDelayCalibrate(&measured, cpuHz)", self.cal)
		self.assertIn("SystemMicroSecLoopCnt = oldUs;", self.delay)
		self.assertIn("SystemnsDelayFactor = oldNs;", self.delay)
		self.assertIn("DWT->CYCCNT - start", self.cal)


	def test_delay_implementation_is_not_replaced(self):
		self.assertIn("static inline __attribute__((always_inline)) void usDelay(", self.delay)
		self.assertIn("static inline __attribute__((always_inline)) void nsDelay(", self.delay)
		self.assertIn('" SUBS %[low], #1\\n"', self.delay)
		self.assertIn('" BNE 1b\\n"', self.delay)

	def test_measured_loop_rate_math(self):
		for cpu_hz in (64000000, 128000000):
			for us_loop_cycles, ns_loop_cycles in ((13, 3), (23, 3), (28, 3)):
				count = 8192
				us_cycles = count * us_loop_cycles
				ns_cycles = count * ns_loop_cycles
				us_rate = cpu_hz * count // us_cycles
				ns_rate = cpu_hz * count // ns_cycles
				us_iterations = (250000 * us_rate + 999999) // 1000000
				ns_iterations = (250000000 * ns_rate + 999999999) // 1000000000

				self.assertLess(abs(us_iterations * us_loop_cycles -
					cpu_hz // 4), 1000)
				self.assertLess(abs(ns_iterations * ns_loop_cycles -
					cpu_hz // 4), 1000)


if __name__ == "__main__":
	unittest.main()
