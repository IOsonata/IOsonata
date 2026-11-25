#!/usr/bin/env python3
"""Validate CoreClockTest's architecture-neutral delay calibration boundary."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
TEST = ROOT / "exemples/misc/core_clock_test.cpp"
ARM = ROOT / "ARM/include/idelay.h"
ARM_CAL = ROOT / "ARM/include/system_core_clock_calibration.h"
RISCV = ROOT / "RISCV/include/idelay.h"


class GenericDelayCalibration(unittest.TestCase):
	def test_core_test_has_no_architecture_dependency(self):
		source = TEST.read_text(encoding="utf-8")
		for forbidden in (
			"ArmDelay", "DWT->", "CoreDebug->", "__arm__", "__thumb__",
			"__riscv",
		):
			with self.subTest(symbol=forbidden):
				self.assertNotIn(forbidden, source)
		self.assertIn("DelayCalibrationMeasure(pSample)", source)
		self.assertIn("DelayCalibrationFromTimer(", source)

	def test_architectures_share_api_without_sharing_mechanisms(self):
		arm = ARM.read_text(encoding="utf-8")
		cal = ARM_CAL.read_text(encoding="utf-8")
		riscv = RISCV.read_text(encoding="utf-8")
		for source in (cal, riscv):
			self.assertIn("} DelayCalibrationPoint_t;", source)
			self.assertIn("bool DelayCalibrationMeasure(", source)
			self.assertIn("bool DelayCalibrationFromTimer(", source)
		self.assertIn("ArmDelayCalibrate(&measured, cpuHz)", cal)
		self.assertNotIn("DWT->", arm)
		self.assertNotIn("NRF52_SERIES", cal)
		self.assertNotIn("ArmDelay", riscv)
		self.assertIn("IOsonataRiscvDelayCycles", riscv)

	def test_interpolation_is_owned_by_system_core(self):
		source = TEST.read_text(encoding="utf-8")
		arm = ARM.read_text(encoding="utf-8")
		cal = ARM_CAL.read_text(encoding="utf-8")
		riscv = RISCV.read_text(encoding="utf-8")
		self.assertNotIn("DelayCalibrationInterpolate(", source)
		self.assertNotIn("ArmDelayClockInterpolate(", source)
		self.assertIn("ArmDelayClockInterpolate(", cal)
		self.assertIn("CoreClockTestDelayDivisors(pSample);", source)
		self.assertEqual(source.count("CoreClockTestDelayCalibrationRecord();"), 2)
		self.assertNotIn("DelayCalibrationInterpolate(", arm)
		self.assertNotIn("DelayCalibrationInterpolate(", riscv)

	def test_only_generic_system_core_clock_header_remains(self):
		canonical = ROOT / "include/coredev/system_core_clock.h"
		self.assertTrue(canonical.is_file())
		legacy = [p for p in (ROOT / "ARM").rglob("system_core_clock.h")
			if p.is_file()]
		self.assertEqual([], legacy)



if __name__ == "__main__":
	unittest.main()
