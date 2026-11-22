#!/usr/bin/env python3
"""Validate CoreClockTest's architecture-neutral delay calibration boundary."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
TEST = ROOT / "exemples/misc/core_clock_test.cpp"
ARM = ROOT / "ARM/include/idelay.h"
RISCV = ROOT / "RISCV/include/idelay.h"


class GenericDelayCalibration(unittest.TestCase):
	def test_core_test_has_no_architecture_dependency(self):
		source = TEST.read_text(encoding="utf-8")
		for forbidden in (
			"ArmDelay", "DWT->", "CoreDebug->", "__arm__", "__thumb__",
			"__riscv", "SystemMicroSecLoopCnt", "SystemnsDelayFactor",
		):
			with self.subTest(symbol=forbidden):
				self.assertNotIn(forbidden, source)
		self.assertIn("DelayCalibrationMeasure(pSample)", source)
		self.assertIn("DelayCalibrationFromTimer(", source)
		self.assertIn("static bool DelayCalibrationInterpolate(", source)

	def test_architectures_share_api_without_sharing_mechanisms(self):
		arm = ARM.read_text(encoding="utf-8")
		riscv = RISCV.read_text(encoding="utf-8")
		for source in (arm, riscv):
			self.assertIn("} DelayCalibrationPoint_t;", source)
			self.assertIn("bool DelayCalibrationMeasure(", source)
			self.assertIn("bool DelayCalibrationFromTimer(", source)
		self.assertIn("ArmDelayCalibrate(&measured, cpuHz)", arm)
		self.assertNotIn("ArmDelay", riscv)
		self.assertIn("IOsonataRiscvDelayCycles", riscv)

	def test_interpolation_is_local_to_core_clock_test(self):
		source = TEST.read_text(encoding="utf-8")
		arm = ARM.read_text(encoding="utf-8")
		riscv = RISCV.read_text(encoding="utf-8")
		self.assertIn("static bool DelayCalibrationInterpolate(", source)
		self.assertIn("pResult->UsLoopsPerSecond", source)
		self.assertIn("pResult->NsLoopsPerSecond", source)
		self.assertNotIn("DelayCalibrationInterpolate(", arm)
		self.assertNotIn("DelayCalibrationInterpolate(", riscv)
		self.assertFalse((ROOT / "include/coredev/delay_calibration.h").exists())


if __name__ == "__main__":
	unittest.main()
