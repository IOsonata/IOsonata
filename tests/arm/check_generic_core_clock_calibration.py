#!/usr/bin/env python3
"""Validate CoreClockTest's architecture-neutral delay calibration boundary."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
TEST = ROOT / "exemples/misc/core_clock_test.cpp"
API = ROOT / "include/coredev/delay_calibration.h"
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
		self.assertIn("DelayCalibrationInterpolate(", source)

	def test_architectures_share_api_without_sharing_mechanisms(self):
		arm = ARM.read_text(encoding="utf-8")
		riscv = RISCV.read_text(encoding="utf-8")
		for source in (arm, riscv):
			self.assertIn('#include "coredev/delay_calibration.h"', source)
			self.assertIn("bool DelayCalibrationMeasure(", source)
			self.assertIn("bool DelayCalibrationFromTimer(", source)
		self.assertIn("ArmDelayCalibrate(&measured, cpuHz)", arm)
		self.assertNotIn("ArmDelay", riscv)
		self.assertIn("IOsonataRiscvDelayCycles", riscv)

	def test_interpolation_is_pure_and_compiles_in_c_and_cpp(self):
		compiler_c = shutil.which("cc")
		compiler_cpp = shutil.which("c++")
		if compiler_c is None or compiler_cpp is None:
			self.skipTest("host C/C++ compilers are unavailable")

		program = """
#include "coredev/delay_calibration.h"
int main(void)
{
	DelayCalibrationPoint_t low = { 16000000U, 40000U, 120000U, true };
	DelayCalibrationPoint_t high = { 64000000U, 160000U, 480000U, true };
	DelayCalibrationPoint_t result = { 0U, 0U, 0U, false };
	if (!DelayCalibrationInterpolate(40000000U, &low, &high, &result)) return 1;
	if (result.UsQ16 != 100000U || result.NsQ16 != 300000U) return 2;
	if (!DelayCalibrationInterpolate(1000000U, &low, &high, &result)) return 3;
	if (result.UsQ16 != low.UsQ16) return 4;
	if (!DelayCalibrationInterpolate(96000000U, &low, &high, &result)) return 5;
	if (result.NsQ16 != high.NsQ16) return 6;
	if (!DelayCalibrationInterpolate(16000000U, &low, &low, &result)) return 7;
	if (result.UsQ16 != low.UsQ16) return 8;
	return 0;
}
"""
		with tempfile.TemporaryDirectory() as directory:
			for compiler, language, standard in (
				(compiler_c, "c", "c11"), (compiler_cpp, "c++", "c++11")
			):
				with self.subTest(language=language):
					binary = str(Path(directory) / ("test-" + language.replace("+", "p")))
					subprocess.run(
						[compiler, "-x", language, "-std=" + standard, "-Wall",
						 "-Wextra", "-Werror", "-I", str(ROOT / "include"),
						 "-o", binary, "-"],
						input=program, text=True, check=True, capture_output=True,
					)
					subprocess.run([binary], check=True)


if __name__ == "__main__":
	unittest.main()
