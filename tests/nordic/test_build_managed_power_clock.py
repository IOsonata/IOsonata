#!/usr/bin/env python3
"""Host tests for Nordic Eclipse CDT build result classification."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).with_name("build_managed_power_clock.py")
SPEC = importlib.util.spec_from_file_location("nordic_managed_build", SCRIPT)
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


class BuildResultTests(unittest.TestCase):
	def check_log(self, output):
		with tempfile.TemporaryDirectory(prefix="nordic-cdt-log-") as temp:
			logfile = Path(temp) / "build.log"
			logfile.write_text(output, encoding="utf-8")
			return RUNNER.log_error(logfile)

	def test_success(self):
		self.assertFalse(self.check_log(
			"Building target: ClockDemo.elf\n"
			"Finished building target: ClockDemo.elf\n"
			"Build Finished. 0 errors, 0 warnings.\n"))

	def test_warning_only(self):
		self.assertFalse(self.check_log(
			"src/test.cpp:42:5: warning: unused parameter\n"
			"Build Finished. 0 errors, 1 warning.\n"))

	def test_compiler_error(self):
		self.assertTrue(self.check_log(
			"src/test.cpp:42:3: error: 'foo' was not declared\n"))

	def test_linker_error(self):
		self.assertTrue(self.check_log(
			"ld: error: undefined symbol: SystemSleep\n"))

	def test_make_failure(self):
		self.assertTrue(self.check_log(
			"make[2]: *** [subdir.mk:12: test.o] Error 1\n"))

	def test_eclipse_failure(self):
		self.assertTrue(self.check_log(
			"Headless build encountered errors\n"))

	def test_missing_dependency(self):
		self.assertTrue(self.check_log(
			"make: *** No rule to make target 'libTaktOS.a'\n"))


if __name__ == "__main__":
	unittest.main()
