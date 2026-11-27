#!/usr/bin/env python3
"""Check nRF5340 consolidated CorePowerTest project wiring."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "exemples/misc/core_power_test.cpp"
BASE = ROOT / "ARM/Nordic/nRF53"


class CorePowerTest(unittest.TestCase):
	def test_all_supported_phases_run_at_runtime(self):
		s = SRC.read_text(encoding="utf-8")
		main = s[s.index("int main(void)"):]
		for name in ("CorePowerTestResult", "CorePowerTestSummary",
			"SystemSleep();", "SystemPowerOff();",
			"CorePowerTestRetention(marker)", "CorePowerTestRunningMode(caps)",
			"CorePowerTestSleep(caps)", "CorePowerTestWakePin(caps)",
			"SystemPowerCapabilitiesGet()"):
			self.assertIn(name, s)
		self.assertNotIn("POWER_DEMO_RESUME_WAKE", s)
		self.assertLess(main.index("CorePowerTestSleep(caps)"),
			main.index("SystemPowerOff();"))

	def test_void_gpio_configuration_call(self):
		s = SRC.read_text(encoding="utf-8")
		self.assertNotIn("ok = IOPinConfig(", s)
		self.assertIn("IOPinConfig(WAKE_BUT_PORT", s)

	def test_one_managed_project_per_core(self):
		for core in ("nRF5340_App", "nRF5340_Net"):
			base = BASE / core / "exemples"
			project = (base / "CorePowerTest/ioc/.project").read_text(encoding="utf-8")
			self.assertIn("<name>CorePowerTest</name>", project)
			self.assertIn("exemples/misc/core_power_test.cpp", project)
			self.assertFalse((base / "StandbyDemo").exists())
			self.assertFalse((base / "PowerOffDemo").exists())

	def test_build_includes_power_api(self):
		s = (ROOT / "tests/nordic/build_managed_power_clock.py").read_text(encoding="utf-8")
		self.assertIn('root / "exemples/CorePowerTest/ioc"', s)
		self.assertNotIn('root / "exemples/PowerOffDemo/ioc"', s)
		self.assertIn('root / "exemples/CoreClockTest/ioc"', s)
		self.assertNotIn('root / "exemples/ClockDemo/ioc"', s)
		self.assertNotIn('root / "exemples/StandbyDemo/ioc"', s)


if __name__ == "__main__":
	unittest.main()
