#!/usr/bin/env python3
"""Check nRF5340 consolidated PowerApiTest project wiring."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "exemples/misc/power_api_test.cpp"
BASE = ROOT / "ARM/Nordic/nRF53"


class PowerApiTest(unittest.TestCase):
	def test_shared_power_api_result_ledger(self):
		s = SRC.read_text(encoding="utf-8")
		for name in ("PowerApiTestResult", "PowerApiTestSummary",
			"SystemSleep();", "SystemPowerOff();",
			"RAM retention after reset", "Standby timer wake",
			"CPU/APB clock after resume"):
			self.assertIn(name, s)

	def test_report_after_ram_check(self):
		s = SRC.read_text(encoding="utf-8")
		standby = s[s.index("int main()"):s.index("#else\n\tuint32_t reason;", s.index("int main()"))]
		self.assertLess(standby.index('PowerApiTestResult("SRAM sample:'),
			standby.rindex("PowerApiTestSummary();"))
		self.assertIn("PowerApiTest complete:", s)

	def test_one_managed_project_per_core(self):
		for core in ("nRF5340_App", "nRF5340_Net"):
			base = BASE / core / "exemples"
			project = (base / "PowerApiTest/ioc/.project").read_text(encoding="utf-8")
			self.assertIn("<name>PowerApiTest</name>", project)
			self.assertIn("exemples/misc/power_api_test.cpp", project)
			self.assertFalse((base / "StandbyDemo").exists())
			self.assertFalse((base / "PowerOffDemo").exists())

	def test_build_includes_power_api(self):
		s = (ROOT / "tests/nordic/build_managed_power_clock.py").read_text(encoding="utf-8")
		self.assertIn('root / "exemples/PowerApiTest/ioc"', s)
		self.assertIn('root / "exemples/PowerOffDemo/ioc"', s)
		self.assertNotIn('root / "exemples/StandbyDemo/ioc"', s)


if __name__ == "__main__":
	unittest.main()
