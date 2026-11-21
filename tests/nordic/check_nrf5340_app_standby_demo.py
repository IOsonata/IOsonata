#!/usr/bin/env python3
"""Check the nRF5340 app-core standby project and backend wiring."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / "ARM/Nordic/nRF53/nRF5340_App/exemples"
POWER = ROOT / "ARM/Nordic/src/system_power_nrfx.cpp"
BUILD = ROOT / "tests/nordic/build_managed_power_clock.py"


class ApplicationStandbyDemo(unittest.TestCase):
	def test_standby_project_is_separate_from_system_off(self):
		project = (APP / "StandbyDemo/ioc/.project").read_text(encoding="utf-8")
		system_off = (APP / "PowerOffDemo/ioc/.project").read_text(encoding="utf-8")
		self.assertIn("<name>StandbyDemo</name>", project)
		self.assertIn("exemples/misc/power_off_demo.cpp", project)
		self.assertIn("<name>PowerOffDemo</name>", system_off)

	def test_standby_configuration_is_generic(self):
		board = (APP / "StandbyDemo/src/board.h").read_text(encoding="utf-8")
		original = (APP / "PowerOffDemo/src/board.h").read_text(encoding="utf-8")
		for token in ("POWER_DEMO_RESUME_WAKE", "POWER_DEMO_TIMER_WAKE_MS",
			"POWER_DEMO_TIMER_WAKE_DEVNO", "POWER_DEMO_STANDBY_RAM_TEST",
			"NRF5340_DK_PCA10095"):
			self.assertIn(token, board)
		self.assertNotIn("POWER_DEMO_RESUME_WAKE", original)
		self.assertNotIn("POWER_DEMO_RAM_RET_TEST", board)

	def test_sleep_backend_on_both_cores(self):
		s = POWER.read_text(encoding="utf-8")
		block = s[s.index("#if defined(NRF5340_XXAA_NETWORK) || defined(NRF5340_XXAA_APPLICATION)"):]
		block = block[:block.index("#endif")]
		self.assertIn("void SystemSleep(void)", block)
		self.assertIn("state = DisableInterrupt();", block)
		self.assertIn("__WFI();", block)
		self.assertIn("SCB->SCR = scr;", block)
		self.assertIn("EnableInterrupt(state);", block)

	def test_managed_build_includes_both_examples(self):
		s = BUILD.read_text(encoding="utf-8")
		self.assertIn('root / "exemples/StandbyDemo/ioc"', s)
		self.assertIn('root / "exemples/PowerOffDemo/ioc"', s)
		self.assertIn('root / "exemples/NetCoreStart/ioc"', s)


if __name__ == "__main__":
	unittest.main()
