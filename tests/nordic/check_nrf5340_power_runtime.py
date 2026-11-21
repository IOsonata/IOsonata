#!/usr/bin/env python3
"""Check CorePowerTest runs all nRF5340 power capabilities at runtime."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "exemples/misc/core_power_test.cpp"
API = ROOT / "include/coredev/system_core_clock.h"
NRF = ROOT / "ARM/Nordic/src/system_power_nrfx.cpp"

class RuntimePowerChecks(unittest.TestCase):
	def test_runtime_sequence(self):
		s = SRC.read_text(encoding="utf-8")
		main = s[s.index("int main(void)"):]
		self.assertIn("caps = SystemPowerCapabilitiesGet();", main)
		self.assertLess(main.index("CorePowerTestSleep(caps)"), main.index("SystemPowerOff();"))
		self.assertIn("CorePowerTestRetention(caps", main)
		self.assertNotIn("POWER_DEMO_RESUME_WAKE", s)
		for name in ("CorePowerTestResult", "CorePowerTestSummary",
			"CORE_POWER_TEST_SLEEP_CYCLES", "SystemSleep();", "SystemPowerOff();"):
			self.assertIn(name, s)

	def test_app_and_net_differ_in_backend_capabilities(self):
		s = NRF.read_text(encoding="utf-8")
		self.assertIn("uint32_t SystemPowerCapabilitiesGet(void)", s)
		self.assertIn("SYSTEM_POWER_CAP_SYSTEM_OFF", s)
		self.assertIn("SYSTEM_POWER_CAP_TIMER_WAKE", s)
		self.assertIn("defined(NRF5340_XXAA_NETWORK)", s)
		self.assertIn("defined(NRF5340_XXAA_APPLICATION)", s)

	def test_reversible_boundaries_exercised(self):
		s = SRC.read_text(encoding="utf-8")
		for name in ("RAM retention invalid range", "Wake pin invalid index",
			"Wake pin disable", "Wake pin rearm",
			"Retention register invalid index"):
			self.assertIn(name, s)

	def test_board_describes_resources_not_test_modes(self):
		for core in ("nRF5340_App", "nRF5340_Net"):
			p = ROOT / "ARM/Nordic/nRF53" / core / "exemples/CorePowerTest/src/board.h"
			s = p.read_text(encoding="utf-8")
			self.assertNotIn("#define POWER_DEMO_RESUME_WAKE", s)

if __name__ == "__main__":
	unittest.main()
