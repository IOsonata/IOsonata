#!/usr/bin/env python3
"""Check nRF91 System ON IDLE integration without changing generic test code."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
POWER = ROOT / "ARM/Nordic/src/system_power_nrfx.cpp"
BUILD = ROOT / "tests/nordic/build_managed_power_clock.py"


class Nrf91PowerClock(unittest.TestCase):
	def test_nrf91_resumable_sleep(self):
		src = POWER.read_text(encoding="utf-8")
		sleep = src[src.index("// Resumable CPU sleep on nRF53"):src.index("#endif", src.index("void SystemSleep(void)"))]
		self.assertIn("defined(NRF91_SERIES)", src[src.index("#if defined(NRF5340_XXAA_NETWORK)"):src.index("// Resumable CPU sleep")])
		for token in ("__get_IPSR()", "DisableInterrupt()", "__WFI()", "EnableInterrupt(state)", "SCB->SCR = scr"):
			self.assertIn(token, sleep)
		self.assertIn("SCB_SCR_SLEEPDEEP_Msk", sleep)
		self.assertIn("SCB_SCR_SLEEPONEXIT_Msk", sleep)

	def test_nrf91_capabilities_match_implemented_sleep(self):
		src = POWER.read_text(encoding="utf-8")
		caps = src[src.index("uint32_t SystemPowerCapabilitiesGet(void)"):src.index("void SystemPowerOff(void)")]
		part = caps[caps.index("#elif defined(NRF91_SERIES)"):caps.index("#else", caps.index("#elif defined(NRF91_SERIES)"))]
		for capability in ("SYSTEM_POWER_CAP_SLEEP", "SYSTEM_POWER_CAP_TIMER_WAKE",
			"SYSTEM_POWER_CAP_SYSTEM_OFF", "SYSTEM_POWER_CAP_WAKE_PIN",
			"SYSTEM_POWER_CAP_RAM_RETENTION", "SYSTEM_POWER_CAP_RET_REG"):
			self.assertIn(capability, part)
		self.assertNotIn("SYSTEM_POWER_CAP_LOW_POWER", part)

	def test_both_nrf91_managed_core_tests(self):
		src = BUILD.read_text(encoding="utf-8")
		for target in ("nrf9160", "nrf91x1"):
			self.assertIn('"' + target + '": ROOT', src)
		for target in ("nRF9160", "nRF91x1"):
			root = ROOT / "ARM/Nordic/nRF91" / target / "exemples"
			for project in ("CoreClockTest", "CorePowerTest"):
				path = root / project / "ioc/.project"
				text = path.read_text(encoding="utf-8")
				self.assertIn("<name>" + project + "</name>", text)
				self.assertIn("exemples/misc/core_" + ("clock" if project == "CoreClockTest" else "power") + "_test.cpp", text)


if __name__ == "__main__":
	unittest.main()
