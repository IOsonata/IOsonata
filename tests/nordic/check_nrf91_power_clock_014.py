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


	def test_lfclk_start_checks_actual_source(self):
		src = (ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp").read_text(encoding="utf-8")
		start = src.index("static bool nRFxLfclkFinishStart(uint32_t Src)")
		end = src.index("static bool nRFxLfclkStart(uint32_t Src)", start)
		section = src[start:end]
		self.assertIn("#if defined(NRF53_SERIES) || defined(NRF91_SERIES)", section)
		self.assertIn("NRFX_LFCLK_STAT_SRC_Msk", section)
		self.assertIn("NRFX_LFCLK_SRC_XTAL", section)
		self.assertIn("NRFX_LFCLK_SRC_RC", section)
		self.assertIn("#if defined(NRF53_SERIES)\n\t\t\t(void)nRFxLfclkCalibrate();", section)
		self.assertIn("#if defined(NRF91_SERIES)", section)

	def test_nrf91_sdelay_uses_shared_lf_timer_and_measured_startup(self):
		timer = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(encoding="utf-8")
		clock = (ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp").read_text(encoding="utf-8")
		self.assertIn("defined(NRF54L_SERIES) || defined(NRF91_SERIES)", timer)
		self.assertIn('extern "C" bool nRFxSleepLfClockAccurate(void);', timer)
		self.assertIn('extern "C" uint32_t nRFxSleepLfClockStartupWaitUs(void);', timer)
		self.assertIn('extern "C" bool nRFxSleepLfClockAccurate(void)\n{', clock)
		self.assertIn("#if defined(NRF5340_XXAA_APPLICATION) || defined(NRF91_SERIES)", clock)
		start = timer.index('extern "C" bool sDelay(uint32_t Seconds)')
		delay = timer[start:]
		for call in ("nRFxSleepLfClockAccurate()", "nRFxSleepLfClockStartupWaitUs()",
					 "TimerInit(&s_SleepTimer", "TimerDisable(&s_SleepTimer)", "__WFE()"):
			self.assertIn(call, delay)
		self.assertNotIn("msDelay(", delay)
		self.assertNotIn("DWT->", delay)

	def test_example_is_generic(self):
		example = (ROOT / "exemples/misc/core_clock_test.cpp").read_text(encoding="utf-8")
		self.assertNotIn("NRF91_SERIES", example)
		self.assertNotIn("NRF9160", example)

	def test_independent_raw_lf_timer_frequency_diagnostics(self):
		example = (ROOT / "exemples/misc/core_clock_test.cpp").read_text(encoding="utf-8")
		begin = example.index("static void CoreClockTestTimer(int DevNo)")
		end = example.index("// Exercise the public Timer trigger API", begin)
		section = example[begin:end]
		self.assertIn("TimerGetTickCount(&s_Timer)", section)
		self.assertIn("TimerGetMicrosecond(&s_RetainedTimer)", section)
		self.assertIn("Independent LF timer rate:", section)
		self.assertIn('CoreClockTestResult("Delay within 2 percent",', section)
		self.assertNotIn("NRF91_SERIES", section)

if __name__ == "__main__":
	unittest.main()
