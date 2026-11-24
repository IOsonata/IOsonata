#!/usr/bin/env python3
"""Check the TimerPowerOff C/C++ declarations and MCU timer drivers."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "include/coredev/timer.h"

class TimerPowerOffChecks(unittest.TestCase):
	def read(self, path):
		return (ROOT / path).read_text(encoding="utf-8")

	def test_timer_interface(self):
		src = HEADER.read_text(encoding="utf-8")
		self.assertIn("bool (*PowerOff)(TimerDev_t * const pTimerDev);", src)
		self.assertIn("static inline bool TimerPowerOff(TimerDev_t * const pTimer)", src)
		self.assertIn("pTimer->PowerOff(pTimer) : false;", src)
		self.assertIn("virtual bool PowerOff() { return TimerPowerOff(&vTimer); }", src)
		self.assertIn("vTimer.PowerOff = NULL;", src)
		self.assertIn("static inline void TimerDisable(", src)
		self.assertNotIn("TimerDeinit", src)

	def test_nordic_timer_shutdown(self):
		for path, callback, owner, guard in (
			("ARM/Nordic/src/timer_hf_nrfx.cpp", "nRFxTimerPowerOff",
				"s_pnRFxTimerDev[devno]", "nRFxTimerOwned"),
			("ARM/Nordic/src/timer_lf_nrfx.cpp", "nRFxRtcPowerOff",
				"s_pnRfxRtcDev[devno]", "nRFxRtcOwned"),
		):
			with self.subTest(timer=path):
				src = self.read(path)
				self.assertIn("static bool " + callback + "(", src)
				begin = src.index("static bool " + callback + "(")
				end = src.index("\n}\n", begin)
				body = src[begin:end]
				self.assertIn(owner + " = nullptr;", body)
				self.assertIn("pTimer->DevNo = -1;", body)
				self.assertIn("pTimer->PowerOff = nullptr;", body)
				self.assertIn("TIMER_TRIG_PEI_SINGLE", body)
				self.assertIn("TIMER_TRIG_PEI_CONTINUOUS", body)
				self.assertIn("pTimer->PowerOff = " + callback + ";", src)
				self.assertIn("static bool " + guard + "(", src)
				self.assertIn("!" + guard + "(pTimer)", src)

	def test_nordic_shared_clock_protection(self):
		hf = self.read("ARM/Nordic/src/timer_hf_nrfx.cpp")
		rtc = self.read("ARM/Nordic/src/timer_lf_nrfx.cpp")
		grtc = self.read("ARM/Nordic/nRF54/src/timer_lf_nrf54.cpp")
		self.assertIn("SystemHighFreqClockDisable();", hf)
		self.assertIn("SystemLowFreqClockDisable();", rtc)
		self.assertIn("static bool nRFxGrtcPowerOff(", grtc)
		begin = grtc.index("static bool nRFxGrtcPowerOff(")
		end = grtc.index("\n}\n", begin)
		body = grtc[begin:end]
		self.assertIn("s_pGrtcOwner[devno] = nullptr;", body)
		self.assertIn("NRF_GRTC->CC[idx].CCEN = 0U;", body)
		self.assertNotIn("NRF_GRTC->TASKS_STOP", body)

	def test_nordic_library_timer_sources(self):
		for family in ("nRF52", "nRF53", "nRF54", "nRF71", "nRF91"):
			projects = list((ROOT / "ARM/Nordic" / family).glob("*/lib/ioc/.project"))
			self.assertTrue(projects, family)
			for project in projects:
				with self.subTest(project=project):
					src = project.read_text()
					self.assertIn("/timer_nrfx.cpp</locationURI>", src)
					self.assertIn("/timer_hf_nrfx.cpp</locationURI>", src)
					lf = "timer_lf_nrf54" if family in ("nRF54", "nRF71") else "timer_lf_nrfx"
					self.assertIn("/" + lf + ".cpp</locationURI>", src)

	def test_generic_example_uses_timer_power_off(self):
		src = self.read("exemples/misc/core_clock_test.cpp")
		begin = src.index("static void CoreClockTestTimerPowerOff(void)")
		end = src.index("int main(void)", begin)
		test = src[begin:end]
		self.assertIn("stopped = TimerPowerOff(&s_RetainedTimer);", test)
		self.assertIn("TimerInit(&s_RetainedTimer, &cfg);", test)
		self.assertIn("TimerPowerOff(&s_RetainedTimer);", test)
		self.assertIn("!TimerPowerOff(&s_RetainedTimer)", test)
		self.assertIn("CoreClockTestTimerPowerOff();", src)
		self.assertNotIn("NRF91_SERIES", test)

if __name__ == "__main__":
	unittest.main()
