#!/usr/bin/env python3
"""Check Nordic clock register ownership and peripheral request lifetimes.

Source-level checks only; build and hardware validation remain separate.
"""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
CLOCK = ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp"
RTC = ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp"
TIMER = ROOT / "ARM/Nordic/src/timer_hf_nrfx.cpp"
USB = ROOT / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp"


class NordicClockOwnership(unittest.TestCase):
	def test_rtc_uses_system_clock(self):
		source = RTC.read_text(encoding="utf-8")
		self.assertNotIn("NRF_CLOCK", source)
		self.assertNotIn("TASKS_LFCLKSTART", source)
		self.assertNotIn("TASKS_LFCLKSTOP", source)
		self.assertIn("SystemLowFreqClockEnable()", source)
		self.assertIn("SystemLowFreqClockDisable();", source)

	def test_clock_refuses_mismatched_rtc_source(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertIn("NRF_CLOCK->NRFX_LFCLKSRCCOPY", source)
		self.assertIn("s_nRFxLfclkReqCnt++", source)
		self.assertIn("effectiveSrc & NRFX_LFCLK_SRC_Msk", source)
		self.assertIn("s_bNRFxLfclkStopPending = true;", source)

	def test_hf_timer_balances_clock_request(self):
		source = TIMER.read_text(encoding="utf-8")
		self.assertNotIn("NRF_CLOCK->", source)
		self.assertNotIn("TASKS_PLLSTART", source)
		self.assertIn("if (!SystemHighFreqClockEnable())", source)
		self.assertIn("SystemHighFreqClockDisable();", source)
		self.assertIn("s_bTimerEnabled[devno] = false;", source)

	def test_usb_releases_held_clock(self):
		source = USB.read_text(encoding="utf-8")
		start = source.index("static inline void UsbdXtalRelease(void)\n{")
		end = source.index("\n}", start)
		release = source[start:end]
		self.assertIn("SystemHighFreqClockDisable();", release)
		self.assertLess(release.index("SystemHighFreqClockDisable("),
			release.index("s_bUsbdXtalHeld = false;"))


if __name__ == "__main__":
	unittest.main()
