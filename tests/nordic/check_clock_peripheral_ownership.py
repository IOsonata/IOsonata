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
		self.assertIn("nRFxLfclkAcquire(s_RtcClockSource[pTimer->DevNo])", source)
		self.assertIn("nRFxLfclkRelease();", source)

	def test_clock_refuses_mismatched_rtc_source(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertEqual(source.count("Source != NRFX_LFCLK_REQUEST_DEFAULT &&"), 2)
		self.assertIn("s_nRFxLfclkReqCnt++", source)
		self.assertIn("if (change && s_nRFxLfclkReqCnt != 0U)", source)
		self.assertIn("s_bNRFxLfclkStopPending = true;", source)

	def test_hf_timer_balances_clock_request(self):
		source = TIMER.read_text(encoding="utf-8")
		self.assertNotIn("NRF_CLOCK->", source)
		self.assertNotIn("TASKS_PLLSTART", source)
		self.assertIn("s_nRfxHFClockSem == 0 && !s_bTimerHfxoHeld", source)
		self.assertIn("if (SystemCoreClockSelect(OSC_TYPE_RC, 0))", source)
		self.assertIn("s_bTimerHfxoHeld = false;", source)

	def test_usb_keeps_refused_clock_release(self):
		source = USB.read_text(encoding="utf-8")
		start = source.index("static inline void UsbdXtalRelease(void)\n{")
		end = source.index("\n}", start)
		release = source[start:end]
		self.assertIn("if (SystemCoreClockSelect(OSC_TYPE_RC, 0))", release)
		self.assertLess(release.index("SystemCoreClockSelect("),
			release.index("s_bUsbdXtalHeld = false;"))


if __name__ == "__main__":
	unittest.main()
