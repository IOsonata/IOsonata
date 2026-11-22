#!/usr/bin/env python3
"""Check nRF5340 NetCoreStart holds LFXO before releasing network reset."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
START = ROOT / "ARM/Nordic/nRF53/nRF5340_App/exemples/NetCoreStart/src/netcore_start.cpp"
APP = ROOT / "ARM/Nordic/nRF53/nRF5340_App/lib/src/system_nrf5340_application.c"
BUILD = ROOT / "tests/nordic/build_managed_power_clock.py"


class NetCoreStartLfxo(unittest.TestCase):
	def test_lfxo_is_started_before_network_core(self):
		s = START.read_text(encoding="utf-8")
		self.assertIn('#include "coredev/system_core_clock.h"', s)
		self.assertIn("g_McuOsc.LowPwrOsc.Type != OSC_TYPE_XTAL", s)
		self.assertIn("SystemLowFreqClockSelect(OSC_TYPE_XTAL,", s)
		self.assertIn("SystemLowFreqClockEnable()", s)
		self.assertIn("return 2;", s)
		self.assertLess(s.index("SystemLowFreqClockEnable()"),
			s.index("NETWORK.FORCEOFF"))
		self.assertNotIn("SystemLowFreqClockDisable()", s)

	def test_secure_system_init_prepares_oscillator(self):
		s = APP.read_text(encoding="utf-8")
		for part in ("NRF_P0_S->PIN_CNF[0]", "NRF_P0_S->PIN_CNF[1]",
			"NRF_OSCILLATORS_S->XOSC32KI.INTCAP"):
			self.assertIn(part, s)

	def test_eclipse_managed_build_includes_starter(self):
		s = BUILD.read_text(encoding="utf-8")
		self.assertIn('if target == "nrf5340_app":', s)
		self.assertIn('root / "exemples/NetCoreStart/ioc"', s)


if __name__ == "__main__":
	unittest.main()
