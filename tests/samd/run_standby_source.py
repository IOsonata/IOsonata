#!/usr/bin/env python3
"""Source regression checks for SAM D11/D21 Standby wake ownership.

These checks do not replace Cortex-M0+ compilation or hardware tests.
"""
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
EIC = (ROOT / "ARM/Microchip/SAMD/src/iopincfg_samd.c").read_text(encoding="utf-8")
SLEEP = (ROOT / "ARM/Microchip/SAMD/src/system_sleep_samd.c").read_text(encoding="utf-8")
CLOCK = (ROOT / "ARM/Microchip/SAMD/src/system_samd.c").read_text(encoding="utf-8")
RTC = (ROOT / "ARM/Microchip/SAMD/src/timer_samd_rtc.cpp").read_text(encoding="utf-8")
ADC = (ROOT / "ARM/Microchip/SAMD/src/adc_samd.cpp").read_text(encoding="utf-8")
DMAC = (ROOT / "ARM/Microchip/SAMD/src/dmac_samd.c").read_text(encoding="utf-8")
TIMER = (ROOT / "ARM/Microchip/SAMD/src/timer_samd.cpp").read_text(encoding="utf-8")


def body(source, signature):
	start = source.index(signature)
	open_brace = source.index("{", start)
	level = 1
	for pos in range(open_brace + 1, len(source)):
		if source[pos] == "{":
			level += 1
		elif source[pos] == "}":
			level -= 1
			if level == 0:
				return source[open_brace + 1:pos]
	raise AssertionError("Unclosed function: " + signature)


class StandbySourceTest(unittest.TestCase):
	def test_eic_requests_and_releases_the_low_frequency_clock(self):
		start = body(EIC, "static bool SamdEicStart(")
		stop = body(EIC, "static bool SamdEicStopIdle(")
		self.assertIn("SystemLowFreqClockEnable()", start)
		self.assertIn("SamdLowFreqClockStandbyEnable()", start)
		self.assertIn("SAMD_GCLK_LF", start)
		self.assertNotIn("SAMD_GCLK_MAIN", EIC)
		self.assertIn("SystemLowFreqClockDisable()", stop)
		self.assertEqual(EIC.count("SystemLowFreqClockEnable()"), 1)

	def test_pin_interrupt_is_the_wake_source(self):
		sense = body(EIC, "static bool SamdEicSense(")
		self.assertIn("EIC_REGS->EIC_WAKEUP |= 1UL << ExtInt", sense)
		self.assertIn("EIC_REGS->EIC_WAKEUP &= ~(1UL << ExtInt)", sense)
		self.assertLess(sense.index("EIC_REGS->EIC_CTRL = 0"), sense.index("EIC_REGS->EIC_WAKEUP |="))
		for name in ("SystemWakePin", "StandbyWake", "EIC_CONFIG_SENSE0_LOW_Val"):
			self.assertNotIn(name, EIC)

	def test_samd11_nmi_pin_is_an_interrupt_line(self):
		find = body(EIC, "static int SamdExtIntFind(")
		self.assertIn("PinNo == 14", find)
		self.assertIn("return SAMD_EIC_NMI_LINE", find)
		enable = body(EIC, "bool IOPinEnableInterrupt(")
		self.assertIn("SamdEicNmiSense(Sense)", enable)
		nmi = body(EIC, "void NMI_Handler(")
		self.assertIn("EIC_REGS->EIC_NMIFLAG = EIC_NMIFLAG_NMI_Msk", nmi)
		self.assertIn("NVIC_SetPendingIRQ(EIC_IRQn)", nmi)
		handler = body(EIC, "void EIC_Handler(")
		self.assertIn("s_SamdPinInt[SAMD_EIC_NMI_LINE].Handler(", handler)

	def test_rtc_requests_standby_oscillator(self):
		clock = body(CLOCK, "bool SamdLowFreqClockStandbyEnable(")
		rtc = body(RTC, "bool SamdRtcSetup(")
		self.assertIn("SamdLowFreqClockStandbyEnable()", rtc)
		self.assertIn("SYSCTRL_OSC32K_RUNSTDBY_Msk", clock)
		self.assertIn("SYSCTRL_XOSC32K_RUNSTDBY_Msk", clock)

	def test_samd11_board_selects_sw0(self):
		board = (ROOT / "ARM/Microchip/SAMD/SAMD11/exemples/CorePowerTest/src/board.h").read_text(encoding="utf-8")
		self.assertIn("#define WAKE_BUT_PIN 14", board)
		self.assertNotIn("#define CORE_POWER_TEST_WAKE_MS", board)

	def test_maskable_wake_is_pending_through_wfi(self):
		s = body(SLEEP, "void SystemSleep(")
		self.assertLess(s.index("DisableInterrupt()"), s.index("__WFI()"))
		self.assertLess(s.index("__WFI()"), s.rindex("EnableInterrupt(state)"))
		self.assertIn("SCB->SCR = previous", s)

	def test_sleep_entry_calls_no_pin_driver(self):
		for name in ("StandbyWake", "EIC_REGS", "IOPin"):
			self.assertNotIn(name, SLEEP)

	def test_sleep_entry_leaves_the_device_drivers_alone(self):
		# The application prepares its devices; SystemSleep only enters Standby.
		source = SLEEP if isinstance(SLEEP, str) else SLEEP.read_text()
		for name in ("PrepareSleep", "ResumeSleep", "AllowStandby"):
			self.assertNotIn(name, source)



if __name__ == "__main__":
	unittest.main()
