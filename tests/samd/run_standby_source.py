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
	def test_interrupt_cannot_take_wake_line(self):
		s = body(EIC, "bool IOPinEnableInterrupt(")
		self.assertIn("s_SamdStandbyWakeLines & (1UL << IntNo)", s)

	def test_gpio_sense_cannot_modify_wake_line(self):
		s = body(EIC, "void IOPinSetSense(")
		self.assertIn("s_SamdStandbyWakeLines & (1UL << intno)", s)

	def test_interrupt_disable_releases_wake_line(self):
		s = body(EIC, "void IOPinDisableInterrupt(")
		self.assertIn("EIC_REGS->EIC_WAKEUP &= ~(1UL << IntNo)", s)
		self.assertIn("s_SamdStandbyWakeLines &= ~(1UL << IntNo)", s)
		self.assertLess(s.index("EIC_REGS->EIC_CTRL = 0U"), s.index("EIC_REGS->EIC_WAKEUP &="))

	def test_wake_uses_unfiltered_level(self):
		s = body(EIC, "bool SystemWakePin(")
		self.assertIn("EIC_CONFIG_SENSE0_LOW_Val", s)
		self.assertIn("EIC_CONFIG_SENSE0_HIGH_Val", s)
		self.assertIn("cfg & ~(0xFUL << shift)", s)
		self.assertIn("EIC_REGS->EIC_WAKEUP |= bit", s)

	def test_samd11_nmi_wake(self):
		s = body(EIC, "static bool SamdStandbyNmiConfig(")
		self.assertIn("PinNo != 14", s)
		self.assertIn("SAMD_GCLK_LF", s)
		self.assertIn("SamdLowFreqClockStandbyEnable()", s)
		self.assertIn("EIC_NMICTRL_NMISENSE_FALL", s)
		self.assertIn("EIC_NMIFLAG_NMI_Msk", s)
		self.assertIn("PORT_PINCFG_PMUXEN_Msk", s)

	def test_rtc_and_nmi_request_standby_oscillator(self):
		clock = body(CLOCK, "bool SamdLowFreqClockStandbyEnable(")
		rtc = body(RTC, "bool SamdRtcSetup(")
		nmi = body(EIC, "static bool SamdStandbyNmiConfig(")
		self.assertIn("SamdLowFreqClockStandbyEnable()", rtc)
		self.assertIn("SamdLowFreqClockStandbyEnable()", nmi)
		self.assertIn("GCLK_GENCTRL_SRC_OSC32K", clock)
		self.assertIn("GCLK_GENCTRL_SRC_XOSC32K", clock)
		self.assertIn("SYSCTRL_OSC32K_RUNSTDBY_Msk", clock)
		self.assertIn("SYSCTRL_XOSC32K_RUNSTDBY_Msk", clock)
		self.assertIn("SYSCTRL_PCLKSR_OSC32KRDY_Msk", clock)
		self.assertIn("SYSCTRL_PCLKSR_XOSC32KRDY_Msk", clock)
		self.assertIn("EnableInterrupt(state)", clock)
		self.assertNotIn("SYSCTRL_REGS->SYSCTRL_OSC32K |=", nmi)

	def test_nmi_handler_clears_flag(self):
		s = body(EIC, "void NMI_Handler(")
		self.assertIn("EIC_REGS->EIC_NMIFLAG = EIC_NMIFLAG_NMI_Msk", s)

	def test_samd11_board_selects_sw0(self):
		board = (ROOT / "ARM/Microchip/SAMD/SAMD11/exemples/PowerOffDemo/src/board.h").read_text(encoding="utf-8")
		self.assertIn("#define WAKE_BUT_PIN 14", board)
		self.assertIn("#define POWER_DEMO_RESUME_WAKE 1", board)

	def test_maskable_wake_is_pending_through_wfi(self):
		s = body(SLEEP, "void SystemSleep(")
		self.assertLess(s.index("DisableInterrupt()"), s.index("SamdStandbyWakeArm()"))
		self.assertLess(s.index("SamdStandbyWakeArm()"), s.index("__WFI()"))
		self.assertLess(s.index("__WFI()"), s.rindex("EnableInterrupt(state)"))
		self.assertIn("SCB->SCR = previous", s)
		self.assertLess(s.index("__WFI()"), s.index("SamdStandbyWakeDone()"))
		self.assertLess(s.index("SamdStandbyWakeDone()"), s.rindex("EnableInterrupt(state)"))
		self.assertIn("SamdUartPrepareSleep()", s)
		self.assertLess(s.index("SamdUartPrepareSleep()"), s.index("SamdStandbyWakeArm()"))
		self.assertLess(s.index("SamdStandbyWakeDone()"), s.rindex("SamdUartResumeSleep()"))

	def test_adc_preserves_enabled_state_through_standby(self):
		prepare = body(ADC, 'extern "C" bool SamdAdcPrepareSleep(')
		resume = body(ADC, 'extern "C" void SamdAdcResumeSleep(')
		sleep = body(SLEEP, 'void SystemSleep(')
		self.assertIn('s_bSamdAdcSleepPaused = true', prepare)
		self.assertIn('ADC_REGS->ADC_CTRLA = ctrl & ~ADC_CTRLA_ENABLE_Msk', prepare)
		self.assertIn('s_pSamdAdc->Enable()', resume)
		self.assertLess(sleep.index('SamdAdcPrepareSleep()'), sleep.index('SamdStandbyWakeArm()'))
		self.assertGreater(sleep.rindex('SamdAdcResumeSleep()'), sleep.index('__WFI()'))
		self.assertIn('SamdUsbAllowStandby()', sleep)
		self.assertIn('SamdDacAllowStandby()', sleep)

	def test_dma_busy_prevents_standby(self):
		check = body(DMAC, "bool SamdDmacAllowStandby(")
		sleep = body(SLEEP, "void SystemSleep(")
		self.assertIn("DMAC_REGS->DMAC_BUSYCH", check)
		self.assertIn("DMAC_REGS->DMAC_PENDCH", check)
		self.assertIn("DMAC_REGS->DMAC_INTSTATUS", check)
		self.assertIn("DMAC_CHCTRLA_ENABLE_Msk", check)
		self.assertIn("DMAC_REGS->DMAC_CHID = selected", check)
		self.assertLess(sleep.index("SamdDmacAllowStandby()"), sleep.index("SamdAdcPrepareSleep()"))

	def test_hf_timer_standby_refusal(self):
		check = body(TIMER, 'extern "C" bool SamdTimerAllowStandby(')
		sleep = body(SLEEP, 'void SystemSleep(')
		self.assertIn('s_SamdSysTickOwner != NULL', check)
		self.assertIn('SAMD_RTC_TIMER_MAXCNT', check)
		self.assertIn('g_SamdTimerData[i].bRunning', check)
		self.assertLess(sleep.index('SamdTimerAllowStandby()'), sleep.index('SamdAdcPrepareSleep()'))

	def test_rearm_does_not_change_gpio_ownership(self):
		s = body(EIC, "void SamdStandbyWakeArm(")
		self.assertIn("s_SamdStandbyWakeLines", s)
		self.assertIn("EIC_REGS->EIC_INTFLAG = mask", s)
		self.assertIn("EIC_REGS->EIC_INTENSET = mask", s)
		self.assertNotIn("SamdEicSense(", s)


if __name__ == "__main__":
	unittest.main()
