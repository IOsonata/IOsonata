#!/usr/bin/env python3
"""Source regression checks for SAM D11/D21 Standby wake ownership.

These checks do not replace Cortex-M0+ compilation or hardware tests.
"""
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
EIC = (ROOT / "ARM/Microchip/SAMD/src/iopincfg_samd.c").read_text(encoding="utf-8")
SLEEP = (ROOT / "ARM/Microchip/SAMD/src/system_sleep_samd.c").read_text(encoding="utf-8")


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
		self.assertIn("SYSCTRL_OSC32K_RUNSTDBY_Msk", s)
		self.assertIn("SYSCTRL_XOSC32K_RUNSTDBY_Msk", s)
		self.assertIn("EIC_NMICTRL_NMISENSE_FALL", s)
		self.assertIn("EIC_NMIFLAG_NMI_Msk", s)
		self.assertIn("PORT_PINCFG_PMUXEN_Msk", s)

	def test_nmi_handler_clears_flag(self):
		s = body(EIC, "void NMI_Handler(")
		self.assertIn("EIC_REGS->EIC_NMIFLAG = EIC_NMIFLAG_NMI_Msk", s)

	def test_samd11_board_selects_sw0(self):
		board = (ROOT / "ARM/Microchip/SAMD/SAMD11/exemples/PowerOffDemo/src/board.h").read_text(encoding="utf-8")
		self.assertIn("#define WAKE_BUT_PIN 14", board)
		self.assertIn("#define POWER_DEMO_RESUME_WAKE 1", board)

	def test_sleep_rearm_wfi_is_atomic(self):
		s = body(SLEEP, "void SystemSleep(")
		self.assertLess(s.index("DisableInterrupt()"), s.index("SamdStandbyWakeArm()"))
		self.assertLess(s.index("SamdStandbyWakeArm()"), s.index("__WFI()"))
		self.assertLess(s.index("__WFI()"), s.index("EnableInterrupt(state)"))
		self.assertIn("SCB->SCR = previous", s)

	def test_rearm_does_not_change_gpio_ownership(self):
		s = body(EIC, "void SamdStandbyWakeArm(")
		self.assertIn("s_SamdStandbyWakeLines", s)
		self.assertIn("EIC_REGS->EIC_INTFLAG = mask", s)
		self.assertIn("EIC_REGS->EIC_INTENSET = mask", s)
		self.assertNotIn("SamdEicSense(", s)


if __name__ == "__main__":
	unittest.main()
