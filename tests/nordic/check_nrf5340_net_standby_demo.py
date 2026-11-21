#!/usr/bin/env python3
"""Check nRF5340 network managed standby demo wiring."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
NET = ROOT / "ARM/Nordic/nRF53/nRF5340_Net/exemples/PowerOffDemo"


class NetworkStandbyDemo(unittest.TestCase):
	def test_project_links_generic_demo(self):
		project = (NET / "ioc/.project").read_text(encoding="utf-8")
		self.assertIn("<name>PowerOffDemo</name>", project)
		self.assertIn("exemples/misc/power_off_demo.cpp", project)
		self.assertIn("src/board.h", project)

	def test_board_uses_lf_timer_wake(self):
		board = (NET / "src/board.h").read_text(encoding="utf-8")
		self.assertIn("POWER_DEMO_RESUME_WAKE", board)
		self.assertIn("POWER_DEMO_TIMER_WAKE_MS", board)
		self.assertIn("POWER_DEMO_TIMER_WAKE_DEVNO", board)
		self.assertIn("POWER_DEMO_STANDBY_RAM_TEST", board)
		self.assertIn("NRF5340_DK_PCA10095", board)
		self.assertIn("UART_RX_PIN\t\t\t8", board)

	def test_generic_timer_is_running_before_wake_compare(self):
		s = (ROOT / "exemples/misc/power_off_demo.cpp").read_text(encoding="utf-8")
		start = s.index("if (TimerSetFrequency(&s_WakeTimer, 1024U)")
		end = s.index("elapsed = TimerGetMicrosecond(&s_WakeTimer) - start;", start)
		cycle = s[start:end]
		self.assertIn('PowerDemoPut("Standby timer enable FAIL', cycle)
		self.assertIn("if (!TimerEnable(&s_WakeTimer))", cycle)
		self.assertLess(cycle.index("TimerSetFrequency(&s_WakeTimer"),
			cycle.index("TimerEnable(&s_WakeTimer)"))
		self.assertLess(cycle.index("TimerEnable(&s_WakeTimer)"),
			cycle.index("nsTimerEnableTrigger(&s_WakeTimer"))
		self.assertLess(cycle.index("nsTimerEnableTrigger(&s_WakeTimer"),
			cycle.index("SystemSleep();"))

	def test_network_core_system_sleep_backend(self):
		power = (ROOT / "ARM/Nordic/src/system_power_nrfx.cpp").read_text(encoding="utf-8")
		start = power.index("#if defined(NRF5340_XXAA_NETWORK) || defined(NRF5340_XXAA_APPLICATION)")
		end = power.index("#endif", start)
		sleep = power[start:end]
		self.assertIn("void SystemSleep(void)", sleep)
		self.assertIn("if (__get_IPSR() != 0U)", sleep)
		self.assertIn("state = DisableInterrupt();", sleep)
		self.assertIn("scr = SCB->SCR;", sleep)
		self.assertIn("SCB_SCR_SLEEPDEEP_Msk", sleep)
		self.assertIn("SCB_SCR_SLEEPONEXIT_Msk", sleep)
		self.assertIn("__DSB();", sleep)
		self.assertIn("__WFI();", sleep)
		self.assertIn("SCB->SCR = scr;", sleep)
		self.assertIn("EnableInterrupt(state);", sleep)
		self.assertLess(sleep.index("DisableInterrupt()"), sleep.index("__WFI()"))
		self.assertLess(sleep.index("__WFI()"), sleep.index("EnableInterrupt(state)"))

	def test_managed_build_includes_net_demo(self):
		build = (ROOT / "tests/nordic/build_managed_power_clock.py").read_text(encoding="utf-8")
		self.assertIn('standard_projects.append(root / "exemples/PowerOffDemo/ioc")', build)
		self.assertNotIn('if target != "nrf5340_net":', build)


if __name__ == "__main__":
	unittest.main()
