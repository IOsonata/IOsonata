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

	def test_managed_build_includes_net_demo(self):
		build = (ROOT / "tests/nordic/build_managed_power_clock.py").read_text(encoding="utf-8")
		self.assertIn('standard_projects.append(root / "exemples/PowerOffDemo/ioc")', build)
		self.assertNotIn('if target != "nrf5340_net":', build)


if __name__ == "__main__":
	unittest.main()
