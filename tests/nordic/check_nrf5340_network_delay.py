#!/usr/bin/env python3
"""Check nRF5340 network-core busy-loop delay calibration."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
SYSTEM = ROOT / "ARM/Nordic/nRF53/nRF5340_Net/lib/src/system_nrf5340_network.c"
DELAY = ROOT / "ARM/include/idelay.h"


class Nrf5340NetworkDelay(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		cls.system = SYSTEM.read_text(encoding="utf-8")
		cls.delay = DELAY.read_text(encoding="utf-8")

	def test_calibration(self):
		micro = re.search(r"SystemMicroSecLoopCnt\s*=\s*"
			r"\(\(uint64_t\)SystemCoreClock\s*<<\s*16\)\s*/\s*(\d+)UL;", self.system)
		nano = re.search(r"SystemnsDelayFactor\s*=\s*"
			r"\(\(uint64_t\)SystemCoreClock\s*<<\s*16\)\s*/\s*(\d+)UL;", self.system)
		self.assertIsNotNone(micro)
		self.assertIsNotNone(nano)
		self.assertEqual(int(micro.group(1)), 23000000)
		self.assertEqual(int(nano.group(1)), 3000000)

	def test_250ms_scale(self):
		frequency = 64000000
		micro = (frequency << 16) // 23000000
		loops = (250000 * micro + 65535) >> 16
		self.assertLess(abs(loops * 23 - 16000000), 5000)
		self.assertIn("usDelay(ms * 1000U)", self.delay)


if __name__ == "__main__":
	unittest.main()
