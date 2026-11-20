#!/usr/bin/env python3
"""Check the Nordic nRF52 delay calibration used by ClockDemo.

The constants calibrate assembly delay loops against nRF52832 hardware.
This checks fixed-point scaling and source assignments, not actual timing
on hardware. The nanosecond factor uses the measured effective loop rate.
"""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
SYSTEM = ROOT / "ARM/Nordic/nRF52/src/system_nrf52.c"
DELAY = ROOT / "ARM/include/idelay.h"


class Nrf52DelayCalibration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.system = SYSTEM.read_text(encoding="utf-8")
        cls.delay = DELAY.read_text(encoding="utf-8")

    def test_microsecond_calibration(self):
        match = re.search(r"SystemMicroSecLoopCnt\s*=\s*"
                          r"\(\(uint64_t\)SystemCoreClock\s*<<\s*16\)\s*/\s*"
                          r"(\d+)UL;", self.system)
        self.assertIsNotNone(match)
        self.assertEqual(int(match.group(1)), 13000000)

    def test_nanosecond_calibration(self):
        match = re.search(r"SystemnsDelayFactor\s*=\s*"
                          r"\(\(uint64_t\)SystemCoreClock\s*<<\s*16\)\s*/\s*"
                          r"(\d+)UL;", self.system)
        self.assertIsNotNone(match)
        self.assertEqual(int(match.group(1)), 3189000)

    def test_fixed_point_loop_units(self):
        self.assertIn("SystemMicroSecLoopCnt + 65535ULL) >> 16", self.delay)
        self.assertIn("SystemnsDelayFactor + 65535999ULL) / 65536000ULL",
                      self.delay)

    def test_250ms_scale_at_64mhz(self):
        frequency = 64000000
        us_factor = (frequency << 16) // 13000000
        ns_factor = (frequency << 16) // 3189000
        long_loops = (250000 * us_factor + 65535) >> 16
        short_loops = (250000000 * ns_factor + 65535999) // 65536000
        self.assertLess(abs(long_loops * 13 - 16000000), 128)
        # Measured 255500 us at the previous 3120000 divisor. The
        # adjusted effective loop calibration is 3.189 cycles.
        self.assertLess(abs(short_loops * 3189 - 16000000000), 128000)


if __name__ == "__main__":
    unittest.main()
