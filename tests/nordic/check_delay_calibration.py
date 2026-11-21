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

    def test_existing_mcu_fallback_divisors_are_preserved(self):
        self.assertIn("ArmDelayClockSetDivisors(SystemCoreClock,", self.system)
        self.assertIn("13000000UL, 3000000UL", self.system)
        self.assertIn('#include "idelay_calibration.h"', self.system)

    def test_clock_change_factor_math_is_generic(self):
        helper = (ROOT / "ARM/include/idelay_calibration.h").read_text(
            encoding="utf-8")
        self.assertIn("bool ArmDelayClockSetDivisors(", helper)
        self.assertIn("usFactor = ((uint64_t)CpuHz << 16) / UsDivisor", helper)
        self.assertIn("nsFactor = ((uint64_t)CpuHz << 16) / NsDivisor", helper)

    def test_fixed_point_loop_units(self):
        self.assertIn("SystemMicroSecLoopCnt + 65535ULL) >> 16", self.delay)
        self.assertIn("SystemnsDelayFactor + 65535999ULL) / 65536000ULL",
                      self.delay)

    def test_250ms_scale_at_64mhz(self):
        frequency = 64000000
        us_factor = (frequency << 16) // 13000000
        ns_factor = (frequency << 16) // 3000000
        long_loops = (250000 * us_factor + 65535) >> 16
        short_loops = (250000000 * ns_factor + 65535999) // 65536000
        self.assertLess(abs(long_loops * 13 - 16000000), 128)
        # The SUBS/BNE loop takes three cycles per iteration on nRF52.
        # A previous divisor of 3189000 measured 235299 us.
        self.assertLess(abs(short_loops * 3000 - 16000000000), 128000)


if __name__ == "__main__":
    unittest.main()
