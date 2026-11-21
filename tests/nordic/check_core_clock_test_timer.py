#!/usr/bin/env python3
"""Check CoreClockTest timer measurement and shutdown ordering."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
TEST = ROOT / "exemples/misc/core_clock_test.cpp"


class CoreClockTestTimer(unittest.TestCase):
    def test_elapsed_read_precedes_disable(self):
        source = TEST.read_text(encoding="utf-8")
        start = source.index("static void CoreClockTestTimer(int DevNo)\n{")
        end = source.index("// Check that disabling one timer", start)
        timer = source[start:end]
        self.assertIn(
            "elapsed = TimerGetMicrosecond(&s_Timer) - start;\n"
            "\t\tTimerDisable(&s_Timer);",
            timer,
        )
        self.assertLess(
            timer.index("elapsed = TimerGetMicrosecond(&s_Timer) - start;"),
            timer.index("count = s_InterruptCount;"),
        )


if __name__ == "__main__":
    unittest.main()
