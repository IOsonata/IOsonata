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

    def test_low_frequency_timer_has_independent_hf_reference(self):
        source = TEST.read_text(encoding="utf-8")
        start = source.index("static void CoreClockTestTimer(int DevNo)\n{")
        end = source.index("// Check that disabling one timer", start)
        timer = source[start:end]
        self.assertIn("DevNo < TimerGetLowFreqDevCount()", timer)
        self.assertIn("TimerGetHighFreqDevNo()", timer)
        self.assertIn("hfCfg.EvtHandler = nullptr;", timer)
        self.assertIn("TimerInit(&s_RetainedTimer, &hfCfg)", timer)
        self.assertIn("TimerEnable(&s_RetainedTimer)", timer)
        self.assertIn("TimerGetMicrosecond(&s_RetainedTimer) - hfStart", timer)
        self.assertIn("LF/HF reference: LF %u us, HF %u us", timer)
        self.assertIn("TimerDisable(&s_RetainedTimer);", timer)
        for forbidden in ("NRF_CLOCK", "LFCLKSTAT", "HFCLKCTRL", "DWT->"):
            self.assertNotIn(forbidden, timer)


class CoreClockTestLfLifecycle(unittest.TestCase):
    def test_enable_sequence_reports_individual_results(self):
        source = TEST.read_text(encoding="utf-8")
        start = source.index("static void CoreClockTestSharedClockTest(void)")
        end = source.index("static void CoreClockTestPowerMode(void)", start)
        lifecycle = source[start:end]
        self.assertIn("first = SystemLowFreqClockEnable();", lifecycle)
        self.assertIn("second = first && SystemLowFreqClockEnable();", lifecycle)
        self.assertIn("third = SystemLowFreqClockEnable();", lifecycle)
        self.assertIn("LF lifecycle: first %u, second %u, after release %u", lifecycle)


class NordicRtcStopOwnership(unittest.TestCase):
    def test_nrf52_stop_completes_before_lfclk_release(self):
        source = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(
            encoding="utf-8")
        start = source.index("static void nRFxRtcDisable(")
        end = source.index("// CLEAR task", start)
        disable = source[start:end]
        self.assertLess(
            disable.index("TASKS_STOP = 1"),
            disable.index("usDelay(50U);"))
        self.assertLess(
            disable.index("usDelay(50U);"),
            disable.index("SystemLowFreqClockDisable();"))
        self.assertIn("#if defined(NRF52_SERIES)", disable)


if __name__ == "__main__":
    unittest.main()
