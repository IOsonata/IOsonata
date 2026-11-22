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


class NordicSleepDelay(unittest.TestCase):
    def test_temporary_rtc_claim_and_release(self):
        rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(
            encoding="utf-8")
        self.assertIn("nRFxRtcSleepClaim(", rtc)
        self.assertIn("nRFxRtcSleepRelease(", rtc)
        self.assertIn("s_pnRfxRtcDev[i] == nullptr", rtc)
        self.assertIn("s_pnRfxRtcSleepOwner[i] == nullptr", rtc)
        self.assertIn("first = 1;", rtc)
        self.assertIn("first = 2;", rtc)
        self.assertIn("i >= first", rtc)
        self.assertIn("TimerDisable(pTimer);", rtc)
        self.assertIn("s_pnRfxRtcDev[devno] = nullptr;", rtc)
        self.assertIn("s_pnRfxRtcSleepOwner[devno] = nullptr;", rtc)
        self.assertIn("s_pnRfxRtcSleepOwner[pCfg->DevNo] != pTimer", rtc)

    def test_sdelay_uses_only_lf_rtc_and_sleep(self):
        src = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(
            encoding="utf-8")
        start = src.index('extern "C" bool sDelay(uint32_t Seconds)')
        delay = src[start:]
        for term in ("nRFxRtcSleepClaim(&s_SleepTimer, &cfg)",
            "nsTimerEnableTrigger(&s_SleepTimer, 0,",
            "TIMER_TRIG_TYPE_SINGLE", "__WFE();",
            "s_bSleepTimerDone", "SCB_SCR_SEVONPEND_Msk",
            "nRFxRtcSleepRelease(&s_SleepTimer)", "NRF_SDELAY_OVERHEAD_TICKS"):
            self.assertIn(term, delay)
        for forbidden in ("msDelay(", "usDelay(", "nsDelay(", "DWT->",
            "SysTick", "malloc(", "new "):
            self.assertNotIn(forbidden, delay)
        self.assertIn('extern "C" bool sDelay(uint32_t Seconds)', src)

    def test_core_clock_test_reports_end_to_end_calibration(self):
        src = TEST.read_text(encoding="utf-8")
        start = src.index("static void CoreClockTestSleepDelay(void)")
        end = src.index("static bool CoreClockTestDelaySample(", start)
        test = src[start:end]
        self.assertIn("started = TimerGetMicrosecond(&s_RetainedTimer);", test)
        self.assertIn("ok = sDelay(1U);", test)
        self.assertIn("TimerDisable(&s_RetainedTimer);", test)
        self.assertIn("sDelay overhead suggestion", test)
        self.assertLess(test.index("ok = sDelay(1U);"),
            test.index("elapsed = TimerGetMicrosecond(&s_RetainedTimer) - started;"))
        self.assertIn("CoreClockTestSleepDelay();", src)


if __name__ == "__main__":
    unittest.main()
