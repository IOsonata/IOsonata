#!/usr/bin/env python3
"""Check CoreClockTest timer measurement and shutdown ordering."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
TEST = ROOT / "exemples/misc/core_clock_test.cpp"


class CoreClockTestTimer(unittest.TestCase):
    def test_one_lf_and_one_hf_timer_keep_counting_after_release(self):
        src = TEST.read_text()
        begin = src.index("running = present &&")
        end = src.index("static void CoreClockTestPowerMode(", begin)
        lf = src[begin:end]
        self.assertEqual(src.count("CoreClockTestSharedClock(false);"), 1)
        self.assertEqual(src.count("CoreClockTestSharedClock(true);"), 1)
        steps = ("CoreClockTestRequests(SystemHighFreqClockEnable, SystemHighFreqClockDisable)",
                 "CoreClockTestRequests(SystemLowFreqClockEnable, SystemLowFreqClockDisable)",
                 "start = TimerGetTickCount", "msDelay(10)", "!= start", "CoreClockTestReleaseTimer(")
        positions = [lf.index(step) for step in steps]
        self.assertEqual(positions, sorted(positions))
        self.assertNotIn("nsTimerEnableTrigger", src)
        self.assertEqual(src.count("static TimerDev_t "), 1)

    def test_public_delay_functions_own_original_countdowns(self):
        delay = (ROOT / "ARM/include/idelay.h").read_text(
            encoding="utf-8")
        us_start = delay.index("static inline __attribute__((always_inline)) "
            "void usDelay(uint32_t cnt)")
        ns_start = delay.index("static inline __attribute__((always_inline)) void nsDelay(uint32_t cnt)", us_start)
        end = delay.index('#include "system_core_clock_calibration.h"', ns_start)
        us_loop = delay[us_start:ns_start]
        ns_loop = delay[ns_start:end]
        self.assertEqual(us_loop.count('" NOP\\n"'), 10)
        for loop in (us_loop, ns_loop):
            self.assertIn('" MOVS %[remaining], %[count]\\n"', loop)
            self.assertIn('" SUBS %[remaining], #1\\n"', loop)
            self.assertIn('" BGT 1b\\n"', loop)
            self.assertIn("chunk = loops > INT32_MAX ? INT32_MAX", loop)
            self.assertNotIn('" BNE ', loop)
            self.assertNotIn("NRF52_SERIES", loop)
        for forbidden in ("ArmDelayUsLoop(", "ArmDelayNsLoop(",
            "DelayCalibrationProbeNs("):
            self.assertNotIn(forbidden, delay)
        self.assertIn("static inline bool ArmDelayCalibrate(", (ROOT / "ARM/include/system_core_clock_calibration.h").read_text(encoding="utf-8"))
        self.assertIn("static inline bool DelayCalibrationMeasure(", (ROOT / "ARM/include/system_core_clock_calibration.h").read_text(encoding="utf-8"))
        self.assertNotIn("ArmDelayCalibrationDwtAllowed", delay)
        self.assertNotIn("NRF52_SERIES", delay)
        self.assertIn('" .p2align 5\\n"', ns_loop)


class CoreClockTestLfLifecycle(unittest.TestCase):
    def test_enable_sequence_reports_individual_results(self):
        source = TEST.read_text(encoding="utf-8")
        start = source.index("static bool CoreClockTestRequests(")
        end = source.index("static void CoreClockTestSharedClock(", start)
        requests = source[start:end]
        self.assertIn("!Enable()", requests)
        self.assertIn("second = Enable();", requests)
        self.assertIn("restarted = Enable();", requests)
        self.assertEqual(requests.count("Disable();"), 3)


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
    def test_temporary_rtc_uses_generic_timer_lifecycle(self):
        rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(
            encoding="utf-8")
        self.assertNotIn("nRFxRtcSleepClaim(", rtc)
        self.assertNotIn("nRFxRtcSleepRelease(", rtc)
        self.assertNotIn("s_pnRfxRtcSleepOwner", rtc)
        self.assertIn("automatic = pCfg->DevNo == -1;", rtc)
        self.assertIn("first = 1;", rtc)
        self.assertIn("first = 2;", rtc)
        self.assertIn("candidate >= first", rtc)
        self.assertIn("s_pnRfxRtcDev[candidate] == nullptr", rtc)
        self.assertNotIn("s_pnRfxRtcSleepOwner", rtc)
        self.assertIn("SystemLowFreqClockDisable();", rtc)

    def test_disable_preserves_selected_timer_until_power_off(self):
        for path, name, assignment in (
            ("src/timer_lf_nrfx.cpp", "nRFxRtc", "s_pnRfxRtcDev[devno] = nullptr;"),
            ("nRF54/src/timer_lf_nrf54.cpp", "nRFxGrtc", "pRtc->pTimer = nullptr;"),
        ):
            with self.subTest(timer=name):
                src = (ROOT / "ARM/Nordic" / path).read_text()
                disable = src.split("static void " + name + "Disable(", 1)[1].split("\n}", 1)[0]
                off = src.split("static bool " + name + "PowerOff(", 1)[1].split("\n}", 1)[0]
                self.assertNotIn(assignment, disable)
                self.assertIn(assignment, off)
                self.assertIn("pTimer->DevNo = -1;", off)

    def test_clock_start_time_counted_for_sdelay(self):
        clock = (ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp").read_text(
            encoding="utf-8")
        timer = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(
            encoding="utf-8")
        lf = clock.split("bool SystemLowFreqClockEnable(void)\n{", 1)[1]
        lf = lf.split("\n}\n", 1)[0]
        hf = clock.split("bool SystemHighFreqClockEnable(void)\n{", 1)[1]
        hf = hf.split("\n}\n", 1)[0]
        self.assertIn("s_nRFxLfclkStartWaitUs = 0U;", lf)
        self.assertIn("s_nRFxHfxoStartWaitUs = 0U;", hf)
        self.assertIn("s_nRFxLfclkStartWaitUs += NRFX_LFCLK_START_POLL_US;", clock)
        self.assertIn("s_nRFxHfxoStartWaitUs += NRFX_HFXO_START_POLL_US;", clock)
        self.assertIn("uint32_t SystemLowFreqClockStartTime(void)", clock)
        self.assertIn("uint32_t SystemHighFreqClockStartTime(void)", clock)
        # The generic src/sdelay.cpp is the only sDelay
        self.assertNotIn("sDelay", timer)
        self.assertNotIn("nRFxSleepLfClock", clock)


if __name__ == "__main__":
    unittest.main()
