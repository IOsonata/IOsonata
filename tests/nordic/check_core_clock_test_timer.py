#!/usr/bin/env python3
"""Check CoreClockTest timer measurement and shutdown ordering."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
TEST = ROOT / "exemples/misc/core_clock_test.cpp"


class CoreClockTestTimer(unittest.TestCase):
    def test_lf_timer_keeps_counting_after_release(self):
        src = TEST.read_text()
        begin = src.index("timerRunning = TimerInit(")
        end = src.index("static void CoreClockTestPowerMode(", begin)
        lf = src[begin:end]
        steps = ("CoreClockTestRequests(SystemLowFreqClockEnable, SystemLowFreqClockDisable)",
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
        end = source.index("static void CoreClockTestSharedClockTest(", start)
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

    def test_sdelay_uses_only_lf_rtc_and_sleep(self):
        src = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(
            encoding="utf-8")
        start = src.index('extern "C" bool sDelay(uint32_t Seconds)')
        delay = src[start:]
        for term in ("TimerInit(&s_SleepTimer, &cfg)",
            "nsTimerEnableTrigger(&s_SleepTimer, 0,",
            "TIMER_TRIG_TYPE_SINGLE", "__WFE();",
            "s_bSleepTimerDone", "SCB_SCR_SEVONPEND_Msk",
            "TimerPowerOff(&s_SleepTimer)",
            "NRF_SDELAY_COLD_OVERHEAD_TICKS",
            "NRF_SDELAY_WARM_OVERHEAD_TICKS",
            "warmClock = nRFxSleepLfClockRunning();"):
            self.assertIn(term, delay)
        for forbidden in ("msDelay(", "usDelay(", "nsDelay(", "DWT->",
            "SysTick", "malloc(", "new "):
            self.assertNotIn(forbidden, delay)
        self.assertIn('extern "C" bool sDelay(uint32_t Seconds)', src)
        self.assertIn("initialized = TimerInit(&s_SleepTimer, &cfg);", delay)
        self.assertIn("if (initialized)", delay)
        self.assertIn("TimerDisableTrigger(&s_SleepTimer, 0);", delay)
        self.assertNotIn("nRFxRtcSleepClaim(", delay)
        self.assertNotIn("nRFxRtcSleepRelease(", delay)

    def test_nrf54l_grtc_distinguishes_cold_lfxo(self):
        source = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(
            encoding="utf-8")
        section = source.split("static bool nRFxGrtcSleepClockWarm(void)", 1)[1]
        section = section.split("static TimerDev_t s_SleepTimer;", 1)[0]
        self.assertIn("NRF_GRTC->MODE & GRTC_MODE_SYSCOUNTEREN_Msk", section)
        self.assertIn("GetLowFreqOscType() != OSC_TYPE_XTAL", section)
        self.assertIn("NRF_CLOCK_S->LFCLK.STAT", section)
        self.assertIn("CLOCK_LFCLK_STAT_STATE_Msk", section)
        self.assertIn("CLOCK_LFCLK_STAT_SRC_Msk", section)
        self.assertIn("CLOCK_LFCLK_SRC_SRC_LFXO", section)
        self.assertNotIn("TASKS_START", section)
        self.assertNotIn("TASKS_STOP", section)
        self.assertIn("warmClock = nRFxGrtcSleepClockWarm();", source)
        self.assertLess(source.index("static bool nRFxGrtcSleepClockWarm(void)"),
                        source.index("int TimerEnablePeiTrigger("))

    def test_nrf54l15_provisional_cold_compensation(self):
        source = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(
            encoding="utf-8")
        self.assertIn("#elif defined(NRF54L15_XXAA)", source)
        self.assertIn("#define NRF_SDELAY_COLD_DEFAULT_TICKS 434775", source)
        self.assertIn("#define NRF_SDELAY_WARM_DEFAULT_TICKS 0", source)
        # The one recorded 1,434,775-us cold call becomes 1 second
        # after shortening only the cold GRTC compare. Warm calls must
        # not receive this correction.
        self.assertEqual(1000000, 1434775 - 434775)
        self.assertEqual(1000047, 1000047 - 0)

    def test_nrf54lm20a_cold_compensation(self):
        source = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(
            encoding="utf-8")
        start = source.index("#elif defined(NRF54LM20A_XXAA)")
        end = source.index("#elif defined(NRF52832_XXAA)", start)
        section = source[start:end]
        self.assertIn("#define NRF_SDELAY_COLD_DEFAULT_TICKS 433611",
                      section)
        self.assertIn("#define NRF_SDELAY_WARM_DEFAULT_TICKS 0",
                      section)
        # GRTC counter is 1 MHz, so 433611 ticks compensate 433611 us.
        for elapsed in (1433700, 1433521):
            self.assertLessEqual(abs(elapsed - 433611 - 1000000), 1000)

    def test_nrf5340_app_uses_measured_startup_for_sdelay(self):
        clock = (ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp").read_text(
            encoding="utf-8")
        delay = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(
            encoding="utf-8")
        self.assertIn("s_nRFxLfclkStartWaitUs", clock)
        self.assertIn("s_nRFxLfclkStartWaitUs = 0U;", clock)
        self.assertIn("s_nRFxLfclkStartWaitUs += NRFX_LFCLK_START_POLL_US;", clock)
        self.assertIn("nRFxSleepLfClockStartupWaitUs(void)", clock)
        self.assertIn("startupWaitUs = initialized && !warmClock ?", delay)
        self.assertIn("nRFxSleepLfClockStartupWaitUs() : 0U;", delay)
        self.assertIn("if (!warmClock && Seconds == chunk)", delay)
        self.assertIn("(uint64_t)startupWaitUs *", delay)
        self.assertIn("NRF_SDELAY_WARM_OVERHEAD_TICKS;", delay)
        # 260.5-ms startup uses nearly 8,539 ticks; immediate startup
        # must not subtract 8,539 ticks from an otherwise warm call.
        frequency = 32768
        for start_us in (0, 200, 260500):
            ticks = (start_us * frequency + 500000) // 1000000 - 11
            self.assertLess(abs(ticks - (start_us * frequency // 1000000)), 12)

    def test_nrf5340_per_core_signed_compensation(self):
        source = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(
            encoding="utf-8")
        clock = (ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp").read_text(
            encoding="utf-8")
        self.assertIn("#define NRF_SDELAY_COLD_DEFAULT_TICKS 8539", source)
        self.assertIn("#define NRF_SDELAY_WARM_DEFAULT_TICKS (-11)", source)
        self.assertIn("#define NRF_SDELAY_COLD_DEFAULT_TICKS 8", source)
        self.assertIn("#define NRF_SDELAY_WARM_DEFAULT_TICKS (-10)", source)
        self.assertIn("#define NRF_SDELAY_COLD_DEFAULT_TICKS 0", source)
        self.assertIn("#define NRF_SDELAY_WARM_DEFAULT_TICKS 0", source)
        self.assertIn("int32_t compensation;", source)
        self.assertIn("if (compensation > 0)", source)
        self.assertIn("else if (compensation < 0)", source)
        self.assertIn("ticks -= (uint32_t)compensation;", source)
        self.assertIn("ticks += (uint32_t)(-(int64_t)compensation);", source)
        self.assertIn('extern "C" bool nRFxSleepLfClockRunning(void)', clock)
        self.assertIn("NRFX_LFCLKSTAT & NRFX_LFCLK_STATE_Msk", clock)

    def test_signed_calibration_math(self):
        for base, adjustment, expected in (
            (32768, 8539, 24229),
            (32768, -11, 32779),
            (32768, 8, 32760),
            (32768, -10, 32778),
            (32768, 0, 32768),
        ):
            ticks = base
            if adjustment > 0:
                ticks = max(2, ticks - adjustment)
            elif adjustment < 0:
                ticks += -adjustment
            self.assertEqual(ticks, expected)


    def test_nrf52832_cold_calibration_and_ns_probe(self):
        clock = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(
            encoding="utf-8")
        self.assertIn("defined(NRF52832_XXAA) || defined(NRF52832_XXAB)", clock)
        self.assertIn("#define NRF_SDELAY_COLD_DEFAULT_TICKS 8604", clock)
        self.assertIn("#elif defined(NRF52840_XXAA)", clock)
        self.assertIn("#define NRF_SDELAY_COLD_DEFAULT_TICKS 8208", clock)
        self.assertIn("#define NRF_SDELAY_WARM_DEFAULT_TICKS 0", clock)


if __name__ == "__main__":
    unittest.main()
