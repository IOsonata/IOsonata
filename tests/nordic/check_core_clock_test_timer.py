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
        self.assertIn("elapsed = TimerGetMicrosecond(pTimer) - start;", timer)
        measure = timer.index("elapsed = TimerGetMicrosecond(pTimer) - start;")
        disable = timer.index("TimerDisable(pTimer);", measure)
        self.assertLess(measure, disable)
        self.assertLess(disable, timer.index("count = s_InterruptCount;", measure))

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


class CoreClockTestInitialization(unittest.TestCase):
    def test_initialization_does_not_restart_lfclk(self):
        source = TEST.read_text(encoding="utf-8")
        start = source.index("static bool CoreClockTestTimerInit(int DevNo, TimerDev_t *pTimer)")
        end = source.index("static void CoreClockTestFlush(void)", start)
        setup = source[start:end]
        self.assertIn("TimerInit(pTimer, &cfg)", setup)
        self.assertNotIn("TimerDisable(pTimer);", setup)

    def test_factors_use_public_nsdelay(self):
        source = TEST.read_text(encoding="utf-8")
        start = source.index("static bool CoreClockTestDelaySample(")
        end = source.index("static void CoreClockTestDelayCalibrationRecord(void)\n{", start)
        sample = source[start:end]
        self.assertIn("nsDelay(100000000U);", sample)
        self.assertIn("if (DelayCalibrationMeasure(pSample))", sample)
        self.assertIn("Delay calibration DWT %u Hz:", sample)
        self.assertIn("DelayCalibrationFromTimer(cpuHz, usElapsed, nsPublicElapsed,", sample)
        self.assertNotIn("DelayCalibrationProbeNs(", sample)
        self.assertNotIn("raw ns loop", sample)
        self.assertIn('CoreClockTestResult("nsDelay repeated HF Timer 100 ms",', sample)
        self.assertNotIn("nsSameSiteElapsed", sample)
        self.assertIn('CoreClockTestResult("nsDelay factor unchanged during timing",', sample)
        self.assertEqual(sample.count("irqState = __get_PRIMASK();"), 3)
        self.assertEqual(sample.count("__disable_irq();"), 3)
        self.assertEqual(sample.count("__set_PRIMASK(irqState);"), 3)
        for call, elapsed in (
            ("msDelay(100);", "usElapsed"),
            ("nsDelay(100000000U);", "nsPublicElapsed"),
            ("nsDelay(100000000U);", "nsRepeatElapsed"),
        ):
            start = sample.index(elapsed + " = TimerGetMicrosecond")
            self.assertIn("__set_PRIMASK(irqState);",
                sample[start:start + 120])
        self.assertLess(sample.rindex("__set_PRIMASK(irqState);"),
            sample.index('CoreClockTestPrint("nsDelay repeat'))

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
        self.assertIn("s_bRtcAutoSelected[devno] = false;", rtc)
        self.assertIn("s_pnRfxRtcDev[candidate] == nullptr", rtc)
        self.assertIn("s_bRtcAutoSelected[pSelectedCfg->DevNo] = automatic;", rtc)
        self.assertNotIn("s_pnRfxRtcSleepOwner", rtc)
        self.assertIn("SystemLowFreqClockDisable();", rtc)

    def test_sdelay_uses_only_lf_rtc_and_sleep(self):
        src = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(
            encoding="utf-8")
        start = src.index('extern "C" bool sDelay(uint32_t Seconds)')
        delay = src[start:]
        for term in ("TimerInit(&s_SleepTimer, &cfg)",
            "nsTimerEnableTrigger(&s_SleepTimer, 0,",
            "TIMER_TRIG_TYPE_SINGLE", "__WFE();",
            "s_bSleepTimerDone", "SCB_SCR_SEVONPEND_Msk",
            "TimerDisable(&s_SleepTimer)",
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

    def test_sdelay_reports_architecture_neutral_time_error(self):
        example = TEST.read_text(encoding="utf-8")
        start = example.index("static void CoreClockTestSleepDelayMeasure(")
        end = example.index("static void CoreClockTestSleepDelay(void)", start)
        section = example[start:end]
        self.assertIn("sDelay %s overhead: %s %u us", section)
        self.assertNotIn("32768U", section)
        self.assertNotIn("LF ticks", section)

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
        example = TEST.read_text(encoding="utf-8")
        self.assertIn("defined(NRF52832_XXAA) || defined(NRF52832_XXAB)", clock)
        self.assertIn("#define NRF_SDELAY_COLD_DEFAULT_TICKS 8604", clock)
        self.assertIn("#elif defined(NRF52840_XXAA)", clock)
        self.assertIn("#define NRF_SDELAY_COLD_DEFAULT_TICKS 8208", clock)
        self.assertIn("#define NRF_SDELAY_WARM_DEFAULT_TICKS 0", clock)
        self.assertIn("nsFactorBefore = SystemnsDelayFactor;", example)
        self.assertIn("nsFactorAfter = SystemnsDelayFactor;", example)
        self.assertIn("nsDelay repeat %u us", example)
        self.assertIn("nsRepeatElapsed = TimerGetMicrosecond", example)


    def test_core_clock_test_reports_end_to_end_calibration(self):
        src = TEST.read_text(encoding="utf-8")
        start = src.index("static void CoreClockTestSleepDelayMeasure(")
        end = src.index("static bool CoreClockTestDelaySample(", start)
        test = src[start:end]
        self.assertIn("started = TimerGetMicrosecond(&s_RetainedTimer);", test)
        self.assertIn("ok = sDelay(1U);", test)
        self.assertIn("TimerDisable(&s_RetainedTimer);", test)
        self.assertIn("sDelay %s overhead:", test)
        self.assertIn('CoreClockTestSleepDelayMeasure("cold");', test)
        self.assertIn('CoreClockTestSleepDelayMeasure("warm");', test)
        self.assertIn('CoreClockTestSleepDelayMeasure("reuse");', test)
        self.assertIn("held = SystemLowFreqClockEnable();", test)
        self.assertIn("SystemLowFreqClockDisable();", test)
        self.assertLess(test.index("ok = sDelay(1U);"),
            test.index("elapsed = TimerGetMicrosecond(&s_RetainedTimer) - started;"))
        self.assertIn("CoreClockTestSleepDelay();", src)


    def test_core_clock_test_validates_timer_trigger_implementations(self):
        source = TEST.read_text(encoding="utf-8")
        self.assertIn("static void CoreClockTestTimerTriggers(TimerDev_t *pTimer)", source)
        self.assertIn("CoreClockTestTimerTriggers(pTimer);", source)
        self.assertIn("TimerGetMaxTrigger(pTimer)", source)
        self.assertIn("pTimer->FindAvailTrigger(pTimer)", source)
        self.assertIn("TIMER_TRIG_TYPE_SINGLE, CoreClockTestTriggerHandler", source)
        self.assertIn("TIMER_TRIG_TYPE_CONTINUOUS, CoreClockTestTriggerHandler", source)
        self.assertIn('CoreClockTestResult("Timer one-shot callback",', source)
        self.assertIn("TimerReset(pTimer);", source)
        self.assertIn("Switch trigger modes while the timer owns its clock.", source)
        self.assertNotIn("enabled = TimerEnable(pTimer);\n\tperiod = enabled ? nsTimerEnableTrigger", source)
        self.assertIn("Timer continuous diagnostics:", source)
        self.assertLess(source.index("TimerReset(pTimer);"),
            source.index("TIMER_TRIG_TYPE_CONTINUOUS, CoreClockTestTriggerHandler"))
        self.assertIn('CoreClockTestResult("Timer continuous callback",', source)
        self.assertIn('CoreClockTestResult("Timer counter advances",', source)
        self.assertIn('CoreClockTestResult("Timer trigger disabled",', source)
        self.assertIn('CoreClockTestResult("Timer trigger slot reusable",', source)
        self.assertNotIn("nRFxRtcSleepClaim(", source)
        self.assertNotIn("nRFxRtcSleepRelease(", source)


    def test_matrix_reuses_the_existing_reference_timer_handle(self):
        source = TEST.read_text(encoding="utf-8")
        matrix_start = source.index("static void CoreClockTestTimer(int DevNo)")
        matrix_end = source.index("// Exercise the public Timer trigger API", matrix_start)
        matrix = source[matrix_start:matrix_end]
        self.assertIn("pTimer = &s_Timer;", matrix)
        self.assertIn("DevNo == TimerGetHighFreqDevNo()", matrix)
        self.assertIn("pTimer = &s_RetainedTimer;", matrix)
        self.assertLess(matrix.index("pTimer = &s_RetainedTimer;"),
            matrix.index("CoreClockTestTimerInit(DevNo, pTimer)"))
        self.assertIn("CoreClockTestTimerTriggers(pTimer);", matrix)
        self.assertNotIn("NRF91_SERIES", matrix)
        # TIMER0 is the first HF reference, and its handle remains owned
        # across Disable. A second independent handle must not claim it.
        self.assertIn("cfg.DevNo = TimerGetHighFreqDevNo();", source)
        self.assertIn("TimerInit(&s_RetainedTimer, &cfg)", source)
        self.assertIn("TimerDisable(&s_RetainedTimer);", source)

    def test_clock_test_runs_contention_after_timer_matrix(self):
        source = TEST.read_text(encoding="utf-8")
        self.assertIn("static void CoreClockTestSleepDelayOwnership(int LowCount)", source)
        self.assertIn("s_LfTimerInitMask |= 1UL << DevNo;", source)
        self.assertIn("CoreClockTestSleepDelayOwnership(lowTimerCount);", source)
        self.assertIn('CoreClockTestResult("sDelay zero with LF timers occupied", sDelay(0U));', source)
        self.assertIn('CoreClockTestResult("sDelay occupied disabled LF timers", blocked);', source)
        self.assertIn("blocked = !sDelay(1U);", source)
        self.assertIn("s_LfTimerInitMask & expectedMask", source)
        self.assertLess(source.index("CoreClockTestConcurrentTimerTest(lowTimerCount, highTimerCount);"),
            source.index("CoreClockTestSleepDelayOwnership(lowTimerCount);"))
        self.assertLess(source.index("CoreClockTestSleepDelayOwnership(lowTimerCount);"),
            source.index("CoreClockTestDelayCalibrationReport();", source.index("int main(void)")))




    def test_generic_timer_power_off_reuses_same_structure(self):
        source = TEST.read_text(encoding="utf-8")
        begin = source.index("static void CoreClockTestTimerPowerOff(void)")
        end = source.index("int main(void)", begin)
        test = source[begin:end]
        self.assertIn("TimerGetHighFreqDevNo()", test)
        self.assertIn("s_RetainedTimer.PowerOff == nullptr", test)
        self.assertIn("stopped = TimerPowerOff(&s_RetainedTimer);", test)
        self.assertIn("TimerInit(&s_RetainedTimer, &cfg);", test)
        self.assertIn("TimerPowerOff(&s_RetainedTimer);", test)
        self.assertIn("!TimerPowerOff(&s_RetainedTimer)", test)
        self.assertLess(test.index("TimerPowerOff(&s_RetainedTimer);"),
            test.index("TimerInit(&s_RetainedTimer, &cfg);"))
        self.assertIn("CoreClockTestTimerPowerOff();", source)
        self.assertNotIn("NRF91_SERIES", test)


if __name__ == "__main__":
    unittest.main()
