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


class CoreClockTestInitialization(unittest.TestCase):
    def test_initialization_does_not_restart_lfclk(self):
        source = TEST.read_text(encoding="utf-8")
        start = source.index("static bool CoreClockTestTimerInit(int DevNo)")
        end = source.index("static void CoreClockTestFlush(void)", start)
        setup = source[start:end]
        self.assertIn("TimerInit(&s_Timer, &cfg)", setup)
        self.assertNotIn("TimerDisable(&s_Timer);", setup)

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
        self.assertIn("pOldTimer != nullptr && pOldTimer != pTimer", rtc)
        self.assertIn("s_bRtcAutoSelected[devno] = false;", rtc)
        self.assertIn("s_pnRfxRtcDev[candidate] == nullptr", rtc)
        self.assertIn("pOldTimer != nullptr && pOldTimer != pTimer", rtc)
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
        self.assertIn("static void CoreClockTestTimerTriggers(void)", source)
        self.assertIn("CoreClockTestTimerTriggers();", source)
        self.assertIn("TimerGetMaxTrigger(&s_Timer)", source)
        self.assertIn("s_Timer.FindAvailTrigger(&s_Timer)", source)
        self.assertIn("TIMER_TRIG_TYPE_SINGLE, CoreClockTestTriggerHandler", source)
        self.assertIn("TIMER_TRIG_TYPE_CONTINUOUS, CoreClockTestTriggerHandler", source)
        self.assertIn('CoreClockTestResult("Timer one-shot callback",', source)
        self.assertIn("TimerReset(&s_Timer);", source)
        self.assertIn("Switch trigger modes while the timer owns its clock.", source)
        self.assertNotIn("enabled = TimerEnable(&s_Timer);\n\tperiod = enabled ? nsTimerEnableTrigger", source)
        self.assertIn("Timer continuous diagnostics:", source)
        self.assertLess(source.index("TimerReset(&s_Timer);"),
            source.index("TIMER_TRIG_TYPE_CONTINUOUS, CoreClockTestTriggerHandler"))
        self.assertIn('CoreClockTestResult("Timer continuous callback",', source)
        self.assertIn('CoreClockTestResult("Timer counter advances",', source)
        self.assertIn('CoreClockTestResult("Timer trigger disabled",', source)
        self.assertIn('CoreClockTestResult("Timer trigger slot reusable",', source)
        self.assertNotIn("nRFxRtcSleepClaim(", source)
        self.assertNotIn("nRFxRtcSleepRelease(", source)

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



if __name__ == "__main__":
    unittest.main()
