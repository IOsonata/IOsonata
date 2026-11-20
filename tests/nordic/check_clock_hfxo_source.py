#!/usr/bin/env python3
"""Inspect Nordic HF/LF clock enable and disable source paths."""

from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
CLOCK = ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp"
MPSL = ROOT / "ARM/Nordic/src/nrf_mpsl.cpp"


def function(source, name, next_name):
    begin = source.index(name)
    end = source.index(next_name, begin)
    return source[begin:end]


class HfxoSourceChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.clock = CLOCK.read_text(encoding="utf-8")
        cls.mpsl = MPSL.read_text(encoding="utf-8")

    def test_public_api_uses_first_enable_last_disable(self):
        source = function(self.clock, "bool SystemHighFreqClockEnable(",
                          "// System OFF requires HFXO stopped")
        self.assertIn("s_nRFxHfxoReqCnt != 0U", source)
        self.assertIn("s_nRFxHfxoReqCnt++", source)
        self.assertIn("s_bNRFxHfxoTransition = true", source)
        self.assertIn("s_bNRFxHfxoTransition = false", source)
        self.assertIn("success = nRFxHfxoStart(&owner)", source)
        disable = function(self.clock, "void SystemHighFreqClockDisable(",
                           "// System OFF requires HFXO stopped")
        self.assertIn("s_nRFxHfxoReqCnt > 1U", disable)
        self.assertIn("s_nRFxHfxoReqCnt--", disable)
        self.assertIn("success = nRFxHfxoStop(owner)", disable)
        self.assertIn("if (success)", disable)
        self.assertNotIn("std::atomic", source)

    def test_generic_clock_sources_and_lifecycle(self):
        header = (ROOT / "include/coredev/system_core_clock.h").read_text(
            encoding="utf-8")
        rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(
            encoding="utf-8")
        for name in ("SystemHighFreqClockEnable",
                     "SystemHighFreqClockDisable",
                     "SystemLowFreqClockEnable",
                     "SystemLowFreqClockDisable"):
            self.assertIn(name + "(void)", header)
            self.assertIn(name + "(void)", self.clock)
        self.assertIn("g_McuOsc.CoreOsc.Type", self.clock)
        self.assertIn("g_McuOsc.LowPwrOsc.Type", self.clock)
        self.assertIn("SystemLowFreqClockEnable()", rtc)
        self.assertIn("SystemLowFreqClockDisable()", rtc)
        self.assertNotIn("nRFxLfclkAcquire", self.clock)
        self.assertNotIn("nRFxLfclkRelease", self.clock)
        self.assertNotIn("nRFxLfclkAcquire", rtc)
        self.assertNotIn("nRFxLfclkRelease", rtc)
        self.assertFalse(
            (ROOT / "ARM/Nordic/include/system_clock_nrfx.h").exists())

    def test_core_select_does_not_enable_hfxo(self):
        select = function(self.clock, "bool SystemCoreClockSelect(",
                          "bool SystemHighFreqClockEnable(")
        self.assertIn("g_McuOsc.CoreOsc.Type = ClkSrc", select)
        self.assertIn("s_nRFxHfxoReqCnt != 0U", select)
        self.assertNotIn("nRFxHfxoStart(", select)
        self.assertNotIn("nRFxHfxoStop(", select)

    def test_low_frequency_select_does_not_start_clock(self):
        select = function(self.clock, "bool SystemLowFreqClockSelect(",
                          "uint32_t SystemPeriphClockGet(")
        self.assertIn("g_McuOsc.LowPwrOsc.Type = ClkSrc", select)
        self.assertIn("s_nRFxLfclkReqCnt != 0U", select)
        self.assertNotIn("nRFxLfclkStart(", select)
        self.assertNotIn("TASKS_LFCLKSTART", select)

    def test_nordic_consumers_use_generic_clock_lifecycle(self):
        hf_timer = (ROOT / "ARM/Nordic/src/timer_hf_nrfx.cpp").read_text(
            encoding="utf-8")
        rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(
            encoding="utf-8")
        usb = (ROOT / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp").read_text(
            encoding="utf-8")
        dtm = (ROOT / "ARM/Nordic/nRF52/src/ble_dtm.c").read_text(
            encoding="utf-8")
        for source in (hf_timer, usb, dtm):
            self.assertIn("SystemHighFreqClockEnable()", source)
            self.assertNotIn("SystemCoreClockSelect(", source)
        self.assertIn("SystemHighFreqClockDisable()", hf_timer)
        self.assertIn("SystemHighFreqClockDisable()", usb)
        self.assertNotIn("s_nRfxHFClockSem", hf_timer)
        self.assertIn("SystemLowFreqClockEnable()", rtc)
        self.assertIn("SystemLowFreqClockDisable()", rtc)

    def test_ble_diagnostic_uses_no_application_clock_mode(self):
        source = (ROOT / "exemples/misc/hfxo_test_ble.cpp").read_text(
            encoding="utf-8")
        self.assertNotIn("HFXO_TEST_PRESTACK", source)
        self.assertNotIn("SystemCoreClockSelect(", source)
        self.assertIn("static int HfxoPreStackTest(void)", source)
        pre = function(source, "static int HfxoPreStackTest(void)",
                       "int main(void)")
        self.assertLess(pre.index("SystemHighFreqClockEnable()"),
                        pre.index("HfxoStackInit()"))
        self.assertLess(pre.index("HfxoStackInit()"),
                        pre.index("Pre-stack request disabled"))
        self.assertIn("Pre-stack handover", pre)
        self.assertNotIn("NRF_CLOCK->TASKS_HFCLKSTOP", pre)

    def test_pre_stack_success_continues_normal_checks(self):
        source = (ROOT / "exemples/misc/hfxo_test_ble.cpp").read_text(
            encoding="utf-8")
        main = function(source, "int main(void)", "return 0;")
        self.assertIn("preStackResult = HfxoPreStackTest()", main)
        self.assertIn("if (preStackResult != 0)", main)
        self.assertLess(main.index("preStackResult = HfxoPreStackTest()"),
                        main.index("initial = HfxoRunning()"))
        self.assertLess(main.index("initial = HfxoRunning()"),
                        main.index('HFXO_PRINT("HfxoTestBle: %d/5 checks PASS'))

    def test_mpsl_external_clock_selected_from_oscillator(self):
        mpsl = self.mpsl
        clock = self.clock
        header = (ROOT / "ARM/Nordic/include/nrf_mpsl.h").read_text(
            encoding="utf-8")
        for source in (mpsl, clock, header):
            self.assertNotIn("IO_NRFX_MPSL_EXT_CLOCK", source)
        self.assertIn("mpsl_clock_ctrl_source_register(", mpsl)
        self.assertIn("externalClock = lfosc->Type == OSC_TYPE_XTAL", mpsl)
        self.assertIn("if (externalClock)", mpsl)
        self.assertIn("mpsl_init(externalClock ? nullptr : &lfclk", mpsl)
        self.assertIn("nRFxMpslExternalClockSet(true)", mpsl)
        self.assertIn("nRFxMpslExternalClockGet()", mpsl)
        self.assertIn("s_MpslExternalLf.accuracy_ppm", mpsl)
        self.assertIn("lfclk_calibration_start = MpslExternalLfCalibrationStart", mpsl)
        self.assertIn("lfclk_calibration_is_enabled = MpslExternalLfCalibrationIsEnabled", mpsl)
        self.assertNotIn("lfclk_calibration_start = nullptr", mpsl)
        self.assertNotIn("lfclk_calibration_is_enabled = nullptr", mpsl)
        self.assertIn("static void MpslExternalLfCalibrationStart(void)", mpsl)
        self.assertIn("static bool MpslExternalLfCalibrationIsEnabled(void)", mpsl)
        self.assertIn("mpsl_init(&lfclk, MPSL_LOW_PRIO_IRQn", mpsl)

    def test_mpsl_hf_callbacks_are_nonblocking(self):
        request = function(self.clock, "void nRFxMpslHfclkRequest(",
                           "void nRFxMpslHfclkRelease(")
        release = function(self.clock, "void nRFxMpslHfclkRelease(",
                           "bool nRFxMpslHfclkIsRunning(")
        for block in (request, release):
            self.assertNotIn("nRFxHfxoStart(", block)
            self.assertNotIn("nRFxHfxoStop(", block)
            self.assertNotIn("mpsl_clock_hfclk_src_request(", block)
            self.assertNotIn("NRFX_HFXO_WAIT_LOOPS", block)
            self.assertNotIn("SystemHighFreqClockEnable(", block)
        self.assertIn("s_nRFxMpslHfxoReqCnt++", request)
        self.assertIn("s_nRFxMpslHfxoReqCnt--", release)
        self.assertIn("NRFX_HFXO_START", request)
        self.assertIn("NRFX_HFXO_STOP", release)

    def test_mpsl_clock_requests_share_hardware_ownership(self):
        disable = function(self.clock, "void SystemHighFreqClockDisable(",
                           "// System OFF requires HFXO stopped")
        self.assertIn("s_nRFxMpslHfxoReqCnt != 0U", disable)
        self.assertIn("s_bNRFxMpslHfxoOwned = true", disable)
        self.assertIn("s_nRFxHfxoReqCnt = 0U", disable)

    def test_softdevice_probe_requires_sdk_runtime_state(self):
        source = function(self.clock, "static bool nRFxClockSdEnabled(",
                          "static bool nRFxHfxoRunning(")
        self.assertIn("nrf_sdh_is_enabled != nullptr", source)
        self.assertIn("nrf_sdh_is_enabled()", source)
        self.assertNotIn("sd_softdevice_is_enabled(", self.clock)
        self.assertNotIn("SOFTDEVICE_INFO_STRUCT_OFFSET", self.clock)
        self.assertIn("#pragma weak nrf_sdh_is_enabled", self.clock)

    def test_disable_uses_original_clock_owner(self):
        source = function(self.clock, "static bool nRFxHfxoStop(",
                          "bool SystemCoreClockSelect(")
        self.assertIn("nRFxMpslHfxo(false) == 0", source)
        self.assertIn("sd_clock_hfclk_release() == NRF_SUCCESS", source)
        self.assertIn("if (nRFxClockSdEnabled())", source)
        self.assertIn("if (nRFxMpslActive() && !s_bNRFxMpslExternalClock)",
                      source)
        self.assertIn("if (nRFxMpslActive())", source)
        self.assertIn("#else", source)
        self.assertIn("#endif", source)
        stop = source.index("NRFX_HFXO_STOP = 1U")
        self.assertLess(source.index("if (nRFxClockSdEnabled())"), stop)
        self.assertLess(source.index(
            "if (nRFxMpslActive() && !s_bNRFxMpslExternalClock)"), stop)
        self.assertLess(source.index("if (nRFxMpslActive())"), stop)
        self.assertIn("NRFX_HFXO_OWNER_NONE", self.clock)

    def test_ble_console_uses_interrupt_tx_completion(self):
        source = (ROOT / "exemples/misc/hfxo_test_ble.cpp").read_text(
            encoding="utf-8")
        self.assertIn(".bIntMode = true", source)
        self.assertIn(".EvtCallback = HfxoUartEvent", source)
        callback = function(source, "static int HfxoUartEvent(",
                            "static const IOPinCfg_t")
        self.assertIn("UART_EVT_TXREADY", callback)
        self.assertIn("__SEV();", callback)
        drain = function(source, "static void HfxoConsoleDrain(void)",
                         "#else")
        self.assertIn("SysLogFlush(SysLogGet())", drain)
        self.assertIn("CFifoPeek(SysLogGet()->hFifo)", drain)
        self.assertIn("CFifoUsed(pUart->hTxFifo)", drain)
        self.assertIn("pUart->bTxReady", drain)
        self.assertIn("__WFE();", drain)
        self.assertNotIn("EVENTS_ENDTX", drain)
        main = function(source, "int main(void)", "return 0;")
        self.assertIn("HfxoConsoleDrain();", main)
        self.assertLess(main.index("HfxoConsoleDrain();"),
                        main.rindex("while (1)"))

    def test_s132_shutdown_rechecks_direct_hfxo_lifecycle(self):
        source = (ROOT / "exemples/misc/hfxo_test_ble.cpp").read_text(
            encoding="utf-8")
        main = function(source, "int main(void)", "return 0;")
        self.assertIn("nrf_sdh_disable_request()", main)
        self.assertIn('HfxoResult("S132 shutdown"', main)
        self.assertIn('HfxoResult("Direct enable after S132"', main)
        self.assertIn('HfxoResult("Direct release after S132"', main)
        self.assertLess(main.index("nrf_sdh_disable_request()"),
                        main.index("enabled = SystemHighFreqClockEnable()"))
        self.assertLess(main.index("SystemHighFreqClockDisable();",
                                   main.index("nrf_sdh_disable_request()")),
                        main.index("released = !HfxoRunning()"))

    def test_s132_release_is_deferred_until_stack_shutdown(self):
        disable = function(self.clock, "void SystemHighFreqClockDisable(",
                           "// System OFF requires HFXO stopped")
        self.assertIn("s_nRFxHfxoOwner == NRFX_HFXO_OWNER_DIRECT", disable)
        self.assertIn("nRFxClockSdEnabled()", disable)
        self.assertIn("s_bNRFxSdDirectStopPending = true", disable)
        self.assertIn("s_nRFxHfxoReqCnt = 0U", disable)
        handler = function(self.clock,
                           "static void nRFxClockSdhStateHandler(",
                           "bool SystemCoreClockSelect(")
        self.assertIn("NRF_SDH_EVT_STATE_DISABLED", handler)
        self.assertIn("NRF_SDH_STATE_OBSERVER(", handler)
        self.assertIn("s_bNRFxSdDirectStopPending", handler)
        self.assertIn("s_nRFxHfxoReqCnt == 0U", handler)
        self.assertIn("NRFX_HFXO_STOP = 1U", handler)
        self.assertIn("s_nRFxHfxoOwner = NRFX_HFXO_OWNER_DIRECT", handler)

    def test_nrf54l_system_off_checks_hfxo(self):
        source = function(self.clock, "bool nRFxClockPrepareSystemOff(",
                          "static bool nRFxLfclkOscSource(")
        self.assertIn("s_nRFxHfxoReqCnt != 0U", source)
        self.assertIn("s_bNRFxHfxoTransition", source)
        self.assertIn("nRFxMpslActive()", source)
        self.assertIn("nRFxClockSdEnabled()", source)
        self.assertNotIn("NRF_CLOCK->TASKS_XOSTOP = 1U", source)
        self.assertIn("nRFxHfxoRunning() == false", source)
        self.assertIn("DisableInterrupt()", source)
        self.assertIn("EnableInterrupt(state)", source)

        power = (ROOT / "ARM/Nordic/src/system_power_nrfx.cpp").read_text(
            encoding="utf-8")
        entry = function(power, "void SystemPowerOff(",
                         "bool SystemWakePin(")
        self.assertLess(entry.index("nRFxClockPrepareSystemOff()"),
                        entry.index("SYS_PWR_REGULATORS->SYSTEMOFF"))

    def test_failed_start_relinquishes_vendor_clock_request(self):
        source = function(self.clock, "static bool nRFxHfxoStart(",
                          "// Stop only through the backend")
        self.assertIn("sd_clock_hfclk_request()", source)
        self.assertGreaterEqual(source.count("sd_clock_hfclk_release()"), 2)
        self.assertIn("NRFX_HFXO_OWNER_SOFTDEVICE", source)
        self.assertIn("NRFX_HFXO_OWNER_MPSL", source)

        mpsl = function(self.mpsl, "int nRFxMpslHfxo(", 'extern "C"')
        self.assertIn("mpsl_clock_hfclk_src_request(", mpsl)
        self.assertGreaterEqual(
            mpsl.count("mpsl_clock_hfclk_src_release("), 3)


if __name__ == "__main__":
    unittest.main()
