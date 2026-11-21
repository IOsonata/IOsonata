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

    def test_hfxo_example_uses_only_generic_clock_services(self):
        source = (ROOT / "exemples/misc/hfxo_test.cpp").read_text(
            encoding="utf-8")
        self.assertNotIn('#include "nrf.h"', source)
        self.assertNotIn("NRF_CLOCK", source)
        self.assertNotIn("HFCLKSTAT", source)
        self.assertNotIn("SystemCoreClockSelect(", source)
        self.assertNotIn("SOFTDEVICE_PRESENT", source)
        self.assertNotIn("NRFXLIB_SDC", source)
        self.assertIn("SystemHighFreqClockEnable()", source)
        self.assertIn("SystemHighFreqClockDisable()", source)
        self.assertIn("3 requests PASS", source)
        self.assertNotIn("HFXO running", source)
        self.assertNotIn("HFXO stopped", source)

    def test_hfxo_console_drains_interrupt_uart(self):
        source = (ROOT / "exemples/misc/hfxo_test.cpp").read_text(
            encoding="utf-8")
        self.assertIn(".bIntMode = true", source)
        self.assertIn(".EvtCallback = HfxoUartEvent", source)
        callback = function(source, "static int HfxoUartEvent(UARTDev_t *pDev, UART_EVT Event,",
                            "static void HfxoConsoleInit(void)")
        self.assertIn("UART_EVT_TXREADY", callback)
        self.assertIn("__SEV();", callback)
        drain = function(source, "static void HfxoConsoleDrain(void)",
                         "#else")
        self.assertIn("SysLogFlush(SysLogGet())", drain)
        self.assertIn("CFifoUsed(pUart->hTxFifo)", drain)
        self.assertIn("pUart->bTxReady", drain)
        self.assertIn("__WFE();", drain)
        self.assertLess(source.rindex("HfxoConsoleDrain();"),
                        source.rindex("while (1)"))

    def test_portable_ble_app_has_no_vendor_stack_selection(self):
        source = (ROOT / "exemples/misc/hfxo_test_ble.cpp").read_text(
            encoding="utf-8")
        self.assertIn('#include "bluetooth/bt_app.h"', source)
        self.assertIn("BtAppInit(&s_BtCfg)", source)
        self.assertIn("SystemHighFreqClockEnable()", source)
        self.assertIn("SystemHighFreqClockDisable()", source)
        self.assertNotIn("SystemCoreClockSelect(", source)
        for forbidden in ("SOFTDEVICE_PRESENT", "NRFXLIB_SDC",
                          "nrf_sdh_", "MpslInit", "#include \"nrf.h\"",
                          "NRF_CLOCK", "stm32", "SHCI_"):
            self.assertNotIn(forbidden, source)
        wba = (ROOT / "ARM/ST/STM32WBAxx/src/system_stm32wbaxx.c").read_text(
            encoding="utf-8")
        self.assertIn("bool SystemHighFreqClockEnable(void)", wba)
        self.assertIn("void SystemHighFreqClockDisable(void)", wba)

    def test_ble_diagnostic_uses_no_application_clock_mode(self):
        source = (ROOT / "tests/nordic/hfxo_test_ble_vendor_diagnostic.cpp").read_text(
            encoding="utf-8")
        self.assertNotIn("HFXO_TEST_PRESTACK", source)
        self.assertNotIn("SystemCoreClockSelect(", source)
        self.assertIn("static int HfxoPreStackTest(void)", source)
        pre = function(source, "static int HfxoPreStackTest(void)",
                       "int main(void)")
        self.assertLess(pre.index("SystemHighFreqClockEnable()"),
                        pre.index("HfxoStackInit()"))
        self.assertLess(pre.index("HfxoStackInit()"),
                        pre.index("Pre-stack request released"))
        self.assertIn("Pre-stack handover", pre)
        self.assertNotIn("NRF_CLOCK->TASKS_HFCLKSTOP", pre)

    def test_managed_build_covers_nrf52840_s140_and_sdc(self):
        build = (ROOT / "tests/nordic/build_managed_power_clock.py").read_text(
            encoding="utf-8")
        self.assertIn('root / "exemples/HfxoTestBle/ioc",', build)
        self.assertIn('if configuration.endswith("_SDC"):', build)
        for target in ("nRF52832", "nRF52840"):
            project = (ROOT / "ARM/Nordic/nRF52" / target /
                       "exemples/HfxoTestBle/ioc/.cproject").read_text(
                encoding="utf-8")
            self.assertIn('name="Release"', project)
            self.assertIn('name="Release_SDC"', project)

    def test_nrf52832_uart_ble_clock_requests_during_gatt_writes(self):
        board = (ROOT / "ARM/Nordic/nRF52/nRF52832/exemples/UartBleDemo/src/board.h").read_text(
            encoding="utf-8")
        source = (ROOT / "exemples/bluetooth/uart_ble.cpp").read_text(
            encoding="utf-8")
        project = (ROOT / "ARM/Nordic/nRF52/nRF52832/exemples/UartBleDemo/ioc/.cproject").read_text(
            encoding="utf-8")
        self.assertIn("OSC_TYPE_RC, 32768U, 500U", board)
        self.assertIn('name="Release_SDC"', project)
        self.assertIn('#include "coredev/system_core_clock.h"', source)
        self.assertIn("static void ClockCheckEvt(", source)
        self.assertIn("AppEvtHandlerQue(0U, nullptr, ClockCheckEvt)", source)
        callback = source.split("static void ClockCheckEvt(", 1)[1].split(
            "void UartTxSrvcCallback(", 1)[0]
        self.assertIn("first = SystemHighFreqClockEnable()", callback)
        self.assertIn("second = first && SystemHighFreqClockEnable()", callback)
        self.assertEqual(callback.count("SystemHighFreqClockDisable();"), 2)
        self.assertIn("s_ClockCheckFailures++", callback)
        self.assertIn("BLE clock requests: %u checks, %u failures", callback)
        self.assertIn("s_ClockCheckCount == 1U", callback)
        self.assertIn("s_bClockCheckPending = false", callback)
        self.assertIn("s_bClockCheckQueued = false", callback)
        self.assertIn("BLE GATT write received", source)
        self.assertIn("s_bClockCheckQueued = AppEvtHandlerQue(0U, nullptr, ClockCheckEvt)", source)
        self.assertIn("if (s_bClockCheckPending && !s_bClockCheckQueued)", source)
        self.assertIn("!s_bClockCheckPending", source)
        gatt = source.split("void UartTxSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len)\n{", 1)[1]
        self.assertIn("g_Uart.Tx(pData, Len);", gatt)
        self.assertNotIn("SystemHighFreqClockEnable()", gatt)
        self.assertNotIn("nRFxMpsl", source)

    def test_nrf52840_ble_boards_select_lfrc(self):
        base = ROOT / "ARM/Nordic/nRF52/nRF52840/exemples"
        for project in ("BleAdvertiser", "UartBleDemo"):
            with self.subTest(project=project):
                board = (base / project / "src/board.h").read_text(
                    encoding="utf-8")
                self.assertIn("#define MCU_OSC", board)
                self.assertIn("OSC_TYPE_XTAL, 32000000U, 20U", board)
                self.assertIn("OSC_TYPE_RC, 32768U, 500U", board)
                project_file = (base / project / "ioc/.cproject").read_text(
                    encoding="utf-8")
                self.assertIn('name="Release_SDC"', project_file)

    def test_nrf52832_advertiser_uses_lfrc_board_configuration(self):
        board = (ROOT / "ARM/Nordic/nRF52/nRF52832/exemples/BleAdvertiser/src/board.h").read_text(
            encoding="utf-8")
        advertiser = (ROOT / "exemples/bluetooth/ble_advertiser.cpp").read_text(
            encoding="utf-8")
        project = (ROOT / "ARM/Nordic/nRF52/nRF52832/exemples/BleAdvertiser/ioc/.cproject").read_text(
            encoding="utf-8")
        self.assertIn("#define MCU_OSC", board)
        self.assertIn("OSC_TYPE_RC, 32768U, 500U", board)
        self.assertIn("OSC_TYPE_XTAL, 32000000U", board)
        self.assertIn("McuOsc_t g_McuOsc = MCU_OSC;", advertiser)
        self.assertIn("BtAppInit(&s_BtAppCfg)", advertiser)
        self.assertIn("BtAdvStart();", advertiser)
        self.assertIn('name="Release_SDC"', project)
        self.assertNotIn("SystemLowFreqClockSelect(", advertiser)
        self.assertNotIn("MpslInit(", advertiser)

    def test_sdc_vendor_diagnostic_uses_lfrc(self):
        diagnostic = (ROOT / "tests/nordic/hfxo_test_ble_vendor_diagnostic.cpp").read_text(
            encoding="utf-8")
        self.assertIn("SystemLowFreqClockSelect(OSC_TYPE_RC, 32768U)", diagnostic)
        self.assertIn("g_McuOsc.LowPwrOsc.Accuracy = 500U", diagnostic)
        self.assertIn("Select LFRC: PASS", diagnostic)
        self.assertIn("Pre-stack physical release", diagnostic)
        self.assertIn("stoppedSamples = 0U", diagnostic)
        self.assertIn("sample < 200U", diagnostic)
        self.assertIn("stopped = stoppedSamples != 0U", diagnostic)
        self.assertIn('HFXO_PRINT("Pre-stack physical release: %s, HFXO stopped %u/200 samples', diagnostic)
        self.assertIn('return stopped ? 0 : 1;', diagnostic)
        portable = (ROOT / "exemples/misc/hfxo_test_ble.cpp").read_text(
            encoding="utf-8")
        self.assertNotIn("SystemLowFreqClockSelect(", portable)
        self.assertNotIn("NRFXLIB_SDC", portable)

    def test_pre_stack_success_continues_normal_checks(self):
        source = (ROOT / "tests/nordic/hfxo_test_ble_vendor_diagnostic.cpp").read_text(
            encoding="utf-8")
        main = function(source, "int main(void)", "return 0;")
        self.assertIn("preStackResult = HfxoPreStackTest()", main)
        self.assertIn("if (preStackResult != 0)", main)
        self.assertLess(main.index("preStackResult = HfxoPreStackTest()"),
                        main.index("initial = HfxoRunning()"))
        self.assertLess(main.index("initial = HfxoRunning()"),
                        main.index('HFXO_PRINT("HfxoTestBle: %d/5 checks PASS'))

    def test_lfrc_periodic_calibration_diagnostic(self):
        source = (ROOT / "tests/nordic/hfxo_test_ble_vendor_diagnostic.cpp").read_text(
            encoding="utf-8")
        test = function(source, "static bool HfxoLfCalibrationTest(void)",
                        "int main(void)")
        self.assertIn("nRFxMpslLfclkCalibrationCount()", test)
        self.assertIn("sample < 120U", test)
        self.assertIn("msDelay(100U)", test)
        self.assertIn("after - before >= 2U", test)
        self.assertIn("stoppedSamples != 0U", test)
        self.assertIn("LFRC periodic calibration: %s", test)
        self.assertIn("(void)HfxoLfCalibrationTest();", source)
        self.assertIn("s_nRFxMpslCalibrationCount++", self.clock)
        self.assertIn("uint32_t nRFxMpslLfclkCalibrationCount(void)", self.clock)
        self.assertIn("NRF_CLOCK->EVENTS_DONE != 0U", self.clock)

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
        self.assertIn("if (nRFxMpslActive())", source)
        self.assertIn("if (s_bNRFxMpslExternalClock)", source)
        self.assertIn("#else", source)
        self.assertIn("#endif", source)
        stop = source.index("NRFX_HFXO_STOP = 1U")
        self.assertLess(source.index("if (nRFxClockSdEnabled())"), stop)
        self.assertLess(source.index("if (nRFxMpslActive())"), stop)
        self.assertLess(source.index("if (nRFxMpslActive())"), stop)
        self.assertIn("NRFX_HFXO_OWNER_NONE", self.clock)

    def test_lfrc_uses_external_clock_and_calibration(self):
        init = function(self.mpsl, "bool MpslInit(void)", "static void nRFxMpslHfxoStarted(")
        self.assertIn("externalClock = lfosc->Type == OSC_TYPE_XTAL ||", init)
        self.assertIn("lfosc->Type == OSC_TYPE_RC", init)
        self.assertIn("mpsl_clock_ctrl_source_register(", init)
        self.assertIn("mpsl_init(externalClock ? nullptr : &lfclk", init)
        self.assertIn("nRFxMpslLfclkCalibrationStart();", init)
        self.assertIn("return nRFxMpslLfclkCalibrationIsEnabled();", self.mpsl)
        self.assertIn("nRFxMpslClockIrq();", self.mpsl)
        start = function(self.clock, "void nRFxMpslLfclkCalibrationStart(",
                         "bool nRFxMpslLfclkCalibrationIsEnabled(")
        irq = function(self.clock, "void nRFxMpslClockIrq(",
                       "// The external MPSL clock driver uses")
        self.assertIn("nRFxMpslHfclkRequest();", start)
        self.assertIn("CLOCK_INTENSET_DONE_Msk", start)
        self.assertIn("nRFxMpslCalibrationBegin();", start)
        self.assertIn("NRF_CLOCK->TASKS_CAL = 1U", self.clock)
        self.assertIn("NRF_CLOCK->EVENTS_DONE", irq)
        self.assertIn("nRFxMpslHfclkRelease();", irq)
        self.assertIn("NRF_CLOCK->CTIV = 16U;", irq)
        self.assertIn("NRF_CLOCK->TASKS_CTSTART = 1U;", irq)
        self.assertIn("s_bNRFxMpslCalibrationPending", self.clock)

    def test_lfrc_calibration_does_not_use_blocking_delay(self):
        calibration = function(self.clock, "void nRFxMpslLfclkCalibrationStart(",
                               "bool nRFxMpslLfclkCalibrationIsEnabled(")
        for call in ("nRFxHfxoStart(", "nRFxMpslHfxo(true)",
                     "SystemHighFreqClockEnable(", "msDelay(", "usDelay("):
            self.assertNotIn(call, calibration)
        self.assertIn("nRFxMpslHfclkRequest();", calibration)
        self.assertIn("CLOCK_INTENSET_HFCLKSTARTED_Msk", calibration)
        self.assertLess(calibration.index("s_bNRFxMpslCalibrationPending = true"),
                        calibration.index("nRFxMpslHfclkRequest();"))
        self.assertLess(calibration.index("nRFxMpslHfclkRequest();"),
                        calibration.index("EnableInterrupt(state);", calibration.index(
                            "s_bNRFxMpslCalibrationPending = true")))

    def test_ble_console_uses_interrupt_tx_completion(self):
        source = (ROOT / "tests/nordic/hfxo_test_ble_vendor_diagnostic.cpp").read_text(
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
        source = (ROOT / "tests/nordic/hfxo_test_ble_vendor_diagnostic.cpp").read_text(
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
                          "// Release HFXO through the owner")
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
