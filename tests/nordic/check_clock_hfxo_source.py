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
        self.assertIn("if (nRFxMpslActive() || nRFxClockSdEnabled())", source)
        guarded = source.split(
            "if (nRFxMpslActive() || nRFxClockSdEnabled())", 1)[1]
        self.assertLess(guarded.index("return false;"),
                        guarded.index("NRFX_HFXO_STOP = 1U"))
        self.assertIn("NRFX_HFXO_OWNER_NONE", self.clock)

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
