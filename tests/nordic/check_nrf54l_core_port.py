#!/usr/bin/env python3
"""Source-level nRF54L System Core power, clock and GRTC port checks."""
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


class Nrf54lCorePort(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grtc = (ROOT / "ARM/Nordic/nRF54/src/timer_lf_nrf54.cpp").read_text()
        cls.timer = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text()
        cls.power = (ROOT / "ARM/Nordic/src/system_power_nrfx.cpp").read_text()

    def test_grtc_auto_select_keeps_mpsl_group_reserved(self):
        source = self.grtc
        self.assertIn("automatic = pCfg->DevNo == -1;", source)
        self.assertIn("TIMER_NRFX_RTC_MAX - 2", source)
        self.assertIn("candidate >= 0", source)
        self.assertIn("selected.DevNo = candidate;", source)
        self.assertIn("rtc.pSysCnt = &NRF_GRTC->SYSCOUNTER[devno];", source)
        self.assertNotIn("SYSCOUNTER[pCfg->DevNo]", source)

    def test_grtc_explicit_ownership_and_temporary_cleanup(self):
        source = self.grtc
        self.assertIn("s_bGrtcAutoSelected[devno] = automatic;", source)
        self.assertIn("s_nRfxGrtcData[devno].pTimer != pTimer", source)
        self.assertIn("candidate != devno && s_nRfxGrtcData[candidate].pTimer == pTimer", source)
        self.assertIn("if (s_bGrtcEnabled[candidate])", source)
        self.assertIn("s_nRfxGrtcData[candidate].Trigger[i].nsPeriod != 0U", source)
        self.assertIn("s_nRfxGrtcData[candidate].pTimer = nullptr;", source)
        self.assertIn("s_nRfxGrtcData[candidate].pInt[2] =", source)


        self.assertIn("s_bGrtcAutoSelected[devno]", source)
        self.assertIn("pRtc->pTimer = nullptr;", source)
        self.assertIn("s_pGrtcOwner[devno] = nullptr;", source)
        self.assertIn("s_pGrtcOwner[devno] = pTimer;", source)
        self.assertIn("s_pGrtcOwner[candidate] == nullptr", source)
        self.assertIn("s_pGrtcOwner[devno] != pTimer", source)
        self.assertIn("state = DisableInterrupt();", source)
        self.assertIn("pRtc->pInt[2] = nRFxGrtcCCMask(*pRtc);", source)
        self.assertIn("NRF_GRTC->CC[idx].CCEN = 0U;", source)
        self.assertIn("NRF_GRTC->EVENTS_COMPARE[idx] = 0U;", source)
        self.assertIn("s_bGrtcAutoSelected[devno] = false;", source)
        self.assertNotIn("GrtcSleepClaim", source)
        self.assertNotIn("GrtcSleepRelease", source)

    def test_released_handle_cannot_reenable_grtc(self):
        source = self.grtc
        start = source.index("static bool nRFxGrtcEnable(")
        end = source.index("// The triggers are kept", start)
        enable = source[start:end]
        self.assertIn("if (pTimer == nullptr)", enable)
        self.assertIn("if (devno < 0 || devno >= TIMER_NRFX_RTC_MAX)", enable)
        self.assertIn("state = DisableInterrupt();", enable)
        self.assertIn("if (pRtc->pTimer != pTimer)", enable)
        self.assertIn("EnableInterrupt(state);", enable)
        self.assertIn("return false;", enable)

    def test_stale_grtc_handles_cannot_touch_reassigned_group(self):
        source = self.grtc
        self.assertIn("return pRtc->pTimer == pTimer ? pRtc : nullptr;", source)

        # After auto-release or handle rebinding, DevNo may still identify
        # hardware now assigned to another TimerDev. Every operation that
        # uses that group must check the current dispatch owner first.
        guarded = (
            "nRFxGrtcReset",
            "nRFxGrtcSetFrequency",
            "nRFxGrtcGetTickCount",
            "nRFxGrtcGetMaxTrigger",
            "nRFxGrtcEnableTrigger",
            "nRFxGrtcDisableTrigger",
            "nRFxGrtcFindAvailTrigger",
        )
        for name in guarded:
            with self.subTest(operation=name):
                start = source.rfind("static ", 0, source.index(name + "("))
                end = source.find("\nstatic ", source.index(name + "("))
                function = source[start:end if end >= 0 else None]
                check = function.index("nRFxGrtcOwned(pTimer)")
                self.assertTrue("pRtc == nullptr" in function or
                                "pRtc != nullptr" in function or
                                "nRFxGrtcOwned(pTimer) == nullptr" in function)
                for hardware in ("NRF_GRTC->CC[", "->pInt[", "->pSysCnt"):
                    if hardware in function:
                        self.assertLess(check, function.index(hardware))

        for name in ("nRFxGrtcReset", "nRFxGrtcEnableTrigger",
                     "nRFxGrtcDisableTrigger", "nRFxGrtcFindAvailTrigger"):
            with self.subTest(atomic_operation=name):
                start = source.index(name + "(")
                end = source.find("\nstatic ", start)
                function = source[start:end if end >= 0 else None]
                self.assertIn("state = DisableInterrupt();", function)
                self.assertIn("EnableInterrupt(state);", function)

    def test_sdelay_uses_generic_timer_and_grtc(self):
        source = self.timer
        self.assertIn("defined(NRF54L_SERIES)", source)
        self.assertIn("warmClock = true;", source)
        self.assertIn("cfg.DevNo = -1;", source)
        self.assertIn("TimerInit(&s_SleepTimer, &cfg)", source)
        self.assertIn("TimerDisableTrigger(&s_SleepTimer, 0);", source)
        self.assertIn("TimerDisable(&s_SleepTimer);", source)

    def test_nrf54l_system_sleep_and_capabilities(self):
        source = self.power
        self.assertIn("defined(NRF54L_SERIES)", source)
        self.assertIn("void SystemSleep(void)", source)
        self.assertIn("SCB_SCR_SLEEPDEEP_Msk", source)
        self.assertIn("__WFI();", source)
        self.assertIn("SYSTEM_POWER_CAP_SLEEP | SYSTEM_POWER_CAP_TIMER_WAKE", source)

    def test_l15_measured_128mhz_delay_factors(self):
        source = (ROOT / "ARM/Nordic/nRF54/src/system_nrf54l.c").read_text()
        self.assertIn("#if defined(NRF54L15_XXAA)", source)
        self.assertIn("if (SystemCoreClock == 128000000UL)", source)
        self.assertIn("8000400UL, 3000150UL", source)
        self.assertIn("16000000UL, 3000000UL", source)

    def test_both_managed_libraries_link_backends(self):
        for mcu in ("nRF54L15", "nRF54LM20x"):
            project = (ROOT / "ARM/Nordic/nRF54" / mcu /
                       "lib/ioc/.project").read_text()
            for filename in ("timer_lf_nrf54.cpp", "timer_nrfx.cpp",
                             "system_power_nrfx.cpp",
                             "system_clock_nrfx.cpp"):
                with self.subTest(mcu=mcu, file=filename):
                    self.assertIn(filename, project)


if __name__ == "__main__":
    unittest.main()
