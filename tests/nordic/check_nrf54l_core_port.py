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
