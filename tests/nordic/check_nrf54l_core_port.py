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
        self.assertIn("if (s_bGrtcEnabled[devno])", source)
        self.assertIn("candidate != devno && s_nRfxGrtcData[candidate].pTimer == pTimer",
                      " ".join(source.split()))
        self.assertIn("if (s_bGrtcEnabled[candidate])", source)
        self.assertIn("s_nRfxGrtcData[candidate].Trigger[i].nsPeriod != 0U", source)
        self.assertIn("s_nRfxGrtcData[candidate].pTimer = nullptr;", source)
        self.assertIn("s_nRfxGrtcData[candidate].pInt[2] =", source)


        self.assertIn("pRtc->pTimer = nullptr;", source)
        self.assertIn("s_pGrtcOwner[devno] = nullptr;", source)
        self.assertIn("s_pGrtcOwner[devno] = pTimer;", source)
        self.assertIn("s_pGrtcOwner[candidate] == nullptr", source)
        self.assertIn("s_pGrtcOwner[devno] != pTimer", source)
        self.assertIn("state = DisableInterrupt();", source)
        self.assertIn("pRtc->pInt[2] = nRFxGrtcCCMask(*pRtc);", source)
        self.assertIn("NRF_GRTC->CC[idx].CCEN = 0U;", source)
        self.assertIn("NRF_GRTC->EVENTS_COMPARE[idx] = 0U;", source)
        self.assertNotIn("GrtcSleepClaim", source)
        self.assertNotIn("GrtcSleepRelease", source)

    def test_board_oscillator_caps_configured_before_first_grtc_start(self):
        system = (ROOT / "ARM/Nordic/nRF54/src/system_nrf54l.c").read_text()
        oscillator = (ROOT / "ARM/Nordic/nRF54/src/osc_nrf54l.c").read_text()
        reset = (ROOT / "ARM/src/ResetEntry.c").read_text()
        for family in ("nRF54L15", "nRF54LM20A", "nRF54LM20B"):
            project = (ROOT / "ARM/Nordic/nRF54" / family /
                       "lib/ioc/.project").read_text()
            self.assertIn("src/osc_nrf54l.c", project)
            self.assertIn("src/system_nrf54l.c", project)

        self.assertIn("SystemOscInit();", system)
        self.assertIn("!defined(NRF_TRUSTZONE_NONSECURE)", system)
        self.assertIn("!defined(NRF_SKIP_CLOCK_CONFIGURATION)", system)
        self.assertLess(system.index("SystemOscInit();"),
                        system.index("/*lint --flb"))
        self.assertLess(reset.index("memset((void *)&__bss_start__"),
                        reset.index("SystemInit();"))
        self.assertIn('#include "hal/nrf_ficr.h"', oscillator)
        self.assertIn("nrf_ficr_hfxo_cap_calculate(NRF_FICR,", oscillator)
        self.assertIn("#if defined(FICR_HFXO_CAP_CALCULATE)", oscillator)
        self.assertIn("#if defined(FICR_LFXO_CAP_CALCULATE)", oscillator)
        self.assertIn("static uint32_t nRF54LHfxoCapCalculate(", oscillator)
        self.assertIn("static uint32_t nRF54LLfxoCapCalculate(", oscillator)
        self.assertIn("nRF54LTrimSlope(", oscillator)
        self.assertIn("g_McuOsc.CoreOsc.LoadCap * 100U);", oscillator)
        self.assertIn("g_McuOsc.LowPwrOsc.LoadCap * 100U);", oscillator)
        self.assertIn("(int64_t)CapFf - 5500LL", oscillator)
        self.assertIn("2LL * CapFf - 12000LL", oscillator)
        self.assertIn("nrf_ficr_lfxo_cap_calculate(NRF_FICR,", oscillator)
        self.assertIn("g_McuOsc.CoreOsc.LoadCap * 100U", oscillator)
        self.assertIn("g_McuOsc.LowPwrOsc.LoadCap * 100U", oscillator)
        self.assertIn("g_McuOsc.CoreOsc.Type == OSC_TYPE_XTAL", oscillator)
        self.assertIn("g_McuOsc.LowPwrOsc.Type == OSC_TYPE_XTAL", oscillator)
        self.assertIn("NRF_CLOCK_S->XO.STAT & CLOCK_XO_STAT_STATE_Msk", oscillator)
        self.assertIn("NRF_CLOCK_S->LFCLK.STAT", oscillator)
        self.assertIn("NRF_GRTC->MODE & GRTC_MODE_SYSCOUNTEREN_Msk", oscillator)
        self.assertIn("NRF_GRTC->CLKCFG & GRTC_CLKCFG_CLKSEL_Msk", oscillator)
        self.assertIn("if (!lfRunning && NRF_OSCILLATORS->XOSC32KI.INTCAP != lfCap)", oscillator)
        self.assertIn("NRF_OSCILLATORS->XOSC32KI.INTCAP = lfCap;", oscillator)
        self.assertNotIn("slope = ((-(slope>>8))<<8) | slope", oscillator)
        self.assertIn("NRF_GRTC->TASKS_START = 1U;", self.grtc)

    def test_grtc_selects_board_lf_source_only_before_start(self):
        source = self.grtc
        start = source.index("static bool nRFxGrtcEnable(")
        end = source.index("// The triggers are kept", start)
        enable = source[start:end]

        self.assertIn('#include "coredev/system_core_clock.h"', source)
        self.assertIn("if ((NRF_GRTC->MODE & GRTC_MODE_SYSCOUNTEREN_Msk) == 0U)", enable)
        self.assertIn("switch (GetLowFreqOscType())", enable)
        self.assertIn("case OSC_TYPE_XTAL:", enable)
        self.assertIn("clkSel = GRTC_CLKCFG_CLKSEL_LFXO;", enable)
        self.assertIn("case OSC_TYPE_RC:", enable)
        self.assertIn("clkSel = GRTC_CLKCFG_CLKSEL_LFLPRC;", enable)
        self.assertIn("NRF_GRTC->CLKCFG = clkCfg;", enable)
        self.assertLess(enable.index("NRF_GRTC->CLKCFG = clkCfg;"),
                        enable.index("NRF_GRTC->TASKS_START = 1U;"))
        self.assertNotIn("NRF_GRTC->TASKS_STOP", enable)
        self.assertNotIn("SystemLowFreqClockSelect(", source)

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
        self.assertIn("warmClock = nRFxGrtcSleepClockWarm();", source)
        self.assertIn("cfg.DevNo = -1;", source)
        self.assertIn("TimerInit(&s_SleepTimer, &cfg)", source)
        self.assertIn("TimerDisableTrigger(&s_SleepTimer, 0);", source)
        self.assertIn("TimerPowerOff(&s_SleepTimer)", source)

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

    def test_lm20a_two_point_delay_interpolation(self):
        source = (ROOT / "ARM/Nordic/nRF54/src/system_nrf54l.c").read_text()
        start = source.index("#elif defined(NRF54LM20A_XXAA)")
        end = source.index("#elif defined(NRF54LM20B_XXAA)", start)
        lm20a = source[start:end]
        self.assertIn("64000000UL, 7998400UL, 21331413UL", lm20a)
        self.assertIn("128000000UL, 15999360UL, 42664533UL", lm20a)
        self.assertIn("ArmDelayClockInterpolate(SystemCoreClock,", lm20a)
        self.assertNotIn("16000000UL, 3000000UL", lm20a)
        self.assertEqual(64000000 * 1000000 // 8001600, 7998400)
        self.assertEqual(128000000 * 1000000 // 8000320, 15999360)
        # Verify interpolation arithmetic at the midpoint (96 MHz).
        us_mid = (7998400 + 15999360 + 1) // 2
        ns_mid = (21331413 + 42664533 + 1) // 2
        self.assertEqual(us_mid, 11998880)
        self.assertEqual(ns_mid, 31997973)

    def test_lm20a_startup_default_restored_to_128mhz(self):
        source = (ROOT / "ARM/Nordic/nRF54/src/system_nrf54l.c").read_text()
        init = source[source.index("void SystemInit(void)"):]
        self.assertIn("NRF_CONFIG_CPU_FREQ_MHZ", init)
        self.assertIn("OSCILLATORS_PLL_FREQ_FREQ_CK64M", init)
        self.assertIn("OSCILLATORS_PLL_FREQ_FREQ_CK128M", init)
        self.assertNotIn("TEMPORARY: LM20A 64-MHz", init)
        self.assertIn('''#else
                NRF_OSCILLATORS->PLL.FREQ = OSCILLATORS_PLL_FREQ_FREQ_CK128M;
            #endif''', init)

    def test_lm20b_remains_unmeasured(self):
        source = (ROOT / "ARM/Nordic/nRF54/src/system_nrf54l.c").read_text()
        start = source.index("#elif defined(NRF54LM20B_XXAA)")
        end = source.index("\n#else\n", start)
        lm20b = source[start:end]
        self.assertIn("8000640UL, 3000150UL", lm20b)
        self.assertNotIn("4000320UL", lm20b)
        l15 = source[source.index("#if defined(NRF54L15_XXAA)"):
                     source.index("#elif defined(NRF54LM20A_XXAA)")]
        self.assertIn("8000400UL, 3000150UL", l15)

    def test_lm20a_cold_sdelay_is_separate_from_l15(self):
        timer = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text()
        l15 = timer[timer.index("#elif defined(NRF54L15_XXAA)"):
                    timer.index("#elif defined(NRF54LM20A_XXAA)")]
        lm20a = timer[timer.index("#elif defined(NRF54LM20A_XXAA)"):
                      timer.index("#elif defined(NRF52832_XXAA)")]
        self.assertIn("#define NRF_SDELAY_COLD_DEFAULT_TICKS 434775", l15)
        self.assertIn("#define NRF_SDELAY_COLD_DEFAULT_TICKS 433611", lm20a)
        self.assertIn("#define NRF_SDELAY_WARM_DEFAULT_TICKS 0", lm20a)
        self.assertEqual((1433700 + 1433521) // 2 - 433611, 999999)

    def test_lm20_managed_projects_use_their_target_variant(self):
        # nRF54LM20A and nRF54LM20B are separate targets. Every managed
        # project of a target defines that target's MCU and no other.
        for variant, other in (("A", "B"), ("B", "A")):
            base = ROOT / ("ARM/Nordic/nRF54/nRF54LM20" + variant)
            own = 'value="NRF54LM20%s_XXAA"' % variant
            foreign = 'value="NRF54LM20%s_XXAA"' % other
            projects = sorted(base.glob("exemples/*/ioc/.cproject"))
            self.assertEqual(len(projects), 40)
            projects.append(base / "lib/ioc/.cproject")
            for path in projects:
                with self.subTest(project=str(path.relative_to(ROOT))):
                    project = path.read_text()
                    self.assertIn(own, project)
                    self.assertNotIn(foreign, project)
                    self.assertIn("IOsonata_nRF54LM20" + variant, project)
                    self.assertNotIn("IOsonata_nRF54LM20" + other, project)
                    self.assertNotIn("gcc_nrf54lm20%s_" % other.lower(), project)
            # Four managed library configs, each with C and C++ definitions.
            library = (base / "lib/ioc/.cproject").read_text()
            self.assertEqual(library.count(own), 8)
        self.assertFalse((ROOT / "ARM/Nordic/nRF54/nRF54LM20x").exists())

    def test_nrf54l15_managed_example_library_paths_match_configuration(self):
        import re

        root = ROOT / "ARM/Nordic/nRF54/nRF54L15/exemples"
        projects = sorted(root.glob("*/ioc/.cproject"))
        self.assertGreaterEqual(len(projects), 2)
        for path in projects:
            with self.subTest(project=str(path.relative_to(ROOT))):
                source = path.read_text()
                sections = list(re.finditer(
                    r'<storageModule buildSystemId="org.eclipse.cdt.managedbuilder.'
                    r'core.configurationDataProvider"[^>]*name="([^"]+)"',
                    source))
                self.assertGreaterEqual(len(sections), 2)
                for i, match in enumerate(sections):
                    config = match.group(1)
                    end = sections[i + 1].start() if i + 1 < len(sections) else len(source)
                    section = source[match.start():end]
                    paths = re.findall(
                        r'lib/ioc/(Debug(?:_SDC)?|Release(?:_SDC)?)',
                        section)
                    expected = "Release" if config.startswith("Release") else "Debug"
                    for linked in paths:
                        self.assertTrue(linked.startswith(expected),
                            str(path) + " " + config + " links " + linked)
                    if paths:
                        self.assertIn("IOsonata_nRF54L15", section)

    def test_lm20_managed_example_library_paths_match_configuration(self):
        # Every example must link its own target's corresponding library
        # build: Debug -> Debug, Release -> Release. The original LM20
        # split accidentally made Release link the Debug archive.
        import re

        for variant in ("A", "B"):
            base = ROOT / ("ARM/Nordic/nRF54/nRF54LM20" + variant)
            projects = sorted(base.glob("exemples/*/ioc/.cproject"))
            self.assertEqual(len(projects), 40)
            for path in projects:
                source = path.read_text()
                sections = list(re.finditer(
                    r'<storageModule buildSystemId="org.eclipse.cdt.managedbuilder.'
                    r'core.configurationDataProvider"[^>]*name="([^"]+)"',
                    source))
                self.assertGreaterEqual(len(sections), 2, str(path))
                for i, match in enumerate(sections):
                    config = match.group(1)
                    end = sections[i + 1].start() if i + 1 < len(sections) else len(source)
                    section = source[match.start():end]
                    library_paths = re.findall(
                        r'lib/ioc/(Debug(?:_SDC)?|Release(?:_SDC)?)',
                        section)
                    expected = "Release" if config.startswith("Release") else "Debug"
                    for linked in library_paths:
                        with self.subTest(project=str(path.relative_to(ROOT)),
                                          config=config, linked=linked):
                            self.assertTrue(linked.startswith(expected))
                    if library_paths:
                        self.assertIn("IOsonata_nRF54LM20" + variant, section)

    def test_lm20a_default_startup_restored_after_64mhz_validation(self):
        source = (ROOT / "ARM/Nordic/nRF54/src/system_nrf54l.c").read_text()
        self.assertNotIn("TEMPORARY: force LM20A default startup", source)
        self.assertNotIn("g_nRF54LM20AClockTrace", source)
        self.assertIn("OSCILLATORS_PLL_FREQ_FREQ_CK128M;", source)
        for relative in ("lib/ioc/.cproject",
                         "exemples/CoreClockTest/ioc/.cproject"):
            project = (ROOT / "ARM/Nordic/nRF54/nRF54LM20A" / relative).read_text()
            self.assertNotIn("NRF_CONFIG_CPU_FREQ_MHZ=64", project)

    def test_both_managed_libraries_link_backends(self):
        for mcu in ("nRF54L15", "nRF54LM20A", "nRF54LM20B"):
            project = (ROOT / "ARM/Nordic/nRF54" / mcu /
                       "lib/ioc/.project").read_text()
            for filename in ("timer_lf_nrf54.cpp", "timer_nrfx.cpp",
                             "system_power_nrfx.cpp",
                             "system_clock_nrfx.cpp"):
                with self.subTest(mcu=mcu, file=filename):
                    self.assertIn(filename, project)


if __name__ == "__main__":
    unittest.main()
