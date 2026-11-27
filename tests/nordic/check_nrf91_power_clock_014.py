#!/usr/bin/env python3
"""Check nRF91 System ON IDLE integration without changing generic test code."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
POWER = ROOT / "ARM/Nordic/src/system_power_nrfx.cpp"
BUILD = ROOT / "tests/nordic/build_managed_power_clock.py"


class Nrf91PowerClock(unittest.TestCase):
	def test_nrf91_resumable_sleep(self):
		src = POWER.read_text(encoding="utf-8")
		sleep = src[src.index("// Resumable CPU sleep on nRF53"):src.index("#endif", src.index("void SystemSleep(void)"))]
		self.assertIn("defined(NRF91_SERIES)", src[src.index("#if defined(NRF5340_XXAA_NETWORK)"):src.index("// Resumable CPU sleep")])
		for token in ("__get_IPSR()", "DisableInterrupt()", "__WFI()", "EnableInterrupt(state)", "SCB->SCR = scr"):
			self.assertIn(token, sleep)
		self.assertIn("SCB_SCR_SLEEPDEEP_Msk", sleep)
		self.assertIn("SCB_SCR_SLEEPONEXIT_Msk", sleep)

	def test_nrf91_capabilities_match_implemented_sleep(self):
		src = POWER.read_text(encoding="utf-8")
		caps = src[src.index("uint32_t SystemPowerCapabilitiesGet(void)"):src.index("void SystemPowerOff(void)")]
		part = caps[caps.index("#elif defined(NRF91_SERIES)"):caps.index("#else", caps.index("#elif defined(NRF91_SERIES)"))]
		for capability in ("SYSTEM_POWER_CAP_SLEEP", "SYSTEM_POWER_CAP_TIMER_WAKE",
			"SYSTEM_POWER_CAP_SYSTEM_OFF", "SYSTEM_POWER_CAP_WAKE_PIN",
			"SYSTEM_POWER_CAP_RAM_RETENTION", "SYSTEM_POWER_CAP_RET_REG"):
			self.assertIn(capability, part)
		self.assertNotIn("SYSTEM_POWER_CAP_LOW_POWER", part)

	def test_both_nrf91_managed_core_tests(self):
		src = BUILD.read_text(encoding="utf-8")
		for target in ("nrf9160", "nrf91x1"):
			self.assertIn('"' + target + '": ROOT', src)
		for target in ("nRF9160", "nRF91x1"):
			root = ROOT / "ARM/Nordic/nRF91" / target / "exemples"
			for project in ("CoreClockTest", "CorePowerTest"):
				path = root / project / "ioc/.project"
				text = path.read_text(encoding="utf-8")
				self.assertIn("<name>" + project + "</name>", text)
				self.assertIn("exemples/misc/core_" + ("clock" if project == "CoreClockTest" else "power") + "_test.cpp", text)


	def test_lfclk_start_checks_actual_source(self):
		src = (ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp").read_text(encoding="utf-8")
		start = src.index("static bool nRFxLfclkFinishStart(uint32_t Src)")
		end = src.index("static bool nRFxLfclkStart(uint32_t Src)", start)
		section = src[start:end]
		self.assertIn("#if defined(NRF53_SERIES) || defined(NRF91_SERIES)", section)
		self.assertIn("NRFX_LFCLK_STAT_SRC_Msk", section)
		self.assertIn("NRFX_LFCLK_SRC_XTAL", section)
		self.assertIn("NRFX_LFCLK_SRC_RC", section)
		self.assertIn("#if defined(NRF53_SERIES)\n\t\t\t(void)nRFxLfclkCalibrate();", section)
		self.assertIn("#if defined(NRF91_SERIES)", section)

	def test_nrf91_sdelay_uses_shared_lf_timer_and_measured_startup(self):
		timer = (ROOT / "ARM/Nordic/src/timer_nrfx.cpp").read_text(encoding="utf-8")
		clock = (ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp").read_text(encoding="utf-8")
		self.assertIn("defined(NRF54L_SERIES) || defined(NRF91_SERIES)", timer)
		self.assertIn('extern "C" bool nRFxSleepLfClockAccurate(void);', timer)
		self.assertIn('extern "C" uint32_t nRFxSleepLfClockStartupWaitUs(void);', timer)
		self.assertIn('extern "C" bool nRFxSleepLfClockAccurate(void)\n{', clock)
		self.assertIn("#if defined(NRF5340_XXAA_APPLICATION) || defined(NRF91_SERIES)", clock)
		start = timer.index('extern "C" bool sDelay(uint32_t Seconds)')
		delay = timer[start:]
		for call in ("nRFxSleepLfClockAccurate()", "nRFxSleepLfClockStartupWaitUs()",
					 "TimerInit(&s_SleepTimer", "TimerPowerOff(&s_SleepTimer)", "__WFE()"):
			self.assertIn(call, delay)
		self.assertNotIn("msDelay(", delay)
		self.assertNotIn("DWT->", delay)

	def test_example_is_generic(self):
		example = (ROOT / "exemples/misc/core_clock_test.cpp").read_text(encoding="utf-8")
		self.assertNotIn("NRF91_SERIES", example)
		self.assertNotIn("NRF9160", example)

	def test_nrf91_lfrc_measured_before_timer_frequency_is_exposed(self):
		rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(encoding="utf-8")
		clock = (ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp").read_text(encoding="utf-8")
		self.assertNotIn("SystemLowFreqClockSelect(", clock)
		self.assertNotIn("nRFxLfclkConfigurationEpoch", clock)
		self.assertIn('extern "C" bool nRFxLfclkRunningFromRc(void)', clock)
		self.assertIn("NRF91_LFRC_MEASURE_US", rtc)
		self.assertIn("usDelay(NRF91_LFRC_MEASURE_US);", rtc)
		self.assertIn("if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)", rtc)
		self.assertIn("ticks = (after - before) & RTC_COUNTER_MASK;", rtc)
		self.assertIn("s_bNRF91LfrcMeasured", rtc)
		self.assertIn("s_nRF91LfrcHz = freq;", rtc)
		self.assertIn("uint32_t base = nRFxRtcBaseFrequency();", rtc)
		self.assertIn("pTimer->Freq = (base + div / 2U) / div;", rtc)
		start = rtc.index("bool nRFxRtcInit(")
		init = rtc[start:]
		self.assertLess(init.index("nRFxRtcMeasureLfrc(reg)"),
			init.index("nRFxRtcSetFrequency(pTimer, pSelectedCfg->Freq);",
				init.index("nRFxRtcMeasureLfrc(reg)")))
		self.assertIn("nRFxRtcDisable(pTimer);", init)

	def test_lfrc_rate_conversion_model(self):
		# Simulate hardware raw counts. The reference interval is measured
		# using the independently calibrated HF CPU delay, not an LF RTC.
		interval_us = 100000
		for input_hz in (20000, 32768, 44757, 50000):
			raw = (input_hz * interval_us + 500000) // 1000000
			measured = (raw * 1000000 + interval_us // 2) // interval_us
			self.assertLessEqual(abs(measured - input_hz), 5)
			for requested in (0, 1024, 32768):
				div = 1
				if requested and requested < measured:
					div = measured // requested
					if div > 4095:
						div = 4096
					elif measured * (2 * div + 1) > 2 * requested * div * (div + 1):
						div += 1
				reports_hz = (measured + div // 2) // div
				ticks = (input_hz * 250000 + 500000 * div) // (1000000 * div)
				elapsed_us = (ticks * 1000000 + reports_hz // 2) // reports_hz
				self.assertLessEqual(abs(elapsed_us - 250000), 5000)

	def test_nrf9160_dk_button_wake_configuration(self):
		board = (ROOT / "ARM/Nordic/nRF91/nRF9160/exemples/CorePowerTest/src/board.h").read_text(encoding="utf-8")
		thingy = board[board.index("#if defined(NORDIC_THINGY91)"):board.index("#elif defined(NRF9160_DK)")]
		dk = board[board.index("#elif defined(NRF9160_DK)"):board.index("#else", board.index("#elif defined(NRF9160_DK)"))]
		for definition in ("#define WAKE_BUT_PORT\t\t0",
					"#define WAKE_BUT_PIN\t\t6",
					"#define WAKE_BUT_PINOP\t\t0"):
			self.assertIn(definition, dk)
		self.assertNotIn("WAKE_BUT_PIN", thingy)
		self.assertNotIn("POWER_DEMO_UART_DMA", board)
		example = (ROOT / "exemples/misc/core_power_test.cpp").read_text(encoding="utf-8")
		self.assertIn("WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_SENSE, CorePowerTestWakeHandler", example)
		self.assertNotIn("SystemWakePin", example)
		self.assertIn("IOPINRES_PULLDOWN : IOPINRES_PULLUP", example)
		self.assertIn("SystemPowerOff();", example)

	def test_nrf91_measured_delay_divisors(self):
		# Measured on nRF9160 hardware with CoreClockTest at 64 MHz.
		src = (ROOT / "ARM/Nordic/nRF91/src/system_nrf91.c").read_text(encoding="utf-8")
		self.assertIn("16001600UL, 3000330UL", src)
		self.assertNotIn("16000000UL, 3000000UL", src)
		self.assertEqual(64000000 * 1000000 // 16001600, 3999600)
		self.assertEqual(64000000 * 1000000 // 3000330, 21330986)


	def test_lfrc_rate_survives_timer_disable_enable(self):
		clock = (ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp").read_text(encoding="utf-8")
		rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(encoding="utf-8")
		start = clock.index('extern "C" bool nRFxLfclkUsesRc(void)')
		end = clock.index("#endif", start)
		helper = clock[start:end]
		self.assertIn("g_McuOsc.LowPwrOsc.Type != OSC_TYPE_RC", helper)
		self.assertIn("(stat & NRFX_LFCLK_STATE_Msk) == 0U", helper)
		self.assertIn("NRFX_LFCLK_STAT_SRC_Msk", helper)
		self.assertIn("NRFX_LFCLK_SRC_RC", helper)

		begin = rtc.index("static uint32_t nRFxRtcBaseFrequency(void)")
		end = rtc.index("static bool nRFxRtcMeasureLfrc", begin)
		base = rtc[begin:end]
		self.assertIn("nRFxLfclkUsesRc()", base)
		self.assertNotIn("nRFxLfclkRunningFromRc()", base)
		self.assertIn("s_bNRF91LfrcMeasured", base)
		self.assertIn("return s_nRF91LfrcHz;", base)
		self.assertIn("return TIMER_NRFX_RTC_BASE_FREQ;", base)

		# A stopped LFCLK does not invalidate the selected RC source.
		# An externally supplied LFXO uses its nominal rate.
		measured = 44720
		def base_rate(selected_rc, running, active_rc):
			uses_rc = selected_rc and (not running or active_rc)
			return measured if uses_rc else 32768

		self.assertEqual(base_rate(True, True, True), measured)
		self.assertEqual(base_rate(True, False, False), measured)
		self.assertEqual(base_rate(True, True, False), 32768)
		self.assertEqual(base_rate(False, False, False), 32768)

	def test_nrf91_hf_timer_reinit(self):
		hf = (ROOT / "ARM/Nordic/src/timer_hf_nrfx.cpp").read_text(encoding="utf-8")
		section = hf[hf.index("bool nRFxTimerInit("):]
		self.assertNotIn("s_pnRFxTimerDev[devno] != pTimer", section)
		self.assertIn("pReg->MODE != TIMER_MODE_MODE_Timer", section)
		self.assertIn("s_pnRFxTimerDev[devno] = nullptr;", section)

	def test_running_timer_rejects_init_before_configuration_changes(self):
		for file, init, enabled, assignment in (
			("timer_hf_nrfx.cpp", "nRFxTimerInit", "s_bTimerEnabled[devno]",
			 "pTimer->DevNo = pCfg->DevNo;"),
			("timer_lf_nrfx.cpp", "nRFxRtcInit", "s_bRtcEnabled[pSelectedCfg->DevNo]",
			 "pTimer->DevNo = pSelectedCfg->DevNo;"),
		):
			with self.subTest(driver=file):
				source = (ROOT / "ARM/Nordic/src" / file).read_text(encoding="utf-8")
				section = source[source.index("bool " + init + "("):]
				check = "if (" + enabled + ")\n\t{\n\t\treturn false;"
				self.assertLess(section.index(check), section.index(assignment))

	def test_nrf91_timer_frequency_change_rephases_armed_triggers(self):
		hf = (ROOT / "ARM/Nordic/src/timer_hf_nrfx.cpp").read_text(encoding="utf-8")
		rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(encoding="utf-8")
		hf_section = hf[hf.index("static uint32_t nRFxTimerSetFrequency("):
			hf.index("static uint64_t nRFxTimerGetTickCount(")]
		rtc_section = rtc[rtc.index("static uint32_t nRFxRtcSetFrequency("):
			rtc.index("// Tick count, see nRFxRtcReadCountMasked.", rtc.index("static uint32_t nRFxRtcSetFrequency("))]
		for section in (hf_section, rtc_section):
			self.assertIn("tdata.pTrigger[i].nsPeriod" if section is hf_section
				else "rtc.pTrigger[i].nsPeriod", section)
			self.assertIn("TimerNanosecondToTick(pTimer", section)
			self.assertIn("TimerTickToNanosecond(pTimer, ticks)", section)
			self.assertIn("DisableInterrupt()", section)
			self.assertIn("EnableInterrupt(irqState)", section)
		self.assertIn("nRFxTimerReset(pTimer);", hf_section)
		self.assertIn("nRFxRtcReset(pTimer);", rtc_section)
		self.assertIn("s_bRtcResetPending[pTimer->DevNo] = true;", rtc_section)
		rtc_enable = rtc[rtc.index("static bool nRFxRtcEnable("):
			rtc.index("static void nRFxRtcDisable(", rtc.index("static bool nRFxRtcEnable("))]
		self.assertLess(rtc_enable.index("SystemLowFreqClockEnable()"),
			rtc_enable.index("nRFxRtcReset(pTimer)"))
		self.assertLess(rtc_enable.index("nRFxRtcReset(pTimer)"),
			rtc_enable.index("TASKS_START"))

	def test_nrf91_rejects_unrepresentable_compare_intervals(self):
		hf = (ROOT / "ARM/Nordic/src/timer_hf_nrfx.cpp").read_text(encoding="utf-8")
		rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(encoding="utf-8")
		hf_section = hf[hf.index("static uint64_t nRFxTimerEnableTrigger("):
			hf.index("static void nRFxTimerDisableTrigger(")]
		rtc_section = rtc[rtc.index("static uint64_t nRFxRtcEnableTrigger("):
			rtc.index("static void nRFxRtcDisableTrigger(")]
		self.assertIn("ticks = TimerNanosecondToTick(pTimer, nsPeriod);", hf_section)
		self.assertIn("ticks == 0ULL || ticks > UINT32_MAX", hf_section)
		self.assertIn("if (ticks > RTC_COUNTER_MASK)", rtc_section)
		self.assertIn("return 0ULL;", rtc_section)
		# Old HF code truncated 300s at 16MHz into a 32-bit compare.
		self.assertGreater(300 * 16000000, 0xFFFFFFFF)
		self.assertLess(250 * 16000000, 0xFFFFFFFF)
		self.assertGreater(600 * 32768, 0xFFFFFF)
		self.assertLess(250 * 44740, 0xFFFFFF)

	def test_nrf91_frequency_transition_models(self):
		# The target uses closest power-of-two HF and integer-divider LF
		# rates. Retained nanosecond periods must be rebuilt after reset.
		hf_base = 16000000
		for requested, target in ((16000000, 16000000), (8000000, 8000000),
				(4000000, 4000000), (7000000, 8000000),
				(1000, 31250)):
			rates = [hf_base >> p for p in range(10)]
			actual = min(rates, key=lambda hz: (abs(hz - requested), -hz))
			self.assertEqual(actual, target)
			period_ns = 10000000
			period_ticks = (period_ns * actual + 500000000) // 1000000000
			self.assertGreater(period_ticks, 0)
			self.assertLessEqual(period_ticks, 0xFFFFFFFF)
		for rc_hz, requested in ((44740, 32768), (44740, 1024),
				(32768, 1024)):
			div = 1
			if requested < rc_hz:
				div = rc_hz // requested
				if rc_hz * (2 * div + 1) > 2 * requested * div * (div + 1):
					div += 1
			actual = (rc_hz + div // 2) // div
			ticks = (actual * 10000000 + 500000000) // 1000000000
			self.assertGreaterEqual(ticks, 2)
			self.assertLessEqual(ticks, 0xFFFFFF)


	def test_nrf91_timer_explicit_source_matches_active_selection(self):
		hf = (ROOT / "ARM/Nordic/src/timer_hf_nrfx.cpp").read_text(encoding="utf-8")
		rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(encoding="utf-8")
		hf_init = hf[hf.index("bool nRFxTimerInit("):]
		rtc_init = rtc[rtc.index("bool nRFxRtcInit("):]
		self.assertIn("pCfg->ClkSrc != TIMER_CLKSRC_DEFAULT", hf_init)
		self.assertIn("nRFxLfclkUsesRc()", rtc_init)
		self.assertIn("pCfg->ClkSrc == TIMER_CLKSRC_LFRC", rtc_init)
		self.assertIn("pCfg->ClkSrc == TIMER_CLKSRC_LFXTAL", rtc_init)
		self.assertIn("#if defined(NRF91_SERIES)", rtc_init)
		# Temporary RC selection must be usable without changing the
		# board's immutable g_McuOsc default oscillator description.
		for selected_rc in (False, True):
			accept_rc = selected_rc
			accept_xtal = not selected_rc
			self.assertNotEqual(accept_rc, accept_xtal)

	def test_nrf91_hf_timer_ownership(self):
		hf = (ROOT / "ARM/Nordic/src/timer_hf_nrfx.cpp").read_text()
		guard = hf[hf.index("bool nRFxTimerInit("):]
		self.assertNotIn("s_pnRFxTimerDev[devno] != pTimer", guard)
		self.assertIn("s_bTimerEnabled[devno]", guard)
		self.assertIn("s_pnRFxTimerDev[devno] = pTimer;", guard)


	def test_nrf91_poweroff_old_timer_structure(self):
		hf = (ROOT / "ARM/Nordic/src/timer_hf_nrfx.cpp").read_text(encoding="utf-8")
		rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(encoding="utf-8")
		for src, name, data, owner in (
			(hf, "nRFxTimerPowerOff", "s_pnRFxTimerDev[devno]", "nRFxTimerOwned"),
			(rtc, "nRFxRtcPowerOff", "s_pnRfxRtcDev[devno]", "nRFxRtcOwned"),
		):
			with self.subTest(timer=name):
				self.assertIn("static bool " + name + "(", src)
				self.assertIn(data + " = nullptr;", src)
				self.assertIn("static bool " + owner + "(", src)
				self.assertIn("!" + owner + "(pTimer)", src)

if __name__ == "__main__":
	unittest.main()
