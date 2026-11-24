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
					 "TimerInit(&s_SleepTimer", "TimerDisable(&s_SleepTimer)", "__WFE()"):
			self.assertIn(call, delay)
		self.assertNotIn("msDelay(", delay)
		self.assertNotIn("DWT->", delay)

	def test_example_is_generic(self):
		example = (ROOT / "exemples/misc/core_clock_test.cpp").read_text(encoding="utf-8")
		self.assertNotIn("NRF91_SERIES", example)
		self.assertNotIn("NRF9160", example)

	def test_independent_raw_lf_timer_frequency_diagnostics(self):
		example = (ROOT / "exemples/misc/core_clock_test.cpp").read_text(encoding="utf-8")
		begin = example.index("static void CoreClockTestTimer(int DevNo)")
		end = example.index("// Exercise the public Timer trigger API", begin)
		section = example[begin:end]
		self.assertIn("TimerGetTickCount(&s_Timer)", section)
		self.assertIn("TimerGetMicrosecond(&s_RetainedTimer)", section)
		self.assertIn("Independent LF timer rate:", section)
		self.assertIn('CoreClockTestResult("Delay within 2 percent",', section)
		self.assertNotIn("NRF91_SERIES", section)


	def test_nrf91_lfrc_measured_before_timer_frequency_is_exposed(self):
		rtc = (ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp").read_text(encoding="utf-8")
		clock = (ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp").read_text(encoding="utf-8")
		self.assertIn("s_nRF91LfclkConfigEpoch++", clock)
		self.assertIn('extern "C" uint32_t nRFxLfclkConfigurationEpoch(void)', clock)
		self.assertIn('extern "C" bool nRFxLfclkRunningFromRc(void)', clock)
		self.assertIn("NRF91_LFRC_MEASURE_US", rtc)
		self.assertIn("usDelay(NRF91_LFRC_MEASURE_US);", rtc)
		self.assertIn("if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)", rtc)
		self.assertIn("ticks = (after - before) & RTC_COUNTER_MASK;", rtc)
		self.assertIn("s_nRF91LfrcEpoch == nRFxLfclkConfigurationEpoch()", rtc)
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

	def test_generic_accuracy_is_not_waived_for_rc(self):
		example = (ROOT / "exemples/misc/core_clock_test.cpp").read_text(encoding="utf-8")
		self.assertIn('CoreClockTestResult("Delay within 2 percent", elapsed >= 245000U && elapsed <= 255000U);', example)
		self.assertIn("timer reports %u Hz", example)
		self.assertNotIn("NRF91_SERIES", example)
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
		self.assertIn("SystemWakePin(WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_SENSE)", example)
		self.assertIn("IOPINRES_PULLDOWN : IOPINRES_PULLUP", example)
		self.assertIn("SystemPowerOff();", example)

	def test_nrf91_measured_delay_divisors(self):
		# Measured on nRF9160 hardware with CoreClockTest at 64 MHz.
		src = (ROOT / "ARM/Nordic/nRF91/src/system_nrf91.c").read_text(encoding="utf-8")
		self.assertIn("16001600UL, 3000330UL", src)
		self.assertNotIn("16000000UL, 3000000UL", src)
		self.assertEqual(64000000 * 1000000 // 16001600, 3999600)
		self.assertEqual(64000000 * 1000000 // 3000330, 21330986)

if __name__ == "__main__":
	unittest.main()
