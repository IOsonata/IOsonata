#!/usr/bin/env python3
"""Check that generic sleep and clock demos use driver interfaces.

These checks detect source-level regressions. Hardware TX completion, sleep,
and wake behavior require separate target testing.
"""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
CLOCK = ROOT / "exemples/misc/clock_demo.cpp"
POWER = ROOT / "exemples/misc/power_off_demo.cpp"
I2C = ROOT / "exemples/i2c/i2c_slave_demo.cpp"
NRF_UART = ROOT / "ARM/Nordic/src/uart_nrfx.cpp"


class DemoDriverOwnership(unittest.TestCase):
	def test_clock_demo(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertNotIn("hTxFifo", source)
		self.assertNotIn("hRxFifo", source)
		self.assertNotIn("CFifoUsed(", source)
		start = source.index("static void ClockDemoFlush(void)\n{")
		end = source.index("\n}", start)
		self.assertNotIn("UARTRx(", source[start:end])
		self.assertIn("msDelay(2);", source[start:end])

	def test_clock_demo_peripheral_clock_interface(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertIn("unsigned dividerIdx;", source)
		self.assertIn("for (dividerIdx = 0U; dividerIdx <= dividerCount;", source)
		self.assertIn("divider = dividerIdx == 0U ? 1U :", source)
		self.assertIn("actual = SystemPeriphClockSet(clockIdx, request);", source)
		self.assertIn("readback = SystemPeriphClockGet(clockIdx);", source)
		self.assertIn("clockIdx < 32", source)
		self.assertIn("SystemPeriphClockGet(-1)", source)
		self.assertIn("SystemPeriphClockGet(32)", source)
		self.assertIn("Peripheral domain scan limit", source)
		self.assertIn("SystemPeriphClockSet(clockIdx, bus)", source)
		self.assertIn("actual != 0U &&", source)
		self.assertIn("readback == actual", source)
		self.assertNotIn("CLOCK_DEMO_APB_READONLY", source)
		self.assertNotIn("CLOCK_DEMO_APB_SHARED_CORE", source)
		self.assertNotIn('ClockDemoResult("APB"', source)

	def test_clock_demo_has_one_generic_clock_path(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertIn("SystemCoreClockSelect(pSource->Type, pSource->Frequency)", source)
		self.assertIn("SystemPeriphClockSet(clockIdx, request)", source)
		self.assertIn("SystemLowFreqClockSelect(pLfSource->Type, pLfSource->Frequency)", source)
		self.assertIn("TimerGetLowFreqDevCount() + TimerGetHighFreqDevCount()", source)
		self.assertIn("ClockDemoTimer(timerDevNo);", source)
		self.assertIn("ClockDemoConcurrentTimerTest(lowTimerCount, highTimerCount);", source)
		self.assertIn("Timer independent clock ownership", source)
		self.assertIn("ClockDemoTimerInit(DevNo)", source)
		for name in ("CLOCK_DEMO_CORE_READONLY", "CLOCK_DEMO_DEFER_TIMER",
			"CLOCK_DEMO_UART_RX_TEST", "CLOCK_DEMO_STOP_UART",
			"CLOCK_DEMO_HF_SOURCE_ENABLE", "CLOCK_DEMO_LF_SOURCE_ENABLE",
			"CLOCK_DEMO_APB_DIVIDERS", "CLOCK_DEMO_APB_READONLY",
			"CLOCK_DEMO_APB_SHARED_CORE"):
			self.assertNotIn(name, source)

	def test_clock_demo_tests_all_reported_timers(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertIn("timerDevNo < timerCount", source)
		self.assertIn("cfg.DevNo = DevNo;", source)
		self.assertIn("TimerSetFrequency(&s_Timer", source)
		self.assertNotIn("CLOCK_DEMO_TIMER_DEVNO", source)
		self.assertNotIn("CLOCK_DEMO_STOP_TIMER_DEVNO", source)
		self.assertNotIn("CLOCK_DEMO_CYCLES", source)
		self.assertIn("TimerDisable(&s_Timer);", source)

	def test_clock_demo_checks_shared_clock_lifecycle(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertIn("ClockDemoSharedClockTest();", source)
		self.assertIn("Configured core clock selection", source)
		self.assertIn("Configured LF clock selection", source)
		for api in ("SystemHighFreqClockEnable",
			"SystemHighFreqClockDisable", "SystemLowFreqClockEnable",
			"SystemLowFreqClockDisable"):
			self.assertIn("#pragma weak " + api, source)
			self.assertIn(api + "();", source)
		self.assertIn('ClockDemoResult("HF shared clock API implemented", false)', source)
		self.assertIn('ClockDemoResult("LF shared clock API implemented", false)', source)
		self.assertIn('ClockDemoResult("HF shared clock lifecycle",', source)
		self.assertIn('ClockDemoResult("LF shared clock lifecycle",', source)

	def test_clock_demo_board_inputs(self):
		paths = sorted(ROOT.glob("**/exemples/ClockDemo/src/board.h"))
		self.assertTrue(paths)
		for path in paths:
			with self.subTest(path=path):
				source = path.read_text(encoding="utf-8")
				if "CLOCK_DEMO_SOURCES" not in source:
					continue
				self.assertIn("CLOCK_DEMO_PERIPH_DIVIDERS", source)
				for name in ("CLOCK_DEMO_TIMER_DEVNO", "CLOCK_DEMO_STOP_TIMER_DEVNO",
					"CLOCK_DEMO_CYCLES",
					"CLOCK_DEMO_APB_DIVIDERS", "CLOCK_DEMO_CORE_READONLY",
					"CLOCK_DEMO_DEFER_TIMER", "CLOCK_DEMO_STOP_UART",
					"CLOCK_DEMO_HF_SOURCE_ENABLE", "CLOCK_DEMO_LF_SOURCE_ENABLE",
					"CLOCK_DEMO_UART_RX_TEST", "CLOCK_DEMO_APB_READONLY",
					"CLOCK_DEMO_APB_SHARED_CORE"):
					self.assertNotIn(name, source)

	def test_clock_demo_finishes_last_print(self):
		source = CLOCK.read_text(encoding="utf-8")
		final = source.index('ClockDemoPrint("ClockDemo complete:')
		end = source.index("while (1)", final)
		self.assertLess(final, end)
		self.assertIn("ClockDemoFlush();", source[final:end])
		self.assertNotIn("UARTPowerOff(&s_Uart);", source)

	def test_nrf_polling_uart_drains_before_power_off(self):
		source = NRF_UART.read_text(encoding="utf-8")
		poll = source.index("static bool nRFUARTDrainPollingTx(")
		poweroff = source.index("void nRFUARTPowerOff(")
		drain = source[poll:poweroff]
		self.assertIn("nRFUARTPollLegacyTx(dev);", drain)
		self.assertIn("CFifoUsed(dev->pUartDev->hTxFifo)", drain)
		self.assertIn("remaining == 0 && dev->pUartDev->bTxReady", drain)
		self.assertIn("timeout >= NRFX_UART_TX_DRAIN_WAIT_LOOPS", drain)
		self.assertIn("!pDev->bDma && !pDev->bIntEn", source[poweroff:])
		self.assertIn("!nRFUARTDrainPollingTx(dev)", source[poweroff:])
		self.assertIn("return;", source[poweroff:])

	def test_nrf_dma_uart_drains_before_power_off(self):
		source = NRF_UART.read_text(encoding="utf-8")
		drain = source.index("static bool nRFUARTDrainDmaTx(")
		poweroff = source.index("void nRFUARTPowerOff(")
		self.assertLess(drain, poweroff)
		body = source[drain:poweroff]
		self.assertIn("CFifoUsed(dev->pUartDev->hTxFifo) == 0", body)
		self.assertIn("dev->pUartDev->bTxReady", body)
		self.assertIn("NRFX_UART_DMA_DRAIN_TIMEOUT_US", body)
		self.assertIn("usDelay(NRFX_UART_DMA_DRAIN_POLL_US)", body)
		self.assertIn("elapsed += NRFX_UART_DMA_DRAIN_POLL_US", body)
		shutdown = source[poweroff:source.index("void *nRFUARTGetHandle(", poweroff)]
		self.assertIn("pDev->bDma && pDev->bIntEn", shutdown)
		self.assertIn("!nRFUARTDrainDmaTx(dev)", shutdown)
		self.assertLess(shutdown.index("!nRFUARTDrainDmaTx(dev)"),
			shutdown.index("IOPinDisable("))

	def test_power_off_demo(self):
		source = POWER.read_text(encoding="utf-8")
		self.assertNotIn("hTxFifo", source)
		self.assertNotIn("hRxFifo", source)
		self.assertNotIn("SysLogGet()->hFifo", source)
		self.assertNotIn("s_Uart.Rx(&unused", source)
		self.assertNotIn("s_Uart.Rx(&dummy", source)
		self.assertNotIn("timer_sam4l.h", source)
		self.assertNotIn("SAM4L_PM->", source)
		self.assertNotIn("SAM4L_AST->", source)
		self.assertNotIn("SAM4L_BPM->", source)
		self.assertNotIn("Sam4lUartSleepDiagnostic", source)
		self.assertIn("SysLogFlush(SysLogGet())", source)

	def test_i2c_sleep_demo(self):
		source = I2C.read_text(encoding="utf-8")
		self.assertNotIn("hTxFifo", source)
		self.assertNotIn("bTxReady", source)
		self.assertNotIn("g_Uart.Disable();", source)
		self.assertNotIn("g_Uart.Enable();", source)
		self.assertIn("SystemSleep();", source)
		self.assertIn("fflush(stdout);", source)

	def test_power_clock_examples_use_plain_engineering_terms(self):
		paths = (
			ROOT / "exemples/misc/clock_demo.cpp",
			ROOT / "exemples/misc/power_off_demo.cpp",
			ROOT / "exemples/misc/hfxo_test.cpp",
			ROOT / "exemples/misc/hfxo_test_ble.cpp",
			ROOT / "exemples/misc/hfxo_test_ble.md",
			ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp",
			ROOT / "ARM/Nordic/src/nrf_mpsl.cpp",
			ROOT / "ARM/Nordic/src/timer_hf_nrfx.cpp",
			ROOT / "ARM/Nordic/src/timer_lf_nrfx.cpp",
			ROOT / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp",
			ROOT / "docs/architecture/system-clock-timer-lifecycle.md",
			ROOT / "docs/architecture/nordic-clock-ownership-review-014.md",
		)
		terms = re.compile(r"\b(?:backend|contract|guarding|guardrail|API surface|source closure)\b", re.I)
		for path in paths:
			with self.subTest(path=path):
				for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
					if "LIABILITY" in line.upper() and "CONTRACT" in line.upper():
						continue
					self.assertIsNone(terms.search(line), f"{path}:{lineno}: {line.strip()}")

	def test_shared_clock_examples_do_not_access_vendor_registers(self):
		paths = (
			ROOT / "exemples/misc/clock_demo.cpp",
			ROOT / "exemples/misc/power_off_demo.cpp",
			ROOT / "exemples/misc/hfxo_test.cpp",
			ROOT / "exemples/misc/hfxo_test_ble.cpp",
		)
		vendor = re.compile(r"\b(?:NRF_CLOCK|NRF_POWER|NRF_RTC|NRF_UARTE|RCC|SAM4L_PM)\s*->|"
		                    r"\b(?:nrf_sdh_enable_request|nrf_sdh_disable_request|MpslInit)\s*\(")
		for path in paths:
			with self.subTest(path=path):
				source = path.read_text(encoding="utf-8")
				self.assertIsNone(vendor.search(source))

	def test_no_private_uart_casts(self):
		for path in (CLOCK, POWER, I2C):
			with self.subTest(path=path.name):
				source = path.read_text(encoding="utf-8")
				self.assertNotRegex(source, r"UARTDev_t\s*\*\s*(?:pUart|uart)\s*=")


	def test_applications_do_not_read_uart_internals(self):
		paths = (
			CLOCK,
			POWER,
			I2C,
			ROOT / "exemples/misc/hfxo_test.cpp",
			ROOT / "exemples/misc/hfxo_test_ble.cpp",
			ROOT / "exemples/bluetooth/uart_ble_central.cpp",
		)
		fields = re.compile(r"\b(?:hTxFifo|hRxFifo|TxDropCnt|bTxReady|pDevData)\b")
		for path in paths:
			with self.subTest(path=path.name):
				source = path.read_text(encoding="utf-8")
				self.assertIsNone(fields.search(source))


	def test_lfrc_diagnostics_are_deferred_and_board_scoped(self):
		clock = (ROOT / "ARM/Nordic/src/system_clock_nrfx.cpp").read_text(encoding="utf-8")
		controller = (ROOT / "ARM/Nordic/src/bt_hci_ctlr_sdc.cpp").read_text(encoding="utf-8")
		project = (ROOT / "ARM/Nordic/nRF52/nRF52840/lib/ioc/.cproject").read_text(encoding="utf-8")
		advertiser = (ROOT / "exemples/bluetooth/ble_advertiser.cpp").read_text(encoding="utf-8")
		self.assertIn("static void nRFxMpslCalibrationLog(", clock)
		self.assertIn("AppEvtHandlerQue(0U, nullptr, nRFxMpslCalibrationLog)", clock)
		self.assertIn('SysLogPrintf(SysLogGet(), "LFRC calibration: completed=%lu', clock)
		self.assertIn("SDC: LFRC diagnostic enabled=%u external=%u completed=%lu", controller)
		self.assertEqual(project.count('value="NRFX_LFRC_CAL_LOG"'), 4)
		self.assertNotIn("nRFxMpslLfclkCalibrationCount(", advertiser)
		self.assertNotIn("NRF_CLOCK->", advertiser)


if __name__ == "__main__":
	unittest.main()
