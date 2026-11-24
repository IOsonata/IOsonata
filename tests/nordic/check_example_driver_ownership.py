#!/usr/bin/env python3
"""Check that generic sleep and clock demos use driver interfaces.

These checks detect source-level regressions. Hardware TX completion, sleep,
and wake behavior require separate target testing.
"""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
CLOCK = ROOT / "exemples/misc/core_clock_test.cpp"
POWER = ROOT / "exemples/misc/power_off_demo.cpp"
I2C = ROOT / "exemples/i2c/i2c_slave_demo.cpp"
NRF_UART = ROOT / "ARM/Nordic/src/uart_nrfx.cpp"


class DemoDriverOwnership(unittest.TestCase):
	def test_core_clock(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertNotIn("hTxFifo", source)
		self.assertNotIn("hRxFifo", source)
		self.assertNotIn("CFifoUsed(", source)
		start = source.index("static void CoreClockTestFlush(void)\n{")
		end = source.index("\n}", start)
		self.assertNotIn("UARTRx(", source[start:end])
		self.assertIn("msDelay(2);", source[start:end])

	def test_nordic_core_clock_uses_irq_dma(self):
		# UART interrupts are on by default in the generic CoreClockTest.
		# Nordic boards opt into EasyDMA without duplicating this default.
		boards = (
			"nRF52/nRF52832",
			"nRF52/nRF52840",
			"nRF53/nRF5340_App",
			"nRF53/nRF5340_Net",
			"nRF54/nRF54L15",
			"nRF54/nRF54LM20A",
			"nRF54/nRF54LM20B",
		)
		for board in boards:
			path = ROOT / "ARM/Nordic" / board / "exemples/CoreClockTest/src/board.h"
			with self.subTest(board=board):
				source = path.read_text(encoding="utf-8")
				self.assertNotIn("#define CORE_CLOCK_TEST_UART_INT", source)
				self.assertIn("#define CORE_CLOCK_TEST_UART_DMA true", source)

		source = CLOCK.read_text(encoding="utf-8")
		self.assertIn("#define CORE_CLOCK_TEST_UART_INT true", source)
		self.assertIn("#define CORE_CLOCK_TEST_UART_DMA false", source)
		self.assertIn(".bIntMode = CORE_CLOCK_TEST_UART_INT", source)
		self.assertIn(".bDMAMode = CORE_CLOCK_TEST_UART_DMA", source)
		self.assertIn("DEVINTRF_EVT_TX_FIFO_EMPTY", source)
		self.assertIn("CoreClockTestFlush();", source)

	def test_core_clock_uart_batches_dma_output(self):
		source = CLOCK.read_text(encoding="utf-8")
		start = source.index("#define CORE_CLOCK_TEST_TX_BLOCK_SIZE")
		end = source.index("\nstatic void CoreClockTestResult(", start)
		formatting = source[start:end]
		self.assertIn("#define CORE_CLOCK_TEST_TX_BLOCK_SIZE\t64U", formatting)
		self.assertIn("CoreClockTestWrite(block, used);", formatting)
		self.assertIn("CoreClockTestAppend(block, &used,", formatting)
		self.assertIn("(int)(Length - offset)", formatting)
		self.assertIn("CORE_CLOCK_TEST_TX_RETRY_LIMIT", formatting)
		self.assertIn("s_UartTxEmpty = false;", formatting)
		self.assertNotIn("UARTTx(&s_Uart, &value, 1)", source)
		self.assertIn("DEVINTRF_EVT_TX_FIFO_EMPTY", source)

	def test_core_clock_uses_existing_delay_calibration(self):
		source = CLOCK.read_text(encoding="utf-8")
		start = source.index("static bool CoreClockTestDelaySample(")
		end = source.index("static void CoreClockTestDelayCalibrationRecord(", start)
		section = source[start:end]

		self.assertIn("if (DelayCalibrationMeasure(pSample))", section)
		self.assertIn("DelayCalibrationFromTimer(cpuHz, usElapsed, nsPublicElapsed,", section)
		self.assertIn("CoreClockTestDelayDivisors(pSample);", section)
		self.assertIn("TimerGetHighFreqDevCount()", section)
		self.assertIn("TimerGetHighFreqDevNo()", section)
		self.assertLess(section.index("DelayCalibrationMeasure(pSample)"),
			section.index("TimerGetHighFreqDevCount()"))
		self.assertIn("TimerGetMicrosecond(&s_RetainedTimer)", section)
		self.assertIn("msDelay(100);", section)
		self.assertIn("nsDelay(100000000U);", section)
		self.assertIn("CoreClockTestDelayCalibrationRecord();", source)
		self.assertIn("CoreClockTestDelayCalibrationReport();", source)
		self.assertNotIn("DelayCalibrationInterpolate(", source)
		self.assertIn("if (core != s_DelayHigh.CpuHz", source)

	def test_core_clock_peripheral_clock_interface(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertIn("unsigned dividerIdx;", source)
		self.assertIn("for (dividerIdx = 1U; dividerIdx <= 4U;", source)
		self.assertIn("divider = dividerIdx;", source)
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
		self.assertNotIn('CoreClockTestResult("APB"', source)

	def test_core_clock_has_one_generic_clock_path(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertIn("SystemCoreClockSelect(initialCoreSource, initialCoreFreq)", source)
		self.assertIn("SystemPeriphClockSet(clockIdx, request)", source)
		self.assertIn("SystemLowFreqClockSelect(initialLfSource, initialLfFreq)", source)
		self.assertIn("lowTimerCount = TimerGetLowFreqDevCount();", source)
		self.assertIn("highTimerCount = TimerGetHighFreqDevCount();", source)
		self.assertIn("CoreClockTestTimer(timerDevNo);", source)
		self.assertIn('CoreClockTestPrint("LF RC timer timing test\\r\\n");', source)
		self.assertIn('CoreClockTestPrint("Restored LF source timer timing test\\r\\n");', source)
		self.assertIn("timerDevNo < TimerGetLowFreqDevCount();", source)
		self.assertLess(source.index('CoreClockTestPrint("LF RC timer timing test'),
			source.index("SystemLowFreqClockSelect(initialLfSource, initialLfFreq);",
				source.index('CoreClockTestPrint("LF RC timer timing test')))

		self.assertIn("CoreClockTestConcurrentTimerTest(lowTimerCount, highTimerCount);", source)
		self.assertIn("Timer independent clock ownership", source)
		self.assertIn("CoreClockTestTimerInit(DevNo, pTimer)", source)
		for name in ("CLOCK_DEMO_CORE_READONLY", "CLOCK_DEMO_DEFER_TIMER",
			"CLOCK_DEMO_UART_RX_TEST", "CLOCK_DEMO_STOP_UART",
			"CLOCK_DEMO_HF_SOURCE_ENABLE", "CLOCK_DEMO_LF_SOURCE_ENABLE",
			"CLOCK_DEMO_APB_DIVIDERS", "CLOCK_DEMO_APB_READONLY",
			"CLOCK_DEMO_APB_SHARED_CORE"):
			self.assertNotIn(name, source)

	def test_core_clock_tests_all_reported_timers(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertIn("timerDevNo < timerCount", source)
		self.assertIn("cfg.DevNo = DevNo;", source)
		self.assertIn("TimerSetFrequency(pTimer", source)
		self.assertNotIn("CLOCK_DEMO_TIMER_DEVNO", source)
		self.assertNotIn("CLOCK_DEMO_STOP_TIMER_DEVNO", source)
		self.assertNotIn("CLOCK_DEMO_CYCLES", source)
		self.assertIn("TimerDisable(pTimer);", source)

	def test_core_clock_checks_shared_clock_lifecycle(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertIn("CoreClockTestSharedClockTest();", source)
		self.assertIn("Configured core source", source)
		self.assertIn("Configured LF source", source)
		for api in ("SystemHighFreqClockEnable",
			"SystemHighFreqClockDisable", "SystemLowFreqClockEnable",
			"SystemLowFreqClockDisable"):
			self.assertIn("#pragma weak " + api, source)
			self.assertIn(api + "();", source)
		self.assertIn('CoreClockTestResult("HF shared clock API implemented", false)', source)
		self.assertIn('CoreClockTestResult("LF shared clock API implemented", false)', source)
		self.assertIn('CoreClockTestResult("HF shared clock lifecycle",', source)
		self.assertIn('CoreClockTestResult("LF shared clock lifecycle",', source)

	def test_core_clock_calibration_api(self):
		source = CLOCK.read_text(encoding="utf-8")
		self.assertIn("#pragma weak SystemHighFreqClockCalibrate", source)
		self.assertIn("#pragma weak SystemLowFreqClockCalibrate", source)
		self.assertIn("CoreClockTestCalibrationTest();", source)
		self.assertLess(source.index("for (cycle = 1U;"),
			source.index("CoreClockTestCalibrationTest();"))
		self.assertLess(source.index("SystemLowFreqClockSelect(OSC_TYPE_RC, 32768U)"),
			source.index("CoreClockTestCalibrationTest();"))
		self.assertIn("SystemLowFreqClockEnable();", source)
		self.assertIn("calibrated = SystemLowFreqClockCalibrate();", source)
		self.assertIn("SystemLowFreqClockDisable();", source)
		self.assertIn('CoreClockTestResult("LF RC calibration", calibrated);', source)
		begin = source.index("static void CoreClockTestCalibrationTest(void)")
		end = source.index("static bool CoreClockTestTimerInit(", begin)
		section = source[begin:end]
		self.assertNotIn("TimerInit(", section)
		self.assertNotIn("AppEvtHandlerQue(", section)
		self.assertNotIn("NRF_CLOCK", section)

	def test_core_clock_board_inputs(self):
		source = CLOCK.read_text(encoding="utf-8")
		macros = ("CLOCK_DEMO_SOURCES", "CLOCK_DEMO_LF_SOURCES",
			"CLOCK_DEMO_PERIPH_DIVIDERS")
		paths = sorted(ROOT.glob("**/exemples/ClockDemo/src/board.h"))

		self.assertTrue(paths)

		for name in macros:
			self.assertNotIn(name, source)

		for path in paths:
			with self.subTest(path=path):
				board = path.read_text(encoding="utf-8")

				for name in macros:
					self.assertNotIn("#define " + name, board)

	def test_core_clock_finishes_last_print(self):
		source = CLOCK.read_text(encoding="utf-8")
		final = source.index('CoreClockTestPrint("CoreClockTest complete:')
		end = source.index("while (1)", final)
		self.assertLess(final, end)
		self.assertIn("CoreClockTestFlush();", source[final:end])
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
			ROOT / "exemples/misc/core_clock_test.cpp",
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
			ROOT / "exemples/misc/core_clock_test.cpp",
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
