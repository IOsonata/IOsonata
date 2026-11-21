#!/usr/bin/env python3
"""Check CorePowerTest runs all nRF5340 power capabilities at runtime."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "exemples/misc/core_power_test.cpp"
API = ROOT / "include/coredev/system_core_clock.h"
NRF = ROOT / "ARM/Nordic/src/system_power_nrfx.cpp"

class RuntimePowerChecks(unittest.TestCase):
	def test_runtime_sequence(self):
		s = SRC.read_text(encoding="utf-8")
		main = s[s.index("int main(void)"):]
		self.assertIn("caps = SystemPowerCapabilitiesGet();", main)
		self.assertLess(main.index("CorePowerTestSleep(caps)"), main.index("SystemPowerOff();"))
		self.assertIn("CorePowerTestRetention(caps", main)
		self.assertNotIn("POWER_DEMO_RESUME_WAKE", s)
		for name in ("CorePowerTestResult", "CorePowerTestSummary",
			"CORE_POWER_TEST_SLEEP_CYCLES", "SystemSleep();", "SystemPowerOff();"):
			self.assertIn(name, s)

	def test_core_power_requires_capability_api_linkage(self):
		s = SRC.read_text(encoding="utf-8")
		self.assertIn("#if defined(CORE_POWER_TEST_LEGACY_OPTIONAL_APIS) && defined(__GNUC__)", s)
		self.assertIn("#pragma weak SystemPowerCapabilitiesGet", s)
		compat = (ROOT / "exemples/misc/power_api_test.cpp").read_text(encoding="utf-8")
		self.assertIn("#define CORE_POWER_TEST_LEGACY_OPTIONAL_APIS 1", compat)
		for core in ("nRF5340_App", "nRF5340_Net"):
			project = (ROOT / "ARM/Nordic/nRF53" / core / "exemples/CorePowerTest/ioc/.project").read_text(encoding="utf-8")
			self.assertIn("exemples/misc/core_power_test.cpp", project)
			self.assertNotIn("exemples/misc/power_api_test.cpp", project)

	def test_app_and_net_differ_in_backend_capabilities(self):
		s = NRF.read_text(encoding="utf-8")
		self.assertIn("uint32_t SystemPowerCapabilitiesGet(void)", s)
		self.assertIn("SYSTEM_POWER_CAP_SYSTEM_OFF", s)
		self.assertIn("SYSTEM_POWER_CAP_TIMER_WAKE", s)
		self.assertIn("defined(NRF5340_XXAA_NETWORK)", s)
		self.assertIn("defined(NRF5340_XXAA_APPLICATION)", s)

	def test_core_power_uses_syslog_without_base_uart_tx_changes(self):
		s = SRC.read_text(encoding="utf-8")
		header = (ROOT / "include/coredev/uart.h").read_text(encoding="utf-8")
		self.assertIn("virtual int Tx(uint8_t *pData, uint32_t Len)", header)
		self.assertIn("UARTTx(UARTDev_t * const pDev, uint8_t *pData", header)
		self.assertNotIn("s_Uart.Tx(", s)
		self.assertIn("SysLogPrintf(SysLogGet(),", s)

	def test_retention_marker_requires_system_off_wake(self):
		s = SRC.read_text(encoding="utf-8")
		self.assertIn("(reason & (SYSTEM_RESET_WAKE_PIN | SYSTEM_RESET_WAKE_TIMER)) != 0U", s)
		self.assertIn("SystemRetRegWrite(1, 0U)", s)

	def test_nrf_dma_launch_serialized_with_tx_irq(self):
		s = (ROOT / "ARM/Nordic/src/uart_nrfx.cpp").read_text(encoding="utf-8")
		start = s.index("static int nRFUARTTxDataDma(")
		end = s.index("static void nRFUARTStopTx(", start)
		tx = s[start:end]
		pos = tx.index("// The completion IRQ uses the same ready flag")
		claim = tx.index("state = DisableInterrupt();", pos)
		ready = tx.index("if (dev->pUartDev->bTxReady)", claim)
		launch = tx.index("NRFX_UARTE_TASKS_STARTTX", ready)
		restore = tx.index("EnableInterrupt(state);", launch)
		self.assertLess(claim, ready)
		self.assertLess(ready, launch)
		self.assertLess(launch, restore)

	def test_console_retries_syslog_after_uart_irq_drain(self):
		s = SRC.read_text(encoding="utf-8")
		self.assertNotIn("SysLogVPrintf(", s)
		self.assertIn("SysLogUartEvent", s)
		self.assertIn("SysLogUartEvent", s)
		self.assertNotIn("CorePowerTestDrainLog(", s)
		self.assertIn("SysLogPrintf(SysLogGet(),", s)
		self.assertNotIn("DeviceIntrfWaitTxComplete(", s)

	def test_syslog_console(self):
		s = SRC.read_text(encoding="utf-8")
		self.assertIn("SysLogInitNoTimer(SysLogGet(), &s_LogCfg", s)
		self.assertIn('SysLogPrintf(SysLogGet(), "%s: %s', s)
		self.assertIn("SYSLOG_MEMSIZE(16, 128)", s)
		self.assertNotIn("s_Uart.printf(", s)
		self.assertNotIn("CorePowerTestPut(", s)

	def test_syslog_uart_owns_tx_ready_callback(self):
		s = SRC.read_text(encoding="utf-8")
		header = (ROOT / "include/syslog.h").read_text(encoding="utf-8")
		impl = (ROOT / "src/syslog.cpp").read_text(encoding="utf-8")
		self.assertIn(".EvtCallback = SysLogUartEvent,", s)
		self.assertIn("int SysLogUartEvent(UARTDev_t * const pDev", header)
		self.assertIn("EvtId != UART_EVT_TXREADY", impl)
		self.assertIn("return SysLogFlush(pLog);", impl)
		self.assertNotIn("CorePowerTestDrainLog(", s)
		self.assertIn("CFifoResv(pLog->hFifo)", impl)

	def test_console_uses_interrupt_driven_uarte(self):
		s = SRC.read_text(encoding="utf-8")
		self.assertIn(".bIntMode = true,", s)
		self.assertIn(".bDMAMode = true,", s)
		self.assertNotIn(".bIntMode = false,", s)
		self.assertNotIn(".bDMAMode = false,", s)
		self.assertNotIn("DeviceIntrfWaitTxComplete", s)

	def test_console_uses_syslog_not_direct_uart_printf(self):
		s = SRC.read_text(encoding="utf-8")
		self.assertIn('SysLogPrintf(SysLogGet(), "%s: %s', s)
		self.assertIn('SysLogPrintf(SysLogGet(), "CorePowerTest complete:', s)
		self.assertIn("SysLogUartEvent", s)
		self.assertNotIn("s_Uart.printf(", s)
		self.assertNotIn("s_Uart.Tx(", s)

	def test_vmc_retention_write_is_verified(self):
		backend = NRF.read_text(encoding="utf-8")
		self.assertIn("SYS_PWR_VMC->RAM[blk].POWER & msk", backend)
		self.assertIn("!= bRetain", backend)
		s = SRC.read_text(encoding="utf-8")
		self.assertIn("RAM non-retention unchanged words", s)
		self.assertIn("unchanged != count", s)

	def test_reversible_boundaries_exercised(self):
		s = SRC.read_text(encoding="utf-8")
		for name in ("RAM retention invalid range", "Wake pin invalid index",
			"Wake pin disable", "Wake pin rearm",
			"Retention register invalid index"):
			self.assertIn(name, s)

	def test_nrf53_preserves_partial_uarte_dma_transmission(self):
		s = (ROOT / "ARM/Nordic/src/uart_nrfx.cpp").read_text(encoding="utf-8")
		self.assertIn("defined(NRF71_SERIES) || defined(NRF53_SERIES)", s)
		self.assertIn("sent = dev->pDmaReg->TXD.AMOUNT;", s)
		self.assertIn("memmove(dev->TxDmaMem, dev->TxDmaMem + sent, remaining);", s)
		self.assertIn("dev->TxDmaLen = remaining;", s)
		self.assertEqual(s.count("defined(NRF71_SERIES) || defined(NRF53_SERIES)"), 5)
		self.assertIn("NRFX_UARTE_TASKS_STARTTX(dev->pDmaReg) = 1;", s)

	def test_board_describes_resources_not_test_modes(self):
		for core in ("nRF5340_App", "nRF5340_Net"):
			p = ROOT / "ARM/Nordic/nRF53" / core / "exemples/CorePowerTest/src/board.h"
			s = p.read_text(encoding="utf-8")
			self.assertNotIn("#define POWER_DEMO_RESUME_WAKE", s)

if __name__ == "__main__":
	unittest.main()
