/**-------------------------------------------------------------------------
@example	hfxo_test_ble.cpp

@brief	Generic Bluetooth initialization and shared HF clock requests

The same application source tests clock requests across Bluetooth initialization
on Nordic S1xx, SDC, and STM32WBA. The IOsonata Bluetooth implementation initializes the
vendor stack and its clock requirements.

@author	Hoang Nguyen Hoan
@date	Nov. 9, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

#include "coredev/system_core_clock.h"
#include "bluetooth/bt_app.h"
#include "app_evt_handler.h"

#ifdef NDEBUG
#include "coredev/uart.h"
#include "syslog.h"
#include "idelay.h"
#include "board.h"
#else
#include <stdio.h>
#endif

static const BtAppCfg_t s_BtCfg = {
	.Role = BTAPP_ROLE_BROADCASTER,
	.PeriphDevMax = 0,
	.CentralDevMax = 1,
	.pDevName = "ClockTest",
	.AdvInterval = 100,
	.ConnLedPort = -1,
	.ConnLedPin = -1,
};

#ifdef NDEBUG
#define HFXO_PRINT(...) SysLogPrintf(SysLogGet(), __VA_ARGS__)

static int HfxoUartEvent(DevIntrf_t *pDev, DEVINTRF_EVT Event,
	uint8_t *pBuffer, int BufferLen);

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.EvtCB = HfxoUartEvent,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

static UART s_Uart;
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, 128)];

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = 128,
	.bBlocking = true,
};

// ENDTX is handled by the UART driver interrupt. Wake the foreground
// drain when DMA completes, without polling UARTE registers.
static int HfxoUartEvent(DevIntrf_t *pDev, DEVINTRF_EVT Event, uint8_t *pBuffer,
	int BufferLen)
{
	(void)pDev;
	(void)pBuffer;
	(void)BufferLen;

	if (Event == DEVINTRF_EVT_TX_READY)
	{
		__SEV();
	}
	return 0;
}

static void HfxoConsoleInit(void)
{
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);
}

static void HfxoConsoleDrain(void)
{
	// SysLogFlush submits queued records to the transport; it does not
	// guarantee physical UART completion. Keep the console alive briefly
	// before leaving this test without inspecting UART driver state.
	(void)SysLogFlush(SysLogGet());
	msDelay(100);

}
#else
#define HFXO_PRINT(...) printf(__VA_ARGS__)

static void HfxoConsoleInit(void)
{
}

static void HfxoConsoleDrain(void)
{
}
#endif

int main(void)
{
	bool before;
	bool bluetooth;
	bool second;
	bool after;

	HfxoConsoleInit();
	HFXO_PRINT("HfxoTestBle\r\n");

	// Exercise the generic shared clock API. The Bluetooth implementation
	// selects the controller and its clocks without application flags.
	before = SystemHighFreqClockEnable();
	HFXO_PRINT("Clock request before Bluetooth: %s\r\n",
		before ? "PASS" : "FAIL");

	bluetooth = before && BtAppInit(&s_BtCfg);
	HFXO_PRINT("Bluetooth initialization: %s\r\n",
		bluetooth ? "PASS" : "FAIL");

	second = bluetooth && SystemHighFreqClockEnable();
	HFXO_PRINT("Second clock request: %s\r\n",
		second ? "PASS" : "FAIL");

	if (second)
	{
		SystemHighFreqClockDisable();
	}
	if (before)
	{
		SystemHighFreqClockDisable();
	}

	after = bluetooth && SystemHighFreqClockEnable();
	HFXO_PRINT("Clock request after Bluetooth: %s\r\n",
		after ? "PASS" : "FAIL");
	if (after)
	{
		SystemHighFreqClockDisable();
	}
	HFXO_PRINT("HfxoTestBle: %s\r\n",
		before && bluetooth && second && after ? "PASS" : "FAIL");

	HfxoConsoleDrain();
	if (before && bluetooth && second && after)
	{
		// Service Bluetooth work through IOsonata, not through a vendor loop.
		AppRun();
	}
	return before && bluetooth && second && after ? 0 : 1;
}
