/**-------------------------------------------------------------------------
@example	hfxo_test_ble.cpp

@brief	HFXO requests with Nordic BLE clock ownership

Exercises the generic HF clock API before and after BLE initialization.
Debug writes through semihosting; Release writes to UART at 1 Mbaud.
The SDC configuration initializes MPSL, which manages HFXO requests.
This does not test advertising, connections or enabling SDC itself.

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

#include "nrf.h"
#include "coredev/system_core_clock.h"
#include "idelay.h"

#if defined(NRFXLIB_SDC)
#include "nrf_mpsl.h"
#elif defined(SOFTDEVICE_PRESENT)
#include "sdk_config.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "sd_dispatch.h"
#else
#error "HfxoTestBle requires SoftDevice or SDC"
#endif

#ifdef NDEBUG
#include "coredev/uart.h"
#include "syslog.h"
#include "board.h"

#define HFXO_PRINT(...) SysLogPrintf(SysLogGet(), __VA_ARGS__)

// ENDTX is handled by the UART driver interrupt. Wake the foreground
// drain when DMA completes, without polling UARTE registers.
static int HfxoUartEvent(UARTDev_t *pDev, UART_EVT Event, uint8_t *pBuffer,
	int BufferLen)
{
	(void)pDev;
	(void)pBuffer;
	(void)BufferLen;

	if (Event == UART_EVT_TXREADY)
	{
		__SEV();
	}
	return 0;
}

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
	.EvtCallback = HfxoUartEvent,
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

static void HfxoConsoleInit(void)
{
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);
}

static void HfxoConsoleDrain(void)
{
	UARTDev_t *pUart;

	pUart = (UARTDev_t *)s_Uart;
	for (;;)
	{
		(void)SysLogFlush(SysLogGet());
		if (CFifoPeek(SysLogGet()->hFifo) == nullptr &&
			CFifoUsed(pUart->hTxFifo) == 0U &&
			pUart->bTxReady)
		{
			break;
		}
		// TXREADY callback posts an event when the active DMA ends.
		// Check the queues before waiting to avoid missed completions.
		__WFE();
	}
}
#else
#include <stdio.h>
#define HFXO_PRINT(...) printf(__VA_ARGS__)

static void HfxoConsoleInit(void)
{
}

static void HfxoConsoleDrain(void)
{
}
#endif

#if defined(SOFTDEVICE_PRESENT) && !defined(NRFXLIB_SDC)
// The SoftDevice sends NRF_EVT_HFCLKSTARTED through SD_EVT_IRQHandler.
// Keep its interrupt handler and SoC event polling function linked.
static void HfxoSdDispatch(void)
{
	nrf_sdh_evts_poll();
}

extern "C" void nrf_sdh_soc_evts_poll(void *pContext);

NRF_SDH_STACK_OBSERVER(s_HfxoSocObserver, NRF_SDH_SOC_STACK_OBSERVER_PRIO) = {
	.handler = nrf_sdh_soc_evts_poll,
	.p_context = NULL,
};
#endif

static bool HfxoStackInit(void)
{
#if defined(NRFXLIB_SDC)
	return MpslInit();
#else
	SetSoftdeviceDispatch(HfxoSdDispatch);
	return nrf_sdh_enable_request() == 0U;
#endif
}

// CLOCK register status: verify the crystal source, not merely HFCLK running.
static bool HfxoRunning(void)
{
	return (NRF_CLOCK->HFCLKSTAT &
		(CLOCK_HFCLKSTAT_STATE_Msk | CLOCK_HFCLKSTAT_SRC_Msk)) ==
		(CLOCK_HFCLKSTAT_STATE_Msk |
		 (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos));
}

static void HfxoResult(const char *pName, bool Result, bool Running)
{
	HFXO_PRINT("%s: %s, HFXO %s\r\n", pName,
		Result ? "PASS" : "FAIL",
		Running ? "running" : "stopped");
}

// Check an outstanding HFXO request across BLE initialization.
// The Nordic clock implementation selects the oscillator and handles BLE startup.
static int HfxoPreStackTest(void)
{
	bool enabled;
	bool running;
	bool stackReady;
	bool survived;
	bool reenabled;
	bool stopped;
#if defined(NRFXLIB_SDC)
	unsigned sample;
	unsigned stoppedSamples;
#endif

	HFXO_PRINT("Pre-stack HFXO ownership test\r\n");
	enabled = SystemHighFreqClockEnable();
	running = HfxoRunning();
	HfxoResult("Pre-stack enable", enabled && running, running);
	if (!enabled || !running)
	{
		return 1;
	}

	stackReady = HfxoStackInit();
	HFXO_PRINT("Stack initialization: %s\r\n", stackReady ? "PASS" : "FAIL");
	if (!stackReady)
	{
		SystemHighFreqClockDisable();
		return 1;
	}

	survived = HfxoRunning();
	HfxoResult("Clock survives stack init", survived, survived);
	if (!survived)
	{
		SystemHighFreqClockDisable();
		return 1;
	}

	SystemHighFreqClockDisable();
	// A pre-stack request must not leave HFXO running when MPSL has
	// no HF request. S132 may defer physical STOP until shutdown.
#if defined(NRFXLIB_SDC)
	// LFRC calibration may briefly request HFXO. Observe several
	// calibration-free intervals rather than checking one instant.
	stoppedSamples = 0U;
	for (sample = 0U; sample < 200U; sample++)
	{
		msDelay(10U);
		if (!HfxoRunning())
		{
			stoppedSamples++;
		}
	}
	stopped = stoppedSamples != 0U;
	HFXO_PRINT("Pre-stack physical release: %s, HFXO stopped %u/200 samples\r\n",
		stopped ? "PASS" : "FAIL", stoppedSamples);
#else
	stopped = true;
	HFXO_PRINT("Pre-stack request released, HFXO %s\r\n",
		HfxoRunning() ? "running" : "stopped");
#endif

	reenabled = SystemHighFreqClockEnable();
	running = HfxoRunning();
	HfxoResult("Enable through stack", reenabled && running, running);
	if (!reenabled || !running)
	{
		if (reenabled)
		{
			SystemHighFreqClockDisable();
		}
		return 1;
	}
	SystemHighFreqClockDisable();
	HFXO_PRINT("Stack request released\r\n");
	HFXO_PRINT("Pre-stack handover: %s\r\n", stopped ? "PASS" : "FAIL");
	return stopped ? 0 : 1;
}

int main(void)
{
	int preStackResult;
	bool initial;
	bool first;
	bool second;
	bool disable1;
	bool disable2;
	bool running1;
	bool running2;
	bool running3;
	bool final;
	bool failed;
	int pass;
#if defined(SOFTDEVICE_PRESENT) && !defined(NRFXLIB_SDC)
	bool stopped;
	bool enabled;
	bool started;
	bool released;
#endif

	HfxoConsoleInit();
	HFXO_PRINT("HfxoTestBle\r\n");
#if defined(NRFXLIB_SDC)
	// Test the LFRC selection while MPSL retains its own calibration.
	// This belongs to the Nordic diagnostic, not the portable application.
	if (!SystemLowFreqClockSelect(OSC_TYPE_RC, 32768U))
	{
		HFXO_PRINT("Select LFRC: FAIL\r\n");
		HfxoConsoleDrain();
		return 1;
	}
	g_McuOsc.LowPwrOsc.Accuracy = 500U;
	HFXO_PRINT("Select LFRC: PASS\r\n");
#endif
	preStackResult = HfxoPreStackTest();
	HFXO_PRINT("Pre-stack test result: %s\r\n",
		preStackResult == 0 ? "PASS" : "FAIL");
	if (preStackResult != 0)
	{
		HfxoConsoleDrain();
		while (1)
		{
			__WFE();
		}
	}
	HFXO_PRINT("Clock owner initialized\r\n");

	initial = HfxoRunning();
	HFXO_PRINT("Initial HFXO: %s\r\n", initial ? "running" : "stopped");

	first = SystemHighFreqClockEnable();
	running1 = HfxoRunning();
	second = SystemHighFreqClockEnable();
	running2 = HfxoRunning();
	SystemHighFreqClockDisable();
	disable1 = true;
	running3 = HfxoRunning();
	SystemHighFreqClockDisable();
	disable2 = true;
	final = HfxoRunning();
	SystemHighFreqClockDisable();
	failed = HfxoRunning() == final;

	pass = 0;
	if (first && running1)
	{
		pass++;
	}
	HfxoResult("First enable", first && running1, running1);
	if (second && running2)
	{
		pass++;
	}
	HfxoResult("Second enable", second && running2, running2);
	if (disable1 && running3)
	{
		pass++;
	}
	HfxoResult("First disable", disable1 && running3, running3);
	if (disable2)
	{
		pass++;
	}
	HfxoResult("Last disable", disable2, final);
	if (failed)
	{
		pass++;
	}
	HfxoResult("Unmatched disable", failed, final);
	HFXO_PRINT("HfxoTestBle: %d/5 checks PASS\r\n", pass);

#if defined(SOFTDEVICE_PRESENT) && !defined(NRFXLIB_SDC)
	// Verify the deferred direct HFXO release at SoftDevice shutdown.
	// A subsequent direct enable/disable must work without stack ownership.
	if (nrf_sdh_disable_request() == 0U)
	{
		stopped = !HfxoRunning();
		HfxoResult("S132 shutdown", stopped, !stopped);
		enabled = SystemHighFreqClockEnable();
		started = HfxoRunning();
		HfxoResult("Direct enable after S132", enabled && started, started);
		if (enabled)
		{
			SystemHighFreqClockDisable();
		}
		released = !HfxoRunning();
		HfxoResult("Direct release after S132", released, !released);
	}
	else
	{
		HFXO_PRINT("S132 shutdown: FAIL\r\n");
	}
#endif

	HfxoConsoleDrain();
	while (1)
	{
		__WFE();
	}

	return 0;
}
