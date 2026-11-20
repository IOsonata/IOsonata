/**-------------------------------------------------------------------------
@example	hfxo_test_ble.cpp

@brief	HFXO requests with Nordic BLE clock ownership

Initializes SoftDevice or MPSL before testing generic HF clock Enable/Disable.
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

// Opt-in diagnostic: 1 = direct-to-stack, 2 = disable-before-stack.
// The default test retains stack-first initialization.
#ifndef HFXO_TEST_PRESTACK
#define HFXO_TEST_PRESTACK 0
#endif

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
	.EvtCallback = nullptr,
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
#else
#include <stdio.h>
#define HFXO_PRINT(...) printf(__VA_ARGS__)

static void HfxoConsoleInit(void)
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

#if HFXO_TEST_PRESTACK
// Check whether a direct HFXO request remains balanced when the BLE
// stack initializes. Never issue a direct CLOCK STOP after stack init.
static int HfxoPreStackTest(void)
{
	bool selected;
	bool enabled;
	bool running;
	bool stackReady;
#if HFXO_TEST_PRESTACK == 1
	bool survived;
#endif
	bool disabled;
	bool reenabled;
	bool cleared;

	HFXO_PRINT("Pre-stack HFXO ownership test\r\n");
	selected = SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U);
	enabled = selected && SystemHighFreqClockEnable();
	running = HfxoRunning();
	HfxoResult("Pre-stack enable", enabled && running, running);
	if (!enabled || !running)
	{
		return 1;
	}

#if HFXO_TEST_PRESTACK == 2
	// A direct clock requirement must end before MPSL/SoftDevice owns CLOCK.
	// It may stop temporarily; this is not a seamless handover.
	SystemHighFreqClockDisable();
	disabled = SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U);
	running = HfxoRunning();
	HfxoResult("Direct clock stopped before stack", disabled && !running,
		running);
	if (!disabled || running)
	{
		return 1;
	}
#endif
	stackReady = HfxoStackInit();
	HFXO_PRINT("Stack initialization: %s\r\n", stackReady ? "PASS" : "FAIL");
	if (!stackReady)
	{
		return 1;
	}

#if HFXO_TEST_PRESTACK == 1
	survived = HfxoRunning();
	HfxoResult("Clock survives stack init", survived, survived);
	if (!survived)
	{
		return 1;
	}

	SystemHighFreqClockDisable();
	// Selection is refused while a shared HFXO request remains active.
	// This catches an outstanding direct request after BLE takes CLOCK.
	disabled = SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U);
	HFXO_PRINT("Pre-stack request disabled: %s\r\n",
		disabled ? "PASS" : "FAIL");
	if (!disabled)
	{
		HFXO_PRINT("Outstanding direct HFXO request after stack init\r\n");
		return 1;
	}
#endif

	reenabled = SystemHighFreqClockEnable();
	running = HfxoRunning();
	HfxoResult("Enable through stack", reenabled && running, running);
	if (!reenabled || !running)
	{
		return 1;
	}
	SystemHighFreqClockDisable();
	cleared = SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U);
	HFXO_PRINT("Stack request disabled: %s\r\n",
		cleared ? "PASS" : "FAIL");
#if HFXO_TEST_PRESTACK == 1
	HFXO_PRINT("Pre-stack handover: %s\r\n", cleared ? "PASS" : "FAIL");
#else
	HFXO_PRINT("Release-before-stack sequence: %s\r\n",
		cleared ? "PASS" : "FAIL");
#endif
	return cleared ? 0 : 1;
}
#endif

int main(void)
{
#if HFXO_TEST_PRESTACK
	int preStackResult;
#endif
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

	HfxoConsoleInit();
	HFXO_PRINT("HfxoTestBle\r\n");
#if HFXO_TEST_PRESTACK
	preStackResult = HfxoPreStackTest();
	HFXO_PRINT("Pre-stack test result: %s\r\n",
		preStackResult == 0 ? "PASS" : "FAIL");
	if (preStackResult != 0)
	{
		while (1)
		{
			__WFE();
		}
	}
#else
	if (!HfxoStackInit())
	{
		HFXO_PRINT("Stack initialization: FAIL\r\n");
		while (1)
		{
			__WFE();
		}
	}
#endif
	HFXO_PRINT("Clock owner initialized\r\n");

	initial = HfxoRunning();
	HFXO_PRINT("Initial HFXO: %s\r\n", initial ? "running" : "stopped");

	first = SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U) &&
		SystemHighFreqClockEnable();
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

	while (1)
	{
		__WFE();
	}

	return 0;
}
