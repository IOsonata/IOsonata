/**-------------------------------------------------------------------------
@example	power_off_demo.cpp

@brief	MCU power off and wake up demo

Logs with SysLog on the console UART the reason of the last reset and a
start count kept in retention register 0, then powers the MCU off. The wake
up button of board.h starts it again: the next start logs a wake up by a pin
and the count one higher. A power on reset starts the count over.

The MCU powers off POWER_DEMO_DELAY msec after the start, once the button is
released. With a debugger attached, power off can be emulated by the MCU and
the demo then stays stopped instead of waking up.

The board.h of the project gives the console UART pins and the wake up
button, a button to ground by default. WAKE_BUT_SENSE
IOPINSENSE_HIGH_TRANSITION selects a button to the supply, for MCU that wake
up only on a rising level. When SystemWakePin refuses the pin, the demo says
so and only the reset pin or a power cycle starts the MCU again.

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2026

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

#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "syslog.h"
#include "idelay.h"

#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE			1000000
#endif

// Time before power off, in msec
#ifndef POWER_DEMO_DELAY
#define POWER_DEMO_DELAY		3000
#endif

// Level of the wake up button that wakes the MCU, a button to ground by default
#ifndef WAKE_BUT_SENSE
#define WAKE_BUT_SENSE			IOPINSENSE_LOW_TRANSITION
#endif

#if defined(POWER_DEMO_RAM_RET_TEST) && POWER_DEMO_RAM_RET_TEST
// RAM1 is not allocated to .data, .bss or the stack by the nRF54L15 linker.
// Use volatile accesses; startup must not overwrite this section before check.
static volatile uint32_t * const s_pRamRet =
	(volatile uint32_t *)POWER_DEMO_RAM_RET_ADDR;

static volatile uint32_t * const s_pRamDrop =
	(volatile uint32_t *)POWER_DEMO_RAM_DROP_ADDR;

#define RAM_TEST_PREPARED 0xA7U

static uint32_t RamDropPattern(uint32_t Idx)
{
	return 0x6C39E2B5UL ^ (Idx * 0x85EBCA6BUL);
}

static bool RamDropCheck(void)
{
	uint32_t i;
	uint32_t unchanged;
	uint32_t count;

	count = POWER_DEMO_RAM_DROP_SIZE / sizeof(uint32_t);
	unchanged = 0U;
	for (i = 0; i < count; i++)
	{
		if (s_pRamDrop[i] == RamDropPattern(i))
		{
			unchanged++;
		}
	}

	SysLogPrintf(SysLogGet(),
		"RAM non-retention %s, %u/%u words unchanged\r\n",
		unchanged == count ? "FAIL" : "PASS",
		(unsigned)unchanged, (unsigned)count);

	return unchanged != count;
}

static void RamDropWrite(void)
{
	uint32_t i;
	uint32_t count;

	count = POWER_DEMO_RAM_DROP_SIZE / sizeof(uint32_t);
	for (i = 0; i < count; i++)
	{
		s_pRamDrop[i] = RamDropPattern(i);
	}
}

static uint32_t RamRetPattern(uint32_t Idx)
{
	return 0xA55A1234UL ^ (Idx * 0x9E3779B9UL);
}

static bool RamRetCheck(void)
{
	uint32_t i;
	uint32_t count = POWER_DEMO_RAM_RET_SIZE / sizeof(uint32_t);

	for (i = 0; i < count; i++)
	{
		if (s_pRamRet[i] != RamRetPattern(i))
		{
			SysLogPrintf(SysLogGet(),
				"RAM retention FAIL at word %u: 0x%08x, expected 0x%08x\r\n",
				(unsigned)i, (unsigned)s_pRamRet[i],
				(unsigned)RamRetPattern(i));
			return false;
		}
	}

	SysLogPrintf(SysLogGet(), "RAM retention PASS, %u words\r\n",
		(unsigned)count);
	return true;
}

static void RamRetWrite(void)
{
	uint32_t i;
	uint32_t count = POWER_DEMO_RAM_RET_SIZE / sizeof(uint32_t);

	for (i = 0; i < count; i++)
	{
		s_pRamRet[i] = RamRetPattern(i);
	}
}
#endif

// Pull resistor and level of the released button
static const IOPINRES s_WakeButRes = WAKE_BUT_SENSE == IOPINSENSE_HIGH_TRANSITION ? IOPINRES_PULLDOWN : IOPINRES_PULLUP;
static const int s_WakeButIdle = WAKE_BUT_SENSE == IOPINSENSE_HIGH_TRANSITION ? 0 : 1;

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_BAUDRATE,
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

// SysLog on the console UART, a record goes out as soon as it is logged.
// 16 records of 128 bytes. A record the UART does not take in full stays in
// the store and the rest goes out at the next log call. bBlocking keeps the
// oldest records when the store is full.
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, 128)];

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = 128,
	.bBlocking = true,
};

static const struct {
	uint32_t Reason;
	const char *pName;
} s_ResetNames[] = {
	{ SYSTEM_RESET_POWERON, "power on" },
	{ SYSTEM_RESET_PIN, "reset pin" },
	{ SYSTEM_RESET_WDT, "watchdog" },
	{ SYSTEM_RESET_SOFT, "software" },
	{ SYSTEM_RESET_LOCKUP, "lockup" },
	{ SYSTEM_RESET_DEBUG, "debug interface" },
	{ SYSTEM_RESET_WAKE_PIN, "wake up by a pin" },
	{ SYSTEM_RESET_WAKE_COMP, "wake up by the comparator" },
	{ SYSTEM_RESET_WAKE_NFC, "wake up by NFC" },
	{ SYSTEM_RESET_WAKE_USB, "wake up by USB" },
	{ SYSTEM_RESET_WAKE_TIMER, "wake up by a timer" },
};

int main()
{
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);

	SysLogPrintf(SysLogGet(), "PowerOffDemo\r\n");

	uint32_t reason = SystemResetReason();

	const char *sep = " ";

	SysLogPrintf(SysLogGet(), "Reset reason 0x%x:", (unsigned)reason);
	for (unsigned i = 0; i < sizeof(s_ResetNames) / sizeof(s_ResetNames[0]); i++)
	{
		if (reason & s_ResetNames[i].Reason)
		{
			SysLogPrintf(SysLogGet(), "%s%s", sep, s_ResetNames[i].pName);
			sep = ", ";
		}
	}
	SysLogPrintf(SysLogGet(), "\r\n");

	// The retention registers are cleared by a power on reset only
	uint32_t cnt = (reason & SYSTEM_RESET_POWERON) ? 0 : SystemRetRegRead(0);

	cnt++;
	SystemRetRegWrite(0, cnt);
	SysLogPrintf(SysLogGet(), "Start count %u, %d retention registers\r\n", (unsigned)cnt, SystemRetRegCount());

#if defined(POWER_DEMO_RAM_RET_TEST) && POWER_DEMO_RAM_RET_TEST
	// Check before writing the next pattern. Startup must leave RAM1 untouched.
	if ((reason & SYSTEM_RESET_WAKE_PIN) &&
		SystemRetRegRead(1) == RAM_TEST_PREPARED)
	{
		(void)RamRetCheck();
		(void)RamDropCheck();
	}
	else
	{
		SysLogPrintf(SysLogGet(), "RAM test: preparing first cycle\r\n");
	}

	if (!SystemRamRetain(POWER_DEMO_RAM_RET_ADDR, POWER_DEMO_RAM_RET_SIZE, true))
	{
		SysLogPrintf(SysLogGet(), "RAM retention configuration FAIL\r\n");
		while (1)
		{
			__WFE();
		}
	}
	if (!SystemRamRetain(POWER_DEMO_RAM_DROP_ADDR, POWER_DEMO_RAM_DROP_SIZE, false))
	{
		SysLogPrintf(SysLogGet(), "RAM non-retention configuration FAIL\r\n");
		while (1)
		{
			__WFE();
		}
	}

	RamRetWrite();
	RamDropWrite();
	if (!SystemRetRegWrite(1, RAM_TEST_PREPARED))
	{
		SysLogPrintf(SysLogGet(), "RAM test marker FAIL\r\n");
		while (1)
		{
			__WFE();
		}
	}
	SysLogPrintf(SysLogGet(), "RAM retention enabled, %u bytes\r\n",
		(unsigned)POWER_DEMO_RAM_RET_SIZE);
	SysLogPrintf(SysLogGet(), "RAM non-retention disabled, %u bytes\r\n",
		(unsigned)POWER_DEMO_RAM_DROP_SIZE);
#endif

	IOPinConfig(WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_PINOP, IOPINDIR_INPUT, s_WakeButRes, IOPINTYPE_NORMAL);

	msDelay(POWER_DEMO_DELAY);

	// The button wakes the MCU when pressed, it must be released before power off
	while (IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN) != s_WakeButIdle);

	if (SystemWakePin(WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_SENSE))
	{
		SysLogPrintf(SysLogGet(), "Power off, press the button to wake up\r\n");
	}
	else
	{
		SysLogPrintf(SysLogGet(), "Power off, the button cannot wake this MCU, use the reset pin\r\n");
	}

	// Time for the UART to send the last record
	msDelay(100);

	SystemPowerOff();

	return 0;
}
