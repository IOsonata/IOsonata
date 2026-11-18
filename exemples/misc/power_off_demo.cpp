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
#include "coredev/timer.h"
#include "coredev/system_core_clock.h"
#include "syslog.h"
#include "idelay.h"

#include "board.h"

// Show the SAM4L AST failure stage on the UART without a debugger.
// The generic demo remains independent of SAM4L on other MCU ports.
#if (defined(SAM4LCXA) || defined(SAM4LCXB) || defined(SAM4LCXC) || \
	defined(SAM4LSXA) || defined(SAM4LSXB) || defined(SAM4LSXC)) && \
	defined(POWER_DEMO_RESUME_WAKE) && POWER_DEMO_RESUME_WAKE && \
	defined(POWER_DEMO_TIMER_WAKE_MS) && POWER_DEMO_TIMER_WAKE_MS > 0
#define POWER_DEMO_SAM4L_AST_DIAG 1
#include "timer_sam4l.h"
#endif

#if (defined(SAM4LCXA) || defined(SAM4LCXB) || defined(SAM4LCXC) || \
	defined(SAM4LSXA) || defined(SAM4LSXB) || defined(SAM4LSXC)) && \
	defined(POWER_DEMO_UART_WAKE) && POWER_DEMO_UART_WAKE
#define POWER_DEMO_SAM4L_UART_DIAG 1
uint32_t Sam4lUartSleepDiagnostic(void);
#endif

// UART wake requires a board profile with a supported sleep receiver.
#ifndef POWER_DEMO_UART_WAKE
#define POWER_DEMO_UART_WAKE 0
#endif
#if POWER_DEMO_UART_WAKE && (!defined(POWER_DEMO_RESUME_WAKE) || !POWER_DEMO_RESUME_WAKE)
#error "UART wake requires resumable sleep"
#endif

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE			1000000
#endif

// UART DMA can be disabled on targets without a DMA transport.
#ifndef POWER_DEMO_UART_DMA
#define POWER_DEMO_UART_DMA true
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
// The board-specific linker script must reserve both test regions outside
// .data, .bss and the stack. Startup must not overwrite them before check.
static volatile uint32_t * const s_pRamRet =
	(volatile uint32_t *)POWER_DEMO_RAM_RET_ADDR;

#if defined(POWER_DEMO_RAM_DROP_ADDR)
static volatile uint32_t * const s_pRamDrop =
	(volatile uint32_t *)POWER_DEMO_RAM_DROP_ADDR;
#endif

#define RAM_TEST_PREPARED 0xA7U

#endif

// Pull resistor and level of the released button
#if defined(WAKE_BUT_PIN)
static const IOPINRES s_WakeButRes = WAKE_BUT_SENSE == IOPINSENSE_HIGH_TRANSITION ? IOPINRES_PULLDOWN : IOPINRES_PULLUP;
static const int s_WakeButIdle = WAKE_BUT_SENSE == IOPINSENSE_HIGH_TRANSITION ? 0 : 1;
#endif

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
#if defined(POWER_DEMO_RESUME_WAKE) && POWER_DEMO_RESUME_WAKE
	.bIntMode = false,
#else
	.bIntMode = true,
#endif
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
#if defined(POWER_DEMO_RESUME_WAKE) && POWER_DEMO_RESUME_WAKE
	.bDMAMode = false,
#else
	.bDMAMode = POWER_DEMO_UART_DMA,
#endif
};

static UART s_Uart;
#if defined(POWER_DEMO_TIMER_WAKE_MS) && POWER_DEMO_TIMER_WAKE_MS > 0
static TimerDev_t s_WakeTimer;
static const TimerCfg_t s_WakeTimerCfg = {
	.DevNo = POWER_DEMO_TIMER_WAKE_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1024U,
	.IntPrio = 6,
};
#endif

#if defined(POWER_DEMO_RESUME_WAKE) && POWER_DEMO_RESUME_WAKE && \
	defined(POWER_DEMO_TIMER_WAKE_MS) && POWER_DEMO_TIMER_WAKE_MS > 0 && !POWER_DEMO_UART_WAKE
static volatile bool s_StandbyTimerWoke;
#endif

#if defined(POWER_DEMO_STANDBY_RAM_TEST) && POWER_DEMO_STANDBY_RAM_TEST
// Sample static and stack storage without overwriting live application RAM.
#define STANDBY_RAM_WORDS 32U
static volatile uint32_t s_StandbyRam[STANDBY_RAM_WORDS];
#endif


#if !defined(POWER_DEMO_RESUME_WAKE) || !POWER_DEMO_RESUME_WAKE
// SysLog on the console UART, a record goes out as soon as it is logged.
// 16 records of 128 bytes. A record the UART does not take in full stays in
// the store and the rest goes out at the next log call. bBlocking keeps the
// oldest records when the store is full.
#if defined(POWER_DEMO_RESUME_WAKE) && POWER_DEMO_RESUME_WAKE
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(4, 96)];
#else
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, 128)];
#endif

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen =
#if defined(POWER_DEMO_RESUME_WAKE) && POWER_DEMO_RESUME_WAKE
		96,
#else
		128,
#endif
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

#endif

#if defined(POWER_DEMO_RAM_RET_TEST) && POWER_DEMO_RAM_RET_TEST
#if defined(POWER_DEMO_RAM_DROP_ADDR)
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

#endif

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

#if defined(POWER_DEMO_RESUME_WAKE) && POWER_DEMO_RESUME_WAKE
// Minimal polling output avoids formatted logging on the 16 KB SAM D11.
static void PowerDemoPut(const char *pText)
{
	uint8_t c;

	while (*pText != 0)
	{
		c = (uint8_t)*pText++;
		while (s_Uart.Tx(&c, 1) != 1)
		{
		}
	}
}

// In SAMD polling mode, Tx() may leave the last bytes in its FIFO.
// Rx() services both FIFOs, so drain TX before shutting down SERCOM.
static void PowerDemoDrain(void)
{
	UARTDev_t *pUart;
	uint8_t dummy;

	pUart = (UARTDev_t *)s_Uart;
	while (CFifoUsed(pUart->hTxFifo) != 0)
	{
		(void)s_Uart.Rx(&dummy, 1);
	}
	// FIFO empty means the last byte reached DATA, not that TX is complete.
	msDelay(2);
}

static void PowerDemoPutCount(uint32_t Value)
{
	uint8_t digit;
	uint8_t c;
	int shift;

	for (shift = 28; shift >= 0; shift -= 4)
	{
		digit = (uint8_t)((Value >> shift) & 0xFU);
		c = (uint8_t)(digit < 10U ? '0' + digit : 'A' + digit - 10U);
		while (s_Uart.Tx(&c, 1) != 1)
		{
		}
	}
}

#endif

#if defined(POWER_DEMO_STANDBY_RAM_TEST) && POWER_DEMO_STANDBY_RAM_TEST
static uint32_t StandbyRamPattern(uint32_t Cycle, uint32_t Idx)
{
	return 0xA55A1234UL ^ (Cycle * 0x85EBCA6BUL) ^ (Idx * 0x9E3779B9UL);
}
#endif

#if defined(POWER_DEMO_RESUME_WAKE) && POWER_DEMO_RESUME_WAKE && \
	defined(POWER_DEMO_TIMER_WAKE_MS) && POWER_DEMO_TIMER_WAKE_MS > 0 && !POWER_DEMO_UART_WAKE
static void PowerDemoStandbyTimerWake(TimerDev_t *pTimer, int Trigger, void *pContext)
{
	(void)pTimer;
	(void)Trigger;
	(void)pContext;
	s_StandbyTimerWoke = true;
}
#endif

// Require a stable release before arming another sleep cycle.
static void PowerDemoWaitRelease(void)
{
#if defined(WAKE_BUT_PIN)
	unsigned idle = 0U;

	while (idle < 20U)
	{
		if (IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN) == s_WakeButIdle)
		{
			idle++;
		}
		else
		{
			idle = 0U;
		}
		msDelay(1);
	}
#endif
}

static bool PowerDemoWakeConfig(void)
{
#if defined(WAKE_BUT_PIN)
	IOPinConfig(WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_PINOP,
		IOPINDIR_INPUT, s_WakeButRes, IOPINTYPE_NORMAL);
	PowerDemoWaitRelease();
	return SystemWakePin(WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_SENSE);
#else
	return false;
#endif
}

int main()
{
#if defined(POWER_DEMO_RESUME_WAKE) && POWER_DEMO_RESUME_WAKE
	uint32_t wakeCount;
#if POWER_DEMO_UART_WAKE
	uint32_t rxWait;
	uint8_t rxByte;
	bool normalRx;
#endif
#if defined(POWER_DEMO_STANDBY_RAM_TEST) && POWER_DEMO_STANDBY_RAM_TEST
	volatile uint32_t stackRam[STANDBY_RAM_WORDS];
#endif

	if (!s_Uart.Init(s_UartCfg))
	{
		return 1;
	}
	PowerDemoPut("PowerOffDemo: Standby\r\n");

#if POWER_DEMO_UART_WAKE
	if (!s_Uart.EnableWake(true))
	{
		PowerDemoPut("UART wake unavailable for this configuration\r\n");
		return 1;
	}
	// Validate the physical USART0 RX connection before testing PICOUART.
	// The host sends one U at the normal 1 Mbps console rate.
	// Do not drain the RX FIFO while waiting for the test byte.
	PowerDemoPut("UART RX: send U at 1000000 baud\r\n");
	rxByte = 0U;
	normalRx = false;
	for (rxWait = 0U; rxWait < 3000U; rxWait++)
	{
		if (s_Uart.Rx(&rxByte, 1) == 1)
		{
			normalRx = rxByte == 'U';
			break;
		}
		msDelay(1);
	}
	PowerDemoPut(normalRx ? "UART RX normal: PASS\r\n" :
		"UART RX normal: FAIL\r\n");
	if (!normalRx)
	{
		return 1;
	}
#elif defined(POWER_DEMO_TIMER_WAKE_MS) && POWER_DEMO_TIMER_WAKE_MS > 0
	if (!TimerInit(&s_WakeTimer, &s_WakeTimerCfg))
	{
		PowerDemoPut("Standby timer initialization FAIL\r\n");
#if defined(POWER_DEMO_SAM4L_AST_DIAG)
		PowerDemoPut("SAM4L TimerInit stage 0x");
		PowerDemoPutCount(g_Sam4lTimerInitStage);
		PowerDemoPut("\r\n");
		PowerDemoPut("BPM_PMCON 0x");
		PowerDemoPutCount(SAM4L_BPM->BPM_PMCON);
		PowerDemoPut("\r\n");
		PowerDemoPut("BSCIF_PCLKSR 0x");
		PowerDemoPutCount(SAM4L_BSCIF->BSCIF_PCLKSR);
		PowerDemoPut("\r\n");
		PowerDemoPut("PM_PBDMASK 0x");
		PowerDemoPutCount(SAM4L_PM->PM_PBDMASK);
		PowerDemoPut("\r\n");
		PowerDemoPut("NVIC_ISER1 0x");
		PowerDemoPutCount(NVIC->ISER[1]);
		PowerDemoPut("\r\n");
		if (g_Sam4lTimerInitStage >= SAM4L_TIMER_INIT_SYNC)
		{
			PowerDemoPut("AST_SR 0x");
			PowerDemoPutCount(SAM4L_AST->AST_SR);
			PowerDemoPut("\r\n");
			PowerDemoPut("AST_CLOCK 0x");
			PowerDemoPutCount(SAM4L_AST->AST_CLOCK);
			PowerDemoPut("\r\n");
			PowerDemoPut("AST_CR 0x");
			PowerDemoPutCount(SAM4L_AST->AST_CR);
			PowerDemoPut("\r\n");
		}
#endif
		return 1;
	}
	TimerDisable(&s_WakeTimer);
#else
	if (!PowerDemoWakeConfig())
	{
		PowerDemoPut("Standby wake pin not available\r\n");
		return 1;
	}
#endif

	wakeCount = 0U;
	while (1)
	{
		uint32_t coreBefore = SystemCoreClockGet();
		uint32_t apbBefore = SystemPeriphClockGet(0);
		uint32_t rateBefore = s_Uart.Rate();
#if POWER_DEMO_UART_WAKE
		PowerDemoPut("Entering UART sleep; send one U at 9600 baud\r\n");
#elif defined(POWER_DEMO_TIMER_WAKE_MS) && POWER_DEMO_TIMER_WAKE_MS > 0
		PowerDemoPut("Entering Standby; automatic timer wake\r\n");
#else
		PowerDemoWaitRelease();
		PowerDemoPut("Entering Standby\r\n");
#endif
		PowerDemoDrain();
		// SystemSleep selects a compatible power state and temporarily
		// suspends idle peripherals. The application does not stop UART.
#if defined(POWER_DEMO_STANDBY_RAM_TEST) && POWER_DEMO_STANDBY_RAM_TEST
		for (uint32_t i = 0; i < STANDBY_RAM_WORDS; i++)
		{
			uint32_t pattern = StandbyRamPattern(wakeCount, i);

			s_StandbyRam[i] = pattern;
			stackRam[i] = ~pattern;
		}
#endif
#if defined(POWER_DEMO_TIMER_WAKE_MS) && POWER_DEMO_TIMER_WAKE_MS > 0 && !POWER_DEMO_UART_WAKE
		if (TimerSetFrequency(&s_WakeTimer, 1024U) == 0U)
		{
			PowerDemoPut("Standby timer frequency FAIL\r\n");
			return 1;
		}
		uint32_t state = DisableInterrupt();
		s_StandbyTimerWoke = false;
		uint32_t start = TimerGetMicrosecond(&s_WakeTimer);
		uint64_t period = nsTimerEnableTrigger(&s_WakeTimer, 0,
			(uint64_t)POWER_DEMO_TIMER_WAKE_MS * 1000000ULL,
			TIMER_TRIG_TYPE_SINGLE, PowerDemoStandbyTimerWake, nullptr);
		if (period != 0U && !s_StandbyTimerWoke)
		{
			SystemSleep();
		}
		EnableInterrupt(state);
		uint32_t elapsed = TimerGetMicrosecond(&s_WakeTimer) - start;
		TimerDisableTrigger(&s_WakeTimer, 0);
		TimerDisable(&s_WakeTimer);
#else
		SystemSleep();
#endif
#if defined(POWER_DEMO_STANDBY_RAM_TEST) && POWER_DEMO_STANDBY_RAM_TEST
		bool ramOk = true;
		for (uint32_t i = 0; i < STANDBY_RAM_WORDS; i++)
		{
			uint32_t pattern = StandbyRamPattern(wakeCount, i);

			if (s_StandbyRam[i] != pattern || stackRam[i] != ~pattern)
			{
				ramOk = false;
			}
		}
#endif
#if POWER_DEMO_UART_WAKE
		uint8_t wakeByte = 0;
		bool received = s_Uart.Rx(&wakeByte, 1) == 1 && wakeByte == 'U';
		// Let the host adapter switch back to its normal console baud.
		msDelay(100);
		PowerDemoPut(received ? "UART wake byte: PASS\r\n" : "UART wake byte: FAIL\r\n");
#if defined(POWER_DEMO_SAM4L_UART_DIAG)
		if (!received)
		{
			PowerDemoPut("SAM4L UART sleep diagnostic 0x");
			PowerDemoPutCount(Sam4lUartSleepDiagnostic());
			PowerDemoPut("\r\n");
		}
#endif
#endif
		// Keep wake verification in PowerOffDemo for every resumable port.
		// UART output here also exercises TX after resume.
		PowerDemoPut(s_Uart.Rate() == rateBefore ? "UART rate after resume: PASS\r\n" :
			"UART rate after resume: FAIL\r\n");
		PowerDemoPut(SystemCoreClockGet() == coreBefore && SystemPeriphClockGet(0) == apbBefore ?
			"CPU/APB clock after resume: PASS\r\n" :
			"CPU/APB clock after resume: FAIL\r\n");
#if POWER_DEMO_UART_WAKE
		if (!received)
		{
			// A refused sleep or an early wake is not a completed cycle.
			// Stop instead of printing another misleading resume count.
			PowerDemoPut("UART wake test stopped\r\n");
			PowerDemoDrain();
			return 1;
		}
#endif
#if defined(POWER_DEMO_TIMER_WAKE_MS) && POWER_DEMO_TIMER_WAKE_MS > 0 && !POWER_DEMO_UART_WAKE
		uint64_t expected = (uint64_t)POWER_DEMO_TIMER_WAKE_MS * 1000ULL;
		PowerDemoPut(period != 0U && s_StandbyTimerWoke &&
			(uint64_t)elapsed >= expected * 98ULL / 100ULL &&
			(uint64_t)elapsed <= expected * 102ULL / 100ULL ?
			"Standby timer wake: PASS\r\n" : "Standby timer wake: FAIL\r\n");
#endif
		wakeCount++;
		PowerDemoPut("Resumed from Standby, count 0x");
		PowerDemoPutCount(wakeCount);
		PowerDemoPut("\r\n");
#if defined(POWER_DEMO_STANDBY_RAM_TEST) && POWER_DEMO_STANDBY_RAM_TEST
		PowerDemoPut(ramOk ? "SRAM sample PASS: 128 static + 128 stack bytes\r\n" :
			"SRAM sample FAIL\r\n");
		if (!ramOk)
		{
			PowerDemoDrain();
			while (1)
			{
			}
		}
#endif
	}
#else
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
	// Check before writing the next pattern. Startup must leave the test region untouched.
	if ((reason & SYSTEM_RESET_WAKE_PIN) &&
		(SystemRetRegCount() < 2 || SystemRetRegRead(1) == RAM_TEST_PREPARED))
	{
		(void)RamRetCheck();
#if defined(POWER_DEMO_RAM_DROP_ADDR)
		(void)RamDropCheck();
#endif
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
#if defined(POWER_DEMO_RAM_DROP_ADDR)
	if (!SystemRamRetain(POWER_DEMO_RAM_DROP_ADDR, POWER_DEMO_RAM_DROP_SIZE, false))
	{
		SysLogPrintf(SysLogGet(), "RAM non-retention configuration FAIL\r\n");
		while (1)
		{
			__WFE();
		}
	}

	RamDropWrite();
#endif
	RamRetWrite();
	if (SystemRetRegCount() >= 2 && !SystemRetRegWrite(1, RAM_TEST_PREPARED))
	{
		SysLogPrintf(SysLogGet(), "RAM test marker FAIL\r\n");
		while (1)
		{
			__WFE();
		}
	}
	SysLogPrintf(SysLogGet(), "RAM retention enabled, %u bytes\r\n",
		(unsigned)POWER_DEMO_RAM_RET_SIZE);
#if defined(POWER_DEMO_RAM_DROP_ADDR)
	SysLogPrintf(SysLogGet(), "RAM non-retention disabled, %u bytes\r\n",
		(unsigned)POWER_DEMO_RAM_DROP_SIZE);
#endif
#endif

	msDelay(POWER_DEMO_DELAY);

#if defined(POWER_DEMO_TIMER_WAKE_MS) && POWER_DEMO_TIMER_WAKE_MS > 0
#if defined(WAKE_BUT_PIN)
	(void)SystemWakePin(WAKE_BUT_PORT, WAKE_BUT_PIN, IOPINSENSE_DISABLE);
#endif
	if (!TimerInit(&s_WakeTimer, &s_WakeTimerCfg))
	{
		SysLogPrintf(SysLogGet(), "Wake timer initialization FAIL\r\n");
		(void)SysLogFlush(SysLogGet());
		return 1;
	}
	TimerDisable(&s_WakeTimer);
	SysLogPrintf(SysLogGet(), "Power off, automatic timer wake in %u ms\r\n",
		(unsigned)POWER_DEMO_TIMER_WAKE_MS);
#else
	// Configure the pin and wait for release before arming System OFF wakeup.
	if (PowerDemoWakeConfig())
	{
		SysLogPrintf(SysLogGet(), "Power off, press the button to wake up\r\n");
	}
	else
	{
		SysLogPrintf(SysLogGet(), "Power off, the button cannot wake this MCU, use the reset pin\r\n");
	}

#endif
	// Allow the final UART record to finish before stopping the transport.
#if defined(POWER_DEMO_UART_OFF) && POWER_DEMO_UART_OFF
	UARTDev_t *pUart = (UARTDev_t *)s_Uart;
	uint8_t unused;

	while (CFifoUsed(SysLogGet()->hFifo) != 0 || CFifoUsed(pUart->hTxFifo) != 0)
	{
		(void)SysLogFlush(SysLogGet());
		(void)s_Uart.Rx(&unused, 1);
	}
	// Allow the last UART character to leave the shift register.
	msDelay(2);
	s_Uart.PowerOff();
#else
	msDelay(100);
#endif

#if defined(POWER_DEMO_TIMER_WAKE_MS) && POWER_DEMO_TIMER_WAKE_MS > 0
	// Keep the alarm pending until Backup entry; the reset starts a new timer.
	uint32_t state = DisableInterrupt();
	if (nsTimerEnableTrigger(&s_WakeTimer, 0,
		(uint64_t)POWER_DEMO_TIMER_WAKE_MS * 1000000ULL,
		TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0U || !TimerEnable(&s_WakeTimer))
	{
		EnableInterrupt(state);
		if (s_Uart.Init(s_UartCfg))
		{
			SysLogPrintf(SysLogGet(), "Wake timer start FAIL\r\n");
			(void)SysLogFlush(SysLogGet());
			msDelay(100);
		}
		return 1;
	}
#endif
	SystemPowerOff();

#if defined(POWER_DEMO_UART_OFF) && POWER_DEMO_UART_OFF
	// If power off returns, rebuild the console to report the failure.
	if (!s_Uart.Init(s_UartCfg))
	{
		return 1;
	}
	msDelay(50);
	SysLogPrintf(SysLogGet(), "SystemPowerOff returned\r\n");
	while (1)
	{
		__WFE();
	}
#endif

#endif
	return 0;
}
