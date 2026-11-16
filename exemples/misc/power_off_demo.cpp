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

#if defined(ESP32C6) && defined(POWER_DEMO_BUTTON_TRACE) && POWER_DEMO_BUTTON_TRACE
extern volatile uint32_t g_Esp32GpioIrqCount;
extern volatile uint32_t g_Esp32GpioLastPending;
extern volatile uint32_t g_Esp32C6SleepStage;
extern volatile uint32_t g_Esp32C6SleepGpio;
extern volatile uint32_t g_Esp32C6SleepWakeEnable;
extern volatile uint32_t g_Esp32C6SleepPmuRaw;
extern volatile uint32_t g_Esp32C6SleepWakeCause;
extern volatile uint32_t g_Esp32C6SleepRejectCause;

volatile uint32_t g_PowerDemoWakeStage;
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

static volatile uint32_t * const s_pRamDrop =
	(volatile uint32_t *)POWER_DEMO_RAM_DROP_ADDR;

#define RAM_TEST_PREPARED 0xA7U

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

#if defined(ESP32C6) && defined(POWER_DEMO_BUTTON_TRACE) && POWER_DEMO_BUTTON_TRACE
// Verify the input and its interrupt before the UART is turned off.
static bool PowerDemoTraceButton(void)
{
	uint32_t level;
	uint32_t pressed;
	uint32_t released;
	uint32_t irqBefore;
	uint32_t irqAfter;
	uint32_t pending;
	uint32_t polls;

	g_PowerDemoWakeStage = 0x01U;
	level = (uint32_t)IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN);
	PowerDemoPut("BOOT test: GPIO level 0x");
	PowerDemoPutCount(level);
	PowerDemoPut("\r\n");
	if (level != (uint32_t)s_WakeButIdle)
	{
		PowerDemoPut("BOOT test: release the button first\r\n");
		while (IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN) != s_WakeButIdle)
		{
			msDelay(10);
		}
		msDelay(20);
	}

	irqBefore = g_Esp32GpioIrqCount;
	PowerDemoPut("BOOT test: press and release BOOT\r\n");
	polls = 0U;
	for (;;)
	{
		pressed = (uint32_t)IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN);
		if (pressed != (uint32_t)s_WakeButIdle)
		{
			msDelay(20);
			if (IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN) != s_WakeButIdle)
			{
				break;
			}
		}
		msDelay(10);
		polls++;
		if (polls >= 100U)
		{
			polls = 0U;
			PowerDemoPut("BOOT wait: GPIO level 0x");
			PowerDemoPutCount((uint32_t)IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN));
			PowerDemoPut("\r\n");
		}
	}
	g_PowerDemoWakeStage = 0x02U;
	PowerDemoPut("BOOT pressed: GPIO level 0x");
	PowerDemoPutCount(pressed);
	PowerDemoPut(", IRQ count 0x");
	PowerDemoPutCount(g_Esp32GpioIrqCount);
	PowerDemoPut("\r\n");

	for (;;)
	{
		released = (uint32_t)IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN);
		if (released == (uint32_t)s_WakeButIdle)
		{
			msDelay(20);
			if (IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN) == s_WakeButIdle)
			{
				break;
			}
		}
		msDelay(10);
	}
	g_PowerDemoWakeStage = 0x03U;
	irqAfter = g_Esp32GpioIrqCount;
	pending = g_Esp32GpioLastPending;
	PowerDemoPut("BOOT released: GPIO level 0x");
	PowerDemoPutCount(released);
	PowerDemoPut(", IRQ count 0x");
	PowerDemoPutCount(irqAfter);
	PowerDemoPut(", last pending 0x");
	PowerDemoPutCount(pending);
	PowerDemoPut("\r\n");
	if (pressed == (uint32_t)s_WakeButIdle ||
		released != (uint32_t)s_WakeButIdle)
	{
		PowerDemoPut("BOOT input FAIL\r\n");
		return false;
	}
	PowerDemoPut("BOOT input PASS\r\n");
	if (irqAfter == irqBefore || (pending & (1UL << WAKE_BUT_PIN)) == 0U)
	{
		PowerDemoPut("BOOT GPIO interrupt NOT OBSERVED\r\n");
		return false;
	}
	PowerDemoPut("BOOT GPIO interrupt PASS\r\n");
	g_PowerDemoWakeStage = 0x04U;
	PowerDemoPut("Press BOOT again after Entering Standby to test wake\r\n");
	return true;
}
#endif
#endif

#if defined(POWER_DEMO_STANDBY_RAM_TEST) && POWER_DEMO_STANDBY_RAM_TEST
static uint32_t StandbyRamPattern(uint32_t Cycle, uint32_t Idx)
{
	return 0xA55A1234UL ^ (Cycle * 0x85EBCA6BUL) ^ (Idx * 0x9E3779B9UL);
}
#endif

int main()
{
#if defined(POWER_DEMO_RESUME_WAKE) && POWER_DEMO_RESUME_WAKE
	uint32_t wakeCount;
#if defined(ESP32C6) && defined(POWER_DEMO_BUTTON_TRACE) && POWER_DEMO_BUTTON_TRACE
	uint32_t irqBefore;
	uint32_t irqAfter;
	bool cpuWake;
#endif
#if defined(POWER_DEMO_STANDBY_RAM_TEST) && POWER_DEMO_STANDBY_RAM_TEST
	volatile uint32_t stackRam[STANDBY_RAM_WORDS];
#endif

	if (!s_Uart.Init(s_UartCfg))
	{
		return 1;
	}
	PowerDemoPut("PowerOffDemo: Standby\r\n");

	IOPinConfig(WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_PINOP,
		IOPINDIR_INPUT, s_WakeButRes, IOPINTYPE_NORMAL);
	if (!SystemWakePin(WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_SENSE))
	{
		PowerDemoPut("Standby wake pin not available\r\n");
		return 1;
	}

#if defined(ESP32C6) && defined(POWER_DEMO_BUTTON_TRACE) && POWER_DEMO_BUTTON_TRACE
	if (!PowerDemoTraceButton())
	{
		PowerDemoPut("Standby skipped: BOOT test failed\r\n");
		for (;;)
		{
			msDelay(1000);
		}
	}
#endif

	wakeCount = 0U;
	while (1)
	{
		while (IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN) != s_WakeButIdle)
		{
			msDelay(10);
		}
#if defined(ESP32C6) && defined(POWER_DEMO_BUTTON_TRACE) && POWER_DEMO_BUTTON_TRACE
		if (wakeCount == 0U)
		{
			PowerDemoPut("CPU WFI baseline: press BOOT after Entering Standby\r\n");
		}
		else
		{
			PowerDemoPut("PMU Light-sleep: press BOOT after Entering Standby\r\n");
		}
#endif
		PowerDemoPut("Entering Standby\r\n");
		PowerDemoDrain();
#if defined(ESP32C6) && defined(POWER_DEMO_BUTTON_TRACE) && POWER_DEMO_BUTTON_TRACE
		g_PowerDemoWakeStage = 0x05U;
#endif
		s_Uart.Disable();
#if defined(ESP32C6) && defined(POWER_DEMO_BUTTON_TRACE) && POWER_DEMO_BUTTON_TRACE
		g_PowerDemoWakeStage = 0x06U;
#endif
#if defined(POWER_DEMO_STANDBY_RAM_TEST) && POWER_DEMO_STANDBY_RAM_TEST
		for (uint32_t i = 0; i < STANDBY_RAM_WORDS; i++)
		{
			uint32_t pattern = StandbyRamPattern(wakeCount, i);

			s_StandbyRam[i] = pattern;
			stackRam[i] = ~pattern;
		}
#endif
#if defined(ESP32C6) && defined(POWER_DEMO_BUTTON_TRACE) && POWER_DEMO_BUTTON_TRACE
		g_PowerDemoWakeStage = 0x07U;
		cpuWake = false;
		if (wakeCount == 0U)
		{
			// First prove that GPIO9 resumes a CPU WFI while UART is off.
			// Do not modify the RTC/PMU state in this diagnostic cycle.
			irqBefore = g_Esp32GpioIrqCount;
			g_PowerDemoWakeStage = 0xA0U;
			while (g_Esp32GpioIrqCount == irqBefore)
			{
				__WFI();
			}
			irqAfter = g_Esp32GpioIrqCount;
			cpuWake = irqAfter != irqBefore &&
				(g_Esp32GpioLastPending & (1UL << WAKE_BUT_PIN)) != 0U;
			g_PowerDemoWakeStage = 0xA1U;
		}
		else
		{
			SystemSleep();
		}
		g_PowerDemoWakeStage = 0x08U;
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
		s_Uart.Enable();
#if defined(ESP32C6) && defined(POWER_DEMO_BUTTON_TRACE) && POWER_DEMO_BUTTON_TRACE
		g_PowerDemoWakeStage = 0x09U;
		if (wakeCount == 0U)
		{
			PowerDemoPut(cpuWake ?
				"CPU WFI baseline PASS: BOOT GPIO interrupt woke CPU\r\n" :
				"CPU WFI baseline FAIL: wake was not GPIO9\r\n");
		}
		else
		{
			PowerDemoPut("PMU sleep request returned to application\r\n");
			PowerDemoPut("PMU stage 0x");
			PowerDemoPutCount(g_Esp32C6SleepStage);
			PowerDemoPut(", flags 0x");
			PowerDemoPutCount(g_Esp32C6SleepPmuRaw);
			PowerDemoPut(", wake 0x");
			PowerDemoPutCount(g_Esp32C6SleepWakeCause);
			PowerDemoPut(", reject 0x");
			PowerDemoPutCount(g_Esp32C6SleepRejectCause);
			PowerDemoPut("\r\n");
			PowerDemoPut("PMU wake enable 0x");
			PowerDemoPutCount(g_Esp32C6SleepWakeEnable);
			PowerDemoPut(", input 0x");
			PowerDemoPutCount(g_Esp32C6SleepGpio);
			PowerDemoPut("\r\n");
		}
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

	// The button must be released before configuring System OFF wakeup.
	while (IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN) != s_WakeButIdle);

	if (SystemWakePin(WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_SENSE))
	{
		SysLogPrintf(SysLogGet(), "Power off, press the button to wake up\r\n");
	}
	else
	{
		SysLogPrintf(SysLogGet(), "Power off, the button cannot wake this MCU, use the reset pin\r\n");
	}

	// Allow the final UART record to finish before stopping the transport.
#if defined(POWER_DEMO_UART_OFF) && POWER_DEMO_UART_OFF
	msDelay(1000);
	s_Uart.Disable();
#else
	msDelay(100);
#endif

	SystemPowerOff();

#if defined(POWER_DEMO_UART_OFF) && POWER_DEMO_UART_OFF
	// If the power function returns, the system did not enter Standby.
	s_Uart.Enable();
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
