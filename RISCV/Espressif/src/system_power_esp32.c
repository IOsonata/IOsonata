/**-------------------------------------------------------------------------
@file	system_power_esp32.c

@brief	ESP32-C3/C6 reset reason and interrupt-resumable CPU sleep.

CPU sleep keeps the peripheral clocks, RAM and stack running. PMU Light-sleep
and reset-on-wake Deep-sleep are not implemented here. Reset causes follow
Espressif ESP-IDF v5.5.1 per-chip reset_reasons.h and register definitions.

@license MIT
Copyright (c) 2026, I-SYST inc., all rights reserved.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "esp32xx.h"

#if defined(ESP32C3)
#define ESP32_RESET_CAUSE_REG (*(volatile uint32_t *)0x60008038UL)
#define ESP32_RESET_CAUSE_MASK 0x3FU
#elif defined(ESP32C6)
#define ESP32_RESET_CAUSE_REG (*(volatile uint32_t *)(ESP32_LP_CLKRST_BASE + 0x10U))
#define ESP32_RESET_CAUSE_MASK 0x1FU
#else
#error "System power is implemented for ESP32-C3 and ESP32-C6"
#endif

uint32_t SystemResetReason(void)
{
	// The status records the last reset. Reading it does not clear it.
	switch (ESP32_RESET_CAUSE_REG & ESP32_RESET_CAUSE_MASK)
	{
		case 0x01U:
		case 0x0FU:
#if defined(ESP32C3)
		case 0x17U:
#endif
			return SYSTEM_RESET_POWERON;
		case 0x03U:
		case 0x0CU:
			return SYSTEM_RESET_SOFT;
		case 0x07U:
		case 0x08U:
		case 0x09U:
		case 0x0BU:
		case 0x0DU:
		case 0x10U:
		case 0x11U:
		case 0x12U:
			return SYSTEM_RESET_WDT;
		case 0x16U:
#if defined(ESP32C6)
		case 0x18U:
#endif
			return SYSTEM_RESET_DEBUG;
		default:
			return 0;
	}
}

void SystemSleep(void)
{
	uintptr_t state = DisableInterrupt();

	// Keep an enabled interrupt pending until WFI completes. Restore the
	// caller's interrupt mask before its handler runs; do not alter mie.
	__WFI();
	EnableInterrupt(state);
}

int SystemRetRegCount(void)
{
	// RTC scratch registers are used by ROM boot and clock calibration.
	return 0;
}

uint32_t SystemRetRegRead(int Idx)
{
	(void)Idx;
	return 0;
}

bool SystemRetRegWrite(int Idx, uint32_t Val)
{
	(void)Idx;
	(void)Val;
	return false;
}

bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
	(void)Addr;
	(void)Size;
	(void)bRetain;
	return false;
}
