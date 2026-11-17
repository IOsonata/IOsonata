/**-------------------------------------------------------------------------
@file	system_power_esp32.c

@brief	ESP32-C3/C6 reset reporting and sleep entry.

SystemSleep requests a resumable RTC/PMU sleep with an armed GPIO wake
source. Unsupported configurations use interrupt-resumable CPU WFI.
SystemPowerOff enters reset-on-wake sleep with a dedicated RTC/LP GPIO
wake source. The application stops active peripherals before entry.
Reset codes follow Espressif ESP-IDF v5.5.1.

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
#define ESP32_WAKE_CAUSE_REG (*(volatile uint32_t *)0x600080F8UL)
#define ESP32_WAKE_PIN_MASK (1UL << 2)
#define ESP32_WAKE_TIMER_MASK (1UL << 3)
#elif defined(ESP32C6)
#define ESP32_RESET_CAUSE_REG (*(volatile uint32_t *)(ESP32_LP_CLKRST_BASE + 0x10U))
#define ESP32_RESET_CAUSE_MASK 0x1FU
#define ESP32_WAKE_CAUSE_REG (*(volatile uint32_t *)0x600B0140UL)
#define ESP32_WAKE_PIN_MASK ((1UL << 0) | (1UL << 1) | (1UL << 2))
#define ESP32_WAKE_TIMER_MASK (1UL << 4)
#else
#error "System power is implemented for ESP32-C3 and ESP32-C6"
#endif

bool Esp32PowerSleep(void);
void Esp32PowerDeepSleep(void);

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
		case 0x05U:
		{
			uint32_t wake = ESP32_WAKE_CAUSE_REG;
			uint32_t reason = 0;

			if (wake & ESP32_WAKE_PIN_MASK)
			{
				reason |= SYSTEM_RESET_WAKE_PIN;
			}
			if (wake & ESP32_WAKE_TIMER_MASK)
			{
				reason |= SYSTEM_RESET_WAKE_TIMER;
			}
			return reason;
		}
		case 0x07U:
		case 0x08U:
		case 0x09U:
		case 0x0BU:
		case 0x0DU:
		case 0x10U:
		case 0x11U:
		case 0x12U:
			return SYSTEM_RESET_WDT;
		case 0x15U:	// Host reset through USB Serial/JTAG serial interface.
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
	uintptr_t state;

	// The controller restores wake routing and its CPU clock before
	// returning. If no RTC/PMU wake was configured, ordinary WFI is used.
	state = DisableInterrupt();
	if (!Esp32PowerSleep())
	{
		__WFI();
	}
	EnableInterrupt(state);
}

void SystemPowerOff(void)
{
	// The RTC/PMU may reject entry if no usable low-power GPIO is armed.
	// Either way the application must restart after this call.
	(void)DisableInterrupt();
	Esp32PowerDeepSleep();
	for (;;)
	{
		__WFI();
	}
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
