/**-------------------------------------------------------------------------
@file	system_power_esp32.c

@brief	ESP32-C3/C6 reset reporting and sleep entry.

SystemSleep requests a resumable RTC/PMU sleep with an armed GPIO wake
source. Unsupported configurations use interrupt-resumable CPU WFI.
SystemPowerOff enters reset-on-wake sleep with a dedicated RTC/LP GPIO
wake source. Idle UART instances are temporarily suspended by SystemSleep; other
peripherals remain under their driver lifecycle control.
Reset codes follow Espressif ESP-IDF v5.5.1.

@license MIT
Copyright (c) 2026, I-SYST inc., all rights reserved.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "esp32xx.h"

#if defined(ESP32C3)
#define ESP32_RESET_CAUSE_REG (*(volatile uint32_t *)0x60008038UL)
#define ESP32_RESET_CAUSE_MASK 0x3FU
#define ESP32_RETAINED_RAM_SIZE 0x2000U
#define ESP32_RTC_MEMORY_POWER_REG (*(volatile uint32_t *)0x60008088UL)
#define ESP32_WAKE_CAUSE_REG (*(volatile uint32_t *)0x600080F8UL)
#define ESP32_WAKE_PIN_MASK (1UL << 2)
#define ESP32_WAKE_TIMER_MASK (1UL << 3)
#elif defined(ESP32C6)
#define ESP32_RESET_CAUSE_REG (*(volatile uint32_t *)(ESP32_LP_CLKRST_BASE + 0x10U))
#define ESP32_RESET_CAUSE_MASK 0x1FU
#define ESP32_RETAINED_RAM_SIZE 0x4000U
#define ESP32_WAKE_CAUSE_REG (*(volatile uint32_t *)0x600B0140UL)
#define ESP32_WAKE_PIN_MASK ((1UL << 0) | (1UL << 1) | (1UL << 2))
#define ESP32_WAKE_TIMER_MASK (1UL << 4)
#else
#error "System power is implemented for ESP32-C3 and ESP32-C6"
#endif

bool Esp32PowerSleep(void);
void Esp32PowerDeepSleep(void);
bool Esp32LightWakePrepare(void);
void Esp32LightWakeRestore(void);
bool Esp32UartPrepareSleep(void) __attribute__((weak));
void Esp32UartResumeSleep(void) __attribute__((weak));

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
	bool handled;

	state = DisableInterrupt();
	// Wake routing belongs to the system, not to a board-specific sleep
	// sequence. Keep ordinary WFI available when no RTC wake was selected.
	if (!Esp32LightWakePrepare())
	{
		__WFI();
		EnableInterrupt(state);
		return;
	}
	// Suspend idle peripherals only for RTC/PMU sleep. UART interrupts
	// remain enabled for a plain CPU WFI when no RTC wake was selected.
	if (Esp32UartPrepareSleep != NULL && !Esp32UartPrepareSleep())
	{
		Esp32LightWakeRestore();
		EnableInterrupt(state);
		return;
	}
	handled = Esp32PowerSleep();
	if (Esp32UartResumeSleep != NULL)
	{
		Esp32UartResumeSleep();
	}
	Esp32LightWakeRestore();
	(void)SystemCoreClockGet();
	if (!handled)
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
	// RTC/LP SRAM remains powered in Deep-sleep. HP SRAM cannot be kept.
	// These targets do not provide independent power-off of RTC/LP SRAM.
	if (!bRetain || Size == 0U || Addr < 0x50000000UL ||
		Addr - 0x50000000UL >= ESP32_RETAINED_RAM_SIZE ||
		Size > ESP32_RETAINED_RAM_SIZE - (Addr - 0x50000000UL))
	{
		return false;
	}
#if defined(ESP32C3)
	uintptr_t state = DisableInterrupt();
	uint32_t value = ESP32_RTC_MEMORY_POWER_REG;
	// Follow rtc_sleep_init: keep RTC fast memory powered in sleep.
	value = (value | (1UL << 16)) & ~(1UL << 15);
	ESP32_RTC_MEMORY_POWER_REG = value;
	bool ready = (ESP32_RTC_MEMORY_POWER_REG & (3UL << 15)) == (1UL << 16);
	EnableInterrupt(state);
	return ready;
#else
	// The PMU sleep sequence puts LP SRAM into its content-retaining
	// deep-sleep state, then restores access on wake. No HP bank is kept.
	return true;
#endif
}
