/**-------------------------------------------------------------------------
@file	system_sleep_esp32c3.c

@brief	ESP32-C3 RTC sleep controller.

Uses the ROM analog-I2C access functions and the RTC sleep sequence from the
ESP-IDF v5.5.1 hardware definitions. Light sleep preserves SRAM, flash power,
the XTAL and a running PLL. Deep sleep is reset-on-wake and requires a
registered GPIO0-5 RTC wake source. The caller stops its peripherals.

@license MIT
Copyright (c) 2026, I-SYST inc., all rights reserved.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "esp32xx.h"
#include "esp32xx_gpio.h"

#define C3_REG(Off) ESP32_REG32(0x60008000UL + (Off))
#define C3_ROOT ESP32_REG32(0x600C0058UL)
#define C3_GPIO_WAKE (1UL << 2)
#define C3_RESULT_MASK 3U
#define C3_XTAL_SEL_MASK ((3UL << 10) | 0x3FFU)
#define C3_PLL_PU ((1UL << 11) | (1UL << 9) | (1UL << 7))
#define C3_PLL_PD ((1UL << 10) | (1UL << 8) | (1UL << 6))
#define C3_RTC_TIMER1_MASK ((0xFFUL << 24) | (0x3FFUL << 14) | (0xFFUL << 6))
#define C3_RTC_TIMER1_DEFAULT ((20UL << 24) | (100UL << 14) | (20UL << 6))
#define C3_RTC_BIAS_MASK ((0xFUL << 18) | (1UL << 16) | (1UL << 14))
#define C3_DIG_PD_MASK ((1UL << 31) | (1UL << 30) | (1UL << 29) | (1UL << 28) | (1UL << 27))
#define C3_WAKE_FIELD (0x1FFFFUL << 15)
#define C3_REJECT_FIELD ((3UL << 30) | (0x3FFFFUL << 12))
#define C3_DBIAS_DEFAULT 23U

typedef struct {
	uint32_t Root;
	uint32_t Options;
	uint32_t Ana;
	uint32_t Dig;
	uint32_t Bias;
	uint32_t Regulator;
	uint32_t Clock;
	uint32_t Sdio;
	uint32_t Timer1;
	uint32_t Timer3;
	uint32_t Timer4;
	uint32_t Timer5;
	uint32_t Timer6;
	uint32_t Wake;
	uint32_t Reject;
	uint8_t RtcBias;
	uint8_t DigBias;
	uint32_t CpuMHz;
} Esp32C3SleepState_t;

bool Esp32LightWakePrepare(void);
void Esp32LightWakeRestore(void);
bool Esp32DeepWakePrepare(void);
void Esp32DeepWakeRestore(void);
uint8_t Esp32RomRegi2cReadMask(uint8_t Block, uint8_t Host, uint8_t Addr,
	uint8_t Msb, uint8_t Lsb);
void Esp32RomRegi2cWriteMask(uint8_t Block, uint8_t Host, uint8_t Addr,
	uint8_t Msb, uint8_t Lsb, uint8_t Data);
void Esp32RomClockUpdate(uint32_t CpuMHz);

static void Esp32C3Read(Esp32C3SleepState_t *pState)
{
	pState->Root = C3_ROOT;
	pState->Options = C3_REG(0x00U);
	pState->Ana = C3_REG(0x34U);
	pState->Dig = C3_REG(0x88U);
	pState->Bias = C3_REG(0x7CU);
	pState->Regulator = C3_REG(0x80U);
	pState->Clock = C3_REG(0x70U);
	pState->Sdio = C3_REG(0x78U);
	pState->Timer1 = C3_REG(0x1CU);
	pState->Timer3 = C3_REG(0x24U);
	pState->Timer4 = C3_REG(0x28U);
	pState->Timer5 = C3_REG(0x2CU);
	pState->Timer6 = C3_REG(0x30U);
	pState->Wake = C3_REG(0x3CU);
	pState->Reject = C3_REG(0x68U);
	pState->RtcBias = Esp32RomRegi2cReadMask(0x6DU, 0U, 5U, 4U, 0U);
	pState->DigBias = Esp32RomRegi2cReadMask(0x6DU, 0U, 7U, 4U, 0U);
	pState->CpuMHz = (SystemCoreClockGet() + 500000U) / 1000000U;
}

static bool Esp32C3Configure(bool bDeep)
{
	uint32_t options;
	uint32_t dig;

	options = C3_REG(0x00U);
	dig = C3_REG(0x88U);
	if (C3_REG(0x80U) & (1UL << 30))
	{
		return false;
	}

	C3_REG(0x1CU) = (C3_REG(0x1CU) & ~C3_RTC_TIMER1_MASK) |
		C3_RTC_TIMER1_DEFAULT;
	// RTC sleep has its own power-up and settle timers. The RTC ROM
	// path may not have initialized them for a direct-boot image.
	C3_REG(0x24U) = (1UL << 25) | (1UL << 16) | (1UL << 9) | 1UL;
	C3_REG(0x28U) = (1UL << 25) | (1UL << 16) | (1UL << 9) | 1UL;
	C3_REG(0x2CU) = (C3_REG(0x2CU) & ~(0xFFUL << 8)) | (2UL << 8);
	C3_REG(0x30U) = (1UL << 25) | (1UL << 16);
	// Keep the RTC regulator and RC_FAST available for GPIO sensing.
	C3_REG(0x80U) = (C3_REG(0x80U) | (1UL << 31)) & ~(1UL << 30);
	C3_REG(0x70U) |= (1UL << 26) | (1UL << 16);
	C3_REG(0x7CU) &= ~C3_RTC_BIAS_MASK;
	if (bDeep)
	{
		C3_REG(0x7CU) |= (1UL << 16) | (1UL << 14);
		dig = (dig & ~C3_DIG_PD_MASK) | C3_DIG_PD_MASK;
		C3_REG(0x34U) &= ~((1UL << 31) | (1UL << 30) |
			(1UL << 28) | (1UL << 27));
		options &= ~((1UL << 13) | C3_PLL_PU);
		C3_REG(0x70U) &= ~(1UL << 28);
		C3_REG(0x78U) = (C3_REG(0x78U) & ~(1UL << 22)) | (1UL << 21);
		// With RC_FAST running, Espressif uses the no-attenuation
		// RTC voltage for deep wake through the RTC GPIO input.
		Esp32RomRegi2cWriteMask(0x6DU, 0U, 5U, 4U, 0U, 0U);
		Esp32RomRegi2cWriteMask(0x6DU, 0U, 7U, 4U, 0U, 0U);
	}
	else
	{
		// XTAL and PLL remain powered. No reduced regulator voltage is
		// selected while either is needed by the running system.
		options = (options | (1UL << 13)) & ~(1UL << 12);
		if ((C3_ROOT & (3UL << 10)) == (1UL << 10))
		{
			options = (options | C3_PLL_PU) & ~C3_PLL_PD;
		}
		dig &= ~C3_DIG_PD_MASK;
		dig |= (1UL << 16) | (1UL << 4);
		C3_REG(0x70U) |= (1UL << 28);
		C3_REG(0x78U) &= ~((1UL << 22) | (1UL << 21));
		Esp32RomRegi2cWriteMask(0x6DU, 0U, 5U, 4U, 0U, C3_DBIAS_DEFAULT);
		Esp32RomRegi2cWriteMask(0x6DU, 0U, 7U, 4U, 0U, C3_DBIAS_DEFAULT);
	}
	C3_REG(0x00U) = options;
	C3_REG(0x88U) = dig;
	C3_REG(0x3CU) = (C3_REG(0x3CU) & ~C3_WAKE_FIELD) |
		(C3_GPIO_WAKE << 15);
	C3_REG(0x68U) = (C3_REG(0x68U) & ~C3_REJECT_FIELD) |
		(3UL << 30) | (C3_GPIO_WAKE << 12);
	return (C3_REG(0x3CU) & C3_WAKE_FIELD) == (C3_GPIO_WAKE << 15) &&
		(C3_REG(0x00U) & (1UL << 13)) == (bDeep ? 0U : (1UL << 13)) &&
		Esp32RomRegi2cReadMask(0x6DU, 0U, 5U, 4U, 0U) ==
			(bDeep ? 0U : C3_DBIAS_DEFAULT) &&
		Esp32RomRegi2cReadMask(0x6DU, 0U, 7U, 4U, 0U) ==
			(bDeep ? 0U : C3_DBIAS_DEFAULT);
}

static void Esp32C3Restore(const Esp32C3SleepState_t *pState)
{
	C3_REG(0x00U) = pState->Options;
	C3_REG(0x34U) = pState->Ana;
	C3_REG(0x88U) = pState->Dig;
	C3_REG(0x7CU) = pState->Bias;
	C3_REG(0x80U) = pState->Regulator;
	C3_REG(0x70U) = pState->Clock;
	C3_REG(0x78U) = pState->Sdio;
	C3_REG(0x1CU) = pState->Timer1;
	C3_REG(0x24U) = pState->Timer3;
	C3_REG(0x28U) = pState->Timer4;
	C3_REG(0x2CU) = pState->Timer5;
	C3_REG(0x30U) = pState->Timer6;
	C3_REG(0x3CU) = pState->Wake;
	C3_REG(0x68U) = pState->Reject;
	Esp32RomRegi2cWriteMask(0x6DU, 0U, 5U, 4U, 0U, pState->RtcBias);
	Esp32RomRegi2cWriteMask(0x6DU, 0U, 7U, 4U, 0U, pState->DigBias);
}

__attribute__((noinline, section(".iram.text.esp32_c3_sleep")))
static uint32_t Esp32C3Request(uint32_t Root, uint32_t CpuMHz, bool bDeep)
{
	uint32_t observed;

	C3_ROOT = Root & ~C3_XTAL_SEL_MASK;
	(void)C3_ROOT;
	Esp32RomClockUpdate(40U);
	C3_REG(0x4CU) = C3_RESULT_MASK;
	C3_REG(0x18U) |= 1UL << 31;
	__asm volatile("fence iorw, iorw" : : : "memory");
	observed = 0U;
	for (uint32_t count = 0U; count < 10000000U; count++)
	{
		observed = C3_REG(0x44U) & C3_RESULT_MASK;
		if (observed != 0U)
		{
			break;
		}
	}
	if (bDeep)
	{
		// Never reselect the pre-sleep PLL after a rejected deep sleep.
		// Wait in instruction RAM for a manual reset.
		for (;;)
		{
			__WFI();
		}
	}
	C3_REG(0x4CU) = C3_RESULT_MASK;
	C3_ROOT = Root;
	(void)C3_ROOT;
	Esp32RomClockUpdate(CpuMHz);
	__asm volatile("fence iorw, iorw" : : : "memory");
	return observed;
}

bool Esp32PowerSleep(void)
{
	Esp32C3SleepState_t state;
	uintptr_t mask;
	bool attempted;
	bool slept;

	mask = DisableInterrupt();
	if (!Esp32LightWakePrepare())
	{
		EnableInterrupt(mask);
		return false;
	}
	Esp32C3Read(&state);
	attempted = state.CpuMHz != 0U && Esp32C3Configure(false);
	slept = false;
	if (attempted)
	{
		slept = (Esp32C3Request(state.Root, state.CpuMHz, false) & 1U) != 0U;
	}
	Esp32C3Restore(&state);
	Esp32LightWakeRestore();
	EnableInterrupt(mask);
	return slept;
}

void Esp32PowerDeepSleep(void)
{
	Esp32C3SleepState_t state;
	uintptr_t mask;
	bool ready;

	mask = DisableInterrupt();
	ready = Esp32DeepWakePrepare();
	if (ready)
	{
		Esp32C3Read(&state);
		ready = state.CpuMHz != 0U && Esp32C3Configure(true);
		if (ready)
		{
			(void)Esp32C3Request(state.Root, state.CpuMHz, true);
		}
		Esp32C3Restore(&state);
		Esp32DeepWakeRestore();
	}
	// Deep-sleep normally resets the MCU. If the RTC rejects entry, do
	// not return to an application that believes it powered off.
	(void)mask;
	for (;;)
	{
		__WFI();
	}
}
