/**-------------------------------------------------------------------------
@file	system_lfclock_esp32.c

@brief	ESP32-C3/C6 low-frequency clock selection and calibration.

The TIMG0 calibration unit measures the selected oscillator against XTAL.
Timer counters and watchdog configuration are not changed. Call before using
RTC timekeeping, LP-core firmware or modem timing. The operation is synchronous
and masks interrupts for up to 121 ms. GPIO0/1 must be reserved for XTAL32K.
No oscillator is selected automatically during system startup.

@license MIT
Copyright (c) 2026, I-SYST inc., all rights reserved.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "esp32xx_gpio.h"
#include "idelay.h"

#if defined(ESP32C3)
#define LF_TIMG_BASE 0x6001F000UL
#define LF_CLOCK_REG ESP32_REG32(0x60008070UL)
#define LF_XTAL_REG ESP32_REG32(0x60008060UL)
#define LF_CAL_PERIOD_REG ESP32_REG32(0x60008054UL)
#define LF_BUS_REG ESP32_REG32(0x600C0010UL)
#define LF_BUS_ENABLE (1UL << 13)
#define LF_SOURCE_SHIFT 30U
#define LF_CAL_RESET_CONFIG 0x00013000UL
#elif defined(ESP32C6)
#define LF_TIMG_BASE 0x60008000UL
#define LF_CLOCK_REG ESP32_REG32(0x600B0400UL)
#define LF_XTAL_REG ESP32_REG32(0x600B042CUL)
#define LF_CAL_PERIOD_REG ESP32_REG32(0x600B1004UL)
#define LF_BUS_REG ESP32_REG32(0x6009603CUL)
#define LF_BUS_ENABLE 1UL
#define LF_SOURCE_SHIFT 0U
#define LF_CAL_RESET_CONFIG 0x00011000UL
#define LF_POWER_REG ESP32_REG32(0x600B00ACUL)
#define LF_DIGITAL_REG ESP32_REG32(0x600B0420UL)
#define LF_CAL_SOURCE_REG ESP32_REG32(0x60096134UL)
#else
#error "Low-frequency clock selection is implemented for ESP32-C3/C6"
#endif

#define LF_CAL_CONFIG_REG ESP32_REG32(LF_TIMG_BASE + 0x68U)
#define LF_CAL_VALUE_REG ESP32_REG32(LF_TIMG_BASE + 0x6CU)
#define LF_CAL_TIMEOUT_REG ESP32_REG32(LF_TIMG_BASE + 0x80U)
#define LF_CAL_START (1UL << 31)
#define LF_CAL_CYCLING (1UL << 12)
#define LF_CAL_READY (1UL << 15)
#define LF_CAL_CYCLES 64U
#define LF_RC_SLOW_FREQ 136000U
#define LF_32K_FREQ 32768U

static bool Esp32LowFreqBusy(void)
{
#if defined(ESP32C3)
	// RTC watchdog, wake sources, RTC alarm interrupt and a pending sleep.
	return (ESP32_REG32(0x60008090UL) & ((1UL << 31) | (1UL << 12))) != 0U ||
		(ESP32_REG32(0x6000803CUL) >> 15) != 0U ||
		(ESP32_REG32(0x60008040UL) & (1UL << 10)) != 0U ||
		(ESP32_REG32(0x60008018UL) & (1UL << 31)) != 0U ||
		(ESP32_REG32(0x60008074UL) & (0xFFUL << 23)) != 0U;
#else
	// LP watchdog, HP/LP timer interrupts, HP wake and LP CPU wake sources.
	return (ESP32_REG32(0x600B1C00UL) & ((1UL << 31) | (1UL << 12))) != 0U ||
		ESP32_REG32(0x600B0128UL) != 0U ||
		(ESP32_REG32(0x600B0C30UL) & (3UL << 30)) != 0U ||
		(ESP32_REG32(0x600B0C40UL) & (3UL << 30)) != 0U ||
		(ESP32_REG32(0x600B0180UL) & 0xFFFFU) != 0U ||
		(ESP32_REG32(0x600AF018UL) & 0xBU) != 0U ||
		(ESP32_REG32(0x600AF01CUL) & 0xBU) != 0U ||
		(ESP32_REG32(0x600AF020UL) & 0x00FF0000U) != 0U;
#endif
}

static bool Esp32LowFreqPinsFree(void)
{
	if ((ESP32_GPIO_ENABLE_REG32 & 3U) != 0U ||
		(ESP32_GPIO_PIN_REG32(0) & ESP32_GPIO_PIN_INT_TYPE_Msk) != 0U ||
		(ESP32_GPIO_PIN_REG32(1) & ESP32_GPIO_PIN_INT_TYPE_Msk) != 0U)
	{
		return false;
	}
#if defined(ESP32C6)
	if ((ESP32_LP_AON_GPIO_MUX_REG & 3U) != 0U)
	{
		return false;
	}
#endif
	for (unsigned signal = 0; signal < ESP32_GPIO_FUNC_IN_COUNT; signal++)
	{
		uint32_t input = ESP32_GPIO_FUNC_IN_REG32(signal);

		if ((input & ESP32_GPIO_SIG_IN_SEL_Msk) != 0U &&
			(input & ESP32_GPIO_FUNC_IN_SEL_Msk) < 2U)
		{
			return false;
		}
	}
	return true;
}

static bool Esp32LowFreqCalibrationPause(uint32_t Config, uint32_t Timeout)
{
	if (Config & LF_CAL_START)
	{
		return false;
	}
	if (!(Config & LF_CAL_CYCLING))
	{
		return true;
	}
	// Only the reset continuous calibration may be borrowed at initialization.
	if ((Config & ~LF_CAL_READY) != LF_CAL_RESET_CONFIG)
	{
		return false;
	}
	LF_CAL_TIMEOUT_REG = (Timeout & 0x7EU) | (1UL << 7);
	for (unsigned retry = 0; retry < 1000U; retry++)
	{
		if ((LF_CAL_CONFIG_REG & LF_CAL_READY) || (LF_CAL_TIMEOUT_REG & 1U))
		{
			LF_CAL_CONFIG_REG = Config & ~LF_CAL_CYCLING;
			LF_CAL_TIMEOUT_REG = Timeout;
			return true;
		}
		usDelay(10);
	}
	LF_CAL_TIMEOUT_REG = Timeout;
	return false;
}

static uint32_t Esp32LowFreqMeasure(uint32_t Source)
{
	uint32_t config = LF_CAL_CONFIG_REG;
	uint32_t timeout = LF_CAL_TIMEOUT_REG;
	uint32_t value = 0;
	uint32_t selection = Source == 0U ? 0U : 2U;

	// The calibration unit is idle before changing its source.
	LF_CAL_CONFIG_REG = (config & 0xFFFU) | (selection << 13) |
		(LF_CAL_CYCLES << 16);
	LF_CAL_TIMEOUT_REG = (timeout & 0x7EU) |
		((LF_CAL_CYCLES << (Source == 0U ? 10 : 12)) << 7);
	LF_CAL_CONFIG_REG |= LF_CAL_START;
	for (unsigned retry = 0; retry < 1000U; retry++)
	{
		if (LF_CAL_CONFIG_REG & LF_CAL_READY)
		{
			value = LF_CAL_VALUE_REG >> 7;
			break;
		}
		if (LF_CAL_TIMEOUT_REG & 1U)
		{
			break;
		}
		usDelay(10);
	}
	LF_CAL_CONFIG_REG &= ~LF_CAL_START;
	LF_CAL_TIMEOUT_REG = timeout;
	LF_CAL_CONFIG_REG = config;
	return value;
}

#if defined(ESP32C6)
// Measure RC_FAST for PMU transition timing. The caller masks interrupts.
// Restore TIMG0 and PCR state; timer counters and watchdogs are untouched.
uint32_t Esp32C6FastClockMeasure(void)
{
	uint32_t bus = LF_BUS_REG;
	uint32_t config;
	uint32_t timeout;
	uint32_t digital;
	uint32_t tick;
	uint32_t count = 0U;
	uint32_t divider;

	if ((bus & 2U) != 0U)
	{
		return 0U;
	}
	LF_BUS_REG = bus | LF_BUS_ENABLE;
	config = LF_CAL_CONFIG_REG;
	timeout = LF_CAL_TIMEOUT_REG;
	if (!Esp32LowFreqCalibrationPause(config, timeout))
	{
		LF_BUS_REG = bus;
		return 0U;
	}
	digital = LF_DIGITAL_REG;
	tick = ESP32_REG32(0x60096130UL);
	LF_DIGITAL_REG = digital | (1UL << 31);
	ESP32_REG32(0x60096130UL) = tick | (255UL << 8) | (1UL << 16);
	// ECO1 and later divide the calibration input by 32.
	divider = (ESP32_REG32(0x600B0850UL) & (63UL << 18)) != 0U ? 32U : 1U;
	LF_CAL_CONFIG_REG = (config & 0xFFFU) | (1UL << 13) | (256UL << 16);
	LF_CAL_TIMEOUT_REG = (timeout & 0x7EU) | (0x100000UL << 7);
	LF_CAL_CONFIG_REG |= LF_CAL_START;
	for (unsigned retry = 0U; retry < 1000U; retry++)
	{
		if ((LF_CAL_CONFIG_REG & LF_CAL_READY) != 0U)
		{
			count = LF_CAL_VALUE_REG >> 7;
			break;
		}
		if ((LF_CAL_TIMEOUT_REG & 1U) != 0U)
		{
			break;
		}
		usDelay(10);
	}
	LF_CAL_CONFIG_REG &= ~LF_CAL_START;
	LF_CAL_TIMEOUT_REG = timeout;
	LF_CAL_CONFIG_REG = config;
	ESP32_REG32(0x60096130UL) = tick;
	LF_DIGITAL_REG = digital;
	LF_BUS_REG = bus;
	return count != 0U ? (40000000ULL * 256U * divider + count / 2U) / count : 0U;
}
#endif

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	uint32_t source = 0;
	uint32_t frequency;
	uint32_t count;
	uint32_t clock;
	uint32_t crystal;
	uint32_t bus;
	uint32_t config;
	uint32_t timeout;
	uintptr_t state;
	bool ready;
#if defined(ESP32C6)
	uint32_t power;
	uint32_t digital;
	uint32_t calibration;
#endif

	if (ClkSrc == OSC_TYPE_XTAL)
	{
		if (OscFreq != 0U && OscFreq != LF_32K_FREQ)
		{
			return false;
		}
		source = 1U;
	}
	else if (ClkSrc == OSC_TYPE_RC)
	{
#if defined(ESP32C6)
		// Zero requests the reset RC_SLOW source. Other values choose nearest.
		if (OscFreq != 0U && OscFreq <= (LF_RC_SLOW_FREQ + LF_32K_FREQ) / 2U)
		{
			source = 2U;
		}
#endif
	}
	else
	{
		return false;
	}
	// These targets use a 40 MHz crystal as the calibration reference.
	if (SystemCoreClockGet() < 1000000U)
	{
		return false;
	}
	state = DisableInterrupt();
	if (Esp32LowFreqBusy() || (source == 1U && !Esp32LowFreqPinsFree()))
	{
		EnableInterrupt(state);
		return false;
	}
	bus = LF_BUS_REG;
#if defined(ESP32C3)
	if (ESP32_REG32(0x600C0018UL) & LF_BUS_ENABLE)
#else
	if (bus & 2U)
#endif
	{
		EnableInterrupt(state);
		return false;
	}
	LF_BUS_REG = bus | LF_BUS_ENABLE;
	config = LF_CAL_CONFIG_REG;
	timeout = LF_CAL_TIMEOUT_REG;
	if (!Esp32LowFreqCalibrationPause(config, timeout))
	{
		LF_BUS_REG = bus;
		EnableInterrupt(state);
		return false;
	}
	clock = LF_CLOCK_REG;
	crystal = LF_XTAL_REG;
#if defined(ESP32C3)
	if (source == 1U)
	{
		// XTAL32K DAC=3, DRES=3, DGM=3, DBUF=1; preserve other fields.
		if (!(crystal & (1UL << 16)) || (crystal & (1UL << 23)))
		{
			LF_XTAL_REG = (crystal & ~((1UL << 23) | (0x3FUL << 17) |
				(7UL << 13) | (7UL << 10))) | (3UL << 17) | (1UL << 16) |
				(3UL << 13) | (3UL << 10) | (1UL << 9);
		}
		LF_CLOCK_REG = clock | (1UL << 8);
		if (!(crystal & (1UL << 16)) || (crystal & (1UL << 23)))
		{
			usDelay(100000);
		}
	}
#else
	power = LF_POWER_REG;
	digital = LF_DIGITAL_REG;
	calibration = LF_CAL_SOURCE_REG;
	if (source == 1U)
	{
		if (!(power & (1UL << 28)))
		{
			LF_XTAL_REG = (crystal & 0x003FFFFFU) | (3UL << 22) |
				(3UL << 25) | (1UL << 28) | (3UL << 29);
		}
		LF_POWER_REG = power | (1UL << 28);
		LF_DIGITAL_REG = digital | (1UL << 28);
		LF_CAL_SOURCE_REG = (calibration & ~3U) | 1U;
		if (!(power & (1UL << 28)))
		{
			usDelay(100000);
		}
	}
	else if (source == 2U)
	{
		LF_POWER_REG = power | (1UL << 29);
		LF_DIGITAL_REG = digital | (1UL << 30);
		LF_CAL_SOURCE_REG = calibration & ~3U;
		if (!(power & (1UL << 29)))
		{
			usDelay(300);
		}
	}
#endif
	count = Esp32LowFreqMeasure(source);
	frequency = count != 0U ? (40000000ULL * LF_CAL_CYCLES + count / 2U) / count : 0U;
	ready = source == 0U ? frequency >= 50000U && frequency <= 300000U :
		source == 1U ? frequency >= 32735U && frequency <= 32801U :
		frequency >= 16000U && frequency <= 64000U;
#if defined(ESP32C6)
	LF_CAL_SOURCE_REG = calibration;
	LF_DIGITAL_REG = digital;
#endif
	if (ready)
	{
#if defined(ESP32C3)
		if (source == 1U)
		{
			clock |= 1UL << 8;
		}
#endif
		LF_CLOCK_REG = (clock & ~(3UL << LF_SOURCE_SHIFT)) | (source << LF_SOURCE_SHIFT);
		usDelay(300);
		LF_CAL_PERIOD_REG = ((uint64_t)count * (1UL << 19) + 20U * LF_CAL_CYCLES) /
			(40U * LF_CAL_CYCLES);
		g_McuOsc.LowPwrOsc.Type = ClkSrc;
		g_McuOsc.LowPwrOsc.Freq = source == 1U ? LF_32K_FREQ : frequency;
	}
	else
	{
		LF_CLOCK_REG = clock;
		LF_XTAL_REG = crystal;
#if defined(ESP32C6)
		LF_POWER_REG = power;
#endif
	}
	LF_CAL_CONFIG_REG = config;
	LF_BUS_REG = bus;
	EnableInterrupt(state);
	return ready;
}

void SystemOscInit(void)
{
	(void)SystemLowFreqClockSelect(g_McuOsc.LowPwrOsc.Type, g_McuOsc.LowPwrOsc.Freq);
}
