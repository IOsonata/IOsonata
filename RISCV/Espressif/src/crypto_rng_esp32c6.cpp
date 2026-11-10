/**-------------------------------------------------------------------------
@file	crypto_rng_esp32c6.cpp

@brief	ESP32-C6 SAR ADC hardware entropy source for Crypto::Random().

The SAR ADC is enabled only during RNG draws and is restored afterward.

@author	IOsonata contributors
@date	Nov. 10, 2026

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
FITNESS FOR A PARTICUAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp32xx.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "crypto/icrypto.h"
#ifndef ESP32_RNG_TEST
#include "idelay.h"
#endif

#if !defined(ESP32C6)
#error ESP32-C6 RNG implementation requires ESP32C6
#endif

#ifndef ESP32_RNG_REG
#define ESP32_RNG_REG(Addr) (*(volatile uint32_t *)(uintptr_t)(Addr))
#endif
#ifndef ESP32_RNG_WAIT_MAX_POLLS
#define ESP32_RNG_WAIT_MAX_POLLS 100000U
#endif

// ESP-IDF v5.5.1 bootloader_random_esp32c6.c and per-chip register headers.
#define RNG_ADC_CTRL		(ESP32_SAR_ADC_BASE + 0x00U)
#define RNG_ADC_CTRL2		(ESP32_SAR_ADC_BASE + 0x04U)
#define RNG_ADC_PATTERN		(ESP32_SAR_ADC_BASE + 0x18U)
#define RNG_ADC_ONETIME		(ESP32_SAR_ADC_BASE + 0x20U)
#define RNG_ADC_DMA		(ESP32_SAR_ADC_BASE + 0x50U)
#define RNG_ADC_CLOCK		(ESP32_PCR_BASE + 0x80U)
#define RNG_ADC_CLKM		(ESP32_PCR_BASE + 0x84U)
#define RNG_ANALOG_POWER		(ESP32_PMU_BASE + 0x154U)
#define RNG_ANALOG_CLOCK		0x600AF018U
#define RNG_ANALOG_CLOCK_MAP		0x600AF020U
#define RNG_ANALOG_CLOCK_MAP_MASK	0xFF000000U
#define RNG_ANALOG_CTRL0		0x600AF800U
#define RNG_ANALOG_READ		0x600AF81CU
#define RNG_ANALOG_SELECT		0x600AF820U
#define RNG_CLOCK		0x600B2800U
#define RNG_DATA		0x600B2808U
#define RNG_TIMER_EN		(1U << 24)
#define RNG_ANALOG_BUSY		(1U << 25)
#define RNG_ANALOG_POWER_MASK		(3U << 26)

// First failure is retained for the board test, with ordinary C++ linkage.
// 1 ADC in use, 2 analog bus timeout, 3 analog selection in use,
// 4 analog readback, 5 ADC trigger, 6 cycle counter, 7 repeated output.
volatile uint32_t g_Esp32C6RngError;
volatile uint32_t g_Esp32C6RngDetail;

// Read-only snapshots: entry, setup complete, first failure before cleanup.
// Register labels are listed in the C6 CryptoTest board report.
volatile uint32_t g_Esp32C6RngTrace[3][13];
volatile uint32_t g_Esp32C6RngTransaction[4];
static uint32_t s_AnalogPhase;
static uint32_t s_AnalogAddress;
static uint32_t s_AnalogCommand;
static uint32_t s_AnalogCount;

static void EspC6RngCapture(unsigned Slot)
{
	static const uintptr_t addresses[] = {
		RNG_ANALOG_CTRL0, RNG_ANALOG_CTRL0 + 4U,
		RNG_ANALOG_CTRL0 + 8U, RNG_ANALOG_CTRL0 + 12U,
		RNG_ANALOG_SELECT, RNG_ANALOG_READ,
		0x600AF010U, RNG_ANALOG_CLOCK, 0x600AF01CU,
		RNG_ANALOG_CLOCK_MAP, 0x600AF024U,
		RNG_ANALOG_POWER, RNG_ADC_CLOCK
	};
	unsigned i;

	for (i = 0U; i < sizeof(addresses) / sizeof(addresses[0]); i++)
	{
		g_Esp32C6RngTrace[Slot][i] = ESP32_RNG_REG(addresses[i]);
	}
}

static bool EspC6RngFail(uint32_t Error, uint32_t Detail)
{
	if (g_Esp32C6RngError == 0U)
	{
		g_Esp32C6RngError = Error;
		g_Esp32C6RngDetail = Detail;
		g_Esp32C6RngTransaction[0] = s_AnalogPhase;
		g_Esp32C6RngTransaction[1] = s_AnalogAddress;
		g_Esp32C6RngTransaction[2] = s_AnalogCommand;
		g_Esp32C6RngTransaction[3] = s_AnalogCount;
		EspC6RngCapture(2U);
	}
	return false;
}

static bool s_RngBusy;
static bool s_RngFault;
static uint8_t s_LastByte;
static unsigned s_RepeatedBytes;

// Save only registers this driver changes. Shared clock/power fields are masked
// during restoration so another peripheral's unrelated changes are preserved.
typedef struct {
	uint32_t Ctrl;
	uint32_t Ctrl2;
	uint32_t Pattern;
	uint32_t Clock;
	uint32_t Clkm;
	uint32_t Power;
	uint32_t AnalogClock;
	uint32_t AnalogClockMap;
	uint32_t RngClock;
	uint8_t Analog[5];
	unsigned ChangedAnalog;
} EspC6RngState_t;

static const uint8_t s_AnalogRegisters[] = { 0U, 1U, 3U, 4U, 7U };
static const uint8_t s_AnalogMasks[] = { 0xFFU, 0x0FU, 0xFFU, 0x0FU, 0x5BU };
static const uint8_t s_AnalogValues[] = { 0x66U, 0x08U, 0x66U, 0x08U, 0x5AU };

static void EspC6RngModify(uintptr_t Addr, uint32_t Mask, uint32_t Value)
{
	ESP32_RNG_REG(Addr) = (ESP32_RNG_REG(Addr) & ~Mask) | (Value & Mask);
}

static bool EspC6RngWaitIdle(uintptr_t Addr)
{
	unsigned i;

	for (i = 0U; i < ESP32_RNG_WAIT_MAX_POLLS; i++)
	{
		if ((ESP32_RNG_REG(Addr) & RNG_ANALOG_BUSY) == 0U)
		{
			return true;
		}
	}
	return EspC6RngFail(2U, (uint32_t)Addr);
}

// Internal analog I2C, SAR block 0x69. Select the master assigned by ROM.
// Keep selection and each read/write together while interrupts are masked.
// Unlike the ROM patch's spin loops, both waits have a finite bound.
static bool EspC6RngAnalog(uint8_t Reg, uint8_t *pValue, bool Write)
{
	uintptr_t state;
	uintptr_t ctrl;
	uint32_t selection;
	uint32_t command;
	bool ok;

	state = DisableInterrupt();
	ctrl = RNG_ANALOG_CTRL0 +
		((ESP32_RNG_REG(RNG_ANALOG_SELECT) & (1U << 11)) != 0U ? 0U : 4U);
	selection = ESP32_RNG_REG(RNG_ANALOG_READ) & 0x00FFFFFFU;
	ESP32_RNG_REG(RNG_ANALOG_READ) = 0x00FFFDFFU;
	command = 0x69U | ((uint32_t)Reg << 8);
	if (Write)
	{
		command |= (1U << 24) | ((uint32_t)*pValue << 16);
	}
	s_AnalogPhase = 1U;
	s_AnalogAddress = (uint32_t)ctrl;
	s_AnalogCommand = command;
	s_AnalogCount++;
	ok = EspC6RngWaitIdle(ctrl);
	if (ok)
	{
		ESP32_RNG_REG(ctrl) = command;
		s_AnalogPhase = 2U;
		ok = EspC6RngWaitIdle(ctrl);
		if (ok && !Write)
		{
			*pValue = (uint8_t)(ESP32_RNG_REG(ctrl) >> 16);
		}
	}
	if (ok)
	{
		s_AnalogPhase = 3U;
	}
	ESP32_RNG_REG(RNG_ANALOG_READ) = selection;
	EnableInterrupt(state);
	return ok;
}

static bool EspC6RngIdle(void)
{
	return (ESP32_RNG_REG(RNG_ADC_CTRL2) & RNG_TIMER_EN) == 0U &&
		(ESP32_RNG_REG(RNG_ADC_DMA) & (1U << 31)) == 0U &&
		(ESP32_RNG_REG(RNG_ADC_ONETIME) & (1U << 29)) == 0U &&
		(ESP32_RNG_REG(RNG_ADC_CTRL) & ((7U << 27) | 3U)) == 0U;
}

static bool EspC6RngStart(EspC6RngState_t *pSaved)
{
	uint8_t value;
	uint8_t readback;
	unsigned i;

	pSaved->ChangedAnalog = 0U;
	pSaved->Ctrl = ESP32_RNG_REG(RNG_ADC_CTRL);
	pSaved->Ctrl2 = ESP32_RNG_REG(RNG_ADC_CTRL2);
	pSaved->Pattern = ESP32_RNG_REG(RNG_ADC_PATTERN);
	pSaved->Clock = ESP32_RNG_REG(RNG_ADC_CLOCK);
	pSaved->Clkm = ESP32_RNG_REG(RNG_ADC_CLKM);
	pSaved->Power = ESP32_RNG_REG(RNG_ANALOG_POWER);
	pSaved->AnalogClock = ESP32_RNG_REG(RNG_ANALOG_CLOCK);
	pSaved->AnalogClockMap = ESP32_RNG_REG(RNG_ANALOG_CLOCK_MAP);
	pSaved->RngClock = ESP32_RNG_REG(RNG_CLOCK);

	EspC6RngModify(RNG_ADC_CLOCK, 0xEU, 4U);
	EspC6RngModify(RNG_ADC_CLKM, 0x7FFFFFU, 1U << 22);
	// Match the PMU sequence: hold reset, power the analog block, release reset.
	// An already powered block may also contain temperature sensor state.
	if ((pSaved->Power & (1U << 27)) == 0U)
	{
		EspC6RngModify(RNG_ANALOG_POWER, 1U << 26, 0U);
		EspC6RngModify(RNG_ANALOG_POWER, 1U << 27, 1U << 27);
	}
	EspC6RngModify(RNG_ANALOG_POWER, 1U << 26, 1U << 26);
	// ESP-IDF configures these domain maps during rtc_clk_init(). Bare-metal
	// startup does not. Permit I2C master and LP APB clocks for each PMU code
	// while drawing, without changing the PMU code or other modem domains.
	EspC6RngModify(RNG_ANALOG_CLOCK_MAP, RNG_ANALOG_CLOCK_MAP_MASK,
		RNG_ANALOG_CLOCK_MAP_MASK);
	EspC6RngModify(RNG_ANALOG_CLOCK, 4U, 4U);
	EspC6RngModify(RNG_CLOCK, 1U << 24, 1U << 24);
	EspC6RngCapture(1U);
	for (i = 0U; i < sizeof(s_AnalogRegisters); i++)
	{
		if (!EspC6RngAnalog(s_AnalogRegisters[i], &pSaved->Analog[i], false))
		{
			return false;
		}
	}
	// Do not replace an existing analog test/calibration/entropy selection.
	if ((pSaved->Analog[4] & 0x5BU) != 0U)
	{
		return EspC6RngFail(3U, pSaved->Analog[4]);
	}
	for (i = 0U; i < sizeof(s_AnalogRegisters); i++)
	{
		pSaved->ChangedAnalog = i + 1U;
		value = (pSaved->Analog[i] & ~s_AnalogMasks[i]) | s_AnalogValues[i];
		if (!EspC6RngAnalog(s_AnalogRegisters[i], &value, true) ||
			!EspC6RngAnalog(s_AnalogRegisters[i], &readback, false))
		{
			return false;
		}
		if (readback != value)
		{
			return EspC6RngFail(4U, ((uint32_t)s_AnalogRegisters[i] << 8) | readback);
		}
	}
	// Reserved ADC2/channel 1, attenuation 1, two identical pattern entries.
	// No external GPIO is sampled. The ADC FSM controls analog power.
	EspC6RngModify(RNG_ADC_CTRL, (0x7FFU << 7) | (1U << 6),
		(15U << 7) | (1U << 15) | (1U << 6));
	EspC6RngModify(RNG_ADC_PATTERN, 0x00FFF000U, (0x25U << 18) | (0x25U << 12));
	EspC6RngModify(RNG_ADC_CTRL2, (0xFFFU << 12) | 1U, 200U << 12);
	ESP32_RNG_REG(RNG_ADC_CTRL) |= 1U << 23;
	ESP32_RNG_REG(RNG_ADC_CTRL) &= ~(1U << 23);
	ESP32_RNG_REG(RNG_ADC_CTRL2) |= RNG_TIMER_EN;
	return (ESP32_RNG_REG(RNG_ADC_CTRL2) & RNG_TIMER_EN) != 0U ||
		EspC6RngFail(5U, ESP32_RNG_REG(RNG_ADC_CTRL2));
}

static bool EspC6RngStop(EspC6RngState_t *pSaved)
{
	uint8_t value;
	uint8_t readback;
	bool ok = true;
	unsigned i;

	ESP32_RNG_REG(RNG_ADC_CTRL2) &= ~RNG_TIMER_EN;
	for (i = pSaved->ChangedAnalog; i > 0U; i--)
	{
		if (!EspC6RngAnalog(s_AnalogRegisters[i - 1U], &value, false))
		{
			ok = false;
			continue;
		}
		value = (value & ~s_AnalogMasks[i - 1U]) |
			(pSaved->Analog[i - 1U] & s_AnalogMasks[i - 1U]);
		if (!EspC6RngAnalog(s_AnalogRegisters[i - 1U], &value, true) ||
			!EspC6RngAnalog(s_AnalogRegisters[i - 1U], &readback, false))
		{
			ok = false;
		}
		else if (readback != value)
		{
			ok = EspC6RngFail(4U, ((uint32_t)s_AnalogRegisters[i - 1U] << 8) | readback);
		}
	}
	ESP32_RNG_REG(RNG_ADC_PATTERN) = pSaved->Pattern;
	ESP32_RNG_REG(RNG_ADC_CTRL) = pSaved->Ctrl;
	ESP32_RNG_REG(RNG_ADC_CTRL2) = pSaved->Ctrl2;
	EspC6RngModify(RNG_ADC_CLKM, 0x7FFFFFU, pSaved->Clkm);
	EspC6RngModify(RNG_ADC_CLOCK, 0xEU, pSaved->Clock);
	EspC6RngModify(RNG_ANALOG_POWER, RNG_ANALOG_POWER_MASK, pSaved->Power);
	EspC6RngModify(RNG_ANALOG_CLOCK, 4U, pSaved->AnalogClock);
	EspC6RngModify(RNG_ANALOG_CLOCK_MAP, RNG_ANALOG_CLOCK_MAP_MASK, pSaved->AnalogClockMap);
	EspC6RngModify(RNG_CLOCK, 1U << 24, pSaved->RngClock);
	return ok;
}

#ifdef ESP32_RNG_TEST
extern uint32_t EspC6RngTestCycles(void);
#define EspC6RngCycles EspC6RngTestCycles
#else
#define EspC6RngCycles IOsonataRiscvCycle32
#endif

static bool EspC6RngWait(uint32_t Cycles)
{
	uint32_t start;
	unsigned i;

	start = EspC6RngCycles();
	for (i = 0U; i < ESP32_RNG_WAIT_MAX_POLLS; i++)
	{
		if ((uint32_t)(EspC6RngCycles() - start) >= Cycles)
		{
			return true;
		}
	}
	return EspC6RngFail(6U, Cycles);
}

bool Esp32C6RngSecureAvailable(void)
{
	return !s_RngFault && !s_RngBusy && EspC6RngIdle();
}

CRYPTO_STATUS Esp32C6RngFill(uint8_t *pOut, size_t Len)
{
	EspC6RngState_t saved;
	uintptr_t state;
	uint32_t cycles;
	uint8_t value;
	bool ok;
	size_t i;

	if (Len == 0U)
	{
		return CRYPTO_STATUS_OK;
	}
	if (pOut == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}
	state = DisableInterrupt();
	if (s_RngBusy)
	{
		EnableInterrupt(state);
		memset(pOut, 0, Len);
		return CRYPTO_STATUS_BUSY;
	}
	if (!s_RngFault)
	{
		g_Esp32C6RngError = 0U;
		g_Esp32C6RngDetail = 0U;
		s_AnalogPhase = 0U;
		s_AnalogAddress = 0U;
		s_AnalogCommand = 0U;
		s_AnalogCount = 0U;
		EspC6RngCapture(0U);
	}
	if (s_RngFault || !EspC6RngIdle())
	{
		(void)EspC6RngFail(1U, ESP32_RNG_REG(RNG_ADC_CTRL));
		EnableInterrupt(state);
		memset(pOut, 0, Len);
		return CRYPTO_STATUS_FAIL;
	}
	s_RngBusy = true;
	EnableInterrupt(state);
	ok = EspC6RngStart(&saved);
	cycles = SystemCoreClockGet() / 1000000U;
	if (ok && (cycles == 0U || cycles > 1000U))
	{
		ok = EspC6RngFail(6U, cycles);
	}
	if (ok)
	{
		ok = EspC6RngWait(cycles * 100U);
	}
	for (i = 0U; ok && i < Len; i++)
	{
		// C6 is harvested one byte at a time. 80 us is slower than the
		// 16 us byte pacing used by ESP-IDF, leaving time for SAR entropy.
		ok = EspC6RngWait(cycles * 80U);
		if (!ok)
		{
			break;
		}
		value = (uint8_t)ESP32_RNG_REG(RNG_DATA);
		s_RepeatedBytes = value == s_LastByte ? s_RepeatedBytes + 1U : 1U;
		s_LastByte = value;
		if (s_RepeatedBytes >= 32U)
		{
			ok = EspC6RngFail(7U, value);
			break;
		}
		pOut[i] = value;
	}
	if (!EspC6RngStop(&saved))
	{
		ok = false;
	}
	state = DisableInterrupt();
	s_RngFault = !ok;
	s_RngBusy = false;
	EnableInterrupt(state);
	if (!ok)
	{
		memset(pOut, 0, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}
