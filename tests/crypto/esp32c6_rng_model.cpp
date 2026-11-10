/**-------------------------------------------------------------------------
@file	esp32c6_rng_model.cpp

@brief	ESP32-C6 RNG register-model tests.

Deterministic modeled data is not cryptographic entropy.

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
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <map>
#include "esp32c6_rng_model.h"
#include "../../RISCV/Espressif/src/crypto_rng_esp32c6.cpp"

static std::map<uintptr_t, uint32_t> s_Values;
static std::map<uintptr_t, EspRngModelReg> s_Registers;
static uint8_t s_Analog[8] = { 0x11U, 0xC2U, 0U, 0x33U, 0xB4U, 0U, 0U, 0xA0U };
static uint32_t s_Cycles;
static uint32_t s_LastReadCycle;
static unsigned s_Reads;
static unsigned s_Writes;
static bool s_StallClock;
static bool s_StallBus;
static bool s_IgnoreAnalog;
static bool s_Stuck;
static bool s_Nested;
static bool s_RestoreFail;
static bool s_BadPowerSequence;
static unsigned s_PowerUps;
static uint32_t s_Data = 0xA5C39D71U;

EspRngModelReg &EspRngModelRegister(uintptr_t Addr)
{
	s_Registers[Addr].Addr = Addr;
	return s_Registers[Addr];
}

EspRngModelReg::operator uint32_t() const
{
	if (Addr == RNG_ANALOG_CTRL0 || Addr == RNG_ANALOG_CTRL0 + 4U)
	{
		if (s_StallBus)
		{
			return RNG_ANALOG_BUSY;
		}
	}
	if (Addr == RNG_DATA)
	{
		assert((s_Values[RNG_ADC_CTRL2] & RNG_TIMER_EN) != 0U);
		assert((s_Values[RNG_ADC_CLKM] & 0x7FFFFFU) == (1U << 22));
		assert((s_Values[RNG_CLOCK] & (1U << 24)) != 0U);
		assert((s_Values[RNG_ADC_CTRL] & (0x7FFU << 7)) == ((15U << 7) | (1U << 15)));
		assert((s_Values[RNG_ADC_PATTERN] & 0xFFF000U) == ((0x25U << 18) | (0x25U << 12)));
		assert((s_Analog[7] & 0x5BU) == 0x5AU);
		assert(s_Analog[0] == 0x66U && (s_Analog[1] & 15U) == 8U);
		assert(s_Analog[3] == 0x66U && (s_Analog[4] & 15U) == 8U);
		assert((uint32_t)(s_Cycles - s_LastReadCycle) >= 6400U);
		s_LastReadCycle = s_Cycles;
		s_Reads++;
		if (!s_Stuck)
		{
			s_Data ^= s_Data << 13;
			s_Data ^= s_Data >> 17;
			s_Data ^= s_Data << 5;
		}
		return s_Data;
	}
	return s_Values[Addr];
}

EspRngModelReg &EspRngModelReg::operator=(uint32_t Value)
{
	uint8_t reg;

	s_Writes++;
	if (Addr == RNG_ANALOG_POWER && (s_Values[Addr] & (1U << 27)) == 0U &&
		(Value & (1U << 27)) != 0U)
	{
		s_BadPowerSequence = (s_Values[Addr] & (1U << 26)) != 0U ||
			(Value & (1U << 26)) != 0U;
		s_PowerUps++;
	}
	if (Addr == RNG_ANALOG_CTRL0 || Addr == RNG_ANALOG_CTRL0 + 4U)
	{
		assert((s_Values[RNG_ANALOG_CLOCK] & 4U) != 0U);
		assert((s_Values[RNG_ANALOG_POWER] & RNG_ANALOG_POWER_MASK) == RNG_ANALOG_POWER_MASK);
		assert(s_Values[RNG_ANALOG_READ] == 0xFFFDFFU);
		assert((Value & 255U) == 0x69U);
		reg = (Value >> 8) & 255U;
		assert(reg < sizeof(s_Analog));
		if ((Value & (1U << 24)) != 0U && !s_IgnoreAnalog && !s_BadPowerSequence &&
			!(s_RestoreFail && s_Reads != 0U && reg == 7U))
		{
			s_Analog[reg] = (uint8_t)(Value >> 16);
		}
		s_Values[Addr] = (Value & 0xFF00FFFFU) | ((uint32_t)s_Analog[reg] << 16);
	}
	else
	{
		s_Values[Addr] = Value;
	}
	return *this;
}

uint32_t EspC6RngTestCycles(void)
{
	uint8_t out[4] = { 1U, 1U, 1U, 1U };

	if (s_Nested)
	{
		s_Nested = false;
		assert(Esp32C6RngFill(out, sizeof(out)) == CRYPTO_STATUS_BUSY);
		assert(out[0] == 0U && out[3] == 0U);
	}
	if (!s_StallClock)
	{
		s_Cycles += 80U;
	}
	return s_Cycles;
}

extern "C" uint32_t SystemCoreClockGet(void)
{
	return 80000000U;
}

int main(int argc, char **argv)
{
	uint8_t out[66];
	uint8_t previous[64];
	uint8_t analog[8];
	std::map<uintptr_t, uint32_t> saved;
	unsigned writes;
	bool failure;

	assert(argc == 2);
	s_Values[RNG_ADC_CTRL] = 0x40038240U;
	s_Values[RNG_ADC_CTRL2] = 0xA000U;
	s_Values[RNG_ADC_PATTERN] = 0xFFFFFFU;
	s_Values[RNG_ADC_CLOCK] = 0x84U;
	s_Values[RNG_ADC_CLKM] = 0x400004U;
	s_Values[RNG_ANALOG_POWER] = strcmp(argv[1], "powered") == 0 ? 0x08001234U : 0x04001234U;
	s_Values[RNG_ANALOG_CLOCK] = 0x81U;
	s_Values[RNG_CLOCK] = 0xEE000000U;
	s_Values[RNG_ANALOG_READ] = 0x2DU;
	s_Values[RNG_ANALOG_SELECT] = strcmp(argv[1], "master0") == 0 ? 1U << 11 : 0U;
	assert(Esp32C6RngSecureAvailable());
	assert(Esp32C6RngFill(nullptr, 0U) == CRYPTO_STATUS_OK);
	assert(Esp32C6RngFill(nullptr, 1U) == CRYPTO_STATUS_FAIL);
	if (strcmp(argv[1], "owner") == 0)
	{
		s_Values[RNG_ADC_CTRL2] |= RNG_TIMER_EN;
		assert(!Esp32C6RngSecureAvailable());
		writes = s_Writes;
		memset(out, 0xCC, sizeof(out));
		assert(Esp32C6RngFill(out, sizeof(out)) == CRYPTO_STATUS_FAIL);
		assert(s_Writes == writes && s_Reads == 0U);
		for (unsigned i = 0U; i < sizeof(out); i++)
		{
			assert(out[i] == 0U);
		}
		return 0;
	}
	s_StallClock = strcmp(argv[1], "clock") == 0;
	s_StallBus = strcmp(argv[1], "bus") == 0;
	s_IgnoreAnalog = strcmp(argv[1], "analog") == 0;
	s_Stuck = strcmp(argv[1], "stuck") == 0;
	s_Nested = strcmp(argv[1], "nested") == 0;
	s_RestoreFail = strcmp(argv[1], "restore") == 0;
	if (strcmp(argv[1], "foreign") == 0)
	{
		s_Analog[7] |= 1U;
	}
	failure = s_RestoreFail || s_StallClock || s_StallBus || s_IgnoreAnalog || s_Stuck ||
		strcmp(argv[1], "foreign") == 0;
	saved = s_Values;
	memcpy(analog, s_Analog, sizeof(analog));
	memset(out, 0xCC, sizeof(out));
	assert(Esp32C6RngFill(out + 1, 64U) == (failure ? CRYPTO_STATUS_FAIL : CRYPTO_STATUS_OK));
	assert(out[0] == 0xCCU && out[65] == 0xCCU);
	for (const auto &reg : saved)
	{
		assert(s_Values[reg.first] == reg.second);
	}
	assert(memcmp(analog, s_Analog, s_RestoreFail ? 7U : sizeof(analog)) == 0);
	if (failure)
	{
		for (unsigned i = 1U; i < 65U; i++)
		{
			assert(out[i] == 0U);
		}
		assert(!Esp32C6RngSecureAvailable());
		assert(g_Esp32C6RngError != 0U);
		assert(Esp32C6RngFill(out + 1, 64U) == CRYPTO_STATUS_FAIL);
	}
	else
	{
		assert(s_Reads == 64U);
		assert(!s_BadPowerSequence);
		assert(s_PowerUps == (strcmp(argv[1], "powered") == 0 ? 0U : 1U));
		assert(g_Esp32C6RngError == 0U);
		memcpy(previous, out + 1, sizeof(previous));
		assert(Esp32C6RngFill(out + 1, 17U) == CRYPTO_STATUS_OK);
		assert(memcmp(previous, out + 1, 17U) != 0);
		assert(memcmp(previous + 17, out + 18, 47U) == 0);
		assert(Esp32C6RngSecureAvailable());
	}
	puts("C6 RNG register model PASS");
}
