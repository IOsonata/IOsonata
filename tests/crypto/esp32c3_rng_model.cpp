// Exercises the production ESP32-C3 entropy-source driver with modeled MMIO.
// Deterministic test data is NOT cryptographic entropy.
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp32xx.h"
#include "crypto/icrypto.h"
#include "esp32c3_rng_model.h"

extern bool Esp32C3RngProbe(void);
extern bool Esp32C3RngSecureAvailable(void);
extern CRYPTO_STATUS Esp32C3RngFill(uint8_t *pOut, size_t Len);

static EspRngModelReg s_Registers[32];
static uint32_t s_Values[32];
static unsigned s_RegCount;
static uint32_t s_Cycles;
static uint32_t s_Data = 0xA5C39D71UL;
static unsigned s_ReadCount;
static unsigned s_AnalogWrites;
static uint8_t s_Analog = 0xA3U;
static bool s_AnalogWriteIgnored;
static bool s_Stuck;

static uint32_t &ModelWord(uintptr_t Addr)
{
	for (unsigned i = 0; i < s_RegCount; i++)
	{
		if (s_Registers[i].Addr == Addr)
		{
			return s_Values[i];
		}
	}
	assert(s_RegCount < 32);
	s_Registers[s_RegCount].Addr = Addr;
	return s_Values[s_RegCount++];
}

EspRngModelReg &EspRngModelRegister(uintptr_t Addr)
{
	(void)ModelWord(Addr);
	for (unsigned i = 0; i < s_RegCount; i++)
	{
		if (s_Registers[i].Addr == Addr)
		{
			return s_Registers[i];
		}
	}
	assert(false);
	return s_Registers[0];
}

EspRngModelReg::operator uint32_t() const
{
	if (Addr == ESP32_SYSCON_BASE + 0xB0U)
	{
		assert((ModelWord(ESP32_ADC_BASE + 0x004U) & (1UL << 24)) != 0U);
		assert((ModelWord(ESP32_ADC_BASE + 0x054U) & (1UL << 20)) != 0U);
		assert((s_Analog & (1U << 6)) != 0U);
		s_ReadCount++;
		if (s_Stuck)
		{
			return 0x13572468U;
		}
		s_Data ^= s_Data << 13;
		s_Data ^= s_Data >> 17;
		s_Data ^= s_Data << 5;
		return s_Data;
	}
	return ModelWord(Addr);
}

EspRngModelReg &EspRngModelReg::operator=(uint32_t Value)
{
	ModelWord(Addr) = Value;
	return *this;
}

uint8_t EspRngTestAnalogRead(uint8_t Msb, uint8_t Lsb)
{
	assert(Msb < 8U && Lsb <= Msb);
	return (uint8_t)((s_Analog >> Lsb) & ((1U << (Msb - Lsb + 1U)) - 1U));
}

void EspRngTestAnalogWrite(uint8_t Msb, uint8_t Lsb, uint8_t Value)
{
	uint8_t mask = (uint8_t)(((1U << (Msb - Lsb + 1U)) - 1U) << Lsb);
	s_AnalogWrites++;
	if (!s_AnalogWriteIgnored)
	{
		s_Analog = (uint8_t)((s_Analog & (uint8_t)~mask) |
			(uint8_t)((Value << Lsb) & mask));
	}
}

uint32_t EspRngTestCycleCount(void)
{
	s_Cycles += 80U;
	return s_Cycles;
}

extern "C" uint32_t SystemCoreClockGet(void)
{
	return 80000000UL;
}

static void CheckClean(void)
{
	assert(ModelWord(ESP32_ADC_BASE + 0x004U) == 0U);
	assert(ModelWord(ESP32_ADC_BASE + 0x054U) == 0U);
	assert(ModelWord(ESP32_LP_MGNT_BASE + 0x11CU) == 0U);
	assert(ModelWord(ESP32_LP_MGNT_BASE + 0x034U) == 0U);
	assert(ModelWord(ESP32_SYSTEM_BASE + 0x010U) == 0U);
	assert(ModelWord(ESP32_SYSTEM_BASE + 0x018U) == 0U);
	assert(ModelWord(ESP32_ADC_BASE + 0x050U) == 0U);
	assert(s_Analog == 0xA3U);
}

int main(void)
{
	uint8_t a[34];
	uint8_t b[32];
	unsigned reads;
	unsigned analog;
	assert(!Esp32C3RngSecureAvailable());
	assert(Esp32C3RngProbe());
	assert(Esp32C3RngSecureAvailable());
	assert(s_ReadCount >= 6);
	CheckClean();

	memset(a, 0xCC, sizeof(a));
	assert(Esp32C3RngFill(a + 1, 17U) == CRYPTO_STATUS_OK);
	assert(a[0] == 0xCCU && a[18] == 0xCCU && a[33] == 0xCCU);
	assert(Esp32C3RngFill(b, sizeof(b)) == CRYPTO_STATUS_OK);
	assert(memcmp(a + 1, b, 17U) != 0);
	CheckClean();

	reads = s_ReadCount;
	analog = s_AnalogWrites;
	ModelWord(ESP32_ADC_BASE + 0x004U) = 1UL << 24;
	assert(!Esp32C3RngSecureAvailable());
	memset(b, 0xCC, sizeof(b));
	assert(Esp32C3RngFill(b, sizeof(b)) == CRYPTO_STATUS_FAIL);
	for (unsigned i = 0; i < sizeof(b); i++)
	{
		assert(b[i] == 0U);
	}
	assert(s_ReadCount == reads && s_AnalogWrites == analog);
	ModelWord(ESP32_ADC_BASE + 0x004U) = 0U;
	assert(Esp32C3RngSecureAvailable());
	CheckClean();

	// A source-start readback failure must restore ALL ADC/RTC state.
	s_AnalogWriteIgnored = true;
	memset(b, 0xCC, sizeof(b));
	assert(Esp32C3RngFill(b, sizeof(b)) == CRYPTO_STATUS_FAIL);
	for (unsigned i = 0; i < sizeof(b); i++)
	{
		assert(b[i] == 0U);
	}
	s_AnalogWriteIgnored = false;
	CheckClean();
	assert(!Esp32C3RngSecureAvailable());
	assert(Esp32C3RngFill(b, sizeof(b)) == CRYPTO_STATUS_FAIL);
	puts("ESP32-C3 RNG MMIO model: PASS (entropy setup, restore, conflict, fault, bounds)");
}
