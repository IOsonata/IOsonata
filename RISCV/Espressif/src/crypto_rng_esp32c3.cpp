/**-------------------------------------------------------------------------
@file	crypto_rng_esp32c3.cpp

@brief	ESP32-C3 secure RNG via the on-chip SAR ADC entropy source

No ESP-IDF HAL or driver. The register sequence matches Espressif's
bootloader_random_esp32c3.c. The two analog-register operations use the
chip's ROM routines at their documented ESP32-C3 addresses.

The SAR ADC is exclusively owned during each operation and fully restored
afterwards. Until a shared ADC/RF ownership manager exists, callers must
not concurrently use the SAR ADC, Wi-Fi, or Bluetooth with this backend.

MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp32xx.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "crypto/icrypto.h"

#if !defined(ESP32C3)
#error ESP32-C3 RNG implementation requires ESP32C3
#endif

#ifndef ESP32_RNG_REG
#define ESP32_RNG_REG(Addr) (*(volatile uint32_t *)(uintptr_t)(Addr))
#endif

// ESP32-C3 register map: RTC_CNTL, SYSTEM, APB_SARADC, SYSCON.
// See the ESP32-C3 TRM and ESP-IDF v5.4 bootloader_random_esp32c3.c.
#define RNG_RTC_SENSOR_CTRL		(ESP32_LP_MGNT_BASE + 0x11CU)
#define RNG_RTC_ANA_CONF		(ESP32_LP_MGNT_BASE + 0x034U)
#define RNG_SYSTEM_CLK_EN		(ESP32_SYSTEM_BASE + 0x010U)
#define RNG_SYSTEM_RST_EN		(ESP32_SYSTEM_BASE + 0x018U)
#define RNG_ADC_CTRL			(ESP32_ADC_BASE + 0x000U)
#define RNG_ADC_CTRL2			(ESP32_ADC_BASE + 0x004U)
#define RNG_ADC_FSM_WAIT		(ESP32_ADC_BASE + 0x00CU)
#define RNG_ADC_PATTERN1		(ESP32_ADC_BASE + 0x018U)
#define RNG_ADC_PATTERN2		(ESP32_ADC_BASE + 0x01CU)
#define RNG_ADC_DMA_CONF		(ESP32_ADC_BASE + 0x050U)
#define RNG_ADC_CLK_CONF		(ESP32_ADC_BASE + 0x054U)
#define RNG_DATA			(ESP32_SYSCON_BASE + 0x0B0U)

#define RNG_ADC_CLK_MASK		(1UL << 28)
#define RNG_ADC_TIMER_EN		(1UL << 24)
#define RNG_ADC_CLK_EN			(1UL << 20)
#define RNG_ADC_TRANSFER		(1UL << 31)
#define RNG_RTC_SAR_POWER_MASK		(3UL << 30)
#define RNG_RTC_ANA_POWER		(1UL << 22)
#define RNG_ADC_SAR_POWER_MASK		(3UL << 27)
#define RNG_ADC_CLK_DIV_MASK		(0xFFUL << 7)
#define RNG_ADC_PATTERN_LEN_MASK	(7UL << 15)
#define RNG_ADC_CLOCK_SEL_MASK		(3UL << 21)
#define RNG_ADC_CLOCK_DIV_MASK		0xFFUL
#define RNG_ADC_CLOCK_GATED		(1UL << 6)
#define RNG_ADC_PATTERN_CLEAR		(1UL << 23)
#define RNG_ADC_TIMER_TARGET_MASK	(0xFFFUL << 12)
#define RNG_ADC_MEAS_LIMIT		(1UL << 0)

// ROM internal analog I2C: I2C_SAR_ADC block 0x69, host 0,
// ADC_SARADC2_ENCAL_REF_ADDR at address 7 bit 6.
#ifndef ESP32_RNG_TEST
typedef uint8_t (*EspRngRomReadMask_t)(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
typedef void (*EspRngRomWriteMask_t)(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
static uint8_t EspRngAnalogRead(uint8_t Msb, uint8_t Lsb)
{
	EspRngRomReadMask_t read = (EspRngRomReadMask_t)(uintptr_t)0x40001958UL;
	return read(0x69U, 0U, 7U, Msb, Lsb);
}
static void EspRngAnalogWrite(uint8_t Msb, uint8_t Lsb, uint8_t Value)
{
	EspRngRomWriteMask_t write = (EspRngRomWriteMask_t)(uintptr_t)0x40001960UL;
	write(0x69U, 0U, 7U, Msb, Lsb, Value);
}
static inline uint32_t EspRngCycleCount(void)
{
	uint32_t count;
	__asm volatile ("csrr %0, 0xC00" : "=r"(count));
	return count;
}
#else
extern uint8_t EspRngTestAnalogRead(uint8_t Msb, uint8_t Lsb);
extern void EspRngTestAnalogWrite(uint8_t Msb, uint8_t Lsb, uint8_t Value);
extern uint32_t EspRngTestCycleCount(void);
#define EspRngAnalogRead EspRngTestAnalogRead
#define EspRngAnalogWrite EspRngTestAnalogWrite
#define EspRngCycleCount EspRngTestCycleCount
#endif

typedef struct {
	uint32_t Sensor;
	uint32_t Ana;
	uint32_t ClockEn;
	uint32_t ResetEn;
	uint32_t Ctrl;
	uint32_t Ctrl2;
	uint32_t Wait;
	uint32_t Pattern1;
	uint32_t Pattern2;
	uint32_t Dma;
	uint32_t ClockConf;
	uint8_t AnalogRef;
	uint8_t AnalogDtest;
	uint8_t AnalogRtc;
	uint8_t AnalogTsens;
} EspRngSnapshot_t;

static bool s_EspRngBusy;
static bool s_EspRngFault;

// Debugger-visible checkpoints, with normal C++ linkage. No ISR is used.
// Stages: 1 request, 2 snapshot, 3 analog ROM calls, 4 analog ready,
// 5 ADC configured, 6 source active, 7 settling, 8 reading,
// 9 restoring, 10 completed. Error 1 = busy, 2 = ADC in use,
// 3 = setup failed, 4 = stalled cycle counter, 5 = repeated RNG words.
volatile uint32_t g_Esp32C3RngStage;
volatile uint32_t g_Esp32C3RngError;

#ifndef ESP32_RNG_WAIT_MAX_POLLS
#define ESP32_RNG_WAIT_MAX_POLLS 100000UL
#endif

static inline void EspRngModify(uintptr_t Addr, uint32_t Mask, uint32_t Value)
{
	ESP32_RNG_REG(Addr) = (ESP32_RNG_REG(Addr) & ~Mask) | (Value & Mask);
}

// Reject the request if another owner has configured the ADC.
// No invisible takeover of an active ADC or RF entropy source.
static bool EspRngAdcIdle(void)
{
	if ((ESP32_RNG_REG(RNG_ADC_CTRL2) & RNG_ADC_TIMER_EN) != 0U ||
		(ESP32_RNG_REG(RNG_ADC_DMA_CONF) & RNG_ADC_TRANSFER) != 0U ||
		(ESP32_RNG_REG(RNG_ADC_CLK_CONF) & RNG_ADC_CLK_EN) != 0U ||
		(ESP32_RNG_REG(RNG_RTC_SENSOR_CTRL) & RNG_RTC_SAR_POWER_MASK) != 0U)
	{
		return false;
	}
	g_Esp32C3RngStage = 6U;
	return true;
}

static void EspRngStop(const EspRngSnapshot_t *pSaved);

static bool EspRngStart(EspRngSnapshot_t *pSaved)
{
	if (!EspRngAdcIdle())
	{
		return false;
	}
	g_Esp32C3RngStage = 2U;
	pSaved->Sensor = ESP32_RNG_REG(RNG_RTC_SENSOR_CTRL);
	pSaved->Ana = ESP32_RNG_REG(RNG_RTC_ANA_CONF);
	pSaved->ClockEn = ESP32_RNG_REG(RNG_SYSTEM_CLK_EN);
	pSaved->ResetEn = ESP32_RNG_REG(RNG_SYSTEM_RST_EN);
	pSaved->Ctrl = ESP32_RNG_REG(RNG_ADC_CTRL);
	pSaved->Ctrl2 = ESP32_RNG_REG(RNG_ADC_CTRL2);
	pSaved->Wait = ESP32_RNG_REG(RNG_ADC_FSM_WAIT);
	pSaved->Pattern1 = ESP32_RNG_REG(RNG_ADC_PATTERN1);
	pSaved->Pattern2 = ESP32_RNG_REG(RNG_ADC_PATTERN2);
	pSaved->Dma = ESP32_RNG_REG(RNG_ADC_DMA_CONF);
	pSaved->ClockConf = ESP32_RNG_REG(RNG_ADC_CLK_CONF);

	EspRngModify(RNG_RTC_SENSOR_CTRL, RNG_RTC_SAR_POWER_MASK,
		RNG_RTC_SAR_POWER_MASK);
	EspRngModify(RNG_RTC_ANA_CONF, RNG_RTC_ANA_POWER, RNG_RTC_ANA_POWER);
	g_Esp32C3RngStage = 3U;
	pSaved->AnalogRef = EspRngAnalogRead(6U, 6U);
	pSaved->AnalogDtest = EspRngAnalogRead(1U, 0U);
	pSaved->AnalogRtc = EspRngAnalogRead(3U, 3U);
	pSaved->AnalogTsens = EspRngAnalogRead(2U, 2U);
	EspRngAnalogWrite(6U, 6U, 1U);
	EspRngAnalogWrite(1U, 0U, 0U);
	EspRngAnalogWrite(3U, 3U, 0U);
	EspRngAnalogWrite(2U, 2U, 0U);
	g_Esp32C3RngStage = 4U;

	ESP32_RNG_REG(RNG_SYSTEM_CLK_EN) |= RNG_ADC_CLK_MASK;
	ESP32_RNG_REG(RNG_SYSTEM_RST_EN) &= ~RNG_ADC_CLK_MASK;
	EspRngModify(RNG_ADC_CLK_CONF, RNG_ADC_CLOCK_SEL_MASK | RNG_ADC_CLK_EN |
		RNG_ADC_CLOCK_DIV_MASK, (2UL << 21) | RNG_ADC_CLK_EN | 15UL);
	EspRngModify(RNG_ADC_CTRL, RNG_ADC_CLOCK_GATED | RNG_ADC_SAR_POWER_MASK |
		RNG_ADC_CLK_DIV_MASK, RNG_ADC_CLOCK_GATED | RNG_ADC_SAR_POWER_MASK |
		(1UL << 7));
	EspRngModify(RNG_ADC_FSM_WAIT, 0x00FFFFFFUL,
		(100UL << 16) | (8UL << 8) | 5UL);
	ESP32_RNG_REG(RNG_ADC_CTRL) |= RNG_ADC_PATTERN_CLEAR;
	ESP32_RNG_REG(RNG_ADC_CTRL) &= ~RNG_ADC_PATTERN_CLEAR;
	EspRngModify(RNG_ADC_CTRL, RNG_ADC_PATTERN_LEN_MASK, 0U);
	EspRngModify(RNG_ADC_PATTERN1, 0x00FFFFFFUL, 0x009CFFFFUL);
	EspRngModify(RNG_ADC_PATTERN2, 0x00FFFFFFUL, 0x00FFFFFFUL);
	EspRngModify(RNG_ADC_CTRL2, RNG_ADC_TIMER_TARGET_MASK |
		RNG_ADC_MEAS_LIMIT, (100UL << 12));
	ESP32_RNG_REG(RNG_ADC_DMA_CONF) |= RNG_ADC_TRANSFER;
	ESP32_RNG_REG(RNG_ADC_CTRL2) |= RNG_ADC_TIMER_EN;
	g_Esp32C3RngStage = 5U;

	if ((ESP32_RNG_REG(RNG_ADC_CTRL2) & RNG_ADC_TIMER_EN) == 0U ||
		(ESP32_RNG_REG(RNG_ADC_CLK_CONF) & RNG_ADC_CLK_EN) == 0U ||
		(EspRngAnalogRead(6U, 6U) & 1U) == 0U)
	{
		// The source is not active; never return without undoing setup.
		EspRngStop(pSaved);
		return false;
	}
	return true;
}

static void EspRngStop(const EspRngSnapshot_t *pSaved)
{
	ESP32_RNG_REG(RNG_ADC_CTRL2) &= ~RNG_ADC_TIMER_EN;
	EspRngAnalogWrite(6U, 6U, pSaved->AnalogRef);
	EspRngAnalogWrite(1U, 0U, pSaved->AnalogDtest);
	EspRngAnalogWrite(3U, 3U, pSaved->AnalogRtc);
	EspRngAnalogWrite(2U, 2U, pSaved->AnalogTsens);
	ESP32_RNG_REG(RNG_ADC_DMA_CONF) = pSaved->Dma;
	ESP32_RNG_REG(RNG_ADC_PATTERN1) = pSaved->Pattern1;
	ESP32_RNG_REG(RNG_ADC_PATTERN2) = pSaved->Pattern2;
	ESP32_RNG_REG(RNG_ADC_FSM_WAIT) = pSaved->Wait;
	ESP32_RNG_REG(RNG_ADC_CTRL) = pSaved->Ctrl;
	ESP32_RNG_REG(RNG_ADC_CTRL2) = pSaved->Ctrl2;
	ESP32_RNG_REG(RNG_ADC_CLK_CONF) = pSaved->ClockConf;
	// Other peripheral drivers may have changed unrelated clock/power bits.
	EspRngModify(RNG_SYSTEM_RST_EN, RNG_ADC_CLK_MASK, pSaved->ResetEn);
	EspRngModify(RNG_SYSTEM_CLK_EN, RNG_ADC_CLK_MASK, pSaved->ClockEn);
	EspRngModify(RNG_RTC_SENSOR_CTRL, RNG_RTC_SAR_POWER_MASK, pSaved->Sensor);
	EspRngModify(RNG_RTC_ANA_CONF, RNG_RTC_ANA_POWER, pSaved->Ana);
}

// Always exit even if the cycle CSR is disabled or stops advancing.
// The normal count is only a few hundred iterations; a large upper bound
// allows optimized and Debug builds without letting the firmware spin forever.
static bool EspRngWaitCycles(uint32_t Count)
{
	uint32_t start;
	uint32_t polls;

	start = EspRngCycleCount();
	for (polls = 0U; polls < ESP32_RNG_WAIT_MAX_POLLS; polls++)
	{
		if ((uint32_t)(EspRngCycleCount() - start) >= Count)
		{
			return true;
		}
	}
	return false;
}

bool Esp32C3RngSecureAvailable(void)
{
	// IsSecure describes the hardware entropy capability, not a completed
	// draw. Every Random() request must still enable and check SAR entropy.
	// Do not probe from Crypto::Init() or from IsSecure().
	return !s_EspRngFault && !s_EspRngBusy && EspRngAdcIdle();
}

CRYPTO_STATUS Esp32C3RngFill(uint8_t *pOut, size_t Len)
{
	EspRngSnapshot_t saved;
	uintptr_t state;
	uint32_t cyclesPerUs;
	uint32_t value;
	uint32_t last = 0U;
	unsigned repeats = 0U;
	size_t i;
	size_t n;
	bool started;
	bool ok;

	if (Len == 0U)
	{
		return CRYPTO_STATUS_OK;
	}
	if (pOut == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}
	state = DisableInterrupt();
	if (s_EspRngBusy)
	{
		g_Esp32C3RngError = 1U;
		EnableInterrupt(state);
		memset(pOut, 0, Len);
		return CRYPTO_STATUS_BUSY;
	}
	if (s_EspRngFault || !EspRngAdcIdle())
	{
		g_Esp32C3RngError = 2U;
		EnableInterrupt(state);
		memset(pOut, 0, Len);
		return CRYPTO_STATUS_FAIL;
	}
	s_EspRngBusy = true;
	g_Esp32C3RngError = 0U;
	g_Esp32C3RngStage = 1U;
	EnableInterrupt(state);

	started = EspRngStart(&saved);
	ok = started;
	if (!started)
	{
		g_Esp32C3RngError = 3U;
	}
	cyclesPerUs = SystemCoreClockGet() / 1000000UL;
	if (cyclesPerUs == 0U || cyclesPerUs > 1000U)
	{
		ok = false;
		g_Esp32C3RngError = 4U;
	}
	if (ok)
	{
		// Stabilize the analog noise source before the first sample.
		g_Esp32C3RngStage = 7U;
		if (!EspRngWaitCycles(cyclesPerUs * 100U))
		{
			ok = false;
			g_Esp32C3RngError = 4U;
		}
	}
	if (ok)
	{
		(void)ESP32_RNG_REG(RNG_DATA);
		(void)ESP32_RNG_REG(RNG_DATA);
		g_Esp32C3RngStage = 8U;
	}
	for (i = 0U; i < Len && ok; i += n)
	{
		// SAR entropy must replenish between reads. This also implements
		// Espressif's conservative bootloader pacing (>= 64 us/word).
		if (!EspRngWaitCycles(cyclesPerUs * 80U))
		{
			ok = false;
			g_Esp32C3RngError = 4U;
			break;
		}
		value = ESP32_RNG_REG(RNG_DATA);
		if (i != 0U && value == last)
		{
			repeats++;
			if (repeats >= 2U)
			{
				ok = false;
				g_Esp32C3RngError = 5U;
				break;
			}
		}
		else
		{
			repeats = 0U;
		}
		last = value;
		n = Len - i < sizeof(value) ? Len - i : sizeof(value);
		memcpy(pOut + i, &value, n);
	}
	if (started)
	{
		if (ok)
		{
			g_Esp32C3RngStage = 9U;
		}
		EspRngStop(&saved);
	}
	state = DisableInterrupt();
	if (!ok && g_Esp32C3RngError != 2U)
	{
		s_EspRngFault = true;
	}
	if (ok)
	{
		g_Esp32C3RngStage = 10U;
	}
	s_EspRngBusy = false;
	EnableInterrupt(state);
	if (!ok)
	{
		memset(pOut, 0, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

bool Esp32C3RngProbe(void)
{
	uint8_t data[16];
	CRYPTO_STATUS status = Esp32C3RngFill(data, sizeof(data));
	memset(data, 0, sizeof(data));
	return status == CRYPTO_STATUS_OK && Esp32C3RngSecureAvailable();
}
