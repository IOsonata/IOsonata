/**-------------------------------------------------------------------------
@file	system_sleep_esp32c6.c

@brief	ESP32-C6 PMU light-sleep and reset-on-wake Deep-sleep control.

The sleep state follows the ESP-IDF v5.5.1 PMU register definitions.
Light sleep retains the CPU, SRAM, SPI flash supply and the running
oscillators. Deep sleep powers off the HP domain and uses LP GPIO wake.
The application stops active peripheral transfers before entry.

@license MIT
Copyright (c) 2026, I-SYST inc., all rights reserved.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "esp32xx.h"
#include "esp32xx_gpio.h"
#include "idelay.h"

#define C6_PMU(Off) ESP32_REG32(0x600B0000UL + (Off))
#define C6_ROOT ESP32_REG32(0x60096110UL)
#define C6_CPU ESP32_REG32(0x60096118UL)
#define C6_AHB ESP32_REG32(0x6009611CUL)
#define C6_LP_CLOCK ESP32_REG32(0x600B0400UL)
#define C6_LP_STORE9 ESP32_REG32(0x600B1024UL)
#define C6_EFUSE_BIAS ESP32_REG32(0x600B084CUL)
#define C6_EFUSE_VERSION ESP32_REG32(0x600B0850UL)
#define C6_SLEEP_WAKE (1UL << 2)
#define C6_DEEP_WAKE (1UL << 1)
#define C6_SLEEP_RESULT ((1UL << 30) | (1UL << 31))
#define C6_PLL_KEEP ((1UL << 28) | (1UL << 29) | (1UL << 30))
#define C6_DEEP_HP_PD ((1UL << 21) | (1UL << 27) | (1UL << 29) | (1UL << 30) | (1UL << 31))
#define C6_DBIAS_MASK (31UL << 27)
#define C6_SLEEP_MODE_MASK (3UL << 16)
#define C6_SLEEP_PROTECT_MODE (2UL << 16)
#define C6_SLEEP_PROTECT_MASK (3UL << 16)
#define C6_SYSCLK_MASK (0x3FUL << 26)
#define C6_SYSCLK_ACTIVE (1UL << 27)
#define C6_SYSCLK_SLEEP ((1UL << 28) | (1UL << 29))
#define C6_ANALOG_CLOCK ESP32_REG32(0x600AF018UL)
#define C6_ANALOG_FORCE ESP32_REG32(0x600AF01CUL)
#define C6_ANALOG_SOURCE ESP32_REG32(0x600AF010UL)
#define C6_ANALOG_READ ESP32_REG32(0x600AF81CUL)
#define C6_ANALOG_SELECT ESP32_REG32(0x600AF820UL)

static const uint8_t s_C6RegulatorAddr[] = { 5U, 7U, 13U };
static const uint8_t s_C6RegulatorMask[] = { 0x80U, 0x80U, 0x0CU };
static const uint8_t s_C6RegulatorValue[] = { 0x80U, 0x80U, 0U };

// Only R/W PMU registers are saved. Status and W1C registers are excluded.
static const uint16_t s_C6SavedOffsets[] = {
	0x00U, 0x04U, 0x08U, 0x0CU, 0x10U, 0x14U, 0x18U, 0x1CU,
	0x20U, 0x24U, 0x28U, 0x2CU, 0x30U,
	0x34U, 0x38U, 0x3CU, 0x40U, 0x44U, 0x48U, 0x4CU, 0x50U,
	0x54U, 0x58U, 0x5CU, 0x60U, 0x64U,
	0x68U, 0x6CU, 0x70U, 0x74U, 0x78U, 0x7CU, 0x80U, 0x84U, 0x88U, 0x8CU, 0x90U, 0x94U,
	0x98U, 0x9CU, 0xA0U, 0xA8U, 0xACU, 0xB4U, 0xB8U, 0xBCU, 0xC0U, 0xC4U,
	0xC8U, 0xECU, 0xF0U, 0xF4U, 0xF8U, 0xFCU, 0x104U, 0x108U, 0x10CU,
	0x11CU, 0x124U, 0x128U, 0x12CU,
	0x134U, 0x13CU,
};

typedef struct {
	uint32_t Root;
	uint32_t Cpu;
	uint32_t Ahb;
	uint32_t LpStore9;
	uint32_t LpClock;
	uint32_t CpuMHz;
	uint32_t Registers[sizeof(s_C6SavedOffsets) / sizeof(s_C6SavedOffsets[0])];
} Esp32C6SleepState_t;

// Sleep progress and hardware status remain in SRAM when the UART is off.
// Inspect these globals in the debugger if the MCU stops responding.
volatile uint32_t g_Esp32C6SleepStage;
volatile uint32_t g_Esp32C6SleepIrq;
volatile uint32_t g_Esp32C6SleepGpio;
volatile uint32_t g_Esp32C6SleepWakeEnable;
volatile uint32_t g_Esp32C6SleepPmuRaw;
volatile uint32_t g_Esp32C6SleepWakeCause;
volatile uint32_t g_Esp32C6SleepRejectCause;
volatile uint32_t g_Esp32C6SleepRoot;

// Optional diagnostic output. The handler and its data must be in RAM.
// It runs with interrupts masked and may run with the cache suspended.
void (*g_Esp32C6SleepTrace)(uint32_t Stage);

uint32_t Esp32C6FastClockMeasure(void);
bool Esp32LightWakePrepare(void);
void Esp32LightWakeRestore(void);
bool Esp32DeepWakePrepare(void);
void Esp32DeepWakeRestore(void);
void Esp32RomClockUpdate(uint32_t CpuMHz);
uint32_t Esp32RomCacheSuspend(void);
void Esp32RomCacheResume(uint32_t Autoload);

static inline __attribute__((always_inline)) void Esp32C6SetSleepStage(uint32_t Stage)
{
	g_Esp32C6SleepStage = Stage;
	if (g_Esp32C6SleepTrace != 0)
	{
		g_Esp32C6SleepTrace(Stage);
	}
}

static bool Esp32C6AnalogWait(uintptr_t Ctrl)
{
	for (unsigned count = 0U; count < 100000U; count++)
	{
		if ((ESP32_REG32(Ctrl) & (1UL << 25)) == 0U)
		{
			return true;
		}
	}
	return false;
}

// DIG_REG block access follows the C6 analog-I2C ROM workaround.
// The caller holds interrupts masked and enables the master clock.
static bool Esp32C6AnalogTransfer(uint8_t Addr, uint8_t *pValue, bool Write)
{
	uintptr_t ctrl;
	uint32_t selection;
	uint32_t command;
	bool ready;

	ctrl = 0x600AF800UL + ((C6_ANALOG_SELECT & (1UL << 12)) != 0U ? 0U : 4U);
	selection = C6_ANALOG_READ & 0x00FFFFFFU;
	C6_ANALOG_READ = 0x00FFFBFFU;
	command = 0x6DU | ((uint32_t)Addr << 8);
	if (Write)
	{
		command |= (1UL << 24) | ((uint32_t)*pValue << 16);
	}
	ready = Esp32C6AnalogWait(ctrl);
	if (ready)
	{
		ESP32_REG32(ctrl) = command;
		ready = Esp32C6AnalogWait(ctrl);
		if (ready && !Write)
		{
			*pValue = (uint8_t)(ESP32_REG32(ctrl) >> 16);
		}
	}
	C6_ANALOG_READ = selection;
	return ready;
}

static bool Esp32C6RegulatorPrepare(void)
{
	uint32_t clock;
	uint32_t force;
	uint32_t source;
	uint8_t saved[3];
	uint8_t value;
	uint8_t readback;
	unsigned changed;
	bool ready;

	clock = C6_ANALOG_CLOCK;
	force = C6_ANALOG_FORCE;
	source = C6_ANALOG_SOURCE;
	C6_ANALOG_CLOCK = clock | (1UL << 2);
	C6_ANALOG_FORCE = force | (1UL << 2);
	C6_ANALOG_SOURCE = source | 1U;
	ready = true;
	changed = 0U;
	for (unsigned i = 0U; i < 3U; i++)
	{
		if (!Esp32C6AnalogTransfer(s_C6RegulatorAddr[i], &saved[i], false))
		{
			ready = false;
			break;
		}
	}
	if (ready)
	{
		// Enable the PMU voltage interface before releasing the analog
		// force-on bits. Active and sleep regulator settings are ready.
		for (unsigned i = 0U; i < 3U; i++)
		{
			value = (saved[i] & ~s_C6RegulatorMask[i]) | s_C6RegulatorValue[i];
			changed = i + 1U;
			if (!Esp32C6AnalogTransfer(s_C6RegulatorAddr[i], &value, true) ||
				!Esp32C6AnalogTransfer(s_C6RegulatorAddr[i], &readback, false) ||
				readback != value)
			{
				ready = false;
				break;
			}
		}
	}
	if (!ready)
	{
		while (changed > 0U)
		{
			changed--;
			(void)Esp32C6AnalogTransfer(s_C6RegulatorAddr[changed], &saved[changed], true);
		}
	}
	C6_ANALOG_SOURCE = source;
	C6_ANALOG_FORCE = force;
	C6_ANALOG_CLOCK = clock;
	return ready;
}

static uint32_t Esp32C6Dbias(uint32_t Shift, uint32_t Default)
{
	uint32_t efuse;
	uint32_t dbias;

	if (((C6_EFUSE_VERSION >> 30) & 3U) < 3U)
	{
		return Default;
	}
	efuse = (C6_EFUSE_BIAS >> Shift) & 31U;
	if (efuse == 0U)
	{
		return Default;
	}
	dbias = efuse + 2U;
	return dbias > 31U ? 31U : dbias;
}

static uint32_t Esp32C6SlowCycles(uint32_t Microseconds)
{
	uint64_t cycles;
	uint32_t freq;

	freq = g_McuOsc.LowPwrOsc.Freq;
	if (freq < 16000U || freq > 300000U)
	{
		freq = 136000U;
	}
	cycles = ((uint64_t)Microseconds * freq + 999999U) / 1000000U;
	return cycles > 255U ? 255U : cycles == 0U ? 1U : (uint32_t)cycles;
}

static void Esp32C6Save(Esp32C6SleepState_t *pState)
{
	pState->Root = C6_ROOT;
	pState->Cpu = C6_CPU;
	pState->Ahb = C6_AHB;
	pState->LpStore9 = C6_LP_STORE9;
	pState->LpClock = C6_LP_CLOCK;
	pState->CpuMHz = (SystemCoreClockGet() + 500000U) / 1000000U;
	for (unsigned i = 0U; i < sizeof(s_C6SavedOffsets) / sizeof(s_C6SavedOffsets[0]); i++)
	{
		pState->Registers[i] = C6_PMU(s_C6SavedOffsets[i]);
	}
}

static void Esp32C6Restore(const Esp32C6SleepState_t *pState)
{
	for (unsigned i = 0U; i < sizeof(s_C6SavedOffsets) / sizeof(s_C6SavedOffsets[0]); i++)
	{
		C6_PMU(s_C6SavedOffsets[i]) = pState->Registers[i];
	}
	C6_LP_STORE9 = pState->LpStore9;
	C6_LP_CLOCK = pState->LpClock;
}

static bool Esp32C6Configure(bool bDeep, bool bPll)
{
	uint32_t hp;
	uint32_t lp;
	uint32_t lowfreq;
	uint32_t hpBias;
	uint32_t lpBias;
	uint32_t lpDebug;
	uint32_t efuse;
	uint32_t wake;
	uint32_t fastFreq = 20000000U;
	uint32_t supplyCycles;

	wake = bDeep ? C6_DEEP_WAKE : C6_SLEEP_WAKE;
	// Read the current clock state, not a write-trigger command register.
	if ((C6_PMU(0x194U) & (1UL << 15)) != 0U)
	{
		return false;
	}
	if (bDeep)
	{
		// Match the C6 startup clock used by the PMU reference sequence.
		C6_PMU(0xACU) |= 1UL << 30;
		usDelay(50);
		fastFreq = Esp32C6FastClockMeasure();
		if (fastFreq < 10000000U || fastFreq > 30000000U)
		{
			return false;
		}
		C6_LP_CLOCK &= ~(1UL << 2);
		usDelay(10);
	}
	supplyCycles = bDeep ? (fastFreq * 2U + 999999U) / 1000000U : 32U;
	hp = C6_PMU(0x68U);
	lp = C6_PMU(0xC0U);
	lowfreq = Esp32C6SlowCycles(450U);
	hpBias = Esp32C6Dbias(0U, 25U);
	lpBias = Esp32C6Dbias(5U, 26U);
	lpDebug = 0U;
	efuse = C6_EFUSE_BIAS;

	if (bDeep)
	{
		// Direct boot leaves power-up, no-reset and no-isolation overrides
		// asserted. Initialize the wake state before giving the PMU control.
		C6_PMU(0x00U) &= ~(C6_DEEP_HP_PD | (1UL << 22) | (15UL << 23));
		C6_PMU(0x04U) = 0xFFFFFFFFU;
		C6_PMU(0x08U) = 0xFFFFFFFFU;
		C6_PMU(0x14U) = (C6_PMU(0x14U) & ~(3UL << 26)) | C6_PLL_KEEP;
		C6_PMU(0x18U) = (C6_PMU(0x18U) & ~(0x3FUL << 26)) | (1UL << 25);
		C6_PMU(0x28U) = (C6_PMU(0x28U) &
			~(C6_DBIAS_MASK | (1UL << 15) | (3UL << 16))) |
			(1UL << 18) | (hpBias << 27);
		C6_PMU(0x30U) |= 1UL << 31;
		C6_PMU(0x9CU) = (C6_PMU(0x9CU) &
			~(C6_DBIAS_MASK | (1UL << 21))) | (1UL << 22) | (lpBias << 27);

		// Complete the active/modem/sleep state tables before releasing
		// the boot-time overrides. No register-DMA retention is used.
		C6_PMU(0x0CU) = 2UL << 30;
		C6_PMU(0x10U) &= ~(63UL << 24);
		C6_PMU(0x1CU) = (2UL << 4) | (2UL << 6) | (1UL << 16) | (2UL << 23);
		C6_PMU(0x20U) = 0xFFFFFFFFU;
		C6_PMU(0x28U) &= ~(255UL << 19);
		C6_PMU(0x2CU) &= 255U;
		C6_PMU(0x34U) = 1UL << 29;
		C6_PMU(0x38U) = 1UL << 20;
		C6_PMU(0x3CU) = 1UL << 9;
		C6_PMU(0x40U) = 1UL << 30;
		// Pause existing watchdogs in sleep; this does not enable them.
		C6_PMU(0x44U) = (1UL << 24) | (1UL << 28) | (1UL << 29);
		C6_PMU(0x48U) = C6_PLL_KEEP;
		C6_PMU(0x4CU) = 0U;
		C6_PMU(0x50U) = (1UL << 4) | (1UL << 20);
		C6_PMU(0x54U) = 0xFFFFFFFFU;
		C6_PMU(0x58U) = (15UL << 27);
		C6_PMU(0x5CU) = (1UL << 18) | (hpBias << 27);
		C6_PMU(0x60U) = 0U;
		C6_PMU(0x64U) = 1UL << 31;
		C6_PMU(0x74U) = 0U;
		C6_PMU(0x78U) = (1UL << 24) | (7UL << 27);
		C6_PMU(0x80U) &= ~(1UL << 25);
		C6_PMU(0x84U) = (2UL << 8) | (5UL << 23) | (4UL << 26);
		C6_PMU(0x88U) = 0xFFFFFFFFU;
		C6_PMU(0x90U) &= ~(255UL << 19);
		C6_PMU(0x94U) &= 255U;
		C6_PMU(0x9CU) &= ~(15UL << 23);
		C6_PMU(0xA0U) &= ~(15UL << 28);
		C6_PMU(0xA8U) &= ~(3UL << 30);
		C6_PMU(0xACU) &= ~(1UL << 31);
		C6_PMU(0xB4U) &= ~(15UL << 23);
		C6_PMU(0xB8U) &= ~(15UL << 28);
		C6_PMU(0xC8U) &= ~(1UL << 25);

		// ESP-IDF pmu_power_domain_force_default(): release each domain's
		// force controls, leaving its timing and power-down masks intact.
		C6_PMU(0xF4U) &= ~0x3FUL;
		C6_PMU(0xF8U) &= ~0x3FUL;
		C6_PMU(0xFCU) &= ~0x3FUL;
		C6_PMU(0x104U) &= ~0x3FUL;
		C6_PMU(0x108U) &= ~0x3FUL;
		C6_PMU(0x10CU) &= ~(15UL << 24);
		hp |= C6_DEEP_HP_PD;
		// LP GPIO is the sole configured Deep-sleep wake source.
		// The LP peripheral power domain must remain available.
		lp = (lp | (1UL << 30)) & ~(1UL << 31);
		hpBias = 0U;
		lpBias = 23U;
		lpDebug = 12U;
		if (((C6_EFUSE_VERSION >> 30) & 3U) >= 3U)
		{
			lpBias = 23U + ((efuse >> 19) & 15U);
			lpDebug = 8U + ((efuse >> 16) & 7U);
		}
		if (lpBias > 31U)
		{
			lpBias = 31U;
		}
	}
	else
	{
		hp &= ~((1UL << 21) | (1UL << 22) |
			(15UL << 23) | (1UL << 27) |
			(1UL << 29) | (1UL << 30) | (1UL << 31));
		lp &= ~((1UL << 30) | (1UL << 31));
	}
	C6_PMU(0x68U) = hp;
	C6_PMU(0xC0U) = lp;
	// HP sleep must not power off the LP GPIO peripheral either.
	C6_PMU(0xA8U) &= ~(1UL << 31);
	// The active LP slow clock remains powered for wake detection.
	C6_PMU(0xACU) |= (1UL << 30);
	if (g_McuOsc.LowPwrOsc.Type == OSC_TYPE_XTAL)
	{
		C6_PMU(0xACU) |= (1UL << 28);
		C6_PMU(0xC4U) |= (1UL << 28);
	}
	else if (g_McuOsc.LowPwrOsc.Freq <= 64000U)
	{
		C6_PMU(0xACU) |= (1UL << 29);
		C6_PMU(0xC4U) |= (1UL << 29);
	}
	C6_PMU(0xC4U) &= ~(1UL << 31);
	if (bDeep)
	{
		C6_PMU(0xC4U) &= ~(1UL << 30);
		// RC_FAST supplies the HP wake sequencer while XTAL restarts.
		C6_PMU(0x98U) &= ~(1UL << 31);
		C6_PMU(0x80U) = (C6_PMU(0x80U) &
			~((15UL << 26) | (3UL << 30))) | (3UL << 30);
		C6_PMU(0xBCU) &= ~(1UL << 31);
		C6_PMU(0x7CU) = (C6_PMU(0x7CU) &
			~C6_PLL_KEEP) | (3UL << 26);
		C6_PMU(0x90U) &= ~((1UL << 16) | (1UL << 17) | (1UL << 18));
		C6_PMU(0xC8U) = (C6_PMU(0xC8U) &
			~((15UL << 26) | (3UL << 30))) |
			(lpDebug << 26) | (3UL << 30);
	}
	else
	{
		C6_PMU(0x98U) |= (1UL << 31);
		C6_PMU(0xBCU) |= (1UL << 31);
		C6_PMU(0x7CU) = (C6_PMU(0x7CU) |
			(3UL << 26) | (bPll ? C6_PLL_KEEP : 0U));
		C6_PMU(0x80U) &= ~((15UL << 26) | (3UL << 30));
		C6_PMU(0x90U) |= (1UL << 18);
		C6_PMU(0xC8U) &= ~((15UL << 26) | (3UL << 30));
	}
	C6_PMU(0x90U) = (C6_PMU(0x90U) & ~C6_DBIAS_MASK) |
		(hpBias << 27);
	C6_PMU(0xB4U) = (C6_PMU(0xB4U) &
		~(C6_DBIAS_MASK | (1UL << 21))) |
		(1UL << 22) | (lpBias << 27);
	// RTC/PMU uses the manufacturer-defined analog and digital waits.
	C6_PMU(0xECU) = (C6_PMU(0xECU) &
		~((0x1FFUL << 14) | (0x1FFUL << 23))) |
		(supplyCycles << 14) | (supplyCycles << 23);
	C6_PMU(0xF0U) = (C6_PMU(0xF0U) &
		~((0x7FUL << 16) | (0x1FFUL << 23))) |
		(supplyCycles << 16) | (supplyCycles << 23);
	// PMU_SLEEP_PROTECT_HP_LP_SLEEP is value 2 in the vendor PMU
	// initialization. Bare-metal startup must set it explicitly.
	C6_PMU(0x12CU) = (C6_PMU(0x12CU) &
		~(0xFFFFUL | C6_SLEEP_PROTECT_MASK)) |
		C6_SLEEP_PROTECT_MODE | (lowfreq << 8) | lowfreq;
	C6_PMU(0x13CU) = (C6_PMU(0x13CU) & ~(0xFFFFUL << 16)) |
		((bDeep ? (uint32_t)(((uint64_t)fastFreq * 154U + 999999U) / 1000000U) : 2419U) << 16);
	C6_PMU(0x134U) = (C6_PMU(0x134U) &
		~((0xFFUL << 24) | 0xFFFFFUL)) |
		((Esp32C6SlowCycles(bDeep ? 500U : 154U) & 0xFFU) << 24) |
		20700U;
	// ESP-IDF v5.5.1 uses a 50 us PLL settle interval.
	// Convert the interval using the measured Deep-sleep fast clock.
	// XTAL-off wake counts fast-clock cycles; XTAL-on uses slow cycles.
	C6_PMU(0x11CU) = (bDeep ?
		(uint32_t)(((uint64_t)fastFreq * 250U + 999999U) / 1000000U) :
		Esp32C6SlowCycles(250U)) |
		((uint32_t)(((uint64_t)fastFreq * 50U + 999999U) / 1000000U) << 16);
	// Both sides of the transition need explicit clock and CPU-stall
	// settings. Direct boot does not run ESP-IDF's pmu_init().
	C6_PMU(0x10U) &= ~(1UL << 29);
	C6_PMU(0x24U) = (C6_PMU(0x24U) & ~C6_SYSCLK_MASK) | C6_SYSCLK_ACTIVE;
	C6_PMU(0x78U) |= (1UL << 27) | (1UL << 29);
	C6_PMU(0x6CU) = 0U;
	C6_PMU(0x70U) = 0U;
	C6_PMU(0x8CU) = (C6_PMU(0x8CU) & ~C6_SYSCLK_MASK) | C6_SYSCLK_SLEEP;
	C6_PMU(0x124U) = (1UL << 31) | wake;
	C6_PMU(0x128U) = wake;
	C6_LP_STORE9 = (C6_LP_STORE9 & ~1U) | (bDeep ? 1U : 0U);
	if (C6_PMU(0x128U) != wake ||
		(C6_PMU(0x24U) & C6_SYSCLK_MASK) != C6_SYSCLK_ACTIVE ||
		(C6_PMU(0x8CU) & C6_SYSCLK_MASK) != C6_SYSCLK_SLEEP ||
		(C6_PMU(0x12CU) & C6_SLEEP_PROTECT_MASK) != C6_SLEEP_PROTECT_MODE ||
		(C6_PMU(0xA8U) & (1UL << 31)) != 0U ||
		(C6_PMU(0xC0U) & (1UL << 31)) != 0U ||
		(C6_PMU(0xB4U) & (1UL << 22)) == 0U ||
		((C6_PMU(0xB4U) >> 27) & 31U) != lpBias ||
		((C6_PMU(0x90U) >> 27) & 31U) != hpBias)
	{
		return false;
	}
	if (bDeep)
	{
		return (C6_PMU(0x90U) & (1UL << 18)) == 0U;
	}
	return (C6_PMU(0x90U) & (1UL << 18)) != 0U &&
		(C6_PMU(0x98U) & (1UL << 31)) != 0U &&
		(C6_PMU(0xBCU) & (1UL << 31)) != 0U &&
		(!bPll || (C6_PMU(0x7CU) & C6_PLL_KEEP) == C6_PLL_KEEP);
}

// No flash-resident instruction is executed between selecting XTAL and
// restoring the original root. Deep-sleep resets instead of returning.
__attribute__((noinline, section(".iram.text.esp32_c6_sleep")))
static uint32_t Esp32C6Request(const Esp32C6SleepState_t *pState, bool bDeep)
{
	uint32_t status;
	uint32_t root;
	uint32_t autoload;

	root = pState->Root;
	Esp32C6SetSleepStage(bDeep ? 0x34U : 0x14U);
	g_Esp32C6SleepRoot = root;
	// Suspend from IRAM and wait for outstanding flash reads before the
	// PMU gates clocks. The ROM result preserves the autoload setting.
	autoload = Esp32RomCacheSuspend();
	Esp32C6SetSleepStage(bDeep ? 0x3AU : 0x1AU);
	C6_CPU = pState->Cpu & ~0xFFU;
	C6_AHB = pState->Ahb & ~0xFFU;
	C6_ROOT = root & ~C6_SLEEP_MODE_MASK;
	(void)C6_ROOT;
	Esp32C6SetSleepStage(bDeep ? 0x35U : 0x15U);
	Esp32RomClockUpdate(40U);
	// Allow the PMU's active/sleep ICG selection to control the clocks.
	if (bDeep)
	{
		C6_PMU(0xDCU) = 1UL << 31;
	}
	C6_PMU(0xD0U) = 1UL << 28;
	C6_PMU(0x130U) = 1UL << 31;
	C6_PMU(0x168U) = C6_SLEEP_RESULT | (1UL << 29);
	Esp32C6SetSleepStage(bDeep ? 0x36U : 0x16U);
	C6_PMU(0x120U) = 1UL << 31;
	__asm volatile("fence iorw, iorw" : : : "memory");
	status = 0U;
	for (uint32_t count = 0U; count < 10000000U; count++)
	{
		status = C6_PMU(0x15CU) & C6_SLEEP_RESULT;
		if (status != 0U)
		{
			break;
		}
	}
	g_Esp32C6SleepPmuRaw = C6_PMU(0x15CU);
	g_Esp32C6SleepWakeCause = C6_PMU(0x140U);
	g_Esp32C6SleepRejectCause = C6_PMU(0x144U);
	Esp32C6SetSleepStage(bDeep ? 0x37U : 0x17U);
	if (bDeep)
	{
		// Deep-sleep resets the MCU. A rejected entry cannot assume
		// the pre-sleep PLL or flash supply is still available.
		for (;;)
		{
			__WFI();
		}
	}
	C6_PMU(0x168U) = C6_SLEEP_RESULT;
	C6_CPU = pState->Cpu;
	C6_AHB = pState->Ahb;
	C6_ROOT = root;
	(void)C6_ROOT;
	C6_PMU(0xD0U) = 1UL << 31;
	Esp32RomClockUpdate(pState->CpuMHz);
	Esp32RomCacheResume(autoload);
	Esp32C6SetSleepStage(0x18U);
	g_Esp32C6SleepRoot = C6_ROOT;
	__asm volatile("fence iorw, iorw" : : : "memory");
	return status;
}

bool Esp32PowerSleep(void)
{
	Esp32C6SleepState_t state;
	uintptr_t mask;
	bool attempted;

	mask = DisableInterrupt();
	Esp32C6SetSleepStage(0x10U);
	g_Esp32C6SleepIrq = mask;
	g_Esp32C6SleepGpio = ESP32_GPIO_IN_REG32;
	g_Esp32C6SleepWakeEnable = C6_PMU(0x128U);
	g_Esp32C6SleepPmuRaw = C6_PMU(0x15CU);
	g_Esp32C6SleepWakeCause = C6_PMU(0x140U);
	g_Esp32C6SleepRejectCause = C6_PMU(0x144U);
	if (!Esp32LightWakePrepare())
	{
		Esp32C6SetSleepStage(0x1EU);
		EnableInterrupt(mask);
		return false;
	}
	Esp32C6SetSleepStage(0x11U);
	Esp32C6Save(&state);
	Esp32C6SetSleepStage(0x12U);
	attempted = state.CpuMHz != 0U &&
		(state.Root & 0xFFU) == 0U &&
		((state.Root >> 16) & 3U) != 3U &&
		Esp32C6Configure(false, ((state.Root >> 16) & 3U) == 1U);
	if (attempted)
	{
		Esp32C6SetSleepStage(0x13U);
		g_Esp32C6SleepWakeEnable = C6_PMU(0x128U);
		(void)Esp32C6Request(&state, false);
	}
	else
	{
		Esp32C6SetSleepStage(0x1DU);
	}
	Esp32C6Restore(&state);
	Esp32LightWakeRestore();
	if (attempted)
	{
		Esp32C6SetSleepStage(0x19U);
	}
	EnableInterrupt(mask);
	// This request was handled even if rejected. Return to the caller
	// instead of hiding the PMU result behind another CPU WFI.
	return true;
}

// Exercise the Deep-sleep pin route with the HP domain retained.
// Used by the board demo to separate wake detection from reset-on-wake.
uint32_t Esp32C6CheckDeepWake(void)
{
	Esp32C6SleepState_t state;
	uintptr_t mask;
	uint32_t result;

	result = 0U;
	mask = DisableInterrupt();
	Esp32C6SetSleepStage(0x40U);
	if (Esp32DeepWakePrepare())
	{
		Esp32C6Save(&state);
		Esp32C6SetSleepStage(0x41U);
		if (state.CpuMHz != 0U && (state.Root & 0xFFU) == 0U &&
			((state.Root >> 16) & 3U) != 3U &&
			Esp32C6Configure(false, ((state.Root >> 16) & 3U) == 1U))
		{
			C6_PMU(0x124U) = (1UL << 31) | C6_DEEP_WAKE;
			C6_PMU(0x128U) = C6_DEEP_WAKE;
			g_Esp32C6SleepWakeEnable = C6_DEEP_WAKE;
			result = Esp32C6Request(&state, false);
		}
		Esp32C6Restore(&state);
		Esp32DeepWakeRestore();
	}
	Esp32C6SetSleepStage(0x42U);
	EnableInterrupt(mask);
	return result;
}

static void Esp32C6DeepSleep(bool KeepSupply)
{
	Esp32C6SleepState_t state;
	uintptr_t mask;
	bool ready;

	mask = DisableInterrupt();
	Esp32C6SetSleepStage(0x30U);
	ready = Esp32DeepWakePrepare();
	if (ready)
	{
		Esp32C6SetSleepStage(0x31U);
		Esp32C6Save(&state);
		Esp32C6SetSleepStage(0x32U);
		ready = state.CpuMHz != 0U &&
			((state.Root >> 16) & 3U) != 3U &&
			Esp32C6Configure(true, false);
		if (ready && KeepSupply)
		{
			// Isolate HP digital reset-on-wake from regulator and oscillator
			// shutdown. This board diagnostic does not measure Deep-sleep power.
			C6_PMU(0x68U) &= ~(1UL << 21);
			C6_PMU(0x7CU) = (C6_PMU(0x7CU) & ~(31UL << 26)) |
				(C6_PMU(0x14U) & (31UL << 26));
			C6_PMU(0x80U) = (C6_PMU(0x80U) & ~(127UL << 25)) |
				(C6_PMU(0x18U) & (127UL << 25));
			C6_PMU(0x90U) = (C6_PMU(0x90U) & ~C6_DBIAS_MASK) |
				(C6_PMU(0x28U) & C6_DBIAS_MASK) | (1UL << 18);
			C6_PMU(0x11CU) = (C6_PMU(0x11CU) & 0xFFFF0000U) |
				Esp32C6SlowCycles(250U);
			C6_PMU(0x98U) |= 1UL << 31;
			C6_PMU(0xBCU) |= 1UL << 31;
			C6_PMU(0xC4U) |= 1UL << 30;
			C6_PMU(0xC8U) &= ~(127UL << 25);
			C6_PMU(0xB4U) = (C6_PMU(0xB4U) & ~C6_DBIAS_MASK) |
				(C6_PMU(0x9CU) & C6_DBIAS_MASK);
			Esp32C6SetSleepStage(0x50U);
		}
		if (ready)
		{
			Esp32C6SetSleepStage(0x3BU);
			ready = Esp32C6RegulatorPrepare();
			Esp32C6SetSleepStage(ready ? 0x3CU : 0x3FU);
		}
		if (ready)
		{
			Esp32C6SetSleepStage(0x33U);
			g_Esp32C6SleepWakeEnable = C6_PMU(0x128U);
			(void)Esp32C6Request(&state, true);
		}
		Esp32C6SetSleepStage(0x3DU);
		Esp32C6Restore(&state);
		Esp32DeepWakeRestore();
	}
	else
	{
		Esp32C6SetSleepStage(0x3EU);
	}
	(void)mask;
	for (;;)
	{
		__WFI();
	}
}

// Temporary board diagnostic: HP digital domains power down, supplies stay on.
void Esp32C6CheckDeepReset(void)
{
	Esp32C6DeepSleep(true);
}

void Esp32PowerDeepSleep(void)
{
	Esp32C6DeepSleep(false);
}
