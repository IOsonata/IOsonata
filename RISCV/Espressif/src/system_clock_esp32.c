/**-------------------------------------------------------------------------
@file	system_clock_esp32.c

@brief	ESP32-C3/C6 direct CPU source and APB clock selection.

Call with peripherals stopped, before starting modem or LP-core firmware.
A UART using XTAL may remain enabled. Existing crystal, PLL and flash clock
settings are preserved. XTAL selects
40 MHz directly; C6 measures RC_FAST against XTAL before selecting it.
C3 measures RC_FAST through its divide-by-256 calibration input. The optional
ESP32C3_CPU_160MHZ / ESP32C6_CPU_160MHZ profile starts the PLL at boot and
selects its 160 MHz CPU output for XTAL requests. The default profile selects
XTAL directly. PLL setup uses the ESP-IDF v5.5.1 per-chip clock sequence.

@license MIT
Copyright (c) 2026, I-SYST inc., all rights reserved.
----------------------------------------------------------------------------*/
#include "esp32xx_gpio.h"
#include "esp32xx_uart.h"
#include "coredev/interrupt.h"
#include "idelay.h"

#if defined(ESP32C3)
#define CLOCK_ROOT_REG ESP32_REG32(0x600C0058UL)
#define CLOCK_SOURCE_SHIFT 10U
#define CLOCK_RC_FREQ 17500000UL
#define CLOCK_POWER_REG ESP32_REG32(0x60008070UL)
#define CLOCK_TIMG0 0x6001F000UL
#define CLOCK_TIMG1 0x60020000UL
#elif defined(ESP32C6)
#define CLOCK_ROOT_REG ESP32_REG32(0x60096110UL)
#define CLOCK_SOURCE_SHIFT 16U
#define CLOCK_RC_FREQ 20000000UL
#define CLOCK_POWER_REG ESP32_REG32(0x600B00ACUL)
#define CLOCK_CPU_REG ESP32_REG32(0x60096118UL)
#define CLOCK_AHB_REG ESP32_REG32(0x6009611CUL)
#define CLOCK_APB_REG ESP32_REG32(0x60096120UL)
#define CLOCK_TIMG0 0x60008000UL
#define CLOCK_TIMG1 0x60009000UL
// USB Serial/JTAG uses the unchanged PLL clock, not the CPU/APB source.
static const uint8_t s_ClockUsers[] = {
	0x20, 0x28, 0x2C, 0x34, 0x5C, 0x64, 0x6C, 0x80,
	0x94, 0x98, 0x9C, 0xA4, 0xB0, 0xBC, 0xC0, 0xC8, 0xCC,
	0xD0, 0xD8, 0xE0, 0xE4,
};
#else
#error "Runtime clock selection requires ESP32C3/C6"
#endif
#define CLOCK_SOURCE_MASK (3UL << CLOCK_SOURCE_SHIFT)

extern uint64_t SystemCoreClockPeriodus;
void SystemCoreClockUpdate(void);
void Esp32RomClockUpdate(uint32_t CpuMHz);
#if defined(ESP32C3)
uint32_t Esp32C3FastClockMeasure(void);
#else
uint32_t Esp32C6FastClockMeasure(void);
#endif

#if defined(ESP32C6)
// Run once during SystemInit, before drivers claim peripherals. PCR bus clocks
// reset enabled; drivers enable their own gates when initialized. Preserve ROM
// flash, UART, interrupt routing, timers and USB Serial/JTAG clocks.
void Esp32C6PeripheralClockInit(void)
{
	for (unsigned i = 0; i < sizeof(s_ClockUsers); i++)
	{
		ESP32_REG32(0x60096000UL + s_ClockUsers[i]) &= ~1UL;
	}
}
#endif

static bool Esp32ClockBusy(void)
{
#if defined(ESP32C3)
	// USB Serial/JTAG keeps its PLL clock independently of CPU/APB selection.
	// System timers, flash, eFuse and UART memory may also remain clocked.
	uint32_t infrastructure = (1UL << 0) | (1UL << 1) | (1UL << 2) |
		(1UL << 3) | (1UL << 5) | (1UL << 13) | (1UL << 14) |
		(1UL << 15) | (1UL << 23) | (1UL << 24) | (1UL << 29);

	if ((ESP32_REG32(0x600C0010UL) & ~infrastructure) != 0U ||
		(ESP32_REG32(0x600C0014UL) & 0x5FEUL) != 0U)
	{
		return true;
	}
#else
	for (unsigned i = 0; i < sizeof(s_ClockUsers); i++)
	{
		if (ESP32_REG32(0x60096000UL + s_ClockUsers[i]) & 1U)
		{
			return true;
		}
	}
	if ((ESP32_REG32(0x600AF018UL) & 0xBU) != 0U ||
		(ESP32_REG32(0x600AF01CUL) & 0xBU) != 0U ||
		(ESP32_REG32(0x600AF020UL) & 0x00FF0000UL) != 0U ||
		ESP32_REG32(0x600B0128UL) != 0U ||
		(ESP32_REG32(0x600B0180UL) & 0xFFFFU) != 0U ||
		(ESP32_REG32(0x600B0194UL) & (1UL << 15)) != 0U)
	{
		return true;
	}
#endif
	// UART may continue from XTAL; changing APB must not change its bit clock.
	for (unsigned uart = 0; uart < 2; uart++)
	{
#if defined(ESP32C3)
		bool enabled = (ESP32_REG32(0x600C0010UL) & (1UL << (uart == 0 ? 2 : 5))) != 0U;
		uint32_t serial = ESP32_UART_CLK_CONF_REG(uart == 0 ? ESP32_UART0_BASE : ESP32_UART1_BASE);
#else
		bool enabled = (ESP32_REG32(0x60096000UL + uart * 12U) & 1U) != 0U;
		uint32_t serial = ESP32_REG32(0x60096004UL + uart * 12U);
#endif
		if (enabled && ((serial >> 20) & 3U) != 3U)
		{
			return true;
		}
	}
	return g_McuOsc.bUSBClk ||
		(ESP32_REG32(CLOCK_TIMG0) & (1UL << 31)) != 0U ||
		(ESP32_REG32(CLOCK_TIMG1) & (1UL << 31)) != 0U ||
		(ESP32_REG32(CLOCK_TIMG0 + 0x48U) & ((1UL << 31) | (1UL << 14))) != 0U ||
		(ESP32_REG32(CLOCK_TIMG1 + 0x48U) & ((1UL << 31) | (1UL << 14))) != 0U;
}

// Keep the mux write and readback in instruction RAM. No flash clock changes.
static __attribute__((section(".iram.text.esp32_clock"), noinline))
void Esp32ClockWrite(uint32_t Root)
{
	CLOCK_ROOT_REG = Root;
	(void)CLOCK_ROOT_REG;
}

static void Esp32ClockUpdate(void)
{
	SystemCoreClockUpdate();
	Esp32RomClockUpdate((uint32_t)SystemCoreClockPeriodus);
#if defined(ESP32C3)
	uint32_t apb = SystemPeriphClockGet(0) >> 12;

	ESP32_REG32(0x600080BCUL) = apb | (apb << 16);
#endif
}

// Internal PLL setup shared by startup and the configured XTAL CPU profile.
// The 160 MHz profile uses a 40 MHz crystal; C6 caps its MSPI PLL input at 80 MHz.
// At least 10 us even at 160 MHz. Keep oscillator settling independent of
// flash code and of the cycle counter during early startup.
static __attribute__((section(".iram.text.esp32_pll"), noinline))
void Esp32PllDelay(void)
{
	uint32_t count = 1600U;
	__asm volatile (
		"1: nop\n\t"
		"addi %0, %0, -1\n\t"
		"bnez %0, 1b"
		: "+r"(count) : : "memory");
}

static __attribute__((section(".iram.text.esp32_pll"), noinline))
bool Esp32PllTransfer(uint8_t Addr, uint8_t *pValue, bool Write)
{
#if defined(ESP32C3)
	if (Write)
	{
		((void (*)(uint8_t, uint8_t, uint8_t, uint8_t))0x4000195CUL)(0x66U, 0U, Addr, *pValue);
	}
	else
	{
		*pValue = ((uint8_t (*)(uint8_t, uint8_t, uint8_t))0x40001954UL)(0x66U, 0U, Addr);
	}
	return true;
#else
	uintptr_t ctrl = 0x600AF800UL + ((ESP32_REG32(0x600AF820UL) & (1UL << 9)) ? 0U : 4U);
	uint32_t selection = ESP32_REG32(0x600AF81CUL) & 0x00FFFFFFU;
	uint32_t command = 0x66U | ((uint32_t)Addr << 8);
	bool ready = false;

	ESP32_REG32(0x600AF81CUL) = 0x00FFFF7FU;
	if (Write)
	{
		command |= (1UL << 24) | ((uint32_t)*pValue << 16);
	}
	for (unsigned phase = 0U; phase < 2U; phase++)
	{
		unsigned count = 100000U;
		while ((ESP32_REG32(ctrl) & (1UL << 25)) != 0U && --count != 0U)
		{
		}
		if (count == 0U)
		{
			break;
		}
		if (phase == 0U)
		{
			ESP32_REG32(ctrl) = command;
		}
		else
		{
			ready = true;
			if (!Write)
			{
				*pValue = (uint8_t)(ESP32_REG32(ctrl) >> 16);
			}
		}
	}
	ESP32_REG32(0x600AF81CUL) = selection;
	return ready;
#endif
}

static __attribute__((section(".iram.text.esp32_pll"), noinline))
bool Esp32PllWrite(uint8_t Addr, uint8_t Mask, uint8_t Value)
{
	uint8_t value;
	if (!Esp32PllTransfer(Addr, &value, false))
	{
		return false;
	}
	value = (value & ~Mask) | Value;
	if (!Esp32PllTransfer(Addr, &value, true))
	{
		return false;
	}
	return Esp32PllTransfer(Addr, &value, false) && (value & Mask) == Value;
}

__attribute__((section(".iram.text.esp32_pll"), noinline))
bool Esp32PllClockSelect(void)
{
	uintptr_t state = DisableInterrupt();
	uint32_t root = CLOCK_ROOT_REG;
	bool ready = false;
	OscDesc_t previous = g_McuOsc.CoreOsc;
#if defined(ESP32C3)
	uint32_t originalCpu = ESP32_REG32(0x600C0008UL);
#else
	uint32_t originalCpu = CLOCK_CPU_REG;
	uint32_t originalAhb = CLOCK_AHB_REG;
	uint32_t originalFlash = ESP32_REG32(0x6009601CUL);
#endif

	if (SystemCoreClockGet() < 1000000U ||
		(root & CLOCK_SOURCE_MASK) == CLOCK_SOURCE_MASK)
	{
		EnableInterrupt(state);
		return false;
	}
	if ((root & CLOCK_SOURCE_MASK) == (1UL << CLOCK_SOURCE_SHIFT))
	{
#if defined(ESP32C3)
		ESP32_REG32(0x600C0008UL) = (ESP32_REG32(0x600C0008UL) & ~3U) | 1U;
#else
		CLOCK_CPU_REG &= ~((0xFFUL << 8) | (1UL << 16));
#endif
		Esp32ClockUpdate();
		ready = SystemCoreClockGet() == 160000000U;
		if (!ready)
		{
#if defined(ESP32C3)
			ESP32_REG32(0x600C0008UL) = originalCpu;
#else
			CLOCK_CPU_REG = originalCpu;
#endif
			Esp32ClockUpdate();
		}
		EnableInterrupt(state);
		return ready;
	}
#if defined(ESP32C3)
	uint32_t options = ESP32_REG32(0x60008000UL);
	uint32_t analog = ESP32_REG32(0x6000E044UL);
	uint32_t calibration = ESP32_REG32(0x6000E040UL);
	uint32_t cpu = ESP32_REG32(0x600C0008UL);
	uint8_t status = 0U;

	if ((options & (1UL << 12)) != 0U ||
		((options & (1UL << 13)) == 0U && (root & CLOCK_SOURCE_MASK) != 0U))
	{
		EnableInterrupt(state);
		return false;
	}
	ESP32_REG32(0x6000E044UL) = analog & ~(1UL << 17);
	if ((options & ((1UL << 10) | (1UL << 8) | (1UL << 6))) == 0U)
	{
		ready = Esp32PllTransfer(8U, &status, false) && (status & 0x80U) != 0U;
	}
	if (!ready)
	{
		ESP32_REG32(0x60008000UL) = options & ~((1UL << 10) | (1UL << 8) | (1UL << 6));
		ESP32_REG32(0x6000E040UL) = (calibration & ~(1UL << 2)) | (1UL << 3);
		ready = Esp32PllWrite(4U, 0xFFU, 0x6BU) &&
			Esp32PllWrite(2U, 0xFFU, 0x50U) && Esp32PllWrite(3U, 0xFFU, 8U) &&
			Esp32PllWrite(5U, 0x77U, 0U) && Esp32PllWrite(6U, 0xFFU, 0x63U) &&
			Esp32PllWrite(9U, 3U, 2U);
		if (ready)
		{
			ready = false;
			for (unsigned count = 0U; count < 1000U; count++)
			{
				if (Esp32PllTransfer(8U, &status, false) && (status & 0x80U) != 0U)
				{
					ready = true;
					break;
				}
				Esp32PllDelay();
			}
		}
		if (ready)
		{
			cpu |= 1UL << 2;
		}
	}
	if (ready)
	{
		ESP32_REG32(0x600C0008UL) = (cpu & ~3U) | 1U;
		Esp32ClockWrite((root & ~(CLOCK_SOURCE_MASK | 0x3FFU)) | (1UL << CLOCK_SOURCE_SHIFT));
	}
	else
	{
		ESP32_REG32(0x60008000UL) = options;
		ESP32_REG32(0x6000E040UL) = calibration;
	}
	ESP32_REG32(0x6000E044UL) = analog;
#else
	uint32_t clock = ESP32_REG32(0x600AF018UL);
	uint32_t force = ESP32_REG32(0x600AF01CUL);
	uint32_t source = ESP32_REG32(0x600AF010UL);
	uint32_t calibration = ESP32_REG32(0x600AF818UL);
	bool running = (ESP32_REG32(0x600B0194UL) & (1UL << 30)) != 0U;

	if ((ESP32_REG32(0x600B0194UL) & 2U) == 0U)
	{
		EnableInterrupt(state);
		return false;
	}
	ready = running;
	if (!running)
	{
		ESP32_REG32(0x600B00CCUL) = (1UL << 28) | (1UL << 29) | (1UL << 30);
		ESP32_REG32(0x600B00CCUL) = 1UL << 25;
		ESP32_REG32(0x600AF018UL) = clock | (1UL << 2);
		ESP32_REG32(0x600AF01CUL) = force | (1UL << 2);
		ESP32_REG32(0x600AF010UL) = source | 1U;
		ESP32_REG32(0x600AF818UL) = (calibration & ~(1UL << 2)) | (1UL << 3);
		ready = Esp32PllWrite(2U, 0xFFU, 0x50U) && Esp32PllWrite(3U, 0xFFU, 8U) &&
			Esp32PllWrite(5U, 0x77U, 0U) && Esp32PllWrite(6U, 0xFFU, 0x73U) &&
			Esp32PllWrite(9U, 3U, 2U);
		if (ready)
		{
			ready = false;
			for (unsigned count = 0U; count < 1000U; count++)
			{
				if (ESP32_REG32(0x600AF818UL) & (1UL << 24))
				{
					ready = true;
					break;
				}
				Esp32PllDelay();
			}
		}
		if (ready)
		{
			Esp32PllDelay();
			ESP32_REG32(0x600AF818UL) = (ESP32_REG32(0x600AF818UL) & ~(1UL << 3)) | (1UL << 2);
		}
		else
		{
			ESP32_REG32(0x600AF818UL) = calibration;
			ESP32_REG32(0x600B00CCUL) = 1U | (1UL << 4) | (1UL << 5);
		}
		ESP32_REG32(0x600AF010UL) = source;
		ESP32_REG32(0x600AF01CUL) = force;
		ESP32_REG32(0x600AF018UL) = clock;
	}
	if (ready && ((root >> 8) & 0xFFU) == 2U)
	{
		uint32_t flash = ESP32_REG32(0x6009601CUL);
		// MSPI's PLL input must not exceed 80 MHz. Preserve a slower setting.
		if (((flash >> 8) & 0xFFU) < 5U)
		{
			ESP32_REG32(0x6009601CUL) = (flash & ~(0xFFUL << 8)) | (5UL << 8);
		}
		CLOCK_CPU_REG &= ~((0xFFUL << 8) | (1UL << 16));
		// Keep AHB at 40 MHz and APB at or below it while selecting PLL.
		CLOCK_AHB_REG = (CLOCK_AHB_REG & ~(0xFFUL << 8)) | (3UL << 8);
		Esp32ClockWrite((root & ~CLOCK_SOURCE_MASK) | (1UL << CLOCK_SOURCE_SHIFT));
	}
	else
	{
		ready = false;
	}
#endif
	if (ready)
	{
		g_McuOsc.CoreOsc.Type = OSC_TYPE_XTAL;
		g_McuOsc.CoreOsc.Freq = 40000000U;
		Esp32ClockUpdate();
		ready = SystemCoreClockGet() == 160000000U;
	}
	if (!ready)
	{
		Esp32ClockWrite(root);
#if defined(ESP32C3)
		ESP32_REG32(0x600C0008UL) = originalCpu;
#else
		CLOCK_CPU_REG = originalCpu;
		CLOCK_AHB_REG = originalAhb;
		ESP32_REG32(0x6009601CUL) = originalFlash;
#endif
		g_McuOsc.CoreOsc = previous;
		Esp32ClockUpdate();
	}
	EnableInterrupt(state);
	return ready;
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	if ((ClkSrc != OSC_TYPE_RC && ClkSrc != OSC_TYPE_XTAL) ||
		(ClkSrc == OSC_TYPE_XTAL && OscFreq != 0U && OscFreq != 40000000UL))
	{
		return false;
	}
	uintptr_t state = DisableInterrupt();
	uint32_t root = CLOCK_ROOT_REG;
	uint32_t source = ClkSrc == OSC_TYPE_RC ? 2U : 0U;
	uint32_t power = CLOCK_POWER_REG;
	uint32_t frequency = ClkSrc == OSC_TYPE_RC ? CLOCK_RC_FREQ : 40000000UL;
#if defined(ESP32C3)
	uint32_t wait = ESP32_REG32(0x6000801CUL);
#else
	uint32_t cpu = CLOCK_CPU_REG;
	uint32_t ahb = CLOCK_AHB_REG;
#endif
	if (SystemCoreClockGet() < 1000000U || Esp32ClockBusy())
	{
		EnableInterrupt(state);
		return false;
	}
	if (ClkSrc == OSC_TYPE_XTAL)
	{
#if defined(ESP32C3)
		uint32_t options = ESP32_REG32(0x60008000UL);
		bool running = !(options & (1UL << 12)) &&
			((options & (1UL << 13)) || (root & CLOCK_SOURCE_MASK) != (2UL << CLOCK_SOURCE_SHIFT));
#else
		bool running = (ESP32_REG32(0x600B0194UL) & 2U) != 0U;
#endif
		if (!running)
		{
			EnableInterrupt(state);
			return false;
		}
	}
#if defined(ESP32C3_CPU_160MHZ) || defined(ESP32C6_CPU_160MHZ)
	if (ClkSrc == OSC_TYPE_XTAL)
	{
		bool ready = Esp32PllClockSelect();
		EnableInterrupt(state);
		return ready;
	}
#endif
	if (ClkSrc == OSC_TYPE_RC)
	{
#if defined(ESP32C3)
		CLOCK_POWER_REG = (power | (1UL << 26) | (1UL << 10)) &
			~((1UL << 25) | (1UL << 6));
		ESP32_REG32(0x6000801CUL) = (wait & ~(0xFFUL << 6)) | (5UL << 6);
#else
		CLOCK_POWER_REG = power | (1UL << 30);
#endif
#if defined(ESP32C3)
		bool enabled = (CLOCK_POWER_REG & (1UL << 6)) == 0U;
#else
		bool enabled = (CLOCK_POWER_REG & (1UL << 30)) != 0U;
#endif
		if (!enabled)
		{
			CLOCK_POWER_REG = power;
#if defined(ESP32C3)
			ESP32_REG32(0x6000801CUL) = wait;
#endif
			EnableInterrupt(state);
			return false;
		}
		// Allow the oscillator to settle before measuring it.
		usDelay(100);
#if defined(ESP32C3)
		frequency = Esp32C3FastClockMeasure();
#else
		frequency = Esp32C6FastClockMeasure();
#endif
		if (frequency < 10000000U || frequency > 30000000U)
		{
			CLOCK_POWER_REG = power;
#if defined(ESP32C3)
			ESP32_REG32(0x6000801CUL) = wait;
#endif
			EnableInterrupt(state);
			return false;
		}
	}
#if defined(ESP32C3)
	uint32_t next = (root & ~(CLOCK_SOURCE_MASK | 0x3FFU)) |
		(source << CLOCK_SOURCE_SHIFT);
#else
	// The root low-speed divider is fixed at one. Keep all PLL dividers.
	if ((root & 0xFFU) != 0U)
	{
		CLOCK_POWER_REG = power;
		EnableInterrupt(state);
		return false;
	}
	CLOCK_CPU_REG = cpu & ~0xFFU;
	CLOCK_AHB_REG = ahb & ~0xFFU;
	uint32_t next = (root & ~CLOCK_SOURCE_MASK) | (source << CLOCK_SOURCE_SHIFT);
#endif
	Esp32ClockWrite(next);
	bool ready = CLOCK_ROOT_REG == next;
#if defined(ESP32C6)
	ready = ready && (CLOCK_CPU_REG & 0xFFU) == 0U &&
		(CLOCK_AHB_REG & 0xFFU) == 0U;
#endif

	if (ready)
	{
		g_McuOsc.CoreOsc.Type = ClkSrc;
		g_McuOsc.CoreOsc.Freq = frequency;
	}
	else
	{
		Esp32ClockWrite(root);
		CLOCK_POWER_REG = power;
#if defined(ESP32C3)
		ESP32_REG32(0x6000801CUL) = wait;
#else
		CLOCK_CPU_REG = cpu;
		CLOCK_AHB_REG = ahb;
#endif
	}
	Esp32ClockUpdate();
	EnableInterrupt(state);
	return ready;
}

uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	if (Idx != 0 || Freq == 0U)
	{
		return 0;
	}
	uintptr_t state = DisableInterrupt();
	uint32_t root = CLOCK_ROOT_REG;
	uint32_t source = (root & CLOCK_SOURCE_MASK) >> CLOCK_SOURCE_SHIFT;
	uint32_t actual = SystemPeriphClockGet(0);
	uint32_t best = 1;
	uint32_t error = UINT32_MAX;
	uint32_t frequency;
#if defined(ESP32C3)
	if (source == 1U)
	{
		// APB has one fixed rate while PLL feeds the CPU.
		EnableInterrupt(state);
		return actual;
	}
	uint32_t input = source == 0U ? 40000000UL :
		(g_McuOsc.CoreOsc.Type == OSC_TYPE_RC ? g_McuOsc.CoreOsc.Freq : CLOCK_RC_FREQ);
	uint32_t maximum = input / 1000000U;
#else
	uint32_t apb = CLOCK_APB_REG;
	uint32_t apbDivider = (apb >> 8) & 0xFFU;
	uint32_t ahbDivider = source == 1U ? (CLOCK_AHB_REG >> 8) & 0xFFU :
		CLOCK_AHB_REG & 0xFFU;
	bool valid = apbDivider == 0U || apbDivider == 1U || apbDivider == 3U;

	if (source == 1U)
	{
		valid = valid && ((root >> 8) & 0xFFU) == 2U &&
			(ahbDivider == 3U || ahbDivider == 7U || ahbDivider == 15U);
	}
	else
	{
		valid = valid && (root & 0xFFU) == 0U && ahbDivider <= 31U &&
			((ahbDivider + 1U) & ahbDivider) == 0U;
	}
	if (!valid)
	{
		EnableInterrupt(state);
		return 0;
	}
	// Derive the input from the source, not a rounded APB readback.
	uint32_t rc = g_McuOsc.CoreOsc.Type == OSC_TYPE_RC ?
		g_McuOsc.CoreOsc.Freq : CLOCK_RC_FREQ;
	uint32_t input = source == 1U ? 480000000UL /
		((((root >> 8) & 0xFFU) + 1U) * (ahbDivider + 1U)) :
		(source == 0U ? 40000000UL : rc) / (ahbDivider + 1U);
	uint32_t maximum = 4;
#endif
	if (actual == 0U || source == 3U)
	{
		EnableInterrupt(state);
		return 0;
	}
	for (uint32_t div = 1; div <= maximum;
#if defined(ESP32C3)
		div++)
#else
		div *= 2)
#endif
	{
		uint32_t rate = input / div;
		uint32_t difference = rate > Freq ? rate - Freq : Freq - rate;

		if (difference < error)
		{
			best = div;
			error = difference;
		}
	}
	frequency = input / best;
	if (frequency != actual)
	{
		if (Esp32ClockBusy())
		{
			EnableInterrupt(state);
			return 0;
		}
#if defined(ESP32C3)
		// Hardware requires clearing PRE_DIV before selecting another divider.
		Esp32ClockWrite(root & ~0x3FFU);
		Esp32ClockWrite((root & ~0x3FFU) | (best - 1U));
#else
		CLOCK_APB_REG = (apb & ~(0xFFUL << 8)) | ((best - 1U) << 8);
#endif
		if (SystemPeriphClockGet(0) != frequency)
		{
#if defined(ESP32C3)
			Esp32ClockWrite(root);
#else
			CLOCK_APB_REG = apb;
#endif
			frequency = 0;
		}
		Esp32ClockUpdate();
	}
	EnableInterrupt(state);
	return frequency;
}
