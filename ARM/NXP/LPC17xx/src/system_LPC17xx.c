/*--------------------------------------------------------------------------
File   : system_LPC17xx.c

Author : Hoang Nguyen Hoan          Jan. 23, 2012

Desc   : Implementation of CMSIS Cortex-M3 Device for LPC17xx Device Series
		 Added SystemSetCoreClock for on the fly clock setting

Copyright (c) 2012, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "LPC17xx.h"
#include "system_LPC17xx.h"
#include "coredev/system_core_clock.h"

#define OSC_FREQ				OSC_FREQ_16MHZ  /* Installed oscillator frequency */
#define IRC_FREQ				(4000000UL)		/* Internal RC freq */
#define RTC_FREQ				(32768UL)		/* RTC frequency */
#define LPC17_OSC_WAIT_MAX		1000000U		/* Bounded status reads */
#define LPC17_PLL_WAIT_MAX		1000000U		/* Bounded status reads */

typedef struct {
	uint64_t Error;
	uint32_t Denominator;
	uint32_t PllCfg;
	uint32_t CpuDiv;
	uint32_t PllFreq;
	uint32_t UsbExact;
} Lpc17ClockConfig_t;

static const uint8_t s_Lpc17PclkDividers[] = {4, 1, 2, 8};

// UM10360 table 26: the only additional multipliers supported for RTC input.
static const uint16_t s_Lpc17RtcMultipliers[] = {
	4272, 4395, 4578, 4725, 4807, 5127, 5188, 5400, 5493, 5859,
	6042, 6075, 6104, 6409, 6592, 6750, 6836, 6866, 6958, 7050,
	7324, 7425, 7690, 7813, 7935, 8057, 8100, 8545, 8789, 9155,
	9613, 10254, 10376, 10986, 11719, 12085, 12207, 12817, 13184, 13672,
	13733, 13916, 14099, 14420, 14648, 15381, 15564, 15625, 15869, 16113,
	16479, 17578, 18127, 18311, 19226, 19775, 20508, 20599, 20874, 21149,
	21973, 23071, 23438, 23804, 24170
};

// Preserve the legacy IRC startup unless the application supplies its board descriptor.
__WEAK McuOsc_t g_McuOsc = {
	{OSC_TYPE_RC, IRC_FREQ, 10000, 0},
	{OSC_TYPE_XTAL, RTC_FREQ, 0, 0},
	false
};

/*
 * This variable indicates actual Core Clock (cclk) in Hz
 * Update this by calling SystemCoreClockUpdate.
 *
 * Default on reset, internal RC is selected
 */
uint32_t SystemCoreClock = IRC_FREQ;/*!< System Clock Frequency (Core Clock)*/
uint32_t SystemClkFreq = IRC_FREQ;	/* System clock frequency, reset default IRC */
extern uint32_t SystemMicroSecLoopCnt;
extern void (* const __Vectors[])(void);

// Optional initialized AHB RAM section supplied by the application linker.
extern unsigned char __data_ahb_loc__[] __attribute__((weak));
extern unsigned char __data_ahb_start__[] __attribute__((weak));
extern unsigned char __data_ahb_size__ __attribute__((weak));

// No system-control register access may interrupt the two feed writes.
static void Lpc17PllFeed(volatile uint32_t *pFeed)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	*pFeed = 0xAA;
	*pFeed = 0x55;
	__set_PRIMASK(primask);
}

// Match the other LPC ports: setters and SystemCoreClockUpdate maintain this value.
// Start of physical local SRAM, independent of application linker placement.
void *SystemRamStart(void)
{
	return (void *)LPC_RAM_BASE;
}

uint32_t SystemCoreClockGet(void)
{
	return SystemCoreClock;
}

uint32_t SystemPeriphClockGet(int Idx)
{
	uint32_t select;
	uint32_t div;

	if (Idx == LPC17_PCLK_RTC)
	{
		return SystemCoreClock / 8U;
	}
	// Reserved PCLKSEL fields have no peripheral clock to report.
	if (Idx < 0 || Idx >= 32 || Idx == 5 || Idx == 9 || Idx == 20 || Idx == 28)
	{
		return 0;
	}
	select = Idx < 16 ? LPC_SC->PCLKSEL0 : LPC_SC->PCLKSEL1;
	select = (select >> (2U * ((uint32_t)Idx & 15U))) & 3U;
	div = s_Lpc17PclkDividers[select];
	if (select == 3U && Idx >= LPC17_PCLK_CAN1 && Idx <= LPC17_PCLK_ACF)
	{
		div = 6;
	}

	return SystemCoreClock / div;
}

uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	uint32_t core = SystemCoreClock;
	uint32_t bestdiv = 0;
	uint32_t bestsel = 0;
	uint64_t besterror = 0;
	uint32_t mask;
	uint32_t value;
	uint32_t primask;
	volatile uint32_t *reg;
	bool can = Idx >= LPC17_PCLK_CAN1 && Idx <= LPC17_PCLK_ACF;

	if (Idx == LPC17_PCLK_RTC)
	{
		return core / 8U;
	}
	if (Idx < 0 || Idx >= 32 || Idx == 5 || Idx == 9 || Idx == 20 || Idx == 28 || core == 0)
	{
		return 0;
	}
	for (uint32_t sel = 0; sel < 4; sel++)
	{
		uint32_t div = can && sel == 3U ? 6U : s_Lpc17PclkDividers[sel];
		uint64_t target = (uint64_t)Freq * div;
		uint64_t error = target > core ? target - core : core - target;

		// Compare rational errors before truncating Hz; ties select the slower clock.
		if (bestdiv == 0 || error * bestdiv < besterror * div ||
			(error * bestdiv == besterror * div && div > bestdiv))
		{
			bestdiv = div;
			bestsel = sel;
			besterror = error;
		}
	}
	reg = Idx < 16 ? &LPC_SC->PCLKSEL0 : &LPC_SC->PCLKSEL1;
	mask = 3U << (2U * ((uint32_t)Idx & 15U));
	value = bestsel << (2U * ((uint32_t)Idx & 15U));
	if (can)
	{
		// CAN1, CAN2 and acceptance filtering must use the same divider.
		mask = 0xFC000000U;
		value = (bestsel << 26) | (bestsel << 28) | (bestsel << 30);
	}
	primask = __get_PRIMASK();
	__disable_irq();
	*reg = (*reg & ~mask) | value;
	__set_PRIMASK(primask);

	return core / bestdiv;
}

uint32_t GetSysClk(void)
{
	uint32_t sysclk;

	switch (LPC_SC->CLKSRCSEL & 0x03)
	{
		case CLKSRCSEL_MAIN:
			sysclk = SystemClkFreq;//OSC_FREQ;
			break;
		case CLKSRCSEL_RTC:
			sysclk = RTC_FREQ;
			break;
		default:
			sysclk = IRC_FREQ;
			break;
	}

	return sysclk;
}

/*
 * Update SystemCoreClock based on registers values
 */
void SystemCoreClockUpdate (void)
{
	uint64_t sysclk = GetSysClk();
	uint32_t pll0stat = LPC_SC->PLL0STAT;
	uint32_t n, m;

	if ((pll0stat & 0x03000000U) == 0x03000000U)
	{
		// PLL0 is enabled and connected. MSEL is bits 14:0; bit 15 is reserved.
		m = (pll0stat & 0x7FFFU) + 1U;
		n = ((pll0stat >> 16) & 0xFFU) + 1U;

		// FCCO = (2 * M * FIN) / N. The product can exceed 32 bits.
		sysclk = sysclk * 2U * m / n;
	}

	// Apply the CPU divider before narrowing the result.
	SystemCoreClock = (uint32_t)(sysclk / ((LPC_SC->CCLKCFG & 0xFFU) + 1U));
	SystemMicroSecLoopCnt = SystemCoreClock / 16000000U;
}

// For one PLL ratio, only the two dividers bracketing the request can win.
// M=0 considers the unmodified input with PLL0 bypassed.
static void Lpc17ClockCandidate(Lpc17ClockConfig_t *pBest, uint32_t Request,
	uint32_t Input, uint32_t M, uint32_t N)
{
	uint64_t numerator = M ? (uint64_t)Input * 2U * M : Input;
	uint32_t mindiv = 1;
	uint32_t div;
	uint32_t usb = 0;

	if (M != 0)
	{
		if (numerator < 275000000ULL * N || numerator > 550000000ULL * N)
		{
			return;
		}
		mindiv = (uint32_t)((numerator + (uint64_t)LPC17_MAX_CORE_FREQ * N - 1U) /
			((uint64_t)LPC17_MAX_CORE_FREQ * N));
		usb = numerator % (96000000ULL * N) == 0;
	}
	div = Request ? (uint32_t)(numerator / ((uint64_t)N * Request)) : 256U;
	if (div < mindiv)
	{
		div = mindiv;
	}
	if (div > 256U)
	{
		div = 256U;
	}
	for (uint32_t end = div < 256U ? div + 1U : div; div <= end; div++)
	{
		uint32_t denominator = N * div;
		uint64_t requested = (uint64_t)Request * denominator;
		uint64_t error = numerator > requested ? numerator - requested : requested - numerator;
		uint64_t left = error * pBest->Denominator;
		uint64_t right = pBest->Error * denominator;

		// Keep an equally accurate bypass choice; it does not need PLL lock.
		// USB-compatible CCO preference applies only between PLL choices.
		if (pBest->Denominator == 0 || left < right ||
			(left == right && pBest->PllFreq != 0 &&
			(M == 0 || usb > pBest->UsbExact)))
		{
			pBest->Error = error;
			pBest->Denominator = denominator;
			pBest->PllCfg = M ? ((N - 1U) << 16) | (M - 1U) : 0;
			pBest->CpuDiv = div;
			pBest->PllFreq = M ? (uint32_t)(numerator / N) : 0;
			pBest->UsbExact = usb;
		}
	}
}

static Lpc17ClockConfig_t Lpc17ClockSelect(uint32_t Request, uint32_t Input)
{
	Lpc17ClockConfig_t best = {0};

	Lpc17ClockCandidate(&best, Request, Input, 0, 1);
	for (uint32_t n = 1; n <= 32; n++)
	{
		if (Input == RTC_FREQ)
		{
			for (unsigned i = 0; i < sizeof(s_Lpc17RtcMultipliers) /
				sizeof(s_Lpc17RtcMultipliers[0]); i++)
			{
				Lpc17ClockCandidate(&best, Request, Input, s_Lpc17RtcMultipliers[i], n);
			}
		}
		else
		{
			for (uint32_t m = 6; m <= 512; m++)
			{
				Lpc17ClockCandidate(&best, Request, Input, m, n);
			}
		}
	}

	return best;
}

static uint32_t Lpc17CoreClockSet(uint32_t CoreFreq, int ClkFreq, uint32_t clksrcsel, bool UsePll)
{
	uint32_t temp = 0;
	Lpc17ClockConfig_t clock;

	// ClkFreq describes a physical input, not the requested CPU rate.
	if (ClkFreq != (int)RTC_FREQ && (ClkFreq < 1000000 || ClkFreq > 25000000))
	{
		return 0;
	}

	if (UsePll)
	{
		clock = Lpc17ClockSelect(CoreFreq, (uint32_t)ClkFreq);
	}
	else
	{
		// Startup recovery must not depend on a working PLL.
		memset(&clock, 0, sizeof(clock));
		clock.CpuDiv = 1;
	}

	// Six flash clocks are safe throughout clock-source and divider changes.
	LPC_SC->FLASHCFG = (LPC_SC->FLASHCFG & ~0xF000U) | 0x5000U;

	/* Init system clock */

	if (clksrcsel == CLKSRCSEL_MAIN)
	{
		uint32_t wait = LPC17_OSC_WAIT_MAX;

		// Select MAIN clk source
		// OSCEN - Oscillator enable
		temp = 0x20;
		clksrcsel = CLKSRCSEL_MAIN;
		if (ClkFreq > 15000000)
			temp |= 0x10;	/* Oscillator range 15MHz-25MHz */

		LPC_SC->SCS = temp;
		// OSCSTAT (bit 6) must be set before selecting the main oscillator.
		while ((LPC_SC->SCS & (1U << 6)) == 0U)
		{
			if (--wait == 0U)
			{
				// The source/divider/PLL and clock globals have not changed.
				return 0;
			}
		}

	}


	// Disconnect the CPU before disabling PLL0 (UM10360 section 4.5.13).
	if ((LPC_SC->PLL0STAT & (1U << 25)) != 0U ||
		(LPC_SC->PLL0CON & 2U) != 0U)
	{
		LPC_SC->PLL0CON = 1;
		Lpc17PllFeed(&LPC_SC->PLL0FEED);
		temp = LPC17_PLL_WAIT_MAX;
		while ((LPC_SC->PLL0STAT & (1U << 25)) != 0U)
		{
			if (--temp == 0U)
			{
				// Keep a possibly active CPU clock enabled and report its rate.
				SystemCoreClockUpdate();

				return 0;
			}
		}
	}

	LPC_SC->PLL0CON = 0;
	Lpc17PllFeed(&LPC_SC->PLL0FEED);
	temp = LPC17_PLL_WAIT_MAX;
	while ((LPC_SC->PLL0STAT & (3U << 24)) != 0U)
	{
		if (--temp == 0U)
		{
			SystemCoreClockUpdate();

			return 0;
		}
	}

	SystemClkFreq = ClkFreq;

	LPC_SC->CCLKCFG = clock.CpuDiv - 1U;
	LPC_SC->CLKSRCSEL = clksrcsel;
	if (clock.PllFreq == 0)
	{
		// The divided input is closer; leave PLL0 stopped and report bypass.
		goto clock_ready;
	}

	LPC_SC->PLL0CFG = clock.PllCfg;
	Lpc17PllFeed(&LPC_SC->PLL0FEED);

	LPC_SC->PLL0CON   = 0x01;
	Lpc17PllFeed(&LPC_SC->PLL0FEED);
	// PLOCK0 is unreliable outside 100 kHz..20 MHz reference frequency.
	temp = ((clock.PllCfg >> 16) & 0xFFU) + 1U;
	if ((uint32_t)ClkFreq < 100000U * temp ||
		(uint32_t)ClkFreq > 20000000U * temp)
	{
		volatile uint32_t wait;

		// CPU still runs at FIN / CpuDiv. UM10360 4.5.6 requires
		// 200 / FREF seconds below 100 kHz, or 500 us above 20 MHz.
		if ((uint32_t)ClkFreq < 100000U * temp)
		{
			wait = (200U * temp + clock.CpuDiv - 1U) / clock.CpuDiv;
		}
		else
		{
			wait = ((uint32_t)ClkFreq + 2000U * clock.CpuDiv - 1U) /
				(2000U * clock.CpuDiv);
		}
		// A volatile decrement consumes at least one CPU cycle per count.
		// Loop overhead and interrupts only extend this minimum delay.
		while (wait != 0)
		{
			__NOP();
			wait--;
		}
	}
	else
	{
		// Within the reliable range, do not connect without reported lock.
		temp = LPC17_PLL_WAIT_MAX;
		while ((LPC_SC->PLL0STAT & (1U << 26)) == 0U)
		{
			if (--temp == 0U)
			{
				SystemCoreClockUpdate();

				return 0;
			}
		}
	}

	LPC_SC->PLL0CON   = 0x03;
	Lpc17PllFeed(&LPC_SC->PLL0FEED);
	temp = LPC17_PLL_WAIT_MAX;
	while ((LPC_SC->PLL0STAT & (1U << 25)) == 0U)
	{
		if (--temp == 0U)
		{
			// Cancel a pending connection, but keep PLL0 enabled in case
			// the hardware connects late or cannot acknowledge disconnect.
			LPC_SC->PLL0CON = 0x01;
			Lpc17PllFeed(&LPC_SC->PLL0FEED);

			temp = LPC17_PLL_WAIT_MAX;
			while ((LPC_SC->PLL0STAT & (1U << 25)) != 0U)
			{
				if (--temp == 0U)
				{
					break;
				}
			}
			SystemCoreClockUpdate();

			return 0;
		}
	}

	// USB needs MAIN input and an even division to exactly 48 MHz.
	// A rounded CPU rate alone does not establish a usable USB clock.
	if (clksrcsel == CLKSRCSEL_MAIN && clock.UsbExact)
	{
		LPC_SC->USBCLKCFG = clock.PllFreq / 48000000U - 1U;
	}

clock_ready:
	SystemCoreClockUpdate();

	// Reduce access time only after the resulting CPU clock is known.
	// FLASHTIM is the number of access clocks minus one (UM10360 table 49).
	temp = 0;
	if (SystemCoreClock > 0U)
	{
		temp = (SystemCoreClock - 1U) / 20000000U;
	}
	if (temp > 5U)
	{
		temp = 5U;
	}
	LPC_SC->FLASHCFG = (LPC_SC->FLASHCFG & ~0xF000U) | (temp << 12);

	return SystemCoreClock;
}

uint32_t SystemCoreClockSet(uint32_t CoreFreq, int ClkFreq)
{
	// Preserve the frequency-based legacy source selection.
	uint32_t source = ClkFreq == (int)IRC_FREQ ? CLKSRCSEL_IRC :
		(ClkFreq == (int)RTC_FREQ ? CLKSRCSEL_RTC : CLKSRCSEL_MAIN);

	return Lpc17CoreClockSet(CoreFreq, ClkFreq, source, true);
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	uint32_t source;

	if (ClkSrc == OSC_TYPE_RC && OscFreq == IRC_FREQ)
	{
		source = CLKSRCSEL_IRC;
	}
	else if ((ClkSrc == OSC_TYPE_XTAL || ClkSrc == OSC_TYPE_TCXO) &&
		OscFreq >= 1000000U && OscFreq <= 25000000U)
	{
		// XTAL1 external drive uses the same oscillator enable/ready sequence
		// as a crystal (UM10360 4.3.2); no separate bypass bit is present.
		source = CLKSRCSEL_MAIN;
	}
	else
	{
		return false;
	}

	if (Lpc17CoreClockSet(CORE_FREQ, (int)OscFreq, source, true) == 0)
	{
		return false;
	}
	if (ClkSrc == OSC_TYPE_RC && g_McuOsc.CoreOsc.Type != OSC_TYPE_RC)
	{
		// External accuracy and crystal load do not describe the internal RC.
		// Retain application calibration when the descriptor already is RC.
		g_McuOsc.CoreOsc.Accuracy = 10000;
		g_McuOsc.CoreOsc.LoadCap = 0;
	}
	g_McuOsc.CoreOsc.Type = ClkSrc;
	g_McuOsc.CoreOsc.Freq = OscFreq;

	return true;
}

uint32_t SystemSetCoreClock(uint32_t CoreFreq, int ClkSel)
{
	int clkfreq;

	switch (ClkSel)
	{
		case CLKSRCSEL_IRC:
			clkfreq = IRC_FREQ;
			break;
		case CLKSRCSEL_MAIN:
			clkfreq = OSC_FREQ;
			break;
		case CLKSRCSEL_RTC:
			clkfreq = RTC_FREQ;
			break;
		default:
			return 0;
	}

	return SystemCoreClockSet(CoreFreq, clkfreq);
}

void SystemConnectPLL1(void)
{
	uint32_t sysclk = SystemClkFreq;
	uint32_t val;
	uint32_t wait;

	// PLL1 is clocked only by the main oscillator. This legacy API knows
	// its frequency only while MAIN is the selected PLL0 input.
	if ((LPC_SC->CLKSRCSEL & 3U) != CLKSRCSEL_MAIN ||
		(LPC_SC->SCS & 0x60U) != 0x60U)
	{
		return;
	}
	if (sysclk != 12000000U && sysclk != 16000000U && sysclk != 24000000U)
	{
		return;
	}

	// Configuration may only change after PLL1 is disabled and disconnected.
	LPC_SC->PLL1CON = 0;
	Lpc17PllFeed(&LPC_SC->PLL1FEED);
	wait = LPC17_PLL_WAIT_MAX;
	while ((LPC_SC->PLL1STAT & (3U << 8)) != 0U)
	{
		if (--wait == 0U)
		{
			return;
		}
	}

	// USB requires 48 MHz. P=2 (PSEL=1) gives a 192 MHz CCO.
	val = (48000000U / sysclk - 1U) | (1U << 5);
	LPC_SC->PLL1CFG = val;
	Lpc17PllFeed(&LPC_SC->PLL1FEED);

	LPC_SC->PLL1CON = 0x01;
	Lpc17PllFeed(&LPC_SC->PLL1FEED);
	// USB must remain stopped until PLL1 is ready.
	wait = LPC17_PLL_WAIT_MAX;
	while ((LPC_SC->PLL1STAT & (1U << 10)) == 0U)
	{
		if (--wait == 0U)
		{
			LPC_SC->PLL1CON = 0;
			Lpc17PllFeed(&LPC_SC->PLL1FEED);

			return;
		}
	}

	LPC_SC->PLL1CON = 0x03;
	Lpc17PllFeed(&LPC_SC->PLL1FEED);
	// USB must remain stopped until PLL1 is ready.
	wait = LPC17_PLL_WAIT_MAX;
	while ((LPC_SC->PLL1STAT & (1U << 9)) == 0U)
	{
		if (--wait == 0U)
		{
			LPC_SC->PLL1CON = 0;
			Lpc17PllFeed(&LPC_SC->PLL1FEED);

			return;
		}
	}
}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
	size_t ahbsize = (size_t)&__data_ahb_size__;

	// Select this image's vectors, including when linked in the DFU slot.
	SCB->VTOR = (uint32_t)(uintptr_t)__Vectors;
	__DSB();
	__ISB();

	// ResetEntry initializes local SRAM before calling SystemInit.
	// Copy the separate AHB load image before application initialization.
	if (ahbsize != 0 && &__data_ahb_start__[0] != &__data_ahb_loc__[0])
	{
		memcpy(__data_ahb_start__, __data_ahb_loc__, ahbsize);
	}

	if (SystemCoreClockSelect(g_McuOsc.CoreOsc.Type, g_McuOsc.CoreOsc.Freq) == false)
	{
		// Recover from a missing external source or PLL failure on bare IRC.
		// The shared transition still requires PLL disconnect/stop acknowledgement.
		if (Lpc17CoreClockSet(IRC_FREQ, IRC_FREQ, CLKSRCSEL_IRC, false) != 0)
		{
			g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
			g_McuOsc.CoreOsc.Freq = IRC_FREQ;
			// The failed crystal's tolerance and load do not describe the IRC.
			g_McuOsc.CoreOsc.Accuracy = 10000;
			g_McuOsc.CoreOsc.LoadCap = 0;
		}
	}
}

