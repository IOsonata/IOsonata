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

#define OSC_FREQ				OSC_FREQ_16MHZ  /* Installed oscillator frequency */
#define IRC_FREQ				(4000000UL)		/* Internal RC freq */
#define RTC_FREQ				(32768UL)		/* RTC frequency */
#define LPC17_OSC_WAIT_MAX		1000000U		/* Bounded status reads */
#define LPC17_PLL_WAIT_MAX		1000000U		/* Bounded status reads */

/* Select system clock oscillator source */
#define DEF_CLK_SRC			IRC_FREQ
//#define CLKSRCSEL_VAL		CLKSRCSEL_MAIN

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

uint32_t SystemCoreClockSet(uint32_t CoreFreq, int ClkFreq)
{
	uint32_t temp = 0;
	uint32_t sysclk = ClkFreq;
	uint32_t clksrcsel = CLKSRCSEL_IRC;	// Default internal RC

	// ClkFreq describes a physical input, not the requested CPU rate.
	// The legacy selector uses 4 MHz for IRC and 32768 Hz for RTC.
	if (ClkFreq != (int)RTC_FREQ && (ClkFreq < 1000000 || ClkFreq > 25000000))
	{
		return 0;
	}

	// Six flash clocks are safe throughout clock-source and divider changes.
	LPC_SC->FLASHCFG = (LPC_SC->FLASHCFG & ~0xF000U) | 0x5000U;

	/* Init system clock */

	if (ClkFreq != (int)IRC_FREQ && ClkFreq != (int)RTC_FREQ)
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
	else if (ClkFreq == (int)RTC_FREQ)
	{
		clksrcsel = CLKSRCSEL_RTC;
	}


	SystemClkFreq = ClkFreq;

	/* Setting PLL0 */

	/* find CCLKDIV & PLLCLK to get closes match */
	int32_t cclkdiv = 1;
	int32_t pllclk = 0;
	int32_t delta = 0, predelta = CoreFreq;
	int i;

	for (i = 1; i < 257; i++)
	{
		pllclk = CoreFreq * i;
		/* pllclk range is 275MHz - 550MHz */
		if (pllclk < 275000000UL)
			continue;
		if (pllclk > 550000000UL)
			break;
		/* try to select closest useable frequency for USB clock */
		delta = CoreFreq - (pllclk / 48000000UL) * 48000000UL / i;
		if (delta < predelta)
		{
			predelta = delta;
			cclkdiv = i;
		}
	}

	pllclk = CoreFreq * cclkdiv;

	/* Set cclk divider */
	LPC_SC->CCLKCFG = (cclkdiv - 1) & 0xff;
	LPC_SC->CLKSRCSEL = clksrcsel; //CLKSRCSEL_VAL;

	/* Find M & N value */
	/* Formula M = (FCCO × N) / (2 × FIN) */
	sysclk <<= 1;

	for (i = 1; i < 33; i++)
	{
		// FCCO * N can exceed 32 bits even for a valid CPU clock.
		uint64_t t = (uint64_t)pllclk * (uint32_t)i;
		temp = t / sysclk;
		if ((t % sysclk) == 0)
			break;
	}

	/* results : i is N, temp is M */
	i--;
	temp--;
	// MSEL occupies bits 14:0; preserve bit 12 and leave bit 15 clear.
	temp = ((i & 0xFFU) << 16) | (temp & 0x7FFFU);
	LPC_SC->PLL0CFG = temp;
	Lpc17PllFeed(&LPC_SC->PLL0FEED);

	LPC_SC->PLL0CON   = 0x01;
	Lpc17PllFeed(&LPC_SC->PLL0FEED);
	// Do not connect PLL0 if it fails to report lock.
	temp = LPC17_PLL_WAIT_MAX;
	while ((LPC_SC->PLL0STAT & (1U << 26)) == 0U)
	{
		if (--temp == 0U)
		{
			// Report the active input/divider instead of the requested PLL rate.
			SystemCoreClockUpdate();

			return 0;
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

	/* USBCLK divider */
	temp = (pllclk / 48000000) - 1;
	LPC_SC->USBCLKCFG = temp & 0xf;

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

	SystemCoreClockSet(CORE_FREQ, DEF_CLK_SRC);
}

