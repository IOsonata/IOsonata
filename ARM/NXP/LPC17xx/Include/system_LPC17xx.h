/*--------------------------------------------------------------------------
File   : system_LPC17xx.h

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
#ifndef __SYSTEM_LPC17xx_H__
#define __SYSTEM_LPC17xx_H__

#include <stdint.h>

/* system clock (sysclk) selections */
#define CLKSRCSEL_IRC		0	/* Internal RC oscillator */
#define CLKSRCSEL_MAIN		1	/* Main oscillator */
#define CLKSRCSEL_RTC		2	/* Realtime clock oscillator */

/**
 * LPC17xx indices for SystemPeriphClockGet/Set in coredev/system_core_clock.h.
 * Values 0..31 follow the two-bit PCLKSEL0/PCLKSEL1 fields; reserved holes
 * are invalid. RTC is its CCLK/8 register-interface clock, not its oscillator.
 * These are configured rates, independent of PCONP power gating.
 * Set selects the closest supported rate (ties choose the lower rate).
 * RTC is fixed; Set returns its current rate without changing the CPU clock.
 * Setting CAN1, CAN2 or ACF updates all three dividers together.
 * Stop affected peripherals first; drivers may restore their own dividers
 * during initialization. Serialize peripheral and CPU clock changes.
 */
typedef enum __Lpc17_Periph_Clock {
	LPC17_PCLK_WDT = 0,
	LPC17_PCLK_TIMER0 = 1,
	LPC17_PCLK_TIMER1 = 2,
	LPC17_PCLK_UART0 = 3,
	LPC17_PCLK_UART1 = 4,
	LPC17_PCLK_PWM1 = 6,
	LPC17_PCLK_I2C0 = 7,
	LPC17_PCLK_SPI = 8,
	LPC17_PCLK_SSP1 = 10,
	LPC17_PCLK_DAC = 11,
	LPC17_PCLK_ADC = 12,
	LPC17_PCLK_CAN1 = 13,
	LPC17_PCLK_CAN2 = 14,
	LPC17_PCLK_ACF = 15,
	LPC17_PCLK_QEI = 16,
	LPC17_PCLK_GPIOINT = 17,
	LPC17_PCLK_PCB = 18,
	LPC17_PCLK_I2C1 = 19,
	LPC17_PCLK_SSP0 = 21,
	LPC17_PCLK_TIMER2 = 22,
	LPC17_PCLK_TIMER3 = 23,
	LPC17_PCLK_UART2 = 24,
	LPC17_PCLK_UART3 = 25,
	LPC17_PCLK_I2C2 = 26,
	LPC17_PCLK_I2S = 27,
	LPC17_PCLK_RIT = 29,
	LPC17_PCLK_SYSCON = 30,
	LPC17_PCLK_MCPWM = 31,
	LPC17_PCLK_RTC = 32,
} LPC17_PCLK;

/* Crystal or Oscillator frequencies in Hz */
#define OSC_FREQ_12MHZ			(12000000UL)    /* LPCXpresso Oscillator frequency     */
#define OSC_FREQ_16MHZ			(16000000UL)    /* IMM-LPC1769 Oscillator frequency    */

/* UM10360: only LPC1759 and LPC1769 support 120 MHz. */
#if defined(LPC1759) || defined(LPC1769)
#define LPC17_MAX_CORE_FREQ	120000000UL
#else
#define LPC17_MAX_CORE_FREQ	100000000UL
#endif

/* Startup target may be lower than the device limit. */
#ifndef CORE_FREQ
#define CORE_FREQ			LPC17_MAX_CORE_FREQ
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
__attribute__((weak)) void SystemInit (void);

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock 
 *         retrieved from cpu registers.
 */
void SystemCoreClockUpdate (void);

/**
 *
 * @param CoreFreq 	- Core target frequency
 * @param ClkSel	- Clock source selection
 *						CLKSRCSEL_IRC - Internal osc
 *						CLKSRCSEL_MAIN - Main osc, existing 16 MHz default
 *						CLKSRCSEL_RTC -	Realtime osc
 *
 * @return
 * 		Return actual frequency in Hz, or zero on failure.
 *
 * @brief	Setup core clock (cclk) & update SystemCoreClock variable
 * Stop clock-dependent peripherals before changing the CPU clock.
 * For a different main oscillator frequency, use SystemCoreClockSet.
 */
uint32_t SystemSetCoreClock(uint32_t CoreFreq, int ClkSel);

/**
 * Configure the CPU clock using a physical input frequency in Hz.
 * ClkFreq is 4000000 for IRC, 32768 for RTC, or 1-25 MHz for MAIN.
 * The legacy 4000000 value selects IRC, not a 4 MHz main oscillator.
 * CoreFreq is the requested CPU frequency, distinct from ClkFreq.
 * Selects the closest nominal rate using PLL0 or the divided input,
 * capped at LPC17_MAX_CORE_FREQ. Zero selects the lowest representable rate.
 * The returned integer Hz value truncates any fractional Hz.
 * USBCLKCFG is updated only for MAIN-driven PLL0 at 288/384/480 MHz;
 * otherwise it is unchanged. CPU-clock success does not imply USB readiness.
 * USB also requires an accurate input and at least 18 MHz CPU clock.
 * Stop clock-dependent peripherals before changing the CPU clock.
 * Returns the actual CPU frequency, or zero on failure.
 */
uint32_t SystemCoreClockSet(uint32_t CoreFreq, int ClkFreq);

/**
 * Configure PLL1 for the fixed 48 MHz USB clock.
 * Requires MAIN selected and ready, with its 12, 16 or 24 MHz frequency
 * recorded by the clock setup. Other inputs leave PLL1 unchanged.
 * The caller must stop USB before configuring its clock.
 * Stops PLL1 before changing its configuration. Stop/lock/connect waits
 * are bounded; stop timeout preserves the configuration and disable request.
 * Lock/connect timeout requests PLL1 disable and returns.
 */
void SystemConnectPLL1(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_LPC17xx_H__ */

