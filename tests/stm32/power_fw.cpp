/**-------------------------------------------------------------------------
@file	power_fw.cpp

@brief	STM32 power functions test firmware

Linked with the production power functions, ARM/ST/src/system_power_stm32.cpp,
for run_power.py. The emulator calls the functions directly; this file gives
it the register addresses and bits of the part from the CMSIS device header,
g_PwrTestAddr, word index:
	0	family: 1 PWR CR/CSR (F0, F4, L1), 2 PWR CR1 to CR4 (L4, G4), 3 H7
	1	reset flag register (RCC CSR or RSR)
	2	RMVF
	3	PINRSTF
	4	power reset flag, PORRSTF or BORRSTF
	5	SFTRSTF
	6	IWDGRSTF or IWDG1RSTF
	7	LPWRRSTF
	8	RTC ISR, 0 none
	9	RTC ISR ALRAF
	10	first backup register, 0 none
	11	number of backup registers
	12	SCB SCR
	13	GPIOA, then GPIOB to GPIOI at 14 to 21, 0 when the port does not exist
	22	retention RAM base, 0 none
	23	retention RAM size
	24	retention RAM alias, 0 none
	25	DBP register
	26	DBP bit
	30 to 45	family registers and bits, see the code

@author	Hoang Nguyen Hoan
@date	Nov. 9, 2026

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
#include <stdint.h>

#include "stm32.h"
#include "coredev/system_core_clock.h"

#define TEST_ADDR(r)			((uint32_t)(uintptr_t)&(r))

#if defined(RCC_RSR_RMVF)
#define TEST_RST				TEST_ADDR(RCC->RSR), RCC_RSR_RMVF, RCC_RSR_PINRSTF, RCC_RSR_PORRSTF, \
								RCC_RSR_SFTRSTF, RCC_RSR_IWDG1RSTF, RCC_RSR_LPWRRSTF
#elif defined(RCC_CSR_PORRSTF)
#define TEST_RST				TEST_ADDR(RCC->CSR), RCC_CSR_RMVF, RCC_CSR_PINRSTF, RCC_CSR_PORRSTF, \
								RCC_CSR_SFTRSTF, RCC_CSR_IWDGRSTF, RCC_CSR_LPWRRSTF
#else
#define TEST_RST				TEST_ADDR(RCC->CSR), RCC_CSR_RMVF, RCC_CSR_PINRSTF, RCC_CSR_BORRSTF, \
								RCC_CSR_SFTRSTF, RCC_CSR_IWDGRSTF, RCC_CSR_LPWRRSTF
#endif

#if defined(RTC_ISR_ALRAF)
#define TEST_RTC				TEST_ADDR(RTC->ISR), RTC_ISR_ALRAF
#else
#define TEST_RTC				0, 0
#endif

#if defined(TAMP_BKP0R)
#define TEST_BKP				TEST_ADDR(TAMP->BKP0R), IOSONATA_STM32_NB_BKP
#elif defined(RTC_BKP0R)
#define TEST_BKP				TEST_ADDR(RTC->BKP0R), IOSONATA_STM32_NB_BKP
#else
#define TEST_BKP				0, 0
#endif

#if defined(GPIOA)
#define TEST_GPIOA				TEST_ADDR(*GPIOA)
#else
#define TEST_GPIOA				0
#endif
#if defined(GPIOB)
#define TEST_GPIOB				TEST_ADDR(*GPIOB)
#else
#define TEST_GPIOB				0
#endif
#if defined(GPIOC)
#define TEST_GPIOC				TEST_ADDR(*GPIOC)
#else
#define TEST_GPIOC				0
#endif
#if defined(GPIOD)
#define TEST_GPIOD				TEST_ADDR(*GPIOD)
#else
#define TEST_GPIOD				0
#endif
#if defined(GPIOE)
#define TEST_GPIOE				TEST_ADDR(*GPIOE)
#else
#define TEST_GPIOE				0
#endif
#if defined(GPIOF)
#define TEST_GPIOF				TEST_ADDR(*GPIOF)
#else
#define TEST_GPIOF				0
#endif
#if defined(GPIOG)
#define TEST_GPIOG				TEST_ADDR(*GPIOG)
#else
#define TEST_GPIOG				0
#endif
#if defined(GPIOH)
#define TEST_GPIOH				TEST_ADDR(*GPIOH)
#else
#define TEST_GPIOH				0
#endif
#if defined(GPIOI)
#define TEST_GPIOI				TEST_ADDR(*GPIOI)
#else
#define TEST_GPIOI				0
#endif

// Retention RAM of the reference manuals: SRAM2 of the L4 and G4 with the
// alias after SRAM1 (none on the L47x), backup SRAM of the H7
#if defined(STM32H753xx)
#define TEST_RAM				D3_BKPSRAM_BASE, 0x1000, 0
#elif defined(STM32L476xx)
#define TEST_RAM				0x10000000, 0x8000, 0
#elif defined(STM32L432xx)
#define TEST_RAM				0x10000000, 0x4000, 0x2000C000
#elif defined(STM32L496xx)
#define TEST_RAM				0x10000000, 0x10000, 0x20040000
#elif defined(STM32L4S9xx)
#define TEST_RAM				0x10000000, 0x10000, 0x20030000
#elif defined(STM32G474xx)
#define TEST_RAM				0x20014000, 0x4000, 0
#else
#define TEST_RAM				0, 0, 0
#endif

#if defined(PWR_CR1_DBP)
#define TEST_DBP				TEST_ADDR(PWR->CR1), PWR_CR1_DBP
#else
#define TEST_DBP				TEST_ADDR(PWR->CR), PWR_CR_DBP
#endif

// Family registers and bits from index 30
#if defined(IOSONATA_STM32_H7)
#define TEST_FAMILY				3
// 30 CR2, 31 CPUCR, 32 WKUPCR, 33 WKUPFR, 34 WKUPEPR, 35 BREN, 36 BRRDY,
// 37 PDDS_D1 | PDDS_D2 | PDDS_D3, 38 CSSF, 39 SBF
#define TEST_PWR				TEST_ADDR(PWR->CR2), TEST_ADDR(PWR->CPUCR), TEST_ADDR(PWR->WKUPCR), \
								TEST_ADDR(PWR->WKUPFR), TEST_ADDR(PWR->WKUPEPR), PWR_CR2_BREN, PWR_CR2_BRRDY, \
								PWR_CPUCR_PDDS_D1 | PWR_CPUCR_PDDS_D2 | PWR_CPUCR_PDDS_D3, PWR_CPUCR_CSSF, \
								PWR_CPUCR_SBF
#elif defined(PWR_SCR_CWUF)
#define TEST_FAMILY				2
// 30 CR1, 31 CR3, 32 CR4, 33 SR1, 34 SCR, 35 PUCRA, 36 LPMS, 37 LPMS
// Standby, 38 LPMS Shutdown, 39 RRS, 40 APC, 41 SBF, 42 WUFI, 43 CSBF, 44 CWUF
#define TEST_PWR				TEST_ADDR(PWR->CR1), TEST_ADDR(PWR->CR3), TEST_ADDR(PWR->CR4), \
								TEST_ADDR(PWR->SR1), TEST_ADDR(PWR->SCR), TEST_ADDR(PWR->PUCRA), PWR_CR1_LPMS, \
								PWR_CR1_LPMS_STANDBY, PWR_CR1_LPMS_SHUTDOWN, PWR_CR3_RRS, PWR_CR3_APC, \
								PWR_SR1_SBF, PWR_SR1_WUFI, PWR_SCR_CSBF, PWR_SCR_CWUF
#else
#define TEST_FAMILY				1
#if defined(PWR_CSR_EWUP1)
#define TEST_EWUP				PWR_CSR_EWUP1
#else
#define TEST_EWUP				PWR_CSR_EWUP
#endif
// 30 CR, 31 CSR, 32 CSBF, 33 CWUF, 34 PDDS, 35 SBF, 36 WUF, 37 EWUP1
#define TEST_PWR				TEST_ADDR(PWR->CR), TEST_ADDR(PWR->CSR), PWR_CR_CSBF, PWR_CR_CWUF, \
								PWR_CR_PDDS, PWR_CSR_SBF, PWR_CSR_WUF, TEST_EWUP
#endif

extern "C" __attribute__((used)) const uint32_t g_PwrTestAddr[] = {
	TEST_FAMILY, TEST_RST, TEST_RTC, TEST_BKP, TEST_ADDR(SCB->SCR),
	TEST_GPIOA, TEST_GPIOB, TEST_GPIOC, TEST_GPIOD, TEST_GPIOE, TEST_GPIOF, TEST_GPIOG, TEST_GPIOH, TEST_GPIOI,
	TEST_RAM, TEST_DBP, 0, 0, 0,
	TEST_PWR,
};

// Keeps the power functions the emulator calls, never run
extern "C" void ResetHandler(void)
{
	SystemRetRegWrite(SystemRetRegCount(), SystemRetRegRead((int)SystemResetReason()));
	(void)SystemWakePin(0, 0, 0);
	(void)SystemRamRetain(0, 0, true);
	SystemPowerOff();
}
