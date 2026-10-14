/**-------------------------------------------------------------------------
@file	g4_test.cpp

@brief	Host test of the STM32G4 system clock source.

Runs ARM/ST/STM32G4xx/src/system_stm32g4xx.c with the ST STM32G474 device
header. The peripheral address range is mapped as plain memory: the test
sets the ready flags itself. Register values only, no timing. The timer
driver is tested by tests/stm32/run_timer.py.

@author	Hoang Nguyen Hoan
@date	October 13, 2026

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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>

#include "stm32g4xx.h"
#include "coredev/system_core_clock.h"

TestScb_t g_TestScb;

// Const has internal linkage in C++ without extern
extern "C" void (* const __Vectors[4])(void);
extern "C" void (* const __Vectors[4])(void) = {};
extern "C" bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq);
extern "C" uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq);

static int s_Fail;
static int s_Checks;

#define CHECK(c)	do { s_Checks++; if (!(c)) { s_Fail++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

static bool s_NvicEn[128];
static uint32_t s_NvicPrio[128];

extern "C" void TestNvic(int Op, int IrqNo, uint32_t Prio)
{
	if (IrqNo < 0 || IrqNo >= 128)
	{
		return;
	}
	switch (Op)
	{
		case 0: s_NvicEn[IrqNo] = true; break;
		case 1: s_NvicEn[IrqNo] = false; break;
		case 3: s_NvicPrio[IrqNo] = Prio; break;
		default: break;
	}
}

// ---- System clock -----------------------------------------------------------

static void ClockReset(void)
{
	memset((void *)RCC, 0, sizeof(RCC_TypeDef));
	memset((void *)PWR, 0, sizeof(PWR_TypeDef));
	memset((void *)FLASH, 0, sizeof(FLASH_TypeDef));
	RCC->CR = RCC_CR_HSION | RCC_CR_HSIRDY;
	PWR->CR5 = PWR_CR5_R1MODE;
}

// PLL fields of PLLCFGR: source, M, N, R
static uint32_t Pll(uint32_t Src, uint32_t M, uint32_t N, uint32_t R)
{
	return Src | ((M - 1) << RCC_PLLCFGR_PLLM_Pos) | (N << RCC_PLLCFGR_PLLN_Pos) |
		   (((R >> 1) - 1) << RCC_PLLCFGR_PLLR_Pos) | RCC_PLLCFGR_PLLREN;
}

static void TestClock(void)
{
	// HSI16: 16 / 4 x 85 / 2 = 170 MHz, boost mode, 4 wait states
	ClockReset();
	RCC->CR |= RCC_CR_PLLRDY;
	RCC->CFGR = RCC_CFGR_SWS_PLL;
	g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
	g_McuOsc.CoreOsc.Freq = 16000000;
	SystemInit();
	CHECK(RCC->PLLCFGR == Pll(RCC_PLLCFGR_PLLSRC_HSI, 4, 85, 2));
	CHECK((RCC->CFGR & RCC_CFGR_SW) == RCC_CFGR_SW_PLL);
	CHECK((RCC->CFGR & (RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) == 0);
	CHECK((RCC->CR & RCC_CR_PLLON) != 0);
	CHECK((PWR->CR5 & PWR_CR5_R1MODE) == 0);
	CHECK((FLASH->ACR & FLASH_ACR_LATENCY) == 4);
	CHECK((FLASH->ACR & (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN)) == (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN));
	CHECK(SystemCoreClock == 170000000);
	CHECK(SystemPeriphClockGet(0) == 170000000 && SystemPeriphClockGet(1) == 170000000);
	CHECK(SystemPeriphClockGet(2) == 0);
	CHECK(g_TestScb.VTOR == (uint32_t)(uintptr_t)__Vectors);

	// HSE crystals of the boards: 24 MHz on the NUCLEO-G474RE, 8 and 12 MHz
	static const struct { uint32_t Freq, M; } hse[] = { { 24000000, 6 }, { 8000000, 2 }, { 12000000, 3 } };
	for (auto &h : hse)
	{
		ClockReset();
		RCC->CR |= RCC_CR_PLLRDY | RCC_CR_HSERDY;
		RCC->CFGR = RCC_CFGR_SWS_PLL;
		CHECK(SystemCoreClockSelect(OSC_TYPE_XTAL, h.Freq));
		CHECK(RCC->PLLCFGR == Pll(RCC_PLLCFGR_PLLSRC_HSE, h.M, 85, 2));
		CHECK((RCC->CR & (RCC_CR_HSEON | RCC_CR_HSEBYP)) == RCC_CR_HSEON);
		CHECK(SystemCoreClock == 170000000);
	}

	// External oscillator: HSE bypass
	ClockReset();
	RCC->CR |= RCC_CR_PLLRDY | RCC_CR_HSERDY;
	RCC->CFGR = RCC_CFGR_SWS_PLL;
	CHECK(SystemCoreClockSelect(OSC_TYPE_TCXO, 24000000));
	CHECK((RCC->CR & (RCC_CR_HSEON | RCC_CR_HSEBYP)) == (RCC_CR_HSEON | RCC_CR_HSEBYP));

	// No crystal: HSE never ready, back to HSI16 and recorded in g_McuOsc
	ClockReset();
	RCC->CR |= RCC_CR_PLLRDY;
	RCC->CFGR = RCC_CFGR_SWS_PLL;
	CHECK(SystemCoreClockSelect(OSC_TYPE_XTAL, 24000000));
	CHECK(RCC->PLLCFGR == Pll(RCC_PLLCFGR_PLLSRC_HSI, 4, 85, 2));
	CHECK((RCC->CR & RCC_CR_HSEON) == 0);
	CHECK(g_McuOsc.CoreOsc.Type == OSC_TYPE_RC && g_McuOsc.CoreOsc.Freq == 16000000);
	CHECK(SystemCoreClock == 170000000);

	// Out of the HSE range
	CHECK(SystemCoreClockSelect(OSC_TYPE_XTAL, 50000000) == false);
	CHECK(SystemCoreClockSelect(OSC_TYPE_XTAL, 3000000) == false);

	// PLL not ready: stays on HSI16
	ClockReset();
	RCC->CFGR = RCC_CFGR_SWS_HSI;
	g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
	SystemInit();
	CHECK((RCC->CR & RCC_CR_PLLON) == 0);
	CHECK((RCC->CFGR & RCC_CFGR_SW) == RCC_CFGR_SW_HSI);
	CHECK(SystemCoreClock == 16000000);

	// SystemCoreClockUpdate from the registers, HSE as SYSCLK and AHB divider
	g_McuOsc.CoreOsc.Freq = 24000000;
	RCC->CFGR = RCC_CFGR_SWS_HSE | RCC_CFGR_HPRE_DIV4;
	SystemCoreClockUpdate();
	CHECK(SystemCoreClock == 6000000);
	RCC->CFGR = RCC_CFGR_SWS_PLL;
	RCC->PLLCFGR = Pll(RCC_PLLCFGR_PLLSRC_HSE, 6, 75, 2);
	SystemCoreClockUpdate();
	CHECK(SystemCoreClock == 150000000);

	// Flash wait states below 150 MHz: range 1 normal mode
	ClockReset();
	RCC->CR |= RCC_CR_PLLRDY;
	g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
	g_McuOsc.CoreOsc.Freq = 16000000;
	SystemInit();
	CHECK((PWR->CR5 & PWR_CR5_R1MODE) == 0);

	// APB prescaler, closest of HCLK / 1, 2, 4, 8, 16
	SystemCoreClock = 160000000;
	static const struct { uint32_t Req, Got, Ppre; } apb[] = {
		{ 160000000, 160000000, 0 }, { 80000000, 80000000, 4 }, { 60000000, 80000000, 4 },
		{ 40000000, 40000000, 5 }, { 20000000, 20000000, 6 }, { 1000, 10000000, 7 }, { 0, 10000000, 7 },
		{ 200000000, 160000000, 0 } };
	for (auto &a : apb)
	{
		CHECK(SystemPeriphClockSet(1, a.Req) == a.Got);
		CHECK(((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos) == a.Ppre);
		CHECK(SystemPeriphClockGet(1) == a.Got);
	}
	CHECK(SystemPeriphClockSet(0, 40000000) == 40000000);
	CHECK(((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos) == 5);
	CHECK(SystemPeriphClockSet(2, 40000000) == 0);
}

int main()
{
	void *p = mmap((void *)0x40000000, 0x30000, PROT_READ | PROT_WRITE,
				   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

	if (p != (void *)0x40000000)
	{
		printf("mmap failed\n");
		return 1;
	}

	TestClock();

	printf("%d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
}
