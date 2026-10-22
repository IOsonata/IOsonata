/**-------------------------------------------------------------------------
@file	rtc_wakeup_host.cpp

@brief	Host checks of STM32 RTC wakeup range and rounding

The production RTC driver runs against mapped H753 registers, with the
existing host core and delay shims. The RTC clock and calendar start in a
running state so Init needs no register transitions. This checks SetWakeup
results and register writes, including preservation after refused periods.
It does not model RTC timing or interrupts. Linux only.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#include "stm32.h"
#include "coredev/system_core_clock.h"
#include "rtc_stm32.h"

McuOsc_t g_McuOsc = { { OSC_TYPE_RC, 16000000, 100, 0 }, { OSC_TYPE_XTAL, 32768, 20, 0 }, false };

extern "C" void TestNvic(int Op, int IrqNo, uint32_t Prio)
{
	(void)Op;
	(void)IrqNo;
	(void)Prio;
}

extern "C" void usDelay(uint32_t Count)
{
	(void)Count;
}

int main()
{
	if (mmap((void *)0x58000000, 0x30000, PROT_READ | PROT_WRITE,
			 MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0) == MAP_FAILED)
	{
		perror("RTC registers mmap");
		return 1;
	}

	RCC->BDCR = RCC_BDCR_LSEON | RCC_BDCR_LSERDY | RCC_BDCR_RTCEN | (1U << RCC_BDCR_RTCSEL_Pos);
	RTC->ISR = RTC_ISR_INITS | RTC_ISR_WUTWF;
	RTC->PRER = (31U << RTC_PRER_PREDIV_A_Pos) | 1023U;
	RTC->DR = (6U << RTC_DR_WDU_Pos) | (1U << RTC_DR_MU_Pos) | (1U << RTC_DR_DU_Pos);

	RtcStm32 rtc;
	RtcCfg_t cfg = {};

	cfg.ClkSrc = RTC_CLKSRC_LFXTAL;
	if (!rtc.Init(cfg))
	{
		printf("FAIL RTC init\n");
		return 1;
	}

	struct Request {
		uint32_t Ms;
		uint32_t Effective;
		uint32_t Sel;
		uint32_t Wut;
	};
	static const Request s_Valid[] = {
		{ 1, 1, 3, 15 }, { 7, 7, 3, 114 }, { 250, 250, 3, 4095 }, { 1000, 1000, 3, 16383 },
		{ 5000, 5000, 2, 40959 }, { 32000, 32000, 0, 65535 }, { 32001, 32000, 4, 31 },
		{ 65536000, 65536000, 4, 65535 }, { 65537000, 65537000, 6, 0 },
		{ 131072000, 131072000, 6, 65535 }, { 131072499, 131072000, 6, 65535 },
	};
	int failures = 0;

	for (const Request &r : s_Valid)
	{
		if (!rtc.SetWakeup(r.Ms) || rtc.Wakeup() != r.Effective || RTC->WUTR != r.Wut ||
			(RTC->CR & RTC_CR_WUCKSEL) != r.Sel ||
			(RTC->CR & (RTC_CR_WUTE | RTC_CR_WUTIE)) != (RTC_CR_WUTE | RTC_CR_WUTIE))
		{
			printf("FAIL valid request %u\n", r.Ms);
			failures++;
		}
	}

	static const uint32_t s_TooLong[] = { 131072500U, UINT32_MAX - 500U, UINT32_MAX - 499U, UINT32_MAX };

	for (uint32_t ms : s_TooLong)
	{
		if (!rtc.SetWakeup(1000))
		{
			printf("FAIL initial period\n");
			return 1;
		}
		uint8_t before[sizeof(RTC_TypeDef)];

		memcpy(before, (const void *)RTC, sizeof(before));
		bool accepted = rtc.SetWakeup(ms);
		bool kept = rtc.Wakeup() == 1000 && memcmp(before, (const void *)RTC, sizeof(before)) == 0;

		if (accepted || !kept)
		{
			printf("FAIL invalid request %u: accepted=%d kept=%d period=%u WUTR=%08x CR=%08x\n",
				   ms, accepted, kept, rtc.Wakeup(), RTC->WUTR, RTC->CR);
			failures++;
		}
	}
	printf("RTC wakeup: 11 valid and 4 refused periods, %d failures\n", failures);

	return failures != 0;
}
