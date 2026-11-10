/**-------------------------------------------------------------------------
@file	crypto_diagnostics.cpp
@brief	ESP32-C6 CryptoTest reset and watchdog diagnostics.

@author	IOsonata contributors
@date	Nov. 8, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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

#include "esp32xx.h"
#include "esp32xx_pcr.h"

// Read-only diagnostics. Offsets follow ESP-IDF v5.5.1 C6 register headers.
void CryptoTestReportState(void)
{
	uint32_t cause;
	uint32_t tg0;
	uint32_t tg1;
	uint32_t rtc;
	uint32_t swd;
	uintptr_t mtvec;
	uintptr_t sp;

	cause = ESP32_REG32(ESP32_LP_CLKRST_BASE + 0x10U) & 0x1FU;
	tg0 = ESP32_REG32(ESP32_TIMG0_BASE + 0x48U);
	tg1 = ESP32_REG32(ESP32_TIMG1_BASE + 0x48U);
	rtc = ESP32_REG32(ESP32_LP_WDT_BASE);
	swd = ESP32_REG32(ESP32_LP_WDT_BASE + 0x1CU);
	__asm volatile("csrr %0, mtvec" : "=r"(mtvec));
	__asm volatile("mv %0, sp" : "=r"(sp));
	printf("C6 reset cause: 0x%02lx, mtvec: 0x%08lx, sp: 0x%08lx\r\n",
		(unsigned long)cause, (unsigned long)mtvec, (unsigned long)sp);
	printf("C6 watchdogs: TG0=%08lx TG1=%08lx RTC=%08lx SWD=%08lx\r\n",
		(unsigned long)tg0, (unsigned long)tg1,
		(unsigned long)rtc, (unsigned long)swd);
	fflush(stdout);
}
