/**-------------------------------------------------------------------------
@file	c6_startup_clock_test.c
@brief	Host MMIO checks for C6 clock decoding and startup register selection.
@license MIT
Copyright (c) 2026, I-SYST inc., all rights reserved.
----------------------------------------------------------------------------*/
#include <assert.h>
#include <stdint.h>
#include <sys/mman.h>
#include <stdio.h>
#define ESP32C6
#include "RISCV/Espressif/src/system_esp32_pcr.c"
unsigned long __data_start__;
#define REG(a) (*(volatile uint32_t *)(uintptr_t)(a))
int main(void)
{
	void *mem = mmap((void *)0x60000000, 0xC0000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	assert(mem != MAP_FAILED);
	REG(0x60096110) = 2U << 8;
	REG(0x60096118) = 0;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 40000000U);
	REG(0x60096118) = 1;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 20000000U);
	REG(0x60096110) = (1U << 16) | (2U << 8);
	REG(0x60096118) = 1U << 8;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 80000000U);
	REG(0x60096118) = 0;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 160000000U);
	REG(0x60096118) = 1U << 16;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 120000000U);
	assert(SystemCoreClockPeriodus == 120U);
	assert(SystemCoreClockPeriodns == 8U);
	// TG0 starts in flash-boot protection; TG1 also has normal WDT enabled.
	// Keep the reset pulse-length fields unchanged when disabling both.
	REG(0x60008048) = (1U << 14) | (1U << 15);
	REG(0x60009048) = (1U << 31) | (1U << 14) | (1U << 18);
	REG(0x600B1C00) = (1U << 31) | (1U << 12);
	REG(0x60096054) = 2;
	REG(0x60096000) = 0x12345678;
	REG(0x60096004) = 0x87654321;
	REG(0x60096064) = 0x11223344;
	Esp32PcrSystemInit(0);
	assert(REG(0x60008048) == (1U << 15));
	assert(REG(0x60009048) == (1U << 18));
	assert(REG(0x60008064) == 0);
	assert(REG(0x60009064) == 0);
	assert(REG(0x600B1C00) == 0);
	assert(REG(0x600B1C1C) == (1U << 30));
	assert(REG(0x600B1C18) == 0);
	assert(REG(0x600B1C20) == 0);
	assert(REG(0x60096054) == 1);
	assert(REG(0x60096000) == 0x12345678);
	assert(REG(0x60096004) == 0x87654321);
	assert(REG(0x60096064) == 0x11223344);
	puts("C6 clock and startup MMIO checks PASS");
}
