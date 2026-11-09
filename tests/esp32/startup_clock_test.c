/**-------------------------------------------------------------------------
@file startup_clock_test.c
@brief C2/C5/C61 boot watchdog and clock decoding checks with host MMIO.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <assert.h>
#include <stdint.h>
#include <sys/mman.h>
#include <stdio.h>

#if defined(ESP32C2)
#include "RISCV/Espressif/src/system_esp32_system.c"
#else
#include "RISCV/Espressif/src/system_esp32_pcr.c"
#endif
unsigned long __data_start__;
#define REG(a) (*(volatile uint32_t *)(uintptr_t)(a))

int main(void)
{
	void *mem;
	uint32_t root;
	uint32_t cpu;
	uint32_t timer;

	mem = mmap((void *)0x60000000, 0xD0000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
	assert(mem != MAP_FAILED);
#if defined(ESP32C2)
	root = 0x600C0058;
	cpu = 0x600C0008;
	timer = 0x6001F000;
	REG(root) = 1U << 10;
	REG(cpu) = 1;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 120000000U);
	REG(cpu) = 0;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 80000000U);
	REG(root) = 1;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 20000000U);
	REG(0x60008084) = (1U << 31) | (1U << 12);
	REG(timer + 0x48) = (1U << 31) | (1U << 14);
	REG(0x60020048) = 0x12345678;
	REG(0x600C0018) = 1U << 29;
	Esp32SystemInit();
	assert(REG(root) == 1);
	assert(REG(0x600C0010) == (1U << 29));
	assert(REG(0x600C0018) == 0);
	assert(REG(0x60020048) == 0x12345678);
	assert(REG(0x60008084) == 0);
	assert(REG(0x6000809C) == 0);
	assert(REG(0x600080A0) == (1U << 30));
	assert(REG(0x600080A4) == 0);
#else
#if defined(ESP32C5)
	root = 0x60096110;
	timer = 0x6009606C;
	REG(root) = 3U << 16;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 240000000U);
#else
	root = 0x600960E8;
	timer = 0x60096058;
#endif
	cpu = root + 8;
	REG(root) = 2U << 16;
	REG(cpu) = 1;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 80000000U);
	REG(cpu) = 0;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 160000000U);
	REG(root + 12) = 3;
	REG(root + 16) = 1U << 8;
	assert(SystemPeriphClockGet(0) == 20000000U);
	assert(SystemPeriphClockGet(1) == 0);
	REG(root) = 0;
	REG(cpu) = 1;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 20000000U);
	REG(0x600B1C00) = (1U << 31) | (1U << 12);
	REG(0x60008048) = (1U << 31) | (1U << 14);
	REG(0x60009048) = (1U << 31) | (1U << 14);
	REG(timer) = 2;
	REG(0x60005000) = 0x12345678;
	Esp32PcrSystemInit(0);
	assert(REG(root) == 0);
	assert(REG(cpu) == 1);
	assert(REG(timer) == 1);
	assert(REG(0x60005000) == 0x12345678);
	assert(REG(0x6000A000) == ((1U << 31) | (1U << 30)));
	assert(REG(0x600B1C00) == 0);
	assert(REG(0x600B1C18) == 0);
	assert(REG(0x600B1C1C) == (1U << 30));
	assert(REG(0x600B1C20) == 0);
	assert(REG(0x60009048) == 0);
	timer = 0x60008000;
#endif
	assert(REG(timer + 0x48) == 0);
	assert(REG(timer + 0x64) == 0);
	assert(SystemCoreClockPeriodus == 20);
	assert(SystemCoreClockPeriodns == 50);
	puts("Startup watchdog/clock MMIO PASS");
	return 0;
}
