// Host register model: real ST peripheral definitions, no hardware timing.
// System clock and EXTI allocation. The UART driver has its own test,
// tests/stm32/run_uart.py.
#include <cassert>
#include <cstring>
#include <sys/mman.h>
#include "stm32f0xx.h"
#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"
static void pin_event(int, void *) {}
static void map(unsigned long addr, unsigned long len) {
	assert(mmap((void *)addr, len, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)addr);
}
int main() {
	map(0x40000000, 0x30000); map(0x48000000, 0x2000);
	RCC->CFGR = RCC_CFGR_SWS_PLL | RCC_CFGR_PLLMUL12;
	SystemCoreClockUpdate(); assert(SystemCoreClockGet() == 48000000);
	RCC->CFGR |= RCC_CFGR_HPRE_DIV2 | RCC_CFGR_PPRE_DIV2;
	SystemCoreClockUpdate(); assert(SystemCoreClockGet() == 24000000);
	assert(SystemPeriphClockGet(0) == 12000000);
	RCC->CFGR = 0; SystemCoreClockUpdate(); assert(SystemCoreClockGet() == 8000000);
	assert(IOPinAllocateInterrupt(1,0,13,IOPINSENSE_TOGGLE,pin_event,nullptr) == 13);
	assert(IOPinAllocateInterrupt(1,1,13,IOPINSENSE_TOGGLE,pin_event,nullptr) < 0);
	IOPinDisableInterrupt(13); IOPinDisableInterrupt(13);
	assert(IOPinAllocateInterrupt(1,1,13,IOPINSENSE_TOGGLE,pin_event,nullptr) == 13);
	assert(!IOPinEnableInterrupt(2,1,0,13,IOPINSENSE_TOGGLE,pin_event,nullptr));
}
