// Host regression for linked vector selection. Does not model Cortex barriers.
#include <assert.h>
#include <sys/mman.h>
#include "stm32f4xx.h"
TestSCB test_scb;
void (* const __Vectors[])(void) = {0};
int main(void)
{
	void *base = (void *)0x40000000UL;
	assert(mmap(base, 0x30000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == base);
	SystemInit();
	assert(SCB->VTOR == (uint32_t)(uintptr_t)__Vectors);
	assert(SCB->VTOR != FLASH_BASE);
}
