/*--------------------------------------------------------------------------
File   : ahb17_test.c

Desc   : LPC17xx SystemInit AHB data copy with optional linker symbols

Copyright (c) 2026 I-SYST inc. MIT License.
----------------------------------------------------------------------------*/
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "LPC17xx.h"

uint32_t SystemMicroSecLoopCnt;
TestScb_t g_TestScb;
static unsigned s_BarrierCount;

#ifdef TEST_AHB_DATA
const uint8_t __data_ahb_loc__[16] = {
	0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
	0xA5, 0x5A, 0, 0xFF, 1, 2, 3, 4
};
// Extra bytes after the linker-sized destination detect an overrun.
uint8_t __data_ahb_start__[24];
#endif

void TestDSB(void)
{
	assert(SCB->VTOR == TEST_VECTOR_BASE);
	assert(s_BarrierCount == 0);
	s_BarrierCount++;
}

void TestISB(void)
{
	assert(s_BarrierCount == 1);
	s_BarrierCount++;
}

int main(void)
{
	uint32_t status = (1U << 24) | (1U << 25) | (1U << 26) | 59U;

	assert(mmap((void *)LPC_SC_BASE, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)LPC_SC_BASE);
	// Clock transitions are not simulated: report PLL ready to let the
	// existing SystemInit complete. This test validates data copying and vector selection.
	memcpy((uint8_t *)LPC_SC + offsetof(LPC_SC_TypeDef, PLL0STAT), &status, sizeof(status));
#ifdef TEST_AHB_DATA
	memset(__data_ahb_start__, 0xCC, sizeof(__data_ahb_start__));
#endif
	SCB->VTOR = 0x123400;
	SystemInit();
	assert(SCB->VTOR == TEST_VECTOR_BASE);
	assert(s_BarrierCount == 2);
#ifdef TEST_AHB_DATA
	assert(memcmp(__data_ahb_start__, __data_ahb_loc__, sizeof(__data_ahb_loc__)) == 0);
	for (unsigned i = 16; i < sizeof(__data_ahb_start__); i++)
	{
		assert(__data_ahb_start__[i] == 0xCC);
	}
#endif
	puts("LPC17xx AHB data initialization and vector selection: PASS");
	return 0;
}
