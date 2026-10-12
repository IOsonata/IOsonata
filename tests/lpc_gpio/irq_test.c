/* LPC11Uxx pin interrupt regressions. Copyright (c) 2026 I-SYST inc.
 * MIT License. Exercises the real driver with vendor peripheral declarations.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "iopinctrl.h"
#include "irq_model.h"

#ifdef __cplusplus
extern "C" {
#endif
void FLEX_INT0_IRQHandler(void);
void FLEX_INT1_IRQHandler(void);
void FLEX_INT2_IRQHandler(void);
void FLEX_INT3_IRQHandler(void);
void FLEX_INT4_IRQHandler(void);
void FLEX_INT5_IRQHandler(void);
void FLEX_INT6_IRQHandler(void);
void FLEX_INT7_IRQHandler(void);
#ifdef __cplusplus
}
#endif

static void (* const s_Handlers[8])(void) = {
	FLEX_INT0_IRQHandler,
	FLEX_INT1_IRQHandler,
	FLEX_INT2_IRQHandler,
	FLEX_INT3_IRQHandler,
	FLEX_INT4_IRQHandler,
	FLEX_INT5_IRQHandler,
	FLEX_INT6_IRQHandler,
	FLEX_INT7_IRQHandler
};
static int s_Context[8];

int main(void)
{
	assert(mmap((void *)0x40000000, 0x100000, PROT_READ | PROT_WRITE,
				MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)0x40000000);
	assert(__NVIC_PRIO_BITS == 2);

	// Bad requests neither allocate nor access a clock-gated peripheral.
	assert(!IOPinEnableInterrupt(-1, 0, 0, 0, IOPINSENSE_TOGGLE, PinEvent, NULL));
	assert(!IOPinEnableInterrupt(8, 0, 0, 0, IOPINSENSE_TOGGLE, PinEvent, NULL));
	assert(!IOPinEnableInterrupt(0, -1, 0, 0, IOPINSENSE_TOGGLE, PinEvent, NULL));
	assert(!IOPinEnableInterrupt(0, 4, 0, 0, IOPINSENSE_TOGGLE, PinEvent, NULL));
	assert(!IOPinEnableInterrupt(0, 0, 0, 24, IOPINSENSE_TOGGLE, PinEvent, NULL));
	assert(!IOPinEnableInterrupt(0, 0, 1, 32, IOPINSENSE_TOGGLE, PinEvent, NULL));
	assert(!IOPinEnableInterrupt(0, 0, UINT32_MAX, 0, IOPINSENSE_TOGGLE, PinEvent, NULL));
	assert(!IOPinEnableInterrupt(0, 0, 0, 0, IOPINSENSE_DISABLE, PinEvent, NULL));
	assert(!IOPinEnableInterrupt(0, 0, 0, 0, IOPINSENSE_TOGGLE, NULL, NULL));
	assert(IOPinAllocateInterrupt(0, -1, 0, IOPINSENSE_TOGGLE, PinEvent, NULL) == -1);
	IOPinDisableInterrupt(0);
	IOPinDisableInterrupt(8);
	IOPinSetSense(0, 0, IOPINSENSE_TOGGLE);
	assert(LPC_SYSCON->SYSAHBCLKCTRL == 0 && s_Enabled == 0);

	LPC_SYSCON->SYSAHBCLKCTRL = 1U << 12;
	LPC_GPIO_PIN_INT->ISEL = 0xFF;
	s_Pending = 0xFF;
	for (int i = 0; i < 8; i++)
	{
		IOPINSENSE sense = i % 3 == 0 ? IOPINSENSE_HIGH_TRANSITION :
						  i % 3 == 1 ? IOPINSENSE_LOW_TRANSITION : IOPINSENSE_TOGGLE;
		int channel = IOPinAllocateInterrupt(i % 4, 1, i, sense, PinEvent, &s_Context[i]);
		assert(channel == i);
		assert(LPC_SYSCON->PINTSEL[i] == 24U + (unsigned)i);
		assert(s_Priority[i] == (unsigned)i % 4U);
		unsigned mask = 1U << i;
		assert((s_Enabled & mask) != 0 && (s_Pending & mask) == 0);
		assert(((LPC_GPIO_PIN_INT->IENR & mask) != 0) == (i % 3 != 1));
		assert(((LPC_GPIO_PIN_INT->IENF & mask) != 0) == (i % 3 != 0));
		LPC_GPIO_PIN_INT->IST = 0;
		s_Handlers[i]();
		assert(s_CallbackCount == (unsigned)i);
		LPC_GPIO_PIN_INT->IST = mask;
		s_Handlers[i]();
		assert(s_CallbackCount == (unsigned)i + 1U);
		assert(s_LastChannel == i && s_LastContext == &s_Context[i]);
	}
	assert(LPC_SYSCON->SYSAHBCLKCTRL == ((1U << 12) | (1U << 19)));
	assert(IOPinAllocateInterrupt(0, 0, 0, IOPINSENSE_TOGGLE, PinEvent, NULL) == -1);
	assert(!IOPinEnableInterrupt(0, 0, 0, 1, IOPINSENSE_TOGGLE, PinEvent, NULL));
	assert(LPC_SYSCON->PINTSEL[0] == 24);

	// Pause/resume keeps channel ownership and preserves other edge enables.
	unsigned rising = LPC_GPIO_PIN_INT->IENR & ~1U;
	unsigned falling = LPC_GPIO_PIN_INT->IENF & ~1U;
	IOPinSetSense(1, 0, IOPINSENSE_DISABLE);
	assert((s_Enabled & 1U) == 0);
	assert(LPC_GPIO_PIN_INT->IENR == rising && LPC_GPIO_PIN_INT->IENF == falling);
	assert(IOPinAllocateInterrupt(0, 0, 1, IOPINSENSE_TOGGLE, PinEvent, NULL) == -1);
	LPC_GPIO_PIN_INT->IST = 1;
	FLEX_INT0_IRQHandler();
	assert(s_CallbackCount == 8);
	IOPinSetSense(1, 0, IOPINSENSE_TOGGLE);
	assert(LPC_GPIO_PIN_INT->IENR == (rising | 1U));
	assert(LPC_GPIO_PIN_INT->IENF == (falling | 1U));
	s_Reedge = true;
	LPC_GPIO_PIN_INT->IST = 1;
	FLEX_INT0_IRQHandler();
	assert(LPC_GPIO_PIN_INT->IST == 1); // New edge survives callback return.
	s_Reedge = false;
	FLEX_INT0_IRQHandler();
	assert(s_CallbackCount == 10);

	// Release from a callback, then reuse the freed channel on P0_23.
	s_Release = true;
	LPC_GPIO_PIN_INT->IST = 1;
	FLEX_INT0_IRQHandler();
	s_Release = false;
	assert((s_Enabled & 1U) == 0);
	assert(IOPinAllocateInterrupt(3, 0, 23, IOPINSENSE_LOW_TRANSITION, PinEvent, NULL) == 0);
	assert(LPC_SYSCON->PINTSEL[0] == 23);
	IOPinDisableInterrupt(7);
	assert(IOPinEnableInterrupt(7, 0, 1, 31, IOPINSENSE_TOGGLE, PinEvent, NULL));
	assert(LPC_SYSCON->PINTSEL[7] == 55);
	for (int i = 0; i < 8; i++)
	{
		IOPinDisableInterrupt(i);
	}
	assert(s_Enabled == 0 && LPC_GPIO_PIN_INT->IENR == 0 && LPC_GPIO_PIN_INT->IENF == 0);
	unsigned count = s_CallbackCount;
	LPC_GPIO_PIN_INT->IST = 1;
	FLEX_INT0_IRQHandler();
	assert(s_CallbackCount == count);
	puts("LPC11Uxx pin interrupt regressions passed");
	return 0;
}
