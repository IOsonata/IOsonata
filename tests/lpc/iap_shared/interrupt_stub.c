/* Interrupt model for shared LPC ROM-command tests. */
#include <assert.h>
#include "coredev/interrupt.h"

static uint32_t s_Mask;

uint32_t DisableInterrupt(void)
{
	uint32_t previous = s_Mask;

	s_Mask = 1;
	return previous;
}

void EnableInterrupt(uint32_t State)
{
	assert(s_Mask == 1);
	s_Mask = State;
}
