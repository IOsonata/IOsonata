/*--------------------------------------------------------------------------
File   : lpc17_atomic_test.c

Desc   : LPC17xx inline GPIO interrupt exclusion and single-bit writes

Copyright (c) 2026 I-SYST inc. MIT License.
----------------------------------------------------------------------------*/
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "LPC17xx.h"
#include "coredev/iopincfg.h"

static LPC_GPIO_TypeDef s_Gpio[5];
static uint32_t s_Mask;
static unsigned s_Accesses;
static unsigned s_Locks;

static uint32_t TestDisable(void)
{
	uint32_t previous = s_Mask;

	s_Mask = 1;
	s_Locks++;
	return previous;
}

static void TestRestore(uint32_t Mask)
{
	assert(s_Mask == 1);
	s_Mask = Mask;
}

static LPC_GPIO_TypeDef *TestGpio(void)
{
	assert(s_Mask == 1);
	s_Accesses++;
	return s_Gpio;
}

#undef LPC_GPIO0
#define LPC_GPIO0 TestGpio()
#define DisableInterrupt TestDisable
#define EnableInterrupt TestRestore
#include "iopinctrl.h"

int main(void)
{
	LPC_GPIO_TypeDef before[5];

	for (unsigned irqmask = 0; irqmask <= 1; irqmask++)
	{
		for (int port = 0; port < 5; port++)
		{
			for (int pin = 0; pin < 32; pin++)
			{
				uint32_t bit = 1U << pin;

				s_Mask = irqmask;
				s_Gpio[port].FIODIR = 0xA5A5A5A5U;
				IOPinSetDir(port, pin, IOPINDIR_OUTPUT);
				assert(s_Gpio[port].FIODIR == (0xA5A5A5A5U | bit));
				assert(s_Mask == irqmask);
				IOPinSetDir(port, pin, IOPINDIR_INPUT);
				assert(s_Gpio[port].FIODIR == (0xA5A5A5A5U & ~bit));
				assert(s_Mask == irqmask);
				for (unsigned high = 0; high <= 1; high++)
				{
					s_Gpio[port].FIOPIN = (0x5A5A5A5AU & ~bit) | (high ? bit : 0);
					s_Gpio[port].FIOSET = 0;
					s_Gpio[port].FIOCLR = 0;
					memcpy(before, s_Gpio, sizeof(before));
					IOPinToggle(port, pin);
					if (high)
					{
						before[port].FIOCLR = bit;
					}
					else
					{
						before[port].FIOSET = bit;
					}
					assert(memcmp(before, s_Gpio, sizeof(before)) == 0);
					assert(s_Mask == irqmask);
				}
			}
		}
	}
	assert(s_Accesses != 0 && s_Locks != 0);
	s_Accesses = 0;
	s_Locks = 0;
	IOPinSetDir(-1, 0, IOPINDIR_OUTPUT);
	IOPinSetDir(5, 0, IOPINDIR_OUTPUT);
	IOPinSetDir(0, -1, IOPINDIR_OUTPUT);
	IOPinSetDir(0, 32, IOPINDIR_OUTPUT);
	IOPinSetDir(0, 0, IOPINDIR_BI);
	IOPinToggle(-1, 0);
	IOPinToggle(5, 0);
	IOPinToggle(0, -1);
	IOPinToggle(0, 32);
	assert(s_Accesses == 0 && s_Locks == 0);
	puts("LPC17xx GPIO interrupt exclusion and single-bit writes: PASS");
	return 0;
}
