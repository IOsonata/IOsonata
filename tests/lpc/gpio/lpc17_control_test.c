/* LPC17xx GPIO access-width regressions using vendor CMSIS declarations.
 * Copyright (c) 2026 I-SYST inc. MIT License.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include "iopinctrl.h"

int main(void)
{
	assert(mmap((void *)LPC_GPIO_BASE, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)LPC_GPIO_BASE);

	// Narrow writes must keep every supplied bit and preserve higher pins.
	for (int port = 0; port < 5; port++)
	{
		IOPinWrite32Port(port, 0xA55A1234U);
		IOPinWrite8Port(port, 0xBCU);
		assert(IOPinReadPort(port) == 0xA55A12BCU);
		IOPinWrite16Port(port, 0xCDEFU);
		assert(IOPinReadPort(port) == 0xA55ACDEFU);
		IOPinWrite16Port(port, 0);
		assert(IOPinReadPort(port) == 0xA55A0000U);
		IOPinWrite16Port(port, UINT16_MAX);
		assert(IOPinReadPort(port) == 0xA55AFFFFU);
		IOPinWritePort(port, 0x12345678U);
		assert(IOPinReadPort(port) == 0x12345678U);
	}

	// P1.31 exercises the sign bit in C as well as C++ builds.
	LPC_GPIO1->FIODIR = 0x12345678U;
	IOPinSetDir(1, 31, IOPINDIR_OUTPUT);
	assert(LPC_GPIO1->FIODIR == 0x92345678U);
	IOPinSetDir(1, 31, IOPINDIR_INPUT);
	assert(LPC_GPIO1->FIODIR == 0x12345678U);
	IOPinSet(1, 31);
	assert(LPC_GPIO1->FIOSET == 0x80000000U);
	IOPinClear(1, 31);
	assert(LPC_GPIO1->FIOCLR == 0x80000000U);
	IOPinWritePort(1, 0x12345678U);
	IOPinToggle(1, 31);
	assert(IOPinReadPort(1) == 0x92345678U);
	assert(IOPinRead(1, 31) == 1);
	IOPinToggle(1, 31);
	assert(IOPinReadPort(1) == 0x12345678U);
	assert(IOPinRead(1, 31) == 0);
	puts("LPC17xx GPIO access-width regressions passed");
	return 0;
}
