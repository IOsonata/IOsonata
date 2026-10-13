/* LPC11Uxx generic USB integration model. MIT License, I-SYST, 2026. */
#define LPC11_USB_INTEGRATION
#include "lifecycle_test.c"

void TestUsbInitialize(void)
{
	assert(mmap((void *)LPC_SYSCON_BASE, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	assert(mmap((void *)LPC_USB_BASE, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	assert(mmap((void *)LPC_GPIO_BASE, 0x3000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	Defaults();
	s_Memory = (void *)0x20004000;
	s_CompleteSkip = true;
	LPC_GPIO->PIN[0] = 1U << 3;
}

Lpc11UsbMemory_t *TestUsbMemory(void)
{
	return s_Memory;
}

void TestUsbPacketDone(unsigned Physical, unsigned Length)
{
	const unsigned capacity = (s_Memory->Command[2 * Physical] >> 16) & 0x3FF;
	assert((s_Memory->Command[2 * Physical] & (1U << 31)) != 0 && Length <= capacity);
	s_Memory->Command[2 * Physical] &= ~((1U << 31) | (0x3FFU << 16));
	s_Memory->Command[2 * Physical] |= (capacity - Length) << 16;
	LPC_USB->INTSTAT |= 1U << Physical;
	USB_IRQHandler();
}

void TestUsbSetup(const UsbSetupData_t *pSetup)
{
	memcpy(s_Memory->Setup, pSetup, sizeof(*pSetup));
	LPC_USB->DEVCMDSTAT |= 1U << 8;
	LPC_USB->INTSTAT |= 1U;
	USB_IRQHandler();
}
