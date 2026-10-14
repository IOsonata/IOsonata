/* LPC11Uxx USB hardware preparation regression. MIT License, I-SYST, 2026. */
#include <assert.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include "LPC11Uxx.h"
#include "usb_hw_lpc11uxx.h"

uint32_t SystemMainClkFreq;
static uint32_t s_CoreClock;
static unsigned s_Disabled;
static unsigned s_Cleared;
static uint32_t s_Priority;
static bool s_CompleteSkip;
static Lpc11UsbMemory_t *s_Memory;
static unsigned s_Pended;

void NVIC_SetPendingIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn);
	++s_Pended;
}

uint32_t SystemCoreClockGet(void) { return s_CoreClock; }
void NVIC_DisableIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn);
	++s_Disabled;
}
void NVIC_ClearPendingIRQ(IRQn_Type Irq)
{
	assert(Irq == USB_IRQn);
	++s_Cleared;
}
void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority)
{
	assert(Irq == USB_IRQn);
	s_Priority = Priority;
}
void TestUsbDsb(void)
{
	if (s_CompleteSkip && LPC_USB->EPSKIP != 0U)
	{
		assert(s_Memory != NULL);
		for (unsigned physical = 0; physical < 10; ++physical)
		{
			if ((LPC_USB->EPSKIP & (1U << physical)) != 0U)
				s_Memory->Command[2 * physical] &= ~(1U << 31);
		}
		LPC_USB->EPSKIP = 0;
	}
}

static void SetReadOnly(volatile const uint32_t *pReg, uint32_t Value)
{
	*(volatile uint32_t *)(uintptr_t)pReg = Value;
}

static void Defaults(void)
{
	memset((void *)LPC_SYSCON, 0, sizeof(*LPC_SYSCON));
	memset((void *)LPC_USB, 0, sizeof(*LPC_USB));
	SystemMainClkFreq = 48000000;
	s_CoreClock = 48000000;
	LPC_SYSCON->SYSAHBCLKDIV = 1;
	LPC_SYSCON->MAINCLKSEL = 3;
	LPC_SYSCON->MAINCLKUEN = 1;
	LPC_SYSCON->SYSPLLCLKSEL = 1;
	LPC_SYSCON->SYSPLLCLKUEN = 1;
	LPC_SYSCON->SYSPLLCTRL = 0x23; // 12 MHz * 4, CCO 192 MHz
	SetReadOnly(&LPC_SYSCON->SYSPLLSTAT, 1);
	LPC_SYSCON->USBPLLCLKSEL = 1;
	LPC_SYSCON->USBPLLCLKUEN = 1;
	LPC_SYSCON->USBPLLCTRL = 0x23;
	SetReadOnly(&LPC_SYSCON->USBPLLSTAT, 1);
	LPC_SYSCON->USBCLKUEN = 1;
	LPC_SYSCON->USBCLKDIV = 1;
	LPC_SYSCON->SYSAHBCLKCTRL = 0x1007F;
	s_Disabled = 0;
	s_Cleared = 0;
	s_CompleteSkip = false;
	s_Priority = UINT_MAX;
}

static void RefusedWithoutChanges(void)
{
	unsigned char before[sizeof(*LPC_SYSCON)];
	memcpy(before, (const void *)LPC_SYSCON, sizeof(before));
	LPC_USB->INFO = 0x12345678;
	LPC_USB->INTEN = 0x87654321;
	assert(Lpc11UsbHwPrepare(1) == NULL);
	assert(memcmp(before, (const void *)LPC_SYSCON, sizeof(before)) == 0);
	assert(LPC_USB->INFO == 0x12345678);
	assert(LPC_USB->INTEN == 0x87654321);
	assert(s_Disabled == 0 && s_Cleared == 0 && s_Priority == UINT_MAX);
}

int main(void)
{
	assert(mmap((void *)LPC_SYSCON_BASE, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	assert(mmap((void *)LPC_USB_BASE, 4096, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	Defaults();
#ifdef TEST_BAD_LAYOUT
	RefusedWithoutChanges();
	puts("USB incorrect SRAM placement refused");
	return 0;
#endif
	// Invalid clocks must fail before touching controller registers or SRAM.
	for (unsigned failure = 0; failure < 16; ++failure)
	{
		Defaults();
		switch (failure)
		{
		case 0: LPC_SYSCON->USBCLKDIV = 0; break;
		case 1: LPC_SYSCON->SYSAHBCLKDIV = 0; break;
		case 2: s_CoreClock = 12000000; break;
		case 3: LPC_SYSCON->PDRUNCFG = 1U << 5; break;
		case 4: LPC_SYSCON->PDRUNCFG = 1U << 10; break;
		case 5: LPC_SYSCON->USBCLKSEL = 2; break;
		case 6: SetReadOnly(&LPC_SYSCON->USBPLLSTAT, 0); break;
		case 7: LPC_SYSCON->USBPLLCLKSEL = 0; break;
		case 8: LPC_SYSCON->USBCLKDIV = 2; break;
		case 9: LPC_SYSCON->USBPLLCTRL = 0x03; break; // invalid CCO
		case 10: LPC_SYSCON->USBCLKUEN = 0; break;
		case 11: LPC_SYSCON->MAINCLKSEL = 0; break;
		case 12: LPC_SYSCON->PDRUNCFG = 1U << 8; break;
		case 13: LPC_SYSCON->SYSPLLCLKSEL = 0; break;
		case 14: SetReadOnly(&LPC_SYSCON->SYSPLLSTAT, 0); break;
		case 15: LPC_SYSCON->USBPLLCLKUEN = 0; break;
		}
		RefusedWithoutChanges();
	}
	// Refuse to overwrite ROM/native active controller state.
	for (unsigned busy = 0; busy < 3; ++busy)
	{
		Defaults();
		if (busy == 0) LPC_USB->DEVCMDSTAT = 1U << 7;
		if (busy == 1) LPC_USB->DEVCMDSTAT = 1U << 16;
		if (busy == 2) LPC_USB->EPSKIP = 1;
		RefusedWithoutChanges();
	}

	const int priority[] = {INT_MIN, -1, 0, 1, 2, 3, 6, INT_MAX};
	for (unsigned test = 0; test < sizeof(priority) / sizeof(priority[0]); ++test)
	{
		Defaults();
		// Also cover clocks already enabled by another user.
		if ((test & 1U) != 0U)
			LPC_SYSCON->SYSAHBCLKCTRL |= (1U << 14) | (1U << 27);
		const uint32_t clocks = LPC_SYSCON->SYSAHBCLKCTRL;
		memset((void *)0x20004000, 0xA5, 832);
		s_Memory = Lpc11UsbHwPrepare(priority[test]);
		assert(s_Memory == (void *)0x20004000);
		assert(s_Priority == (uint32_t)(priority[test] < 0 ? 0 :
			(priority[test] > 3 ? 3 : priority[test])));
		assert(s_Disabled == 1 && s_Cleared == 1);
		assert(LPC_SYSCON->SYSAHBCLKCTRL == (clocks | (1U << 14) | (1U << 27)));
		assert(LPC_USB->EPLISTSTART == 0x20004000);
		assert(LPC_USB->DATABUFSTART == 0x20000000);
		assert(LPC_USB->INTEN == 0 && LPC_USB->INTROUTING == 0);
		assert(LPC_USB->EPINUSE == 0 && LPC_USB->EPBUFCFG == 0);
		assert((LPC_USB->DEVCMDSTAT & ((1U << 7) | (1U << 16) | (1U << 11))) == 0);
		assert(LPC_USB->INTSTAT == 0xC00003FF); // W1C write value
		assert(s_Memory->Command[1] == (0x4080U >> 6));
		assert(s_Memory->Command[3] == 0);
		for (unsigned physical = 0; physical < 10; ++physical)
		{
			const uint32_t command = s_Memory->Command[physical * 2];
			assert(((command & 0xFFFFU) << 6) == 0x40C0U + physical * 64);
			assert((command >> 16) == (physical < 2 ? 0U : 0x4000U));
			if (physical >= 2)
				assert(s_Memory->Command[physical * 2 + 1] == (1U << 30));
			for (unsigned byte = 0; byte < 64; ++byte)
				assert(s_Memory->Packet[physical][byte] == 0);
		}
		for (unsigned byte = 0; byte < sizeof(s_Memory->Setup); ++byte)
			assert(s_Memory->Setup[byte] == 0);
		assert(Lpc11UsbHwPrepare(2) == NULL);

		// Plain mapped registers record writes; this checks W1C masks, not
		// a claim that host RAM emulates hardware W1C behavior.
		LPC_USB->DEVCMDSTAT = 0xFFFFFFFF;
		Lpc11UsbHwAckStatus(1U << 25);
		assert(LPC_USB->DEVCMDSTAT == (0x0001FAFFU | (1U << 17) | (1U << 19) | (1U << 25)));
		LPC_USB->DEVCMDSTAT = 0; // No suspend seen at read: still write no-wake ones.
		Lpc11UsbHwAckStatus(0);
		assert(LPC_USB->DEVCMDSTAT == ((1U << 17) | (1U << 19)));

		// Timeout must retain DMA ownership, SRAM and register clocks.
		s_Memory->Command[18] |= 1U << 31; // endpoint 4 IN
		assert(!Lpc11UsbHwRelease());
		assert((s_Memory->Command[18] & (1U << 31)) != 0);
		assert(LPC_USB->EPSKIP == (1U << 9));
		assert(LPC_SYSCON->SYSAHBCLKCTRL == (clocks | (1U << 14) | (1U << 27)));
		assert(Lpc11UsbHwPrepare(0) == NULL);
		s_CompleteSkip = true;
		assert(Lpc11UsbHwRelease());
		assert(LPC_SYSCON->SYSAHBCLKCTRL == clocks);
		assert((LPC_USB->DEVCMDSTAT & ((1U << 7) | (1U << 16))) == 0);
		assert(Lpc11UsbHwRelease());
	}

	// Alternate valid inputs: 24 MHz oscillator, divided USB PLL,
	// minimum 16 MHz AHB, and system PLL instead of the dedicated USB PLL.
	for (unsigned setup = 0; setup < 4; ++setup)
	{
		Defaults();
		if (setup == 0)
		{
			LPC_SYSCON->SYSPLLCTRL = 0x21;
			LPC_SYSCON->USBPLLCTRL = 0x21;
		}
		if (setup == 1)
		{
			LPC_SYSCON->USBPLLCTRL = 7; // 96 MHz, CCO 192 MHz
			LPC_SYSCON->USBCLKDIV = 2;
		}
		if (setup == 2)
		{
			LPC_SYSCON->SYSAHBCLKDIV = 3;
			s_CoreClock = 16000000;
		}
		if (setup == 3)
		{
			LPC_SYSCON->USBCLKSEL = 1;
			LPC_SYSCON->PDRUNCFG = 1U << 8;
			SetReadOnly(&LPC_SYSCON->USBPLLSTAT, 0);
		}
		assert(Lpc11UsbHwPrepare(2) != NULL);
		assert(Lpc11UsbHwRelease());
	}
	puts("LPC11Uxx USB hardware preparation: PASS");
	return 0;
}
