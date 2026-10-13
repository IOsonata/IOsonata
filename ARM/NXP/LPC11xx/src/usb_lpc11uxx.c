/**-------------------------------------------------------------------------
@file usb_lpc11uxx.c
@brief LPC11Uxx native USB hardware preparation.

Private first stage of the controller port. Generic USB continues to own
Chapter 9 and device classes. No ROM USB stack is initialized here. Endpoint
submission and USB_IRQHandler are not implemented yet; this module cannot
connect or enumerate a device by itself.

Register sequences: NXP UM10462, chapters 3 and 11. Register declarations
come from the vendor CMSIS header. Only the DMA memory layout is defined here.

@author Hoang Nguyen Hoan
@date Oct. 13, 2026
@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>

#include "LPC11Uxx.h"
#include "coredev/system_core_clock.h"
#include "usb_hw_lpc11uxx.h"

#define LPC11_USB_AHB_CLOCKS ((1UL << 14) | (1UL << 27))
#define LPC11_USB_ENABLE (1UL << 7)
#define LPC11_USB_CONNECT (1UL << 16)
#define LPC11_USB_PLL_ON (1UL << 9)
#define LPC11_USB_NO_WAKE ((1UL << 17) | (1UL << 19))
#define LPC11_USB_STATUS_W1C ((1UL << 8) | (7UL << 24))
#define LPC11_USB_COMMAND_MASK (0xFFUL | (1UL << 9) | (1UL << 11) | (0x1FUL << 12))
#define LPC11_USB_INTERRUPTS (0x3FFUL | (3UL << 30))
#define LPC11_USB_ACTIVE (1UL << 31)
#define LPC11_USB_DISABLED (1UL << 30)
#define LPC11_USB_WAIT_COUNT 1024U

// Maintained by the existing LPC11Uxx clock implementation, including its
// legacy SystemCoreClockSet entry point (which need not update g_McuOsc).
extern uint32_t SystemMainClkFreq;

// NOLOAD: initialized explicitly only AFTER the USB SRAM clock is enabled.
// No KEEP in the linker script: a non-USB program pays no memory cost.
static Lpc11UsbMemory_t s_Memory
	__attribute__((section(".usb_ram"), aligned(256)));
static bool s_Prepared;
static uint32_t s_AddedClocks;

_Static_assert(offsetof(Lpc11UsbMemory_t, Setup) == 128, "SETUP alignment");
_Static_assert(offsetof(Lpc11UsbMemory_t, Packet) == 192, "packet alignment");
_Static_assert(sizeof(Lpc11UsbMemory_t) == 832, "USB SRAM storage size");

static bool Lpc11UsbClockReady(void)
{
	const uint32_t ahbdiv = LPC_SYSCON->SYSAHBCLKDIV & 0xFFU;
	const uint32_t usbdiv = LPC_SYSCON->USBCLKDIV & 0xFFU;
	const uint32_t main = LPC_SYSCON->MAINCLKSEL & MAINCLKSEL_MASK;
	const uint32_t power = LPC_SYSCON->PDRUNCFG;
	const uint32_t sysmul = (LPC_SYSCON->SYSPLLCTRL & 0x1FU) + 1U;

	// The supported clock setup uses SYSOSC for the CPU and USB. Do not
	// infer SYSOSC from g_McuOsc: the legacy clock setter may have changed it.
	if (ahbdiv == 0U || usbdiv == 0U || SystemCoreClockGet() < 16000000U ||
		SystemMainClkFreq / ahbdiv != SystemCoreClockGet() ||
		(power & (PDRUNCFG_SYSOSC_PD | PDRUNCFG_USBPAD_PD)) != 0U ||
		(LPC_SYSCON->SYSPLLCLKSEL & SYSPLLCLKSEL_MASK) != SYSPLLCLKSEL_SYSOSC ||
		(LPC_SYSCON->SYSPLLCLKUEN & 1U) == 0U ||
		(LPC_SYSCON->MAINCLKUEN & 1U) == 0U ||
		(LPC_SYSCON->USBCLKUEN & 1U) == 0U ||
		(main != MAINCLKSEL_PLLOUT && main != MAINCLKSEL_PLLIN))
		return false;
	if (main == MAINCLKSEL_PLLOUT &&
		((power & PDRUNCFG_SYSPLL_PD) != 0U ||
		 (LPC_SYSCON->SYSPLLSTAT & 1U) == 0U || SystemMainClkFreq % sysmul != 0U))
		return false;

	const uint32_t input = main == MAINCLKSEL_PLLOUT ?
		SystemMainClkFreq / sysmul : SystemMainClkFreq;
	if (input < 10000000U || input > 25000000U)
		return false;

	const uint32_t source = LPC_SYSCON->USBCLKSEL & 3U;
	if (source == USBCLKSEL_MAINCLK)
		return main == MAINCLKSEL_PLLOUT && SystemMainClkFreq == 48000000U && usbdiv == 1U;
	if (source != USBCLKSEL_PLLOUT ||
		(power & PDRUNCFG_USBPLL_PD) != 0U ||
		(LPC_SYSCON->USBPLLCLKSEL & USBPLLCLKSEL_MASK) != USBPLLCLKSEL_SYSOSC ||
		(LPC_SYSCON->USBPLLCLKUEN & 1U) == 0U ||
		(LPC_SYSCON->USBPLLSTAT & 1U) == 0U)
		return false;

	const uint32_t control = LPC_SYSCON->USBPLLCTRL;
	const uint64_t output = (uint64_t)input * ((control & 0x1FU) + 1U);
	const uint64_t cco = output * (2U << ((control >> 5) & 3U));
	return output == (uint64_t)48000000U * usbdiv &&
		cco >= 156000000U && cco <= 320000000U;
}

void Lpc11UsbHwAckStatus(uint32_t Flags)
{
	if (!s_Prepared)
		return;
	// Reading and writing all bits would clear unrelated W1C events.
	// DSUS/LPM_SUS must be written as one even if the read saw zero: the
	// controller may have suspended between that read and this write.
	LPC_USB->DEVCMDSTAT = (LPC_USB->DEVCMDSTAT & LPC11_USB_COMMAND_MASK) |
		LPC11_USB_NO_WAKE | (Flags & LPC11_USB_STATUS_W1C);
}

Lpc11UsbMemory_t *Lpc11UsbHwPrepare(int IntPrio)
{
	const uintptr_t address = (uintptr_t)&s_Memory;
	if (s_Prepared || address < 0x20004000U ||
		address > 0x20004800U - sizeof(s_Memory) || (address & 255U) != 0U ||
		!Lpc11UsbClockReady())
		return NULL;

	const uint32_t clocks = LPC_SYSCON->SYSAHBCLKCTRL;
	LPC_SYSCON->SYSAHBCLKCTRL = clocks | LPC11_USB_AHB_CLOCKS;
	__DSB();
	// Do not take over a live native/ROM controller, or its DMA list.
	if ((LPC_USB->DEVCMDSTAT & (LPC11_USB_ENABLE | LPC11_USB_CONNECT)) != 0U ||
		(LPC_USB->EPSKIP & 0x3FFU) != 0U)
	{
		LPC_SYSCON->SYSAHBCLKCTRL = clocks;
		return NULL;
	}

	NVIC_DisableIRQ(USB_IRQn);
	LPC_USB->INTEN = 0U;
	LPC_USB->DEVCMDSTAT = LPC11_USB_PLL_ON | LPC11_USB_NO_WAKE | LPC11_USB_STATUS_W1C;
	memset(&s_Memory, 0, sizeof(s_Memory));
	for (unsigned physical = 0U; physical < 10U; ++physical)
	{
		const uint32_t buffer = (uint32_t)(uintptr_t)s_Memory.Packet[physical];
		s_Memory.Command[physical * 2U] = ((buffer >> 6) & 0xFFFFU) |
			(physical >= 2U ? LPC11_USB_DISABLED : 0U);
		if (physical >= 2U)
			s_Memory.Command[physical * 2U + 1U] = LPC11_USB_DISABLED;
	}
	s_Memory.Command[1] = ((uint32_t)(uintptr_t)s_Memory.Setup >> 6) & 0xFFFFU;
	__DMB();
	LPC_USB->EPLISTSTART = (uint32_t)address;
	LPC_USB->DATABUFSTART = (uint32_t)address & 0xFFC00000U;
	LPC_USB->EPINUSE = 0U;
	LPC_USB->EPBUFCFG = 0U;
	LPC_USB->INTROUTING = 0U;
	LPC_USB->INTSTAT = LPC11_USB_INTERRUPTS;
	NVIC_ClearPendingIRQ(USB_IRQn);
	const int maxprio = (1 << __NVIC_PRIO_BITS) - 1;
	NVIC_SetPriority(USB_IRQn, IntPrio < 0 ? 0U :
		(IntPrio > maxprio ? (uint32_t)maxprio : (uint32_t)IntPrio));
	s_AddedClocks = LPC11_USB_AHB_CLOCKS & ~clocks;
	s_Prepared = true;
	return &s_Memory;
}

bool Lpc11UsbHwRelease(void)
{
	if (!s_Prepared)
		return true;
	NVIC_DisableIRQ(USB_IRQn);
	LPC_USB->INTEN = 0U;
	// Disconnect but leave DEV_EN set until outstanding buffers retire.
	LPC_USB->DEVCMDSTAT = (LPC_USB->DEVCMDSTAT &
		(LPC11_USB_ENABLE | 0x7FU)) | LPC11_USB_PLL_ON | LPC11_USB_NO_WAKE;
	uint32_t active = 0U;
	for (unsigned physical = 0U; physical < 10U; ++physical)
	{
		if ((s_Memory.Command[physical * 2U] & LPC11_USB_ACTIVE) != 0U)
			active |= 1U << physical;
	}
	if (active != 0U)
		LPC_USB->EPSKIP = active;
	__DSB();
	uint32_t count = LPC11_USB_WAIT_COUNT;
	while ((LPC_USB->EPSKIP & 0x3FFU) != 0U)
	{
		if (--count == 0U)
			return false;
	}
	__DMB();
	for (unsigned physical = 0U; physical < 10U; ++physical)
	{
		if ((s_Memory.Command[physical * 2U] & LPC11_USB_ACTIVE) != 0U)
			return false;
	}
	LPC_USB->DEVCMDSTAT = LPC11_USB_NO_WAKE | LPC11_USB_STATUS_W1C;
	LPC_USB->INTSTAT = LPC11_USB_INTERRUPTS;
	__DSB();
	NVIC_ClearPendingIRQ(USB_IRQn);
	LPC_SYSCON->SYSAHBCLKCTRL &= ~s_AddedClocks;
	s_Prepared = false;
	return true;
}
