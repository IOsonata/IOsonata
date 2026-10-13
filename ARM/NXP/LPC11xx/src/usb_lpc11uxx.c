/**-------------------------------------------------------------------------
@file usb_lpc11uxx.c
@brief LPC11Uxx native USB hardware preparation.

Hardware preparation and EP0 transport. Generic USB owns Chapter 9 and device
classes. No ROM USB stack is initialized here. Public lifecycle and data
endpoints remain unfinished; this module cannot connect a device by itself.

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
#include "coredev/interrupt.h"
#include "usb/usb.h"
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
#define LPC11_USB_STALL (1UL << 29)
#define LPC11_USB_SETUP (1UL << 8)
#define LPC11_USB_NAK_CONTROL (3UL << 14)
#define LPC11_USB_RESET (1UL << 26)
#define LPC11_USB_SUSPEND_CHANGE (1UL << 25)
#define LPC11_USB_CONNECT_CHANGE (1UL << 24)
#define LPC11_USB_DEVICE_INTERRUPT (1UL << 31)

// Maintained by the existing LPC11Uxx clock implementation, including its
// legacy SystemCoreClockSet entry point (which need not update g_McuOsc).
extern uint32_t SystemMainClkFreq;

typedef struct __Lpc11_Usb_Ep0_State {
	uint32_t Generation;
	uint16_t RemainingOut;
	uint16_t InLength;
	uint16_t OutLength;
	uint8_t OldAddress;
	bool Enabled;
	bool InBusy;
	bool OutBusy;
	bool DirectionIn;
	bool StatusOut;
	bool Halted;
	bool Aborted;
	bool AddressPending;
} Lpc11UsbEp0State_t;

// NOLOAD: initialized explicitly only AFTER the USB SRAM clock is enabled.
// No KEEP in the linker script: a non-USB program pays no memory cost.
static Lpc11UsbMemory_t s_Memory
	__attribute__((section(".usb_ram"), aligned(256)));

static Lpc11UsbEp0State_t s_Ep0;
static bool s_Prepared;
static uint32_t s_AddedClocks;

static bool Lpc11UsbRetire(uint32_t Mask);

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
	s_Ep0.Enabled = false;
	++s_Ep0.Generation;
	if (!s_Prepared)
		return true;
	NVIC_DisableIRQ(USB_IRQn);
	LPC_USB->INTEN = 0U;
	// Disconnect but leave DEV_EN set until outstanding buffers retire.
	LPC_USB->DEVCMDSTAT = (LPC_USB->DEVCMDSTAT &
		(LPC11_USB_ENABLE | 0x7FU)) | LPC11_USB_PLL_ON | LPC11_USB_NO_WAKE;
	if (!Lpc11UsbRetire(0x3FFU))
		return false;
	LPC_USB->DEVCMDSTAT = LPC11_USB_NO_WAKE | LPC11_USB_STATUS_W1C;
	LPC_USB->INTSTAT = LPC11_USB_INTERRUPTS;
	__DSB();
	NVIC_ClearPendingIRQ(USB_IRQn);
	LPC_SYSCON->SYSAHBCLKCTRL &= ~s_AddedClocks;
	s_Prepared = false;
	return true;
}

static void Lpc11UsbCommand(uint32_t Clear, uint32_t Set, uint32_t Ack)
{
	LPC_USB->DEVCMDSTAT = ((LPC_USB->DEVCMDSTAT & LPC11_USB_COMMAND_MASK) & ~Clear) |
		Set | LPC11_USB_NO_WAKE | (Ack & LPC11_USB_STATUS_W1C);
}

static uint32_t Lpc11UsbBufferAddress(unsigned Physical)
{
	return ((uint32_t)(uintptr_t)s_Memory.Packet[Physical] >> 6) & 0xFFFFU;
}

static bool Lpc11UsbRetire(uint32_t Mask)
{
	uint32_t active = 0U;
	for (unsigned physical = 0U; physical < 10U; ++physical)
	{
		if ((Mask & (1U << physical)) != 0U &&
			(s_Memory.Command[physical * 2U] & LPC11_USB_ACTIVE) != 0U)
			active |= 1U << physical;
	}
	if (active != 0U)
		LPC_USB->EPSKIP = active;
	__DSB();
	uint32_t count = LPC11_USB_WAIT_COUNT;
	while ((LPC_USB->EPSKIP & Mask) != 0U)
	{
		if (--count == 0U)
			return false;
	}
	__DMB();
	for (unsigned physical = 0U; physical < 10U; ++physical)
	{
		if ((Mask & (1U << physical)) != 0U &&
			(s_Memory.Command[physical * 2U] & LPC11_USB_ACTIVE) != 0U)
			return false;
	}
	return true;
}

static void Lpc11UsbEp0Fault(void)
{
	// Retain DMA storage on a failed skip. Stop bus/IRQ activity without
	// pretending the hardware has returned ownership of an active buffer.
	s_Ep0.Enabled = false;
	++s_Ep0.Generation;
	NVIC_DisableIRQ(USB_IRQn);
	LPC_USB->INTEN = 0U;
	Lpc11UsbCommand(LPC11_USB_CONNECT | LPC11_USB_NAK_CONTROL, 0U, 0U);
}

static void Lpc11UsbEp0Reset(void)
{
	const uint32_t generation = s_Ep0.Generation + 1U;
	memset(&s_Ep0, 0, sizeof(s_Ep0));
	s_Ep0.Generation = generation;
	s_Ep0.Enabled = true;
	s_Memory.Command[0] = Lpc11UsbBufferAddress(0U);
	s_Memory.Command[2] = Lpc11UsbBufferAddress(1U);
	__DMB();
}

bool Lpc11UsbEp0Enable(void)
{
	if (!s_Prepared || s_Ep0.Enabled)
		return false;
	const uint32_t state = DisableInterrupt();
	if ((LPC_USB->DEVCMDSTAT & LPC11_USB_CONNECT) != 0U ||
		!Lpc11UsbRetire(3U))
	{
		EnableInterrupt(state);
		return false;
	}
	Lpc11UsbEp0Reset();
	LPC_USB->INTSTAT = LPC11_USB_INTERRUPTS;
	Lpc11UsbCommand(0x7FU | LPC11_USB_NAK_CONTROL,
		LPC11_USB_ENABLE, LPC11_USB_STATUS_W1C);
	LPC_USB->INTEN = 3U | LPC11_USB_DEVICE_INTERRUPT;
	EnableInterrupt(state);
	return true;
}

static bool Lpc11UsbEp0Current(void)
{
	return s_Prepared && s_Ep0.Enabled && !s_Ep0.Halted &&
		(LPC_USB->DEVCMDSTAT & (LPC11_USB_SETUP | LPC11_USB_RESET)) == 0U;
}

static void Lpc11UsbEp0ArmOut(uint16_t Length)
{
	s_Ep0.OutLength = Length;
	s_Ep0.OutBusy = true;
	__DMB();
	s_Memory.Command[0] = Lpc11UsbBufferAddress(0U) |
		((uint32_t)Length << 16) | LPC11_USB_ACTIVE;
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	if (DevNo != 0 || Length < 0 || (Length != 0 && pBuffer == NULL))
		return -1;
	const uint32_t state = DisableInterrupt();
	if (!Lpc11UsbEp0Current() || s_Ep0.InBusy ||
		(s_Memory.Command[2] & LPC11_USB_ACTIVE) != 0U)
	{
		EnableInterrupt(state);
		return -1;
	}
	const uint16_t length = Length > 64 ? 64U : (uint16_t)Length;
	if (length != 0U)
		memcpy(s_Memory.Packet[1], pBuffer, length);
	if (!Lpc11UsbEp0Current())
	{
		EnableInterrupt(state);
		return -1;
	}
	s_Ep0.InLength = length;
	s_Ep0.InBusy = true;
	LPC_USB->INTSTAT = 2U;
	__DMB();
	s_Memory.Command[2] = Lpc11UsbBufferAddress(1U) |
		((uint32_t)length << 16) | LPC11_USB_ACTIVE;
	EnableInterrupt(state);
	return length;
}

bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr)
{
	if (DevNo != 0 || (EpAddr != 0U && EpAddr != 0x80U))
		return false;
	const uint32_t state = DisableInterrupt();
	if (!Lpc11UsbEp0Current())
	{
		EnableInterrupt(state);
		return false;
	}
	bool accepted = false;
	if (EpAddr == 0x80U && !s_Ep0.OutBusy)
	{
		accepted = UsbCtrlrEp0Send(0, NULL, 0) == 0;
		if (accepted)
		{
			s_Ep0.RemainingOut = 0U;
			Lpc11UsbCommand(LPC11_USB_NAK_CONTROL, 0U, 0U);
		}
	}
	else if (EpAddr == 0U && !s_Ep0.OutBusy && !s_Ep0.InBusy &&
		(s_Memory.Command[0] & LPC11_USB_ACTIVE) == 0U)
	{
		s_Ep0.StatusOut = true;
		Lpc11UsbCommand(LPC11_USB_NAK_CONTROL, 0U, 0U);
		Lpc11UsbEp0ArmOut(0U);
		accepted = true;
	}
	EnableInterrupt(state);
	return accepted;
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	if (DevNo != 0 || Address > 127U)
		return;
	const uint32_t state = DisableInterrupt();
	if (Lpc11UsbEp0Current())
	{
		// UM10462 table 197: program BEFORE the SET_ADDRESS status phase.
		if (!s_Ep0.AddressPending)
			s_Ep0.OldAddress = LPC_USB->DEVCMDSTAT & 0x7FU;
		s_Ep0.AddressPending = true;
		Lpc11UsbCommand(0x7FU, Address, 0U);
	}
	EnableInterrupt(state);
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool bIn)
{
	(void)bIn;
	// Data endpoints will be added with their transfer ownership state.
	if (DevNo != 0 || EpNo != 0U)
		return;
	const uint32_t state = DisableInterrupt();
	if (Lpc11UsbEp0Current())
	{
		if (!Lpc11UsbRetire(3U))
			Lpc11UsbEp0Fault();
		else if (Lpc11UsbEp0Current())
		{
			s_Ep0.InBusy = false;
			s_Ep0.OutBusy = false;
			s_Ep0.Halted = true;
			s_Ep0.RemainingOut = 0U;
			Lpc11UsbCommand(LPC11_USB_NAK_CONTROL, 0U, 0U);
			s_Memory.Command[0] = Lpc11UsbBufferAddress(0U) | LPC11_USB_STALL;
			s_Memory.Command[2] = Lpc11UsbBufferAddress(1U) | LPC11_USB_STALL;
			LPC_USB->INTSTAT = 2U;
		}
	}
	EnableInterrupt(state);
}

static void Lpc11UsbEp0Complete(bool In, uint16_t Length, bool Cancelled)
{
	UsbCtrlrEvt_t event = {0};
	event.Type = USB_CTRLR_EVT_XFER_CMPL;
	event.Xfer.EpAddr = In ? 0x80U : 0U;
	event.Xfer.Length = Length;
	event.Xfer.Result = Cancelled ? USB_CTRLR_XFER_CANCELLED : USB_CTRLR_XFER_SUCCESS;
	event.Xfer.pBuffer = In ? NULL : s_Memory.Packet[0];
	UsbDevProcessEvent(0, &event);
}

static void Lpc11UsbEp0Setup(uint32_t Pending)
{
	// A new SETUP can be observed together with the preceding address
	// status ACK. Commit that ACK to the generic core before aborting the
	// old control state; otherwise the next SET_CONFIGURATION sees address 0.
	if (s_Ep0.AddressPending && s_Ep0.InBusy && s_Ep0.InLength == 0U &&
		(Pending & 2U) != 0U && (s_Memory.Command[2] & LPC11_USB_ACTIVE) == 0U)
	{
		const uint32_t generation = s_Ep0.Generation;
		s_Ep0.AddressPending = false;
		s_Ep0.InBusy = false;
		Lpc11UsbEp0Complete(true, 0U, false);
		if (!s_Prepared || !s_Ep0.Enabled || s_Ep0.Generation != generation)
			return;
	}
	UsbCtrlrEvt_t event = {0};
	event.Type = USB_CTRLR_EVT_SETUP;
	__DMB();
	memcpy(&event.Setup, s_Memory.Setup, sizeof(event.Setup));
	if (s_Ep0.AddressPending)
		Lpc11UsbCommand(0x7FU, s_Ep0.OldAddress, 0U);
	// Figure 21 explicitly requires clearing both EP0 ACTIVE/STALL words
	// while SETUP holds IN/OUT NAKed, BEFORE acknowledging SETUP. Hardware
	// resets both control data toggles to DATA1 on reception of SETUP.
	Lpc11UsbEp0Reset();
	s_Ep0.DirectionIn = (event.Setup.bmRequestType & 0x80U) != 0U;
	s_Ep0.RemainingOut = s_Ep0.DirectionIn ? 0U : event.Setup.wLength;
	LPC_USB->INTSTAT = 2U;
	Lpc11UsbCommand(LPC11_USB_NAK_CONTROL,
		s_Ep0.DirectionIn ? (1U << 14) :
		(s_Ep0.RemainingOut != 0U ? (1U << 15) : 0U), LPC11_USB_SETUP);
	const uint32_t generation = s_Ep0.Generation;
	UsbDevProcessEvent(0, &event);
	if (Lpc11UsbEp0Current() && s_Ep0.Generation == generation &&
		s_Ep0.RemainingOut != 0U && !s_Ep0.OutBusy)
		Lpc11UsbEp0ArmOut(64U);
}

static void Lpc11UsbEp0Interrupt(uint32_t Pending)
{
	// EP0 OUT shares its interrupt with SETUP. Acknowledge BEFORE reading
	// SETUP, once only. A SETUP arriving afterward keeps its new interrupt;
	// one arriving before this write is handled by the following status read.
	LPC_USB->INTSTAT = Pending & 3U;
	if ((LPC_USB->DEVCMDSTAT & LPC11_USB_SETUP) != 0U)
	{
		Lpc11UsbEp0Setup(Pending);
		return;
	}
	if (!Lpc11UsbEp0Current())
		return;
	const uint32_t generation = s_Ep0.Generation;
	// IN completion first: the generic callback may arm status OUT. A NAK
	// interrupt already latched for that OUT must not complete the new bank.
	if ((Pending & 2U) != 0U)
	{
		if (s_Ep0.InBusy && (s_Memory.Command[2] & LPC11_USB_ACTIVE) == 0U)
		{
			const uint16_t length = s_Ep0.InLength;
			s_Ep0.InBusy = false;
			if (s_Ep0.AddressPending && length == 0U)
				s_Ep0.AddressPending = false;
			Lpc11UsbEp0Complete(true, length, s_Ep0.Aborted);
		}
		else if (!s_Ep0.InBusy && s_Ep0.RemainingOut != 0U &&
			!((Pending & 1U) != 0U && s_Ep0.OutBusy &&
			  (s_Memory.Command[0] & LPC11_USB_ACTIVE) == 0U))
		{
			// Host ends a control write early. Retire OUT before acknowledging
			// its IN status request. Notify the core after the status ACK.
			if (!Lpc11UsbRetire(1U))
			{
				Lpc11UsbEp0Fault();
				return;
			}
			if (!Lpc11UsbEp0Current())
				return;
			s_Ep0.OutBusy = false;
			s_Ep0.RemainingOut = 0U;
			s_Ep0.Aborted = true;
			Lpc11UsbCommand(LPC11_USB_NAK_CONTROL, 0U, 0U);
			(void)UsbCtrlrEp0Send(0, NULL, 0);
		}
	}
	if (!Lpc11UsbEp0Current() || s_Ep0.Generation != generation)
		return;
	if ((Pending & 1U) != 0U)
	{
		if (s_Ep0.OutBusy && (s_Memory.Command[0] & LPC11_USB_ACTIVE) == 0U)
		{
			__DMB();
			const uint16_t remaining = (s_Memory.Command[0] >> 16) & 0x3FFU;
			const uint16_t length = remaining <= s_Ep0.OutLength ?
				(uint16_t)(s_Ep0.OutLength - remaining) : 0U;
			const bool invalid = remaining > s_Ep0.OutLength ||
				(!s_Ep0.StatusOut && length > s_Ep0.RemainingOut);
			s_Ep0.OutBusy = false;
			if (!invalid && !s_Ep0.StatusOut)
				s_Ep0.RemainingOut -= length;
			if (invalid || length < 64U)
				s_Ep0.RemainingOut = 0U;
			Lpc11UsbEp0Complete(false, length, invalid || s_Ep0.Aborted);
			if (Lpc11UsbEp0Current() && s_Ep0.Generation == generation &&
				s_Ep0.RemainingOut != 0U && !s_Ep0.OutBusy)
				Lpc11UsbEp0ArmOut(64U);
		}
		else if (!s_Ep0.OutBusy && s_Ep0.DirectionIn && !s_Ep0.StatusOut)
		{
			// Premature OUT status aborts an unfinished control read.
			if (!Lpc11UsbRetire(2U))
			{
				Lpc11UsbEp0Fault();
				return;
			}
			if (!Lpc11UsbEp0Current())
				return;
			s_Ep0.InBusy = false;
			s_Ep0.Aborted = true;
			s_Ep0.StatusOut = true;
			Lpc11UsbCommand(LPC11_USB_NAK_CONTROL, 0U, 0U);
			Lpc11UsbEp0ArmOut(0U);
		}
	}
}

void USB_IRQHandler(void)
{
	const uint32_t state = DisableInterrupt();
	if (!s_Prepared || !s_Ep0.Enabled)
	{
		EnableInterrupt(state);
		return;
	}
	const uint32_t generation = s_Ep0.Generation;
	const uint32_t pending = LPC_USB->INTSTAT & LPC_USB->INTEN;
	const uint32_t status = LPC_USB->DEVCMDSTAT;
	if ((pending & LPC11_USB_DEVICE_INTERRUPT) != 0U)
	{
		LPC_USB->INTSTAT = LPC11_USB_DEVICE_INTERRUPT;
		if ((status & LPC11_USB_RESET) != 0U)
		{
			LPC_USB->INTSTAT = 0x3FFU;
			if (!Lpc11UsbRetire(0x3FFU))
				Lpc11UsbEp0Fault();
			else
			{
				Lpc11UsbEp0Reset();
				for (unsigned physical = 2U; physical < 10U; ++physical)
				{
					s_Memory.Command[physical * 2U] = LPC11_USB_DISABLED;
					s_Memory.Command[physical * 2U + 1U] = LPC11_USB_DISABLED;
				}
				Lpc11UsbCommand(0x7FU | LPC11_USB_NAK_CONTROL, 0U,
					status & (LPC11_USB_RESET | LPC11_USB_CONNECT_CHANGE | LPC11_USB_SUSPEND_CHANGE));
				if ((LPC_USB->DEVCMDSTAT & LPC11_USB_SETUP) != 0U)
					LPC_USB->INTSETSTAT = 1U;
				UsbCtrlrEvt_t event = {0};
				event.Type = USB_CTRLR_EVT_RESET;
				UsbDevProcessEvent(0, &event);
			}
			__DSB();
			EnableInterrupt(state);
			return;
		}
		Lpc11UsbHwAckStatus(status & (LPC11_USB_CONNECT_CHANGE | LPC11_USB_SUSPEND_CHANGE));
		if ((status & LPC11_USB_CONNECT_CHANGE) != 0U)
			UsbProcessQue(0);
		if ((status & LPC11_USB_SUSPEND_CHANGE) != 0U)
		{
			UsbCtrlrEvt_t event = {0};
			event.Type = (status & (1U << 17)) != 0U ?
				USB_CTRLR_EVT_SUSPEND : USB_CTRLR_EVT_RESUME;
			UsbDevProcessEvent(0, &event);
		}
	}
	if (s_Prepared && s_Ep0.Enabled && s_Ep0.Generation == generation &&
		(LPC_USB->DEVCMDSTAT & LPC11_USB_RESET) == 0U && (pending & 3U) != 0U)
		Lpc11UsbEp0Interrupt(pending);
	__DSB();
	EnableInterrupt(state);
}
