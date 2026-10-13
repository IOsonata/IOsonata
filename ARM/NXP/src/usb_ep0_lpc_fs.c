/**-------------------------------------------------------------------------
@file usb_ep0_lpc_fs.c
@brief Shared LPC full-speed USB control endpoint state machine.

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

#include "coredev/interrupt.h"
#include "usb_ep0_lpc_fs.h"

#define LPCUSB_NO_WAKE ((1UL << 17) | (1UL << 19))
#define LPCUSB_STATUS_W1C ((1UL << 8) | (7UL << 24))
#define LPCUSB_COMMAND_MASK (0xFFUL | (1UL << 9) | (1UL << 11) | (0x1FUL << 12))
#define LPCUSB_ACTIVE (1UL << 31)
#define LPCUSB_WAIT_COUNT 1024
#define LPCUSB_STALL (1UL << 29)
#define LPCUSB_SETUP (1UL << 8)
#define LPCUSB_NAK_CONTROL (3UL << 14)
#define LPCUSB_RESET (1UL << 26)

void LpcUsbFsEp0Command(LpcUsbFsEp0_t *pEp0, uint32_t Clear, uint32_t Set, uint32_t Ack)
{
	pEp0->pReg->DEVCMDSTAT = ((pEp0->pReg->DEVCMDSTAT & LPCUSB_COMMAND_MASK) & ~Clear) |
		Set | LPCUSB_NO_WAKE | (Ack & LPCUSB_STATUS_W1C);
}

uint32_t LpcUsbFsEp0BufferAddress(LpcUsbFsEp0_t *pEp0, unsigned Physical)
{
	return ((uint32_t)(uintptr_t)pEp0->pMemory->Packet[Physical] >> 6) & 0xFFFFU;
}

bool LpcUsbFsEp0Retire(LpcUsbFsEp0_t *pEp0, uint32_t Mask)
{
	uint32_t active = 0U;
	for (unsigned physical = 0U; physical < 10U; ++physical)
	{
		if ((Mask & (1U << physical)) != 0U &&
			(pEp0->pMemory->Command[physical * 2U] & LPCUSB_ACTIVE) != 0U)
			active |= 1U << physical;
	}
	if (active != 0U)
		pEp0->pReg->EPSKIP = active;
	__DSB();
	int count = LPCUSB_WAIT_COUNT;
	while ((pEp0->pReg->EPSKIP & Mask) != 0U)
	{
		if (--count == 0)
			return false;
	}
	__DMB();
	for (unsigned physical = 0U; physical < 10U; ++physical)
	{
		if ((Mask & (1U << physical)) != 0U &&
			(pEp0->pMemory->Command[physical * 2U] & LPCUSB_ACTIVE) != 0U)
			return false;
	}
	return true;
}

void LpcUsbFsEp0Reset(LpcUsbFsEp0_t *pEp0)
{
	const uint32_t generation = pEp0->Generation + 1U;
	memset(pEp0, 0, offsetof(LpcUsbFsEp0_t, pReg));
	pEp0->Generation = generation;
	pEp0->Enabled = true;
	pEp0->pMemory->Command[0] = LpcUsbFsEp0BufferAddress(pEp0, 0U);
	pEp0->pMemory->Command[2] = LpcUsbFsEp0BufferAddress(pEp0, 1U);
	__DMB();
}

static bool LpcUsbFsEp0Current(LpcUsbFsEp0_t *pEp0)
{
	return pEp0->Ready() && pEp0->Enabled && !pEp0->Halted &&
		(pEp0->pReg->DEVCMDSTAT & (LPCUSB_SETUP | LPCUSB_RESET)) == 0U;
}

static void LpcUsbFsEp0ArmOut(LpcUsbFsEp0_t *pEp0, uint16_t Length)
{
	pEp0->OutLength = Length;
	pEp0->OutBusy = true;
	__DMB();
	pEp0->pMemory->Command[0] = LpcUsbFsEp0BufferAddress(pEp0, 0U) |
		((uint32_t)Length << 16) | LPCUSB_ACTIVE;
}

int LpcUsbFsEp0Send(LpcUsbFsEp0_t *pEp0, int DevNo, uint8_t *pBuffer, int Length)
{
	if (DevNo != 0 || Length < 0 || (Length != 0 && pBuffer == NULL))
		return -1;
	const uint32_t state = DisableInterrupt();
	if (!LpcUsbFsEp0Current(pEp0) || pEp0->InBusy ||
		(pEp0->pMemory->Command[2] & LPCUSB_ACTIVE) != 0U)
	{
		EnableInterrupt(state);
		return -1;
	}
	const uint16_t length = Length > 64 ? 64U : (uint16_t)Length;
	if (length != 0U)
		memcpy(pEp0->pMemory->Packet[1], pBuffer, length);
	if (!LpcUsbFsEp0Current(pEp0))
	{
		EnableInterrupt(state);
		return -1;
	}
	pEp0->InLength = length;
	pEp0->InBusy = true;
	pEp0->pReg->INTSTAT = 2U;
	__DMB();
	pEp0->pMemory->Command[2] = LpcUsbFsEp0BufferAddress(pEp0, 1U) |
		((uint32_t)length << 16) | LPCUSB_ACTIVE;
	EnableInterrupt(state);
	return length;
}

bool LpcUsbFsEp0Status(LpcUsbFsEp0_t *pEp0, int DevNo, uint8_t EpAddr)
{
	if (DevNo != 0 || (EpAddr != 0U && EpAddr != 0x80U))
		return false;
	const uint32_t state = DisableInterrupt();
	if (!LpcUsbFsEp0Current(pEp0))
	{
		EnableInterrupt(state);
		return false;
	}
	bool accepted = false;
	if (EpAddr == 0x80U && !pEp0->OutBusy)
	{
		accepted = LpcUsbFsEp0Send(pEp0, 0, NULL, 0) == 0;
		if (accepted)
		{
			pEp0->RemainingOut = 0U;
			LpcUsbFsEp0Command(pEp0, LPCUSB_NAK_CONTROL, 0U, 0U);
		}
	}
	else if (EpAddr == 0U && !pEp0->OutBusy && !pEp0->InBusy &&
		(pEp0->pMemory->Command[0] & LPCUSB_ACTIVE) == 0U)
	{
		pEp0->StatusOut = true;
		LpcUsbFsEp0Command(pEp0, LPCUSB_NAK_CONTROL, 0U, 0U);
		LpcUsbFsEp0ArmOut(pEp0, 0U);
		accepted = true;
	}
	EnableInterrupt(state);
	return accepted;
}

void LpcUsbFsEp0SetAddress(LpcUsbFsEp0_t *pEp0, int DevNo, uint8_t Address)
{
	if (DevNo != 0 || Address > 127U)
		return;
	const uint32_t state = DisableInterrupt();
	if (LpcUsbFsEp0Current(pEp0))
	{
		// UM10462 table 197: program BEFORE the SET_ADDRESS status phase.
		if (!pEp0->AddressPending)
			pEp0->OldAddress = pEp0->pReg->DEVCMDSTAT & 0x7FU;
		pEp0->AddressPending = true;
		LpcUsbFsEp0Command(pEp0, 0x7FU, Address, 0U);
	}
	EnableInterrupt(state);
}

static void LpcUsbFsEp0Complete(LpcUsbFsEp0_t *pEp0, bool In, uint16_t Length, bool Cancelled)
{
	UsbCtrlrEvt_t event = {0};
	event.Type = USB_CTRLR_EVT_XFER_CMPL;
	event.Xfer.EpAddr = In ? 0x80U : 0U;
	event.Xfer.Length = Length;
	event.Xfer.Result = Cancelled ? USB_CTRLR_XFER_CANCELLED : USB_CTRLR_XFER_SUCCESS;
	event.Xfer.pBuffer = In ? NULL : pEp0->pMemory->Packet[0];
	pEp0->Event(&event);
}

static void LpcUsbFsEp0Setup(LpcUsbFsEp0_t *pEp0, uint32_t Pending)
{
	// A new SETUP can share an IRQ with the preceding control-write status
	// ACK. Complete it first so acknowledged class requests are committed
	// before the generic core starts the new request. A control-read data
	// ZLP is not a status ACK and must not take this path.
	if (!pEp0->DirectionIn && pEp0->RemainingOut == 0U &&
		pEp0->InBusy && pEp0->InLength == 0U &&
		(Pending & 2U) != 0U && (pEp0->pMemory->Command[2] & LPCUSB_ACTIVE) == 0U)
	{
		const uint32_t generation = pEp0->Generation;
		pEp0->AddressPending = false;
		pEp0->InBusy = false;
		LpcUsbFsEp0Complete(pEp0, true, 0U, pEp0->Aborted);
		if (!pEp0->Ready() || !pEp0->Enabled || pEp0->Generation != generation)
			return;
	}
	else if (pEp0->StatusOut && pEp0->OutBusy &&
		(Pending & 1U) != 0U && (pEp0->pMemory->Command[0] & LPCUSB_ACTIVE) == 0U)
	{
		// OUT status shares the SETUP interrupt. Preserve its completion
		// before the new request replaces the active class callback.
		const uint32_t generation = pEp0->Generation;
		__DMB();
		const bool invalid = ((pEp0->pMemory->Command[0] >> 16) & 0x3FFU) != 0U;
		pEp0->OutBusy = false;
		LpcUsbFsEp0Complete(pEp0, false, 0U, invalid || pEp0->Aborted);
		if (!pEp0->Ready() || !pEp0->Enabled || pEp0->Generation != generation)
			return;
	}
	UsbCtrlrEvt_t event = {0};
	event.Type = USB_CTRLR_EVT_SETUP;
	__DMB();
	memcpy(&event.Setup, pEp0->pMemory->Setup, sizeof(event.Setup));
	if (pEp0->AddressPending)
		LpcUsbFsEp0Command(pEp0, 0x7FU, pEp0->OldAddress, 0U);
	// Figure 21 explicitly requires clearing both EP0 ACTIVE/STALL words
	// while SETUP holds IN/OUT NAKed, BEFORE acknowledging SETUP. Hardware
	// resets both control data toggles to DATA1 on reception of SETUP.
	LpcUsbFsEp0Reset(pEp0);
	pEp0->DirectionIn = (event.Setup.bmRequestType & 0x80U) != 0U;
	pEp0->RemainingOut = pEp0->DirectionIn ? 0U : event.Setup.wLength;
	pEp0->pReg->INTSTAT = 2U;
	LpcUsbFsEp0Command(pEp0, LPCUSB_NAK_CONTROL,
		pEp0->DirectionIn ? (1U << 14) :
		(pEp0->RemainingOut != 0U ? (1U << 15) : 0U), LPCUSB_SETUP);
	const uint32_t generation = pEp0->Generation;
	pEp0->Event(&event);
	if (LpcUsbFsEp0Current(pEp0) && pEp0->Generation == generation &&
		pEp0->RemainingOut != 0U && !pEp0->OutBusy)
		LpcUsbFsEp0ArmOut(pEp0, 64U);
}

void LpcUsbFsEp0Interrupt(LpcUsbFsEp0_t *pEp0, uint32_t Pending)
{
	// EP0 OUT shares its interrupt with SETUP. Acknowledge BEFORE reading
	// SETUP, once only. A SETUP arriving afterward keeps its new interrupt;
	// one arriving before this write is handled by the following status read.
	pEp0->pReg->INTSTAT = Pending & 3U;
	if ((pEp0->pReg->DEVCMDSTAT & LPCUSB_SETUP) != 0U)
	{
		LpcUsbFsEp0Setup(pEp0, Pending);
		return;
	}
	if (!LpcUsbFsEp0Current(pEp0))
		return;
	const uint32_t generation = pEp0->Generation;
	// IN completion first: the generic callback may arm status OUT. A NAK
	// interrupt already latched for that OUT must not complete the new bank.
	if ((Pending & 2U) != 0U)
	{
		if (pEp0->InBusy && (pEp0->pMemory->Command[2] & LPCUSB_ACTIVE) == 0U)
		{
			const uint16_t length = pEp0->InLength;
			pEp0->InBusy = false;
			if (pEp0->AddressPending && length == 0U)
				pEp0->AddressPending = false;
			LpcUsbFsEp0Complete(pEp0, true, length, pEp0->Aborted);
		}
		else if (!pEp0->InBusy && pEp0->RemainingOut != 0U &&
			!((Pending & 1U) != 0U && pEp0->OutBusy &&
			  (pEp0->pMemory->Command[0] & LPCUSB_ACTIVE) == 0U))
		{
			// Host ends a control write early. Retire OUT before acknowledging
			// its IN status request. Notify the core after the status ACK.
			if (!LpcUsbFsEp0Retire(pEp0, 1U))
			{
				pEp0->Fault();
				return;
			}
			if (!LpcUsbFsEp0Current(pEp0))
				return;
			pEp0->OutBusy = false;
			pEp0->RemainingOut = 0U;
			pEp0->Aborted = true;
			LpcUsbFsEp0Command(pEp0, LPCUSB_NAK_CONTROL, 0U, 0U);
			(void)LpcUsbFsEp0Send(pEp0, 0, NULL, 0);
		}
	}
	if (!LpcUsbFsEp0Current(pEp0) || pEp0->Generation != generation)
		return;
	if ((Pending & 1U) != 0U)
	{
		if (pEp0->OutBusy && (pEp0->pMemory->Command[0] & LPCUSB_ACTIVE) == 0U)
		{
			__DMB();
			const uint16_t remaining = (pEp0->pMemory->Command[0] >> 16) & 0x3FFU;
			const uint16_t length = remaining <= pEp0->OutLength ?
				(uint16_t)(pEp0->OutLength - remaining) : 0U;
			const bool invalid = remaining > pEp0->OutLength ||
				(!pEp0->StatusOut && length > pEp0->RemainingOut);
			pEp0->OutBusy = false;
			if (!invalid && !pEp0->StatusOut)
				pEp0->RemainingOut -= length;
			if (invalid || length < 64U)
				pEp0->RemainingOut = 0U;
			LpcUsbFsEp0Complete(pEp0, false, length, invalid || pEp0->Aborted);
			if (LpcUsbFsEp0Current(pEp0) && pEp0->Generation == generation &&
				pEp0->RemainingOut != 0U && !pEp0->OutBusy)
				LpcUsbFsEp0ArmOut(pEp0, 64U);
		}
		else if (!pEp0->OutBusy && pEp0->DirectionIn && !pEp0->StatusOut)
		{
			// Premature OUT status aborts an unfinished control read.
			if (!LpcUsbFsEp0Retire(pEp0, 2U))
			{
				pEp0->Fault();
				return;
			}
			if (!LpcUsbFsEp0Current(pEp0))
				return;
			pEp0->InBusy = false;
			pEp0->Aborted = true;
			pEp0->StatusOut = true;
			LpcUsbFsEp0Command(pEp0, LPCUSB_NAK_CONTROL, 0U, 0U);
			LpcUsbFsEp0ArmOut(pEp0, 0U);
		}
	}
}

void LpcUsbFsEp0Stall(LpcUsbFsEp0_t *pEp0)
{
	uint32_t state = DisableInterrupt();
	if (LpcUsbFsEp0Current(pEp0))
	{
		if (!LpcUsbFsEp0Retire(pEp0, 3U))
			pEp0->Fault();
		else if (LpcUsbFsEp0Current(pEp0))
		{
			pEp0->InBusy = false;
			pEp0->OutBusy = false;
			pEp0->Halted = true;
			pEp0->RemainingOut = 0U;
			LpcUsbFsEp0Command(pEp0, LPCUSB_NAK_CONTROL, 0U, 0U);
			pEp0->pMemory->Command[0] = LpcUsbFsEp0BufferAddress(pEp0, 0U) | LPCUSB_STALL;
			pEp0->pMemory->Command[2] = LpcUsbFsEp0BufferAddress(pEp0, 1U) | LPCUSB_STALL;
			pEp0->pReg->INTSTAT = 2U;
		}
	}
	EnableInterrupt(state);
}
