/**-------------------------------------------------------------------------
@file	usb_control_lpc17xx.c

@brief	Private LPC17xx control endpoint event handling.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#include "LPC17xx.h"
#include "usb_hw_lpc17xx.h"
#include "usb_control_lpc17xx.h"

#define LPC17_EP_FULL 1U
#define LPC17_EP_SETUP 4U
#define LPC17_EP_OVERWRITE 8U
#define LPC17_EP_NAK 16U

void Lpc17UsbControlReset(Lpc17UsbControl_t *pControl, bool Enable)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	++pControl->Generation;
	pControl->RemainingOut = 0;
	pControl->InLength = 0;
	pControl->Enabled = Enable && pControl->Event != NULL && pControl->Fault != NULL;
	pControl->InBusy = false;
	pControl->StatusIn = false;
	pControl->StatusOut = false;
	pControl->DirectionIn = false;
	pControl->Halted = false;
	__set_PRIMASK(state);
}

static void Lpc17UsbControlFault(Lpc17UsbControl_t *pControl)
{
	Lpc17UsbControlReset(pControl, false);
	pControl->Fault();
}

// A SETUP or bus state change can arrive while PRIMASK is held. Leave it for
// the ISR before submitting or reporting another packet for the old request.
static bool Lpc17UsbControlCurrent(Lpc17UsbControl_t *pControl)
{
	if (!pControl->Enabled || pControl->Halted ||
		(LPC_USB->USBDevIntSt & (1U << 3)) != 0U)
		return false;
	uint8_t status;
	if (!Lpc17UsbSieRead(0, &status))
	{
		Lpc17UsbControlFault(pControl);
		return false;
	}
	return (status & (LPC17_EP_SETUP | LPC17_EP_OVERWRITE)) == 0U;
}

int Lpc17UsbControlSend(Lpc17UsbControl_t *pControl, const uint8_t *pData, int Length)
{
	if (Length < 0 || (Length != 0 && pData == NULL))
		return -1;
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	int result = -1;
	if (!pControl->InBusy && Lpc17UsbControlCurrent(pControl))
	{
		uint8_t status;
		const int length = Length > 64 ? 64 : Length;
		// Retire old IN interrupt state before validating the next packet.
		if (!Lpc17UsbEp0Ack(true, &status) || !Lpc17UsbEp0Write(pData, length))
			Lpc17UsbControlFault(pControl);
		else if (Lpc17UsbControlCurrent(pControl))
		{
			pControl->InLength = (uint16_t)length;
			pControl->InBusy = true;
			result = length;
		}
	}
	__set_PRIMASK(state);
	return result;
}

bool Lpc17UsbControlStatus(Lpc17UsbControl_t *pControl, uint8_t EpAddr)
{
	if (EpAddr != 0 && EpAddr != 0x80)
		return false;
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	bool accepted = false;
	if (!pControl->InBusy && Lpc17UsbControlCurrent(pControl))
	{
		if (EpAddr == 0x80)
		{
			accepted = Lpc17UsbControlSend(pControl, NULL, 0) == 0;
			pControl->StatusIn = accepted;
		}
		else
		{
			// The CPU FIFO accepts OUT automatically when its buffer is empty.
			pControl->StatusOut = true;
			accepted = true;
		}
	}
	__set_PRIMASK(state);
	return accepted;
}

void Lpc17UsbControlStall(Lpc17UsbControl_t *pControl)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	if (Lpc17UsbControlCurrent(pControl))
	{
		pControl->Halted = true;
		pControl->InBusy = false;
		pControl->StatusIn = false;
		pControl->StatusOut = false;
		pControl->RemainingOut = 0;
		if (!Lpc17UsbSieWrite(0x40, 1) || !Lpc17UsbSieWrite(0x41, 1))
			Lpc17UsbControlFault(pControl);
	}
	__set_PRIMASK(state);
}

static void Lpc17UsbControlComplete(Lpc17UsbControl_t *pControl, bool In,
		uint16_t Length, const uint8_t *pData, bool Invalid)
{
	UsbCtrlrEvt_t event = {0};
	event.Type = USB_CTRLR_EVT_XFER_CMPL;
	event.Xfer.EpAddr = In ? 0x80 : 0;
	event.Xfer.Length = Length;
	event.Xfer.pBuffer = pData;
	event.Xfer.Result = Invalid ? USB_CTRLR_XFER_CANCELLED : USB_CTRLR_XFER_SUCCESS;
	pControl->Event(&event);
}

static void Lpc17UsbControlDispatch(Lpc17UsbControl_t *pControl)
{
	if (!pControl->Enabled || (LPC_USB->USBDevIntSt & (1U << 3)) != 0U)
		return;
	uint8_t status[2];
	const int pending = Lpc17UsbEp0Collect(status);
	if (pending < 0)
	{
		Lpc17UsbControlFault(pControl);
		return;
	}
	if (pending == 0 || (LPC_USB->USBDevIntSt & (1U << 3)) != 0U)
		return;
	const uint32_t generation = pControl->Generation;
	bool inDone = (pending & 2) != 0 && pControl->InBusy &&
		(status[1] & (LPC17_EP_FULL | LPC17_EP_NAK | 2U)) == 0U;
	// An acknowledged status belongs to the preceding request even when a
	// new SETUP arrives after collection or overwrites the OUT FIFO below.
	// Control NAK interrupts must be disabled by the controller initializer.
	if (inDone && pControl->StatusIn)
	{
		pControl->InBusy = false;
		pControl->StatusIn = false;
		inDone = false;
		Lpc17UsbControlComplete(pControl, true, 0, NULL, false);
		if (!pControl->Enabled || pControl->Generation != generation)
			return;
	}
	uint8_t packet[64];
	int length = -1;
	if ((pending & 1) != 0 && (status[0] & LPC17_EP_FULL) != 0U)
	{
		length = Lpc17UsbEp0Read(packet, sizeof(packet));
		if (length == -2)
			return; // newer SETUP stays pending; retry on the next IRQ entry
		if (length < 0)
		{
			Lpc17UsbControlFault(pControl);
			return;
		}
	}
	if ((pending & 1) != 0 && (status[0] & LPC17_EP_SETUP) != 0U)
	{
		// SETUP automatically unstalls EP0 and invalidates its IN buffer.
		pControl->Halted = false;
		if (!Lpc17UsbControlCurrent(pControl))
			return;
		if (length != (int)sizeof(UsbSetupData_t))
		{
			Lpc17UsbControlStall(pControl);
			return;
		}
		UsbCtrlrEvt_t event = {0};
		event.Type = USB_CTRLR_EVT_SETUP;
		memcpy(&event.Setup, packet, sizeof(event.Setup));
		Lpc17UsbControlReset(pControl, true);
		pControl->DirectionIn = (event.Setup.bmRequestType & 0x80U) != 0U;
		pControl->RemainingOut = pControl->DirectionIn ? 0 : event.Setup.wLength;
		pControl->Event(&event);
		return;
	}
	if (!Lpc17UsbControlCurrent(pControl))
		return;
	if (inDone)
	{
		const uint16_t sent = pControl->InLength;
		pControl->InBusy = false;
		pControl->StatusIn = false;
		Lpc17UsbControlComplete(pControl, true, sent, NULL, false);
	}
	if (!pControl->Enabled || pControl->Generation != generation ||
		!Lpc17UsbControlCurrent(pControl))
		return;
	if (length >= 0)
	{
		const bool invalid = pControl->StatusOut ? length != 0 :
			(pControl->DirectionIn || pControl->RemainingOut == 0 ||
			 length > pControl->RemainingOut);
		if (!invalid && !pControl->StatusOut)
			pControl->RemainingOut -= (uint16_t)length;
		if (invalid || length < 64)
			pControl->RemainingOut = 0;
		pControl->StatusOut = false;
		Lpc17UsbControlComplete(pControl, false, (uint16_t)length, packet, invalid);
	}
}

void Lpc17UsbControlInterrupt(Lpc17UsbControl_t *pControl)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	Lpc17UsbControlDispatch(pControl);
	__set_PRIMASK(state);
}
