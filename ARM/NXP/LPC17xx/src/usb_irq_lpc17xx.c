/**-------------------------------------------------------------------------
@file	usb_irq_lpc17xx.c

@brief	LPC17xx CPU-mode USB interrupt dispatch.

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

static Lpc17UsbControl_t *s_pControl;

bool Lpc17UsbIrqBind(Lpc17UsbControl_t *pControl)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	const bool accepted = s_pControl == NULL && pControl != NULL &&
		pControl->Enabled && pControl->Event != NULL && pControl->Fault != NULL;
	if (accepted)
		s_pControl = pControl;
	__set_PRIMASK(state);
	return accepted;
}

void Lpc17UsbIrqUnbind(void)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	s_pControl = NULL;
	__set_PRIMASK(state);
}

static void Lpc17UsbIrqStop(Lpc17UsbControl_t *pControl)
{
	LPC_USB->USBDevIntEn = 0U;
	LPC_USB->USBEpIntEn = 0U;
	Lpc17UsbControlReset(pControl, false);
	pControl->Fault();
}

static void Lpc17UsbIrqSummary(void)
{
	// UM10360 11.10.2.4: endpoint interrupts precede their summary clear.
	if (LPC_USB->USBEpIntSt == 0U)
	{
		LPC_USB->USBDevIntClr = 4U;
		// An endpoint can arrive between the test and W1C. Restore its
		// summary so the next IRQ services it; never spin in this handler.
		if ((LPC_USB->USBEpIntSt & LPC_USB->USBEpIntEn) != 0U)
			LPC_USB->USBDevIntSet = 4U;
	}
}

void USB_IRQHandler(void)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	Lpc17UsbControl_t *pControl = s_pControl;
	if (pControl == NULL || !pControl->Enabled ||
		(LPC_SC->PCONP & (1UL << 31)) == 0U)
	{
		__set_PRIMASK(state);
		return;
	}
	const uint32_t pending = LPC_USB->USBDevIntSt & LPC_USB->USBDevIntEn;
	// This stage supports EP0 in CPU mode and DEV_STAT, with slow routing.
	if ((pending & ~0x0CU) != 0U || (LPC_USB->USBEpIntEn & ~3U) != 0U)
	{
		Lpc17UsbIrqStop(pControl);
		__set_PRIMASK(state);
		return;
	}
	if ((pending & 8U) != 0U)
	{
		uint8_t status;
		const int result = Lpc17UsbDeviceStatus(&status);
		if (result < 0 || (result > 0 && (status & 3U) == 2U))
		{
			// VBUS loss requires the same stop/reinitialize path as a fault.
			Lpc17UsbIrqStop(pControl);
			__set_PRIMASK(state);
			return;
		}
		if (result > 0 && (status & 0x10U) != 0U)
		{
			Lpc17UsbControlBusReset(pControl);
			if (s_pControl == pControl && pControl->Enabled)
				Lpc17UsbIrqSummary();
			__set_PRIMASK(state);
			return; // never dispatch the pre-reset endpoint snapshot
		}
		if (result > 0 && (status & 8U) != 0U)
		{
			const uint32_t generation = pControl->Generation;
			UsbCtrlrEvt_t event = {0};
			event.Type = (status & 4U) != 0U ? USB_CTRLR_EVT_SUSPEND : USB_CTRLR_EVT_RESUME;
			pControl->Event(&event);
			if (s_pControl != pControl || !pControl->Enabled ||
				pControl->Generation != generation)
			{
				__set_PRIMASK(state);
				return;
			}
		}
	}
	if ((pending & 4U) != 0U)
	{
		Lpc17UsbControlInterrupt(pControl);
		if (s_pControl == pControl && pControl->Enabled)
			Lpc17UsbIrqSummary();
	}
	__set_PRIMASK(state);
}
