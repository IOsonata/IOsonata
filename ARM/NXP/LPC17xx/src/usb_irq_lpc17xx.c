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
#include "usb_data_lpc17xx.h"

static Lpc17UsbControl_t *s_pControl;
static Lpc17UsbDataEndpoint_t *s_pData;

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

bool Lpc17UsbIrqBindData(Lpc17UsbDataEndpoint_t Endpoint[30])
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	const bool accepted = s_pControl != NULL && s_pControl->Enabled &&
		s_pData == NULL && Endpoint != NULL;
	if (accepted)
		s_pData = Endpoint;
	__set_PRIMASK(state);
	return accepted;
}

void Lpc17UsbIrqUnbind(void)
{
	const uint32_t state = __get_PRIMASK();
	__disable_irq();
	s_pControl = NULL;
	s_pData = NULL;
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

static void Lpc17UsbIrqData(Lpc17UsbControl_t *pControl)
{
	Lpc17UsbDataEndpoint_t *data = s_pData;
	if (data == NULL)
		return;
	const uint32_t controlGeneration = pControl->Generation;
	uint32_t generations[30];
	uint8_t status[32];
	uint32_t pending;
	const uint32_t enabled = LPC_USB->USBEpIntEn & ~3U;
	for (unsigned i = 0; i < 30U; ++i)
	{
		generations[i] = data[i].Generation;
		if ((enabled & (1UL << (i + 2U))) != 0U &&
			(!data[i].Enabled || data[i].EpNo != (i + 2U) / 2U ||
			data[i].In != ((i & 1U) != 0U)))
		{
			Lpc17UsbIrqStop(pControl);
			return;
		}
	}
	if (!Lpc17UsbDataCollect(&pending, status))
	{
		// Partial acknowledgements cannot be replayed, including if a bus
		// event interrupted capture. Disconnect rather than lose ownership.
		Lpc17UsbIrqStop(pControl);
		return;
	}
	for (unsigned i = 0; i < 30U; ++i)
	{
		if (s_pControl != pControl || s_pData != data || !pControl->Enabled ||
			pControl->Generation != controlGeneration)
			return;
		if ((LPC_USB->USBDevIntSt & 8U) != 0U)
		{
			Lpc17UsbIrqStop(pControl);
			return;
		}
		if ((pending & (1UL << (i + 2U))) != 0U &&
			data[i].Generation == generations[i])
			Lpc17UsbDataDispatch(&data[i], generations[i], status[i + 2U]);
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
	// CPU-mode control/data endpoints use slow routing; ISO stays disabled.
	if ((pending & ~0x0CU) != 0U ||
		(LPC_USB->USBEpIntEn & ~(s_pData != NULL ? 0xFCF3CF3FU : 3U)) != 0U)
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
			if (s_pData != NULL)
			{
				// Hardware RST must have retired all data FIFO resources.
				if ((LPC_USB->USBReEp & ~3U) != 0U || LPC_USB->USBEpDMASt != 0U ||
					LPC_USB->USBDMAIntEn != 0U)
				{
					Lpc17UsbIrqStop(pControl);
					__set_PRIMASK(state);
					return;
				}
				Lpc17UsbDataEndpoint_t *data = s_pData;
				const uint32_t generation = pControl->Generation;
				LPC_USB->USBEpIntEn &= 3U;
				pControl->Enabled = false;
				for (unsigned i = 0; i < 30U; ++i)
				{
					Lpc17UsbDataForget(&data[i]);
					if (s_pControl != pControl || s_pData != data ||
						pControl->Generation != generation)
					{
						__set_PRIMASK(state);
						return;
					}
				}
				pControl->Enabled = true;
			}
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
			Lpc17UsbIrqData(pControl);
		if (s_pControl == pControl && pControl->Enabled)
			Lpc17UsbIrqSummary();
	}
	__set_PRIMASK(state);
}
