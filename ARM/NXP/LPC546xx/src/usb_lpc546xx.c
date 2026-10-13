/**-------------------------------------------------------------------------
@file usb_lpc546xx.c
@brief Private LPC546xx USB0 control transfers and interrupt handling.

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
#include "coredev/interrupt.h"
#include "usb_control_lpc546xx.h"
#include "usb_ep0_lpc_fs.h"
#include "usb_hw_lpc546xx.h"

#define LPCUSB_DEVICE (1UL << 31)
#define LPCUSB_RESET (1UL << 26)
#define LPCUSB_CONNECT_CHANGE (1UL << 24)
#define LPCUSB_SUSPEND_CHANGE (1UL << 25)
#define LPCUSB_SETUP (1UL << 8)
#define LPCUSB_ENABLE (1UL << 7)
#define LPCUSB_CONNECT (1UL << 16)
#define LPCUSB_NAK_CONTROL (3UL << 14)

static bool Lpc546xxUsbControlReady(void);
static void Lpc546xxUsbControlFault(void);

static bool s_Prepared;
static bool s_Closing;
static LpcUsbFsEp0_t s_Ep0 = {
	.pReg = USB0,
	.Ready = Lpc546xxUsbControlReady,
	.Fault = Lpc546xxUsbControlFault,
};

static bool Lpc546xxUsbControlReady(void)
{
	return s_Prepared && !s_Closing;
}

static void Lpc546xxUsbControlFault(void)
{
	s_Ep0.Enabled = false;
	++s_Ep0.Generation;
	USB0->INTEN = 0U;
	NVIC_DisableIRQ(USB0_IRQn);
	LpcUsbFsEp0Command(&s_Ep0, LPCUSB_CONNECT, 0U, 0U);
	// Keep storage and clocks owned until Release can retire every bank.
}

bool Lpc546xxUsbControlPrepare(int IntPrio, void (*Event)(const UsbCtrlrEvt_t *))
{
	const uint32_t state = DisableInterrupt();
	if (s_Prepared || Event == NULL)
	{
		EnableInterrupt(state);
		return false;
	}
	LpcUsbFsMemory_t *memory = Lpc546xxUsbHwPrepare(IntPrio);
	if (memory == NULL)
	{
		EnableInterrupt(state);
		return false;
	}
	s_Ep0.pMemory = memory;
	s_Ep0.Event = Event;
	s_Prepared = true;
	s_Closing = false;
	LpcUsbFsEp0Reset(&s_Ep0);
	USB0->INTSTAT = 0xC00003FFU;
	LpcUsbFsEp0Command(&s_Ep0, 0x7FU | LPCUSB_NAK_CONTROL,
		LPCUSB_ENABLE, LPCUSB_SETUP | (7U << 24));
	USB0->INTEN = LPCUSB_DEVICE | 3U;
	EnableInterrupt(state);
	return true;
}

bool Lpc546xxUsbControlRelease(void)
{
	const uint32_t state = DisableInterrupt();
	if (!s_Prepared)
	{
		EnableInterrupt(state);
		return true;
	}
	s_Closing = true;
	Lpc546xxUsbControlFault();
	if (!LpcUsbFsEp0Retire(&s_Ep0, 0x3FFU))
	{
		EnableInterrupt(state);
		return false;
	}
	LpcUsbFsEp0Command(&s_Ep0, LPCUSB_ENABLE | LPCUSB_NAK_CONTROL, 0U, 0U);
	const bool released = Lpc546xxUsbHwRelease();
	if (released)
	{
		s_Prepared = false;
		s_Closing = false;
		s_Ep0.pMemory = NULL;
		s_Ep0.Event = NULL;
	}
	EnableInterrupt(state);
	return released;
}

int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length)
{
	return LpcUsbFsEp0Send(&s_Ep0, DevNo, pBuffer, Length);
}

bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr)
{
	return LpcUsbFsEp0Status(&s_Ep0, DevNo, EpAddr);
}

void UsbCtrlrSetAddress(int DevNo, uint8_t Address)
{
	LpcUsbFsEp0SetAddress(&s_Ep0, DevNo, Address);
}

void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool bIn)
{
	(void)bIn;
	if (DevNo == 0 && EpNo == 0U)
		LpcUsbFsEp0Stall(&s_Ep0);
}

static void Lpc546xxUsbBusReset(uint32_t Status)
{
	if (!LpcUsbFsEp0Retire(&s_Ep0, 0x3FFU))
	{
		Lpc546xxUsbControlFault();
		return;
	}
	LpcUsbFsEp0Reset(&s_Ep0);
	USB0->INTSTAT = 0x3FFU;
	// A SETUP following bus reset must survive this interrupt. Cable loss
	// instead discards the old request, even if VBUS has returned already.
	const bool clearSetup = (Status & LPCUSB_RESET) == 0U;
	LpcUsbFsEp0Command(&s_Ep0, 0x7FU | LPCUSB_NAK_CONTROL, 0U,
		(Status & (LPCUSB_RESET | LPCUSB_CONNECT_CHANGE | LPCUSB_SUSPEND_CHANGE)) |
		(clearSetup ? LPCUSB_SETUP : 0U));
	if (!clearSetup && (USB0->DEVCMDSTAT & LPCUSB_SETUP) != 0U)
		USB0->INTSETSTAT = 1U;
	UsbCtrlrEvt_t event = {0};
	event.Type = USB_CTRLR_EVT_RESET;
	s_Ep0.Event(&event);
}

void USB0_IRQHandler(void)
{
	const uint32_t state = DisableInterrupt();
	if (!s_Prepared || !s_Ep0.Enabled || s_Closing)
	{
		EnableInterrupt(state);
		return;
	}
	const uint32_t generation = s_Ep0.Generation;
	const uint32_t pending = USB0->INTSTAT & USB0->INTEN;
	const uint32_t status = USB0->DEVCMDSTAT;
	if ((pending & LPCUSB_DEVICE) != 0U)
	{
		USB0->INTSTAT = LPCUSB_DEVICE;
		if ((status & (LPCUSB_RESET | LPCUSB_CONNECT_CHANGE)) != 0U)
		{
			Lpc546xxUsbBusReset(status);
			__DSB();
			EnableInterrupt(state);
			return;
		}
		LpcUsbFsEp0Command(&s_Ep0, 0U, 0U, status & LPCUSB_SUSPEND_CHANGE);
		if ((status & LPCUSB_SUSPEND_CHANGE) != 0U)
		{
			UsbCtrlrEvt_t event = {0};
			event.Type = (status & (1U << 17)) != 0U ?
				USB_CTRLR_EVT_SUSPEND : USB_CTRLR_EVT_RESUME;
			s_Ep0.Event(&event);
		}
	}
	if (Lpc546xxUsbControlReady() && s_Ep0.Enabled && s_Ep0.Generation == generation &&
		(USB0->DEVCMDSTAT & LPCUSB_RESET) == 0U && (pending & 3U) != 0U)
		LpcUsbFsEp0Interrupt(&s_Ep0, pending);
	__DSB();
	EnableInterrupt(state);
}
