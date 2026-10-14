/**-------------------------------------------------------------------------
@file	usb_data_lpc17xx.c

@brief	Private LPC17xx CPU-mode data transfer state.

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
#include "coredev/interrupt.h"
#include "usb_data_lpc17xx.h"
#include "usb_hw_lpc17xx.h"

static bool Lpc17UsbDataCurrent(Lpc17UsbDataEndpoint_t *pEp)
{
	return pEp != NULL && pEp->Enabled && pEp->Ready != NULL && pEp->Ready();
}

static void Lpc17UsbDataNotify(Lpc17UsbDataEndpoint_t *pEp, UsbCtrlrEvtType_t Event, uint16_t Length)
{
	if (pEp->Handler != NULL)
		pEp->Handler(Event, Length, pEp->pContext);
}

bool Lpc17UsbDataOpen(Lpc17UsbDataEndpoint_t *pEp, uint8_t EpNo, bool In, uint16_t Mps)
{
	if (pEp == NULL || EpNo == 0 || EpNo > 15 || Mps == 0 || Mps > 64 ||
		(EpNo % 3 == 0 && EpNo != 15) || (EpNo % 3 != 1 &&
		(Mps != 8 && Mps != 16 && Mps != 32 && Mps != 64)))
		return false;
	const uint32_t state = DisableInterrupt();
	bool accepted = !pEp->Enabled && !pEp->Busy && pEp->Ready != NULL &&
		pEp->Fault != NULL && pEp->Kick != NULL && pEp->Ready();
	if (accepted)
	{
		pEp->EpNo = EpNo;
		pEp->In = In;
		pEp->Mps = Mps;
		pEp->pBuffer = NULL;
		pEp->Enabled = true;
		pEp->Halted = false;
		++pEp->Generation;
	}
	EnableInterrupt(state);
	return accepted;
}

void Lpc17UsbDataForget(Lpc17UsbDataEndpoint_t *pEp)
{
	if (pEp == NULL)
		return;
	const uint32_t state = DisableInterrupt();
	const bool busy = pEp->Busy;
	pEp->Enabled = false;
	pEp->Busy = false;
	pEp->Halted = false;
	pEp->pBuffer = NULL;
	pEp->Mps = 0;
	pEp->Remaining = 0;
	pEp->Transferred = 0;
	pEp->PacketLength = 0;
	++pEp->Generation;
	if (busy)
		Lpc17UsbDataNotify(pEp, USB_CTRLR_EVT_CANCEL, 0);
	EnableInterrupt(state);
}

static bool Lpc17UsbDataStage(Lpc17UsbDataEndpoint_t *pEp)
{
	pEp->PacketLength = pEp->Remaining < pEp->Mps ? pEp->Remaining : pEp->Mps;
	return Lpc17UsbDataWrite(pEp->EpNo, pEp->pBuffer, pEp->PacketLength);
}

bool Lpc17UsbDataSend(Lpc17UsbDataEndpoint_t *pEp, uint8_t *pBuffer, uint16_t Length)
{
	if (Length != 0 && pBuffer == NULL)
		return false;
	const uint32_t state = DisableInterrupt();
	bool accepted = Lpc17UsbDataCurrent(pEp) && pEp->In && !pEp->Busy;
	if (accepted)
	{
		pEp->pBuffer = pBuffer;
		pEp->Remaining = Length;
		pEp->Transferred = 0;
		accepted = pEp->Halted || Lpc17UsbDataStage(pEp);
		if (accepted)
		{
			pEp->Busy = true;
			++pEp->Generation;
		}
		else
		{
			pEp->pBuffer = NULL;
			pEp->Fault();
		}
	}
	EnableInterrupt(state);
	return accepted;
}

bool Lpc17UsbDataReceive(Lpc17UsbDataEndpoint_t *pEp, uint8_t *pBuffer, uint16_t Capacity)
{
	if (pBuffer == NULL)
		return false;
	const uint32_t state = DisableInterrupt();
	bool accepted = Lpc17UsbDataCurrent(pEp) && !pEp->In && !pEp->Busy && Capacity >= pEp->Mps;
	if (accepted)
	{
		pEp->pBuffer = pBuffer;
		pEp->Busy = true;
		if (!pEp->Halted)
			pEp->Kick(pEp->EpNo);
	}
	EnableInterrupt(state);
	return accepted;
}

void Lpc17UsbDataDispatch(Lpc17UsbDataEndpoint_t *pEp, uint32_t Generation, uint8_t Status)
{
	const uint32_t state = DisableInterrupt();
	if (!Lpc17UsbDataCurrent(pEp) || pEp->Halted || pEp->Generation != Generation)
	{
		EnableInterrupt(state);
		return;
	}
	if ((Status & 2U) != 0U)
	{
		pEp->Fault();
		EnableInterrupt(state);
		return;
	}
	if (pEp->In)
	{
		// NAKs are not acknowledgements. Only one IN buffer is staged.
		if (pEp->Busy && (Status & 0x11U) == 0U)
		{
			pEp->Transferred += pEp->PacketLength;
			pEp->Remaining -= pEp->PacketLength;
			if (pEp->Remaining != 0)
			{
				pEp->pBuffer += pEp->PacketLength;
				if (!Lpc17UsbDataStage(pEp))
					pEp->Fault();
			}
			else
			{
				const uint16_t length = pEp->Transferred;
				pEp->Busy = false;
				pEp->pBuffer = NULL;
				Lpc17UsbDataNotify(pEp, USB_CTRLR_EVT_XFER_CMPL, length);
			}
		}
	}
	else if ((Status & 1U) != 0U)
	{
		if (!pEp->Busy)
			Lpc17UsbDataNotify(pEp, USB_CTRLR_EVT_DRDY, 0);
		if (Lpc17UsbDataCurrent(pEp) && pEp->Generation == Generation && pEp->Busy)
		{
			int length = Lpc17UsbDataRead(pEp->EpNo, pEp->pBuffer, pEp->Mps);
			if (length >= 0)
			{
				pEp->Busy = false;
				pEp->pBuffer = NULL;
				Lpc17UsbDataNotify(pEp, USB_CTRLR_EVT_XFER_CMPL, length);
				// A second bulk OUT packet may have shared the cleared IRQ.
				if (Lpc17UsbDataCurrent(pEp) && pEp->Generation == Generation)
					pEp->Kick(pEp->EpNo);
			}
			else if (length != -2)
				pEp->Fault();
		}
	}
	EnableInterrupt(state);
}

void Lpc17UsbDataHalt(Lpc17UsbDataEndpoint_t *pEp, bool Halt)
{
	const uint32_t state = DisableInterrupt();
	if (Lpc17UsbDataCurrent(pEp))
	{
		const bool wasHalted = pEp->Halted;
		pEp->Halted = Halt;
		const uint32_t generation = ++pEp->Generation;
		if (pEp->Busy && !wasHalted)
		{
			const uint16_t length = pEp->In ? pEp->Transferred : 0;
			pEp->Busy = false;
			pEp->pBuffer = NULL;
			Lpc17UsbDataNotify(pEp, USB_CTRLR_EVT_XFER_FAILED, length);
		}
		if (Lpc17UsbDataCurrent(pEp) && pEp->Generation == generation)
		{
			pEp->Halted = Halt;
			if (!Halt)
			{
				if (pEp->In && pEp->Busy)
				{
					if (!Lpc17UsbDataStage(pEp))
						pEp->Fault();
				}
				else if (!pEp->In)
					pEp->Kick(pEp->EpNo);
			}
		}
	}
	EnableInterrupt(state);
}
