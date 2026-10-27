/**-------------------------------------------------------------------------
@file	usbd_iso.cpp

@brief	USB device ISO function using the role-neutral transport.

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
#include "usb/usbd_iso.h"

UsbdIso::UsbdIso() = default;

bool UsbdIso::Init(const UsbdIsoCfg_t &Cfg)
{
	const bool internal = Cfg.pRxFifoMem == nullptr && Cfg.pTxFifoMem == nullptr;
	const uint16_t capacity = internal ? USB_ISO_INTRF_MAX_MPS : Cfg.BufferSize;
	if ((!internal && (Cfg.pRxFifoMem == nullptr || Cfg.pTxFifoMem == nullptr ||
		 Cfg.pRxFifoMem == Cfg.pTxFifoMem ||
		 (((uintptr_t)Cfg.pRxFifoMem | (uintptr_t)Cfg.pTxFifoMem) & 3U) != 0U)) ||
		capacity > USB_ISO_INTRF_MAX_MPS ||
		!Register(Cfg.Function, USB_ENDPATT_TRANS_ISO, capacity))
	{
		return false;
	}
	UsbIsoIntrfCfg_t cfg = {};
	cfg.DevNo = Cfg.Function.DevNo;
	cfg.EpNo = Endpoint();
	cfg.BufferSize = capacity;
	cfg.pRxFifoMem = Cfg.pRxFifoMem;
	cfg.pTxFifoMem = Cfg.pTxFifoMem;
	cfg.EvtCB = Cfg.EvtCB;
	cfg.pContext = Cfg.pContext;
	if (!UsbIsoIntrf::Init(cfg))
	{
		Rollback();
		return false;
	}
	return true;
}

void UsbdIso::CloseTransport(void)
{
	UsbIsoIntrf_t *iso = *this;
	if (iso->pData != nullptr)
	{
		UsbIsoIntrf::Close();
	}
}

void UsbdIso::Reset(void)
{
	UsbdPeriodic::Reset();
	UsbIsoIntrf_t *iso = *this;
	if (iso->pData != nullptr)
	{
		UsbIsoIntrf::Reset();
	}
}

void UsbdIso::Process(void)
{
	// Bus reset and SET_INTERFACE can run in interrupt context. Do not let
	// a stale foreground resume override a close/reset in between checks.
	const uint32_t state = DisableInterrupt();
	if (Configured() && Alternate() != 0U)
	{
		if (UsbSuspended(DevNo()))
		{
			UsbIsoIntrf::Suspend();
		}
		else
		{
			(void)UsbIsoIntrf::Resume();
		}
	}
	EnableInterrupt(state);
}
