/**-------------------------------------------------------------------------
@file	usbd_int.cpp

@brief	USB device Interrupt function using the role-neutral transport.

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
#include "usb/usbd_int.h"

UsbdInt::UsbdInt() = default;

bool UsbdInt::Init(const UsbdIntCfg_t &Cfg)
{
	if (Cfg.pRxBuffer == nullptr || Cfg.pTxBuffer == nullptr ||
		Cfg.pRxBuffer == Cfg.pTxBuffer ||
		(((uintptr_t)Cfg.pRxBuffer | (uintptr_t)Cfg.pTxBuffer) & 3U) != 0U ||
		!Register(Cfg.Function, USB_ENDPATT_TRANS_INT, USB_INT_INTRF_MAX_MPS))
	{
		return false;
	}
	UsbIntIntrfCfg_t cfg = {};
	cfg.DevNo = Cfg.Function.DevNo;
	cfg.EpNo = Endpoint();
	cfg.EvtCB = Cfg.EvtCB;
	cfg.pContext = Cfg.pContext;
	cfg.pRxBuffer = Cfg.pRxBuffer;
	cfg.pTxBuffer = Cfg.pTxBuffer;
	if (!UsbIntIntrf::Init(cfg))
	{
		Rollback();
		return false;
	}
	return true;
}

void UsbdInt::CloseTransport(void)
{
	if (vUsbIntIntrf.pData != nullptr)
	{
		UsbIntIntrf::Close();
	}
}

void UsbdInt::Reset(void)
{
	UsbdPeriodic::Reset();
	if (vUsbIntIntrf.pData != nullptr)
	{
		UsbIntIntrf::Reset();
	}
}
