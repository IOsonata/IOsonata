/**-------------------------------------------------------------------------
@file	usbd_epalloc.cpp

@brief	Internal USB device interface/endpoint allocator.

Classes register once before the controller starts. Resource ownership is
therefore monotonic for one UsbInit() lifetime: keep compact masks for the
already assigned interfaces and endpoint directions, select the lowest free
resources once, then register that topology with the core.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc. All rights reserved.

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
#include <string.h>

#include "usb/usbd_epalloc.h"

typedef struct __Usbd_EpAlloc_State {
	uint16_t InterfaceMask;
	uint16_t InMask;
	uint16_t OutMask;
} UsbdEpAllocState_t;

static UsbdEpAllocState_t s_State[USB_CTRLR_CNT];

static bool EpAllocInterfaces(uint8_t Count, uint16_t Used,
							 uint8_t *pFirst, uint16_t *pMask)
{
	if (Count == 0U)
	{
		*pFirst = 0U;
		*pMask = 0U;
		return true;
	}

	const uint32_t width = (1UL << Count) - 1UL;
	for (uint8_t first = 0U; first <= 16U - Count; first++)
	{
		const uint16_t mask = (uint16_t)(width << first);
		if ((Used & mask) == 0U)
		{
			*pFirst = first;
			*pMask = mask;
			return true;
		}
	}
	return false;
}

static bool EpAllocOne(uint16_t Used, uint8_t Limit, uint8_t *pEp)
{
	uint16_t mask = (uint16_t)((1UL << Limit) - 1UL);
	mask &= (uint16_t)~Used;
	mask &= (uint16_t)~1U;
	if (mask == 0U)
	{
		return false;
	}
	*pEp = (uint8_t)__builtin_ctz((unsigned)mask);
	return true;
}

void UsbdEpAllocReset(int DevNo)
{
	if (DevNo >= 0 && DevNo < USB_CTRLR_CNT)
	{
		memset(&s_State[DevNo], 0, sizeof(s_State[DevNo]));
	}
}

bool UsbdEpAlloc(int DevNo, const UsbdEpAllocReq_t *pReq,
				 UsbDeviceClass *pClass, UsbdEpAllocRes_t *pRes)
{
	if (DevNo < 0 || DevNo >= USB_CTRLR_CNT ||
		pReq == nullptr || pClass == nullptr || pRes == nullptr ||
		pReq->InterfaceCount > 16U ||
		pReq->BidirectionalCount > USBD_EPALLOC_EP_MAXCNT ||
		pReq->InCount > USBD_EPALLOC_EP_MAXCNT ||
		pReq->OutCount > USBD_EPALLOC_EP_MAXCNT)
	{
		return false;
	}

	UsbdEpAllocState_t &state = s_State[DevNo];
	uint16_t inMask = pReq->FixedInMask;
	uint16_t outMask = pReq->FixedOutMask;
	if ((inMask & state.InMask) != 0U || (outMask & state.OutMask) != 0U)
	{
		return false;
	}

	const uint8_t inLimit = USB_EPIN_CNT(DevNo) < 16U ?
		USB_EPIN_CNT(DevNo) : 16U;
	const uint8_t outLimit = USB_EPOUT_CNT(DevNo) < 16U ?
		USB_EPOUT_CNT(DevNo) : 16U;
	const uint8_t pairLimit = inLimit < outLimit ? inLimit : outLimit;

	uint8_t firstInterface;
	uint16_t interfaceMask;
	if (!EpAllocInterfaces(pReq->InterfaceCount, state.InterfaceMask,
		&firstInterface, &interfaceMask))
	{
		return false;
	}

	memset(pRes, 0, sizeof(*pRes));
	pRes->FirstInterface = firstInterface;

	for (uint8_t i = 0U; i < pReq->InCount; i++)
	{
		uint8_t ep;
		if (!EpAllocOne((uint16_t)(state.InMask | inMask), inLimit, &ep))
		{
			return false;
		}
		inMask |= (uint16_t)(1U << ep);
		pRes->In[i] = ep;
	}

	for (uint8_t i = 0U; i < pReq->BidirectionalCount; i++)
	{
		uint8_t ep;
		const uint16_t used = (uint16_t)(
			state.InMask | state.OutMask | inMask | outMask);
		if (!EpAllocOne(used, pairLimit, &ep))
		{
			return false;
		}
		const uint16_t bit = (uint16_t)(1U << ep);
		inMask |= bit;
		outMask |= bit;
		pRes->Bidirectional[i] = ep;
	}

	for (uint8_t i = 0U; i < pReq->OutCount; i++)
	{
		uint8_t ep;
		if (!EpAllocOne((uint16_t)(state.OutMask | outMask), outLimit, &ep))
		{
			return false;
		}
		outMask |= (uint16_t)(1U << ep);
		pRes->Out[i] = ep;
	}

	if (!UsbClassRegister(DevNo, pClass, firstInterface,
		pReq->InterfaceCount, inMask, outMask))
	{
		return false;
	}

	state.InterfaceMask |= interfaceMask;
	state.InMask |= inMask;
	state.OutMask |= outMask;
	return true;
}
