/**-------------------------------------------------------------------------
@file	usbd_epalloc.cpp

@brief	Internal USB device interface/endpoint allocator.

Recursive backtracking search over the endpoint space, lowest numbers
first, IN only endpoints placed before bidirectional pairs, pairs before
OUT only. Controller-constrained endpoints requested through the fixed
masks are excluded from the search and merged into the registered masks.
Each complete candidate is offered to the core; the first placement the
core accepts is returned to the caller.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

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
#include <string.h>

#include "usb/usbd_epalloc.h"

typedef struct __Usbd_EpAlloc_State {
	int DevNo;
	const UsbdEpAllocReq_t *pReq;
	const UsbdClassCfg_t *pCfg;
	UsbDeviceClass *pClass;
	UsbdEpAllocRes_t *pRes;
	uint8_t FirstInterface;
	uint8_t InLimit;
	uint8_t OutLimit;
	uint8_t PairLimit;
} UsbdEpAllocState_t;

static void EpAllocStore(uint8_t *pEp, unsigned Count, uint16_t Mask)
{
	unsigned count = 0;

	for (uint8_t ep = 1U; ep < 16U && count < Count; ep++)
	{
		if ((Mask & (uint16_t)(1U << ep)) != 0U)
		{
			pEp[count++] = ep;
		}
	}
}

static unsigned EpAllocMaskCount(uint16_t Mask)
{
	unsigned count = 0U;
	while (Mask != 0U)
	{
		Mask &= (uint16_t)(Mask - 1U);
		count++;
	}
	return count;
}

static bool EpAllocTryOut(const UsbdEpAllocState_t *pState, uint8_t Needed,
						  uint8_t StartEp, uint16_t InMask,
						  uint16_t PairMask, uint16_t OutMask)
{
	if (Needed == 0U)
	{
		UsbdClassCfg_t cfg = *pState->pCfg;
		cfg.FirstInterface = pState->FirstInterface;
		cfg.InterfaceCount = pState->pReq->InterfaceCount;
		cfg.EpInMask = InMask | PairMask;
		cfg.EpOutMask = OutMask | PairMask;

		const bool registered = pState->pClass != nullptr ?
			UsbClassRegister(pState->DevNo, &cfg, pState->pClass) :
			UsbdClassRegister(pState->DevNo, &cfg);
		if (!registered)
		{
			return false;
		}

		memset(pState->pRes, 0, sizeof(*pState->pRes));
		pState->pRes->FirstInterface = pState->FirstInterface;
		EpAllocStore(pState->pRes->Bidirectional,
			pState->pReq->BidirectionalCount, PairMask);
		EpAllocStore(pState->pRes->In, pState->pReq->InCount,
			(uint16_t)(InMask & ~pState->pReq->FixedInMask));
		EpAllocStore(pState->pRes->Out, pState->pReq->OutCount,
			(uint16_t)(OutMask & ~pState->pReq->FixedOutMask));
		return true;
	}

	for (uint8_t ep = StartEp; ep < pState->OutLimit; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);

		if ((PairMask & bit) != 0U ||
			(pState->pReq->FixedOutMask & bit) != 0U)
		{
			continue;
		}

		if (EpAllocTryOut(pState, (uint8_t)(Needed - 1U),
						  (uint8_t)(ep + 1U), InMask, PairMask,
						  OutMask | bit))
		{
			return true;
		}
	}

	return false;
}

static bool EpAllocTryPair(const UsbdEpAllocState_t *pState, uint8_t Needed,
						   uint8_t StartEp, uint16_t InMask,
						   uint16_t PairMask)
{
	if (Needed == 0U)
	{
		return EpAllocTryOut(pState, pState->pReq->OutCount, 1U,
							 InMask, PairMask, pState->pReq->FixedOutMask);
	}

	for (uint8_t ep = StartEp; ep < pState->PairLimit; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);

		if (((InMask | pState->pReq->FixedOutMask) & bit) != 0U)
		{
			continue;
		}

		if (EpAllocTryPair(pState, (uint8_t)(Needed - 1U),
						   (uint8_t)(ep + 1U), InMask, PairMask | bit))
		{
			return true;
		}
	}

	return false;
}

static bool EpAllocTryIn(const UsbdEpAllocState_t *pState, uint8_t Needed,
						 uint8_t StartEp, uint16_t InMask)
{
	if (Needed == 0U)
	{
		return EpAllocTryPair(pState, pState->pReq->BidirectionalCount,
							  1U, InMask, 0U);
	}

	for (uint8_t ep = StartEp; ep < pState->InLimit; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);
		if ((InMask & bit) != 0U)
		{
			continue;
		}

		if (EpAllocTryIn(pState, (uint8_t)(Needed - 1U),
						 (uint8_t)(ep + 1U), InMask | bit))
		{
			return true;
		}
	}

	return false;
}

bool UsbdEpAlloc(int DevNo, const UsbdEpAllocReq_t *pReq,
				 const UsbdClassCfg_t *pCfg, UsbDeviceClass *pClass,
				 UsbdEpAllocRes_t *pRes)
{
	if (pReq == nullptr || pCfg == nullptr || pRes == nullptr ||
		DevNo < 0 || DevNo >= USB_CTRLR_CNT ||
		pReq->InterfaceCount > 16U ||
		pReq->BidirectionalCount > USBD_EPALLOC_EP_MAXCNT ||
		pReq->InCount > USBD_EPALLOC_EP_MAXCNT ||
		pReq->OutCount > USBD_EPALLOC_EP_MAXCNT)
	{
		return false;
	}

	const uint8_t inLimit = USB_EPIN_CNT(DevNo) < 16 ?
		USB_EPIN_CNT(DevNo) : 16U;
	const uint8_t outLimit = USB_EPOUT_CNT(DevNo) < 16 ?
		USB_EPOUT_CNT(DevNo) : 16U;
	const uint8_t pairLimit = inLimit < outLimit ? inLimit : outLimit;
	const uint16_t inMask = (uint16_t)((1UL << inLimit) - 1UL);
	const uint16_t outMask = (uint16_t)((1UL << outLimit) - 1UL);
	const uint16_t fixedInDynamic = pReq->FixedInMask & inMask;
	const uint16_t fixedOutDynamic = pReq->FixedOutMask & outMask;

	if (inLimit < 1U || outLimit < 1U ||
		((pReq->FixedInMask | pReq->FixedOutMask) & 1U) != 0U ||
		EpAllocMaskCount(fixedInDynamic) +
			(unsigned)pReq->BidirectionalCount + pReq->InCount >
			(unsigned)inLimit - 1U ||
		EpAllocMaskCount(fixedOutDynamic) +
			(unsigned)pReq->BidirectionalCount + pReq->OutCount >
			(unsigned)outLimit - 1U)
	{
		return false;
	}

	UsbdEpAllocState_t state = {};
	state.DevNo = DevNo;
	state.pReq = pReq;
	state.pCfg = pCfg;
	state.pClass = pClass;
	state.pRes = pRes;
	state.InLimit = inLimit;
	state.OutLimit = outLimit;
	state.PairLimit = pairLimit;

	const unsigned lastFirst = 16U - pReq->InterfaceCount;
	for (unsigned first = 0; first <= lastFirst; first++)
	{
		state.FirstInterface = (uint8_t)first;
		if (EpAllocTryIn(&state, pReq->InCount, 1U, pReq->FixedInMask))
		{
			return true;
		}
	}

	return false;
}

bool UsbdEpAlloc(int DevNo, const UsbdEpAllocReq_t *pReq,
				 const UsbdClassCfg_t *pCfg, UsbdEpAllocRes_t *pRes)
{
	return UsbdEpAlloc(DevNo, pReq, pCfg, nullptr, pRes);
}
