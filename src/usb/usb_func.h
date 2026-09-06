/**-------------------------------------------------------------------------
@file	usb_func.h

@brief	Internal USB function resource allocation.

USB classes describe how many interfaces and endpoint directions they need.
This helper finds the lowest free placement accepted by the USB core and
returns the assigned numbers to the class implementation. Applications do not
choose interface or endpoint numbers.

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
#ifndef __USB_FUNC_H__
#define __USB_FUNC_H__

#include <stdint.h>
#include <string.h>

#include "usb/usb.h"

#define USB_FUNC_EP_MAXCNT		4U

#pragma pack(push, 4)

typedef struct __Usb_Func_Requirement {
	uint8_t InterfaceCount;
	uint8_t BidirectionalCount;
	uint8_t InCount;
	uint8_t OutCount;
} UsbFuncReq_t;

typedef struct __Usb_Func_Allocation {
	uint8_t FirstInterface;
	uint8_t Bidirectional[USB_FUNC_EP_MAXCNT];
	uint8_t In[USB_FUNC_EP_MAXCNT];
	uint8_t Out[USB_FUNC_EP_MAXCNT];
} UsbFuncAlloc_t;

#pragma pack(pop)

typedef struct __Usb_Func_Alloc_State {
	int DevNo;
	const UsbFuncReq_t *pReq;
	const UsbFuncCfg_t *pCfg;
	UsbFuncAlloc_t *pAlloc;
	uint8_t FirstInterface;
	uint8_t InLimit;
	uint8_t OutLimit;
	uint8_t PairLimit;
} UsbFuncAllocState_t;

static inline void UsbFuncStoreEndpoints(uint8_t *pEp, unsigned Count,
										 uint16_t Mask)
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

static inline bool UsbFuncTryOut(const UsbFuncAllocState_t *pState,
								 uint8_t Needed, uint8_t StartEp,
								 uint16_t InMask, uint16_t PairMask,
								 uint16_t OutMask)
{
	if (Needed == 0U)
	{
		UsbFuncCfg_t cfg = *pState->pCfg;
		cfg.FirstInterface = pState->FirstInterface;
		cfg.InterfaceCount = pState->pReq->InterfaceCount;
		cfg.EpInMask = InMask | PairMask;
		cfg.EpOutMask = OutMask | PairMask;

		if (!UsbRegisterFunc(pState->DevNo, &cfg))
		{
			return false;
		}

		memset(pState->pAlloc, 0, sizeof(*pState->pAlloc));
		pState->pAlloc->FirstInterface = pState->FirstInterface;
		UsbFuncStoreEndpoints(pState->pAlloc->Bidirectional,
			pState->pReq->BidirectionalCount, PairMask);
		UsbFuncStoreEndpoints(pState->pAlloc->In,
			pState->pReq->InCount, InMask);
		UsbFuncStoreEndpoints(pState->pAlloc->Out,
			pState->pReq->OutCount, OutMask);
		return true;
	}

	for (uint8_t ep = StartEp; ep < pState->OutLimit; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);

		if ((PairMask & bit) != 0U)
		{
			continue;
		}

		if (UsbFuncTryOut(pState, (uint8_t)(Needed - 1U),
							  (uint8_t)(ep + 1U), InMask, PairMask,
							  OutMask | bit))
		{
			return true;
		}
	}

	return false;
}

static inline bool UsbFuncTryPair(const UsbFuncAllocState_t *pState,
								  uint8_t Needed, uint8_t StartEp,
								  uint16_t InMask, uint16_t PairMask)
{
	if (Needed == 0U)
	{
		return UsbFuncTryOut(pState, pState->pReq->OutCount, 1U,
							 InMask, PairMask, 0U);
	}

	for (uint8_t ep = StartEp; ep < pState->PairLimit; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);

		if ((InMask & bit) != 0U)
		{
			continue;
		}

		if (UsbFuncTryPair(pState, (uint8_t)(Needed - 1U),
							   (uint8_t)(ep + 1U), InMask,
							   PairMask | bit))
		{
			return true;
		}
	}

	return false;
}

static inline bool UsbFuncTryIn(const UsbFuncAllocState_t *pState,
								uint8_t Needed, uint8_t StartEp,
								uint16_t InMask)
{
	if (Needed == 0U)
	{
		return UsbFuncTryPair(pState, pState->pReq->BidirectionalCount,
							  1U, InMask, 0U);
	}

	for (uint8_t ep = StartEp; ep < pState->InLimit; ep++)
	{
		const uint16_t bit = (uint16_t)(1U << ep);

		if (UsbFuncTryIn(pState, (uint8_t)(Needed - 1U),
							 (uint8_t)(ep + 1U), InMask | bit))
		{
			return true;
		}
	}

	return false;
}

static inline bool UsbRegisterFuncAuto(int DevNo, const UsbFuncReq_t *pReq,
									  const UsbFuncCfg_t *pCfg,
									  UsbFuncAlloc_t *pAlloc)
{
	if (pReq == nullptr || pCfg == nullptr || pAlloc == nullptr ||
		DevNo < 0 || DevNo >= USB_CTRLR_CNT ||
		pReq->InterfaceCount > 16U ||
		pReq->BidirectionalCount > USB_FUNC_EP_MAXCNT ||
		pReq->InCount > USB_FUNC_EP_MAXCNT ||
		pReq->OutCount > USB_FUNC_EP_MAXCNT)
	{
		return false;
	}

	const uint8_t inLimit = USB_EPIN_CNT(DevNo) < 16 ?
		USB_EPIN_CNT(DevNo) : 16U;
	const uint8_t outLimit = USB_EPOUT_CNT(DevNo) < 16 ?
		USB_EPOUT_CNT(DevNo) : 16U;
	const uint8_t pairLimit = inLimit < outLimit ? inLimit : outLimit;

	if (inLimit < 1U || outLimit < 1U ||
		(unsigned)pReq->BidirectionalCount + pReq->InCount >
			(unsigned)inLimit - 1U ||
		(unsigned)pReq->BidirectionalCount + pReq->OutCount >
			(unsigned)outLimit - 1U)
	{
		return false;
	}

	UsbFuncAllocState_t state = {};
	state.DevNo = DevNo;
	state.pReq = pReq;
	state.pCfg = pCfg;
	state.pAlloc = pAlloc;
	state.InLimit = inLimit;
	state.OutLimit = outLimit;
	state.PairLimit = pairLimit;

	const unsigned lastFirst = 16U - pReq->InterfaceCount;
	for (unsigned first = 0; first <= lastFirst; first++)
	{
		state.FirstInterface = (uint8_t)first;
		if (UsbFuncTryIn(&state, pReq->InCount, 1U, 0U))
		{
			return true;
		}
	}

	return false;
}

#endif	// __USB_FUNC_H__
