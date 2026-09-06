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

/** @addtogroup USB
  * @{
  */

#define USB_FUNC_EP_MAXCNT		4U

#pragma pack(push, 4)

/// Resources one USB function needs. Endpoint transfer type is deliberately
/// not part of allocation; the class applies Bulk/Interrupt/Iso in descriptors.
typedef struct __Usb_Func_Requirement {
	uint8_t InterfaceCount;
	uint8_t BidirectionalCount;	//!< Endpoint numbers used in both directions
	uint8_t InCount;				//!< Additional IN-only endpoint numbers
	uint8_t OutCount;			//!< Additional OUT-only endpoint numbers
} UsbFuncReq_t;

/// Placement returned to the class implementation after registration.
typedef struct __Usb_Func_Allocation {
	uint8_t FirstInterface;
	uint8_t Bidirectional[USB_FUNC_EP_MAXCNT];
	uint8_t In[USB_FUNC_EP_MAXCNT];
	uint8_t Out[USB_FUNC_EP_MAXCNT];
} UsbFuncAlloc_t;

#pragma pack(pop)

static inline unsigned UsbFuncPopCount(uint16_t Mask)
{
	unsigned count = 0;

	while (Mask != 0U)
	{
		count += Mask & 1U;
		Mask >>= 1;
	}

	return count;
}

static inline uint16_t UsbFuncEpMask(unsigned Count)
{
	if (Count >= 16U)
	{
		return 0xfffeU;
	}

	return (uint16_t)(((1UL << Count) - 1UL) & ~1UL);
}

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

/**
 * @brief	Allocate and register one USB function.
 *
 * The existing core registration remains the authority for overlap. Candidate
 * placements are tried in ascending order and failed candidates do not alter
 * core state. Extra IN resources are placed before bidirectional pairs so CDC
 * and HCI naturally receive notification/event EP1 and data EP2.
 */
static inline bool UsbRegisterFuncAuto(int DevNo, const UsbFuncReq_t *pReq,
									  const UsbFuncCfg_t *pCfg,
									  UsbFuncAlloc_t *pAlloc)
{
	if (pReq == nullptr || pCfg == nullptr || pAlloc == nullptr ||
		DevNo != 0 ||
		pReq->InterfaceCount > 16U ||
		pReq->BidirectionalCount > USB_FUNC_EP_MAXCNT ||
		pReq->InCount > USB_FUNC_EP_MAXCNT ||
		pReq->OutCount > USB_FUNC_EP_MAXCNT)
	{
		return false;
	}

	const unsigned inCount = USB_EPIN_CNT(0) > 16 ? 16U : USB_EPIN_CNT(0);
	const unsigned outCount = USB_EPOUT_CNT(0) > 16 ? 16U : USB_EPOUT_CNT(0);
	const uint16_t inAvail = UsbFuncEpMask(inCount);
	const uint16_t outAvail = UsbFuncEpMask(outCount);
	const uint16_t pairAvail = inAvail & outAvail;

	if ((unsigned)pReq->BidirectionalCount + pReq->InCount > inCount - 1U ||
		(unsigned)pReq->BidirectionalCount + pReq->OutCount > outCount - 1U)
	{
		return false;
	}

	const unsigned lastFirst = 16U - pReq->InterfaceCount;

	for (unsigned first = 0; first <= lastFirst; first++)
	{
		for (uint32_t inValue = 0; inValue <= inAvail; inValue++)
		{
			const uint16_t inMask = (uint16_t)inValue;
			if ((inMask & (uint16_t)~inAvail) != 0U ||
				UsbFuncPopCount(inMask) != pReq->InCount)
			{
				continue;
			}

			for (uint32_t pairValue = 0; pairValue <= pairAvail; pairValue++)
			{
				const uint16_t pairMask = (uint16_t)pairValue;
				if ((pairMask & (uint16_t)~pairAvail) != 0U ||
					(pairMask & inMask) != 0U ||
					UsbFuncPopCount(pairMask) != pReq->BidirectionalCount)
				{
					continue;
				}

				for (uint32_t outValue = 0; outValue <= outAvail; outValue++)
				{
					const uint16_t outMask = (uint16_t)outValue;
					if ((outMask & (uint16_t)~outAvail) != 0U ||
						(outMask & pairMask) != 0U ||
						UsbFuncPopCount(outMask) != pReq->OutCount)
					{
						continue;
					}

					UsbFuncCfg_t cfg = *pCfg;
					cfg.FirstInterface = (uint8_t)first;
					cfg.InterfaceCount = pReq->InterfaceCount;
					cfg.EpInMask = pairMask | inMask;
					cfg.EpOutMask = pairMask | outMask;

					if (!UsbRegisterFunc(DevNo, &cfg))
					{
						continue;
					}

					memset(pAlloc, 0, sizeof(*pAlloc));
					pAlloc->FirstInterface = (uint8_t)first;
					UsbFuncStoreEndpoints(pAlloc->Bidirectional,
						pReq->BidirectionalCount, pairMask);
					UsbFuncStoreEndpoints(pAlloc->In, pReq->InCount, inMask);
					UsbFuncStoreEndpoints(pAlloc->Out, pReq->OutCount, outMask);
					return true;
				}
			}
		}
	}

	return false;
}

/** @} End of group USB */

#endif	// __USB_FUNC_H__
