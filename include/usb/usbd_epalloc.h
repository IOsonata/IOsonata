/**-------------------------------------------------------------------------
@file	usbd_epalloc.h

@brief	Internal USB device interface/endpoint allocator.

USB classes describe how many interfaces and endpoint directions they need.
The allocator finds the lowest free placement accepted by the USB core and
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
#ifndef __USBD_EPALLOC_H__
#define __USBD_EPALLOC_H__

#include <stdint.h>

#include "usb/usb.h"

#define USBD_EPALLOC_EP_MAXCNT		4U

#pragma pack(push, 4)

/// What a class needs : interface count and endpoint counts per direction
typedef struct __Usbd_EpAlloc_Req {
	uint8_t InterfaceCount;
	uint8_t BidirectionalCount;
	uint8_t InCount;
	uint8_t OutCount;
	uint16_t FixedInMask;		//!< Controller-constrained IN endpoints
	uint16_t FixedOutMask;		//!< Controller-constrained OUT endpoints
} UsbdEpAllocReq_t;

/// What the allocator assigned : concrete interface and endpoint numbers
typedef struct __Usbd_EpAlloc_Res {
	uint8_t FirstInterface;
	uint8_t Bidirectional[USBD_EPALLOC_EP_MAXCNT];
	uint8_t In[USBD_EPALLOC_EP_MAXCNT];
	uint8_t Out[USBD_EPALLOC_EP_MAXCNT];
} UsbdEpAllocRes_t;

#pragma pack(pop)

/// Find the lowest free interface/endpoint placement satisfying pReq,
/// register pCfg with the core at that placement and return the assigned
/// numbers in pRes. Returns false when no placement fits.
bool UsbdEpAlloc(int DevNo, const UsbdEpAllocReq_t *pReq,
				 const UsbdClassCfg_t *pCfg, UsbdEpAllocRes_t *pRes);

#ifdef __cplusplus
/// Atomically register pClass with the topology selected by the allocator.
bool UsbdEpAlloc(int DevNo, const UsbdEpAllocReq_t *pReq,
				 const UsbdClassCfg_t *pCfg, UsbDeviceClass *pClass,
				 UsbdEpAllocRes_t *pRes);
#endif

#endif	// __USBD_EPALLOC_H__
