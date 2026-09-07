/**-------------------------------------------------------------------------
@file	usb_iso_intrf.h

@brief	Reusable bidirectional USB isochronous endpoint interface.

UsbIsoIntrf owns the USB transport mechanics for one bidirectional isochronous
endpoint number. It owns fixed DMA staging buffers, opens and closes both
endpoint directions, keeps one OUT transfer armed, accepts at most one IN frame
at a time, and reports frame completions to the class above it. The class owns
all protocol packet assembly and segmentation.

The interface has no packet-format knowledge and performs no dynamic
allocation. A zero-length frame is a valid isochronous frame. Failed or
cancelled transfers are reported and counted rather than retried as protocol
data. Suspend stops new submissions; resume restores the OUT arm.

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2026

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
#ifndef __USB_ISO_INTRF_H__
#define __USB_ISO_INTRF_H__

#include <stdbool.h>
#include <stdint.h>

#include "usb/usb.h"

/** @addtogroup USBD
  * @{
  */

// IOsonata currently exposes one USB controller per target. Keep the staging
// bound compile-time so every UsbIsoIntrf instance is self contained.
#define USB_ISO_INTRF_MAX_MPS		((uint16_t)USB_PKT_MAXLEN(0, ISO))
#define USB_ISO_INTRF_BUFFER_WORDS \
	((USB_ISO_INTRF_MAX_MPS > 0U ? USB_ISO_INTRF_MAX_MPS : 1U) + 3U) / 4U

typedef struct __Usb_Iso_Interf UsbIsoIntrf_t;

typedef void (*UsbIsoIntrfRxHandler_t)(UsbIsoIntrf_t *pIntrf,
									 const uint8_t *pData, uint16_t Length,
									 UsbCtrlrXferResult_t Result,
									 void *pContext);
typedef void (*UsbIsoIntrfTxHandler_t)(UsbIsoIntrf_t *pIntrf,
									 uint16_t Length,
									 UsbCtrlrXferResult_t Result,
									 void *pContext);

#pragma pack(push, 4)

typedef struct __Usb_Iso_Interf_Config {
	int DevNo;					//!< USB controller number
	uint8_t EpNo;				//!< Bidirectional isochronous endpoint number
	UsbIsoIntrfRxHandler_t RxHandler;
	UsbIsoIntrfTxHandler_t TxHandler;
	void *pContext;
} UsbIsoIntrfCfg_t;

#pragma pack(pop)

struct __Usb_Iso_Interf {
	int DevNo;
	void *pContext;
	UsbIsoIntrfRxHandler_t RxHandler;
	UsbIsoIntrfTxHandler_t TxHandler;
	uint32_t RxMissCnt;
	uint32_t TxMissCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
	uint16_t Mps;
	uint16_t TxLength;
	uint8_t EpNo;
	uint8_t Interval;
	bool Opened;
	bool Suspended;
	bool RxArmed;
	bool TxActive;
	uint32_t RxBuffer[USB_ISO_INTRF_BUFFER_WORDS];
	uint32_t TxBuffer[USB_ISO_INTRF_BUFFER_WORDS];
};

#ifdef __cplusplus
extern "C" {
#endif

/** Register the fixed DMA buffers and completion callbacks. */
bool UsbIsoIntrfInit(UsbIsoIntrf_t *pIntrf, const UsbIsoIntrfCfg_t *pCfg);

/** Open both endpoint directions and arm OUT for the first service interval. */
bool UsbIsoIntrfOpen(UsbIsoIntrf_t *pIntrf, uint16_t Mps, uint8_t Interval);

/** Close both directions and discard active USB frame state. */
void UsbIsoIntrfClose(UsbIsoIntrf_t *pIntrf);

/** Bus reset lifecycle entry. Closes the endpoint and clears frame state. */
void UsbIsoIntrfReset(UsbIsoIntrf_t *pIntrf);

/** Pause submissions while the bus is suspended. */
void UsbIsoIntrfSuspend(UsbIsoIntrf_t *pIntrf);

/** Resume submissions and restore the OUT arm. */
bool UsbIsoIntrfResume(UsbIsoIntrf_t *pIntrf);

/**
 * Submit exactly one IN frame. Length may be zero through the active MPS.
 * Returns false while another IN frame is active or while suspended/closed.
 */
bool UsbIsoIntrfSendFrame(UsbIsoIntrf_t *pIntrf, const uint8_t *pData,
						  uint16_t Length);

static inline bool UsbIsoIntrfTxReady(const UsbIsoIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->Opened && !pIntrf->Suspended &&
		!pIntrf->TxActive;
}

#ifdef __cplusplus
}
#endif

/** @} End of group USBD */

#endif	// __USB_ISO_INTRF_H__
