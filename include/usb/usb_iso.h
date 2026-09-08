/**-------------------------------------------------------------------------
@file	usb_iso.h

@brief	USB isochronous endpoint-pair interface built on UsbIntrf.

UsbIsoIntrf adds isochronous endpoint lifecycle and frame callbacks to the
reusable UsbIntrf data path. UsbIntrf owns the fixed DMA staging buffers,
CFifo transport, controller registration and transfer completion hot path.
UsbIsoIntrf only opens/closes the endpoint pair, preserves one-frame-at-a-time
TX semantics for its frame API and translates DeviceIntrf events to frame
callbacks.

The controller is registered non-blocking for ISO OUT. A received frame goes
directly from the controller event to EasyDMA and reaches UsbIntrf only at
transfer completion. There is no DRDY round trip and no receive arm state.

@author	Hoang Nguyen Hoan
@date	Sep. 8, 2026

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
#ifndef __USB_ISO_H__
#define __USB_ISO_H__

#include <stdbool.h>
#include <stdint.h>

#include "usb/usb_intrf.h"

/** @addtogroup USBD
  * @{
  */

#define USB_ISO_INTRF_MAX_MPS		((uint16_t)USB_PKT_MAXLEN(0, ISO))
#define USB_ISO_INTRF_BUFFER_WORDS \
	(((USB_ISO_INTRF_MAX_MPS > 0U ? USB_ISO_INTRF_MAX_MPS : 1U) + 3U) / 4U)
#define USB_ISO_INTRF_PKT_BLKSIZE \
	USB_INTRF_PKT_BLKSIZE(USB_ISO_INTRF_MAX_MPS)
#define USB_ISO_INTRF_FIFO_MEMSIZE \
	CFIFO_TOTAL_MEMSIZE(1U, USB_ISO_INTRF_PKT_BLKSIZE)
#define USB_ISO_INTRF_FIFO_WORDS \
	((USB_ISO_INTRF_FIFO_MEMSIZE + 3U) / 4U)
#define USB_ISO_INTRF_PACKET_WORDS \
	((USB_ISO_INTRF_PKT_BLKSIZE + 3U) / 4U)

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
	int DevNo;
	uint8_t EpNo;
	UsbIsoIntrfRxHandler_t RxHandler;
	UsbIsoIntrfTxHandler_t TxHandler;
	void *pContext;
} UsbIsoIntrfCfg_t;

#pragma pack(pop)

struct __Usb_Iso_Interf {
	UsbDevIntrf_t *pIntrfData;
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
	bool TxActive;

	// C callers use LocalData. The C++ derived class binds pIntrfData to the
	// UsbIntrf base object instead, so both paths use the same implementation.
	UsbDevIntrf_t LocalData;

	// One frame is sufficient because frame callbacks are delivered from the
	// completion path and SendFrame intentionally permits only one active IN.
	uint32_t RxFifoMem[USB_ISO_INTRF_FIFO_WORDS];
	uint32_t TxFifoMem[USB_ISO_INTRF_FIFO_WORDS];
	uint32_t RxBuffer[USB_ISO_INTRF_BUFFER_WORDS];
	uint32_t TxBuffer[USB_ISO_INTRF_BUFFER_WORDS];
	uint32_t TxPacket[USB_ISO_INTRF_PACKET_WORDS];
};

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize a C instance using its embedded UsbIntrf data object. */
bool UsbIsoIntrfInit(UsbIsoIntrf_t *pIntrf, const UsbIsoIntrfCfg_t *pCfg);

/** Bind an ISO wrapper to an existing UsbIntrf data object. */
bool UsbIsoIntrfInitData(UsbIsoIntrf_t *pIntrf, UsbDevIntrf_t *pData,
						 const UsbIsoIntrfCfg_t *pCfg);

bool UsbIsoIntrfOpen(UsbIsoIntrf_t *pIntrf, uint16_t Mps, uint8_t Interval);
void UsbIsoIntrfClose(UsbIsoIntrf_t *pIntrf);
void UsbIsoIntrfReset(UsbIsoIntrf_t *pIntrf);
void UsbIsoIntrfSuspend(UsbIsoIntrf_t *pIntrf);
bool UsbIsoIntrfResume(UsbIsoIntrf_t *pIntrf);
bool UsbIsoIntrfSendFrame(UsbIsoIntrf_t *pIntrf, const uint8_t *pData,
						  uint16_t Length);

static inline bool UsbIsoIntrfTxReady(const UsbIsoIntrf_t *pIntrf)
{
	return pIntrf != nullptr && pIntrf->Opened && !pIntrf->Suspended &&
		!pIntrf->TxActive;
}

#ifdef __cplusplus
}

class UsbIsoIntrf : public UsbIntrf {
public:
	UsbIsoIntrf() = default;

	bool Init(const UsbIsoIntrfCfg_t &Cfg) {
		return UsbIsoIntrfInitData(&vUsbIsoIntrf, &vUsbDevIntrf, &Cfg);
	}

	bool Open(uint16_t Mps, uint8_t Interval) {
		return UsbIsoIntrfOpen(&vUsbIsoIntrf, Mps, Interval);
	}

	void Close(void) { UsbIsoIntrfClose(&vUsbIsoIntrf); }
	void Reset(void) { UsbIsoIntrfReset(&vUsbIsoIntrf); }
	void Suspend(void) { UsbIsoIntrfSuspend(&vUsbIsoIntrf); }
	bool Resume(void) { return UsbIsoIntrfResume(&vUsbIsoIntrf); }

	bool SendFrame(const uint8_t *pData, uint16_t Length) {
		return UsbIsoIntrfSendFrame(&vUsbIsoIntrf, pData, Length);
	}

	bool TxReady(void) const { return UsbIsoIntrfTxReady(&vUsbIsoIntrf); }
	DevIntrf_t *Data(void) { return static_cast<DevIntrf_t *>(*this); }

private:
	UsbIsoIntrf_t vUsbIsoIntrf = {};
};
#endif

/** @} End of group USBD */

#endif	// __USB_ISO_H__
