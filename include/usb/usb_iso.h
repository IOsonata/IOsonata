/**-------------------------------------------------------------------------
@file	usb_iso.h

@brief	Reusable USB isochronous interface.

UsbIsoIntrf is the isochronous specialization of UsbIntrf and follows the
DeviceIntrf model the other USB classes use. UsbIntrf owns the endpoint pair
data path: packet mode with blocking FIFOs of two frames per direction (the
controller reads the TX head in place; a full RX FIFO leaves the next OUT
frame without a destination, so the newest frame is dropped, since an
isochronous endpoint cannot hold the host off), the OUT endpoint, the RX FIFO and DeviceIntrf itself. The application owns the
event callback (EvtCB in the configuration) and pulls received frames with
RxData, one frame per call, when UsbIntrf raises DEVINTRF_EVT_RX_DATA. It
queues frames to send with TxData, one frame per call up to the packet size.

What is isochronous is the IN side timing, and that is all this file adds:
the IN endpoint callback offers the head of the TX FIFO to the controller
once per service interval (UsbCtrlrIsoSend) and, when the frame's DMA has
ended, pops it and raises DEVINTRF_EVT_TX_READY (more frames queued) or
DEVINTRF_EVT_TX_FIFO_EMPTY (queue drained) to the application callback with
the captured DMA length. A failed frame raises DEVINTRF_EVT_TX_TIMEOUT.

The controller moves each received OUT frame straight into the RX FIFO
block UsbIntrf reserved and registered as the endpoint's DMA buffer, and
reports the frame length; UsbIntrf publishes the block. Zero-length frames
are counted in RxEmptyCnt and skipped by RxData.

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
#include <stddef.h>
#include <stdint.h>

#include "usb/usb_intrf.h"

/** @addtogroup USBD
  * @{
  */

#define USB_ISO_INTRF_MAX_MPS		((uint16_t)USB_CTRLR_PKT_LEN_MAX(0, ISO))

// Frames queued per direction. Full speed moves one packet per direction
// per frame, so two slots hold the frame the controller is sending and the
// one the application has already prepared; more only adds latency.
#define USB_ISO_INTRF_FIFO_PKTCNT	2U
#define USB_ISO_INTRF_FIFO_MEMSIZE(Mps) \
	USB_INTRF_RXMEM_SIZE(USB_ISO_INTRF_FIFO_PKTCNT, Mps)
#define USB_ISO_INTRF_FIFO_WORDS \
	((USB_ISO_INTRF_FIFO_MEMSIZE(USB_ISO_INTRF_MAX_MPS) + 3U) / 4U)

typedef struct __Usb_Iso_Interf UsbIsoIntrf_t;

#pragma pack(push, 4)

typedef struct __Usb_Iso_Interf_Config {
	int DevNo;
	uint8_t EpNo;					//!< Internally allocated ISO endpoint number
	uint16_t BufferSize;			//!< Maximum ISO payload bytes, up to USB_ISO_INTRF_MAX_MPS
	uint8_t *pRxFifoMem;			//!< Word-aligned USB_ISO_INTRF_FIFO_MEMSIZE(BufferSize)
	uint8_t *pTxFifoMem;			//!< Word-aligned USB_ISO_INTRF_FIFO_MEMSIZE(BufferSize)
	DevIntrfEvtHandler_t EvtCB;		//!< Application event callback
	void *pContext;					//!< Application context, see UsbIsoIntrfContext
} UsbIsoIntrfCfg_t;

#pragma pack(pop)

struct __Usb_Iso_Interf {
	UsbDevIntrf_t *pData;		//!< Shared endpoint data path
	uint16_t Mps;			//!< Requested packet size, retained while disabled
	uint8_t EpNo;
	uint8_t Interval;
	bool Opened;			//!< Endpoint pair is active
	bool Suspended;
	void *pContext;
	uint32_t RxMissCnt;			//!< Frames larger than the packet size
	uint32_t TxMissCnt;			//!< Frames the controller failed
	uint32_t RxEmptyCnt;		//!< Zero-length frames received
	uint32_t TxEmptyCnt;		//!< Zero-length frames sent
};

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize an ISO interface with caller-owned endpoint data storage. */
bool UsbIsoIntrfInit(UsbIsoIntrf_t *pIntrf, UsbDevIntrf_t *pData,
					 const UsbIsoIntrfCfg_t *pCfg);

/**
 * Configure the endpoint pair as isochronous and open it when enabled.
 * DeviceIntrfDisable closes the endpoints and discards queued frames;
 * DeviceIntrfEnable restores this configuration. Close and Reset discard it.
 */
bool UsbIsoIntrfOpen(UsbIsoIntrf_t *pIntrf, uint16_t Mps, uint8_t Interval);
void UsbIsoIntrfClose(UsbIsoIntrf_t *pIntrf);
void UsbIsoIntrfReset(UsbIsoIntrf_t *pIntrf);
void UsbIsoIntrfSuspend(UsbIsoIntrf_t *pIntrf);
bool UsbIsoIntrfResume(UsbIsoIntrf_t *pIntrf);

/**
 * Queue one IN frame for a later service interval. This is what TxData does
 * for one call. Returns false when the interface is closed, disabled or
 * suspended, the frame is oversize, or the queue already holds
 * USB_ISO_INTRF_FIFO_PKTCNT frames.
 */
bool UsbIsoIntrfSendFrame(UsbIsoIntrf_t *pIntrf, const uint8_t *pData,
						  uint16_t Length);

/** Queue space is available; DevIntrf.bTxReady instead means TX has drained. */
static inline bool UsbIsoIntrfTxReady(const UsbIsoIntrf_t *pIntrf)
{
	return pIntrf != NULL &&
		pIntrf->Opened && !pIntrf->Suspended &&
		CFifoAvail(pIntrf->pData->hTxFifo) > 0;
}

/** The ISO interface behind the DevIntrf_t handed to the event callback. */
static inline UsbIsoIntrf_t *UsbIsoIntrfFromDev(DevIntrf_t * const pDev)
{
	UsbDevIntrf_t *pData = (UsbDevIntrf_t *)pDev->pDevData;
	return (UsbIsoIntrf_t *)pData->pClassContext;
}

/** The application context given at Init, from the event callback. */
static inline void *UsbIsoIntrfContext(DevIntrf_t * const pDev)
{
	return UsbIsoIntrfFromDev(pDev)->pContext;
}

#ifdef __cplusplus
}

class UsbIsoIntrf : public UsbIntrf {
public:
	UsbIsoIntrf() = default;
	UsbIsoIntrf(const UsbIsoIntrf &) = delete;
	UsbIsoIntrf &operator = (const UsbIsoIntrf &) = delete;

	bool Init(const UsbIsoIntrfCfg_t &Cfg) {
		UsbIsoIntrfCfg_t cfg = Cfg;
		if (cfg.pRxFifoMem == nullptr && cfg.pTxFifoMem == nullptr)
		{
			cfg.BufferSize = USB_ISO_INTRF_MAX_MPS;
			cfg.pRxFifoMem = reinterpret_cast<uint8_t *>(vRxFifo);
			cfg.pTxFifoMem = reinterpret_cast<uint8_t *>(vTxFifo);
		}
		return UsbIsoIntrfInit(&vUsbIsoIntrf, &vUsbDevIntrf, &cfg);
	}

	using UsbIntrf::operator DevIntrf_t *;
	operator UsbIsoIntrf_t * () { return &vUsbIsoIntrf; }

	bool Open(uint16_t Mps, uint8_t Interval) {
		return UsbIsoIntrfOpen(&vUsbIsoIntrf, Mps, Interval);
	}

	void Close(void) { UsbIsoIntrfClose(&vUsbIsoIntrf); }
	void Reset(void) override { UsbIsoIntrfReset(&vUsbIsoIntrf); }
	void Suspend(void) { UsbIsoIntrfSuspend(&vUsbIsoIntrf); }
	bool Resume(void) { return UsbIsoIntrfResume(&vUsbIsoIntrf); }

	bool TxReady(void) const { return UsbIsoIntrfTxReady(&vUsbIsoIntrf); }

private:
	UsbIsoIntrf_t vUsbIsoIntrf = {};
	uint32_t vRxFifo[USB_ISO_INTRF_FIFO_WORDS] = {};
	uint32_t vTxFifo[USB_ISO_INTRF_FIFO_WORDS] = {};
};
#endif

/** @} End of group USBD */

#endif	// __USB_ISO_H__
