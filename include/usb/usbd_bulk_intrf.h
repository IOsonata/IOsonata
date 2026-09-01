/**-------------------------------------------------------------------------
@file	usbd_bulk_intrf.h

@brief	Generic USB device bulk data interface.

Provides the reusable DeviceIntrf, CFifo data plane and Bulk IN transfer
engine for USB device functions.

TX policy: the application only fills the TX CFifo. A write that finds the
endpoint idle and the FIFO empty is the start of a transaction and is sent
at once, whatever its size. While earlier data is queued, writes aggregate
into full packets: a full packet starts a transfer and the completion
interrupt chains the next full packet, so a producer that outruns the bus
never emits short packets. A partial tail left behind is sent by the SOF
handler after it has stayed partial and idle for one full frame, without
any application-level polling. A terminating ZLP is sent the same way when
the stream stopped on an exact packet boundary and no data followed.

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2026

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
#ifndef __USBD_BULK_INTRF_H__
#define __USBD_BULK_INTRF_H__

#include <stdbool.h>
#include <stdint.h>

#include "cfifo.h"
#include "device_intrf.h"
#include "usb/usbd_ctrlr.h"

/** @addtogroup USBD
  * @{
  */

#pragma pack(push, 4)

typedef struct __UsbdBulk_Dev_Interf UsbdBulkDevIntrf_t;

typedef void (*UsbdBulkKickHandler_t)(UsbdBulkDevIntrf_t * const pIntrf,
									 void *pContext);

typedef bool (*UsbdBulkTxReadyHandler_t)(UsbdBulkDevIntrf_t * const pIntrf,
										void *pContext);

typedef struct __UsbdBulk_Interf_Config {
	bool bBlocking;
	int RxFifoMemSize;
	uint8_t *pRxFifoMem;
	int TxFifoMemSize;
	uint8_t *pTxFifoMem;
	UsbdBulkKickHandler_t RxKick;
	UsbdBulkTxReadyHandler_t TxReady;
	void *pContext;
	DevIntrfEvtHandler_t EvtCB;
} UsbdBulkIntrfCfg_t;

struct __UsbdBulk_Dev_Interf {
	DevIntrf_t DevIntrf;
	hCFifo_t hRxFifo;
	hCFifo_t hTxFifo;
	UsbdBulkKickHandler_t RxKick;
	UsbdBulkTxReadyHandler_t TxReady;
	void *pContext;
	uint8_t TxEpAddr;			//!< Bulk IN endpoint, 0 until configured
	uint8_t *pTxBuffer;			//!< One packet staging buffer, TxMps bytes
	uint16_t TxMps;				//!< Bulk IN max packet size
	bool bEnabled;
	bool TxZlpRequired;			//!< Last packet was full and nothing followed
	bool TxTailArmed;			//!< Partial tail seen idle at previous SOF
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool UsbdBulkIntrfInit(UsbdBulkDevIntrf_t *pIntrf,
					   const UsbdBulkIntrfCfg_t *pCfg);

/**
 * @brief Queue received endpoint data into the RX CFifo.
 */
int UsbdBulkIntrfPutRxData(UsbdBulkDevIntrf_t *pIntrf,
						   const uint8_t *pData, int DataLen);

/**
 * @brief Bind the Bulk IN endpoint after SET_CONFIGURATION.
 *
 * pBuffer must hold Mps bytes and stay valid while configured.
 */
void UsbdBulkIntrfConfigTx(UsbdBulkDevIntrf_t *pIntrf,
						   uint8_t EpAddr, uint16_t Mps,
						   uint8_t *pBuffer);

/**
 * @brief Drop endpoint transfer state on bus reset or unconfigure.
 *
 * Queued FIFO data is kept.
 */
void UsbdBulkIntrfResetTx(UsbdBulkDevIntrf_t *pIntrf);

/**
 * @brief Bulk IN transfer completion, called from the USB interrupt.
 */
void UsbdBulkIntrfTxXferComplete(UsbdBulkDevIntrf_t *pIntrf,
								 uint16_t Length,
								 UsbdCtrlrXferResult_t Result);

/**
 * @brief Start of frame, called from the USB interrupt.
 *
 * Sends a partial tail or terminating ZLP once it has stayed idle for one
 * full frame. Full packets are never split by this call.
 */
void UsbdBulkIntrfSof(UsbdBulkDevIntrf_t *pIntrf);

bool UsbdBulkIntrfRequestToSend(UsbdBulkDevIntrf_t *pIntrf, int NbBytes);

int UsbdBulkIntrfRxUsed(UsbdBulkDevIntrf_t *pIntrf);

int UsbdBulkIntrfTxUsed(UsbdBulkDevIntrf_t *pIntrf);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class UsbdBulkIntrf : public DeviceIntrf {
public:
	bool Init(const UsbdBulkIntrfCfg_t &Cfg) {
		return UsbdBulkIntrfInit(&vUsbDevIntrf, &Cfg);
	}

	operator DevIntrf_t * () override {
		return &vUsbDevIntrf.DevIntrf;
	}

	uint32_t Rate(uint32_t DataRate) override {
		return DeviceIntrfSetRate(&vUsbDevIntrf.DevIntrf, DataRate);
	}

	uint32_t Rate(void) override {
		return DeviceIntrfGetRate(&vUsbDevIntrf.DevIntrf);
	}

	bool RequestToSend(int NbBytes) {
		return UsbdBulkIntrfRequestToSend(&vUsbDevIntrf, NbBytes);
	}

protected:
	UsbdBulkDevIntrf_t vUsbDevIntrf = {};
};

#endif

/** @} End of group USBD */

#endif	// __USBD_BULK_INTRF_H__
