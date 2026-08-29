/**-------------------------------------------------------------------------
@file	usbd_bulk_intrf.h

@brief	Generic USB device bulk data interface.

Provides the reusable DeviceIntrf and CFifo data plane for a USB device
function that carries data over one Bulk OUT and one Bulk IN endpoint.

Endpoint allocation, descriptors, USB class requests and controller access are
owned by the USB function and device stack. This interface only queues bytes
between the application and that function.

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

/** @addtogroup USBD
  * @{
  */

#pragma pack(push, 4)

typedef struct __UsbdBulk_Dev_Interf UsbdBulkDevIntrf_t;

/**
 * @brief	Ask the owning USB function to service queued Bulk IN data.
 *
 * The callback may start a transfer immediately or only schedule service.
 * UsbdBulkIntrf never assumes an endpoint number, controller type or execution
 * context.
 */
typedef void (*UsbdBulkTxKickHandler_t)(UsbdBulkDevIntrf_t * const pIntrf,
										void *pContext);

typedef struct __UsbdBulk_Interf_Config {
	bool bBlocking;					//!< CFifo full behavior
	int RxFifoMemSize;				//!< RX FIFO memory size including CFifo header
	uint8_t *pRxFifoMem;			//!< Word-aligned RX FIFO memory
	int TxFifoMemSize;				//!< TX FIFO memory size including CFifo header
	uint8_t *pTxFifoMem;			//!< Word-aligned TX FIFO memory
	UsbdBulkTxKickHandler_t TxKick;	//!< Optional Bulk IN service callback
	void *pContext;					//!< Opaque context passed to TxKick
	DevIntrfEvtHandler_t EvtCB;		//!< Optional DeviceIntrf event callback
} UsbdBulkIntrfCfg_t;

struct __UsbdBulk_Dev_Interf {
	DevIntrf_t DevIntrf;			//!< Device interface, must be first
	hCFifo_t hRxFifo;				//!< USB OUT -> application
	hCFifo_t hTxFifo;				//!< Application -> USB IN
	UsbdBulkTxKickHandler_t TxKick;	//!< Owning USB function service callback
	void *pContext;					//!< Opaque callback context
	bool bEnabled;					//!< Interface data plane enabled
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize a generic USB device Bulk interface.
 *
 * FIFO storage belongs to the caller and must remain valid for the life of the
 * interface. No memory is allocated internally.
 *
 * @param	pIntrf	: Interface instance
 * @param	pCfg	: Configuration
 *
 * @return	true on success
 */
bool UsbdBulkIntrfInit(UsbdBulkDevIntrf_t *pIntrf,
					   const UsbdBulkIntrfCfg_t *pCfg);

/**
 * @brief	Push bytes received by a Bulk OUT transfer to the RX FIFO.
 *
 * In non-blocking mode CFifo keeps its normal drop-oldest behavior. In
 * blocking mode only bytes for which FIFO space exists are accepted.
 *
 * @param	pIntrf	: Interface instance
 * @param	pData	: Received USB data
 * @param	DataLen	: Number of bytes received
 *
 * @return	Number of bytes accepted
 */
int UsbdBulkIntrfPutRxData(UsbdBulkDevIntrf_t *pIntrf,
						   const uint8_t *pData, int DataLen);

/**
 * @brief	Copy queued TX bytes into a buffer owned by the USB function.
 *
 * The owning function must keep the returned bytes in its transfer buffer until
 * the Bulk IN request completes. Only one logical Bulk IN request should be
 * outstanding for one UsbdBulkIntrf instance.
 *
 * @param	pIntrf		: Interface instance
 * @param	pBuffer		: USB transfer buffer
 * @param	BufferLen	: Maximum bytes to copy
 *
 * @return	Number of bytes copied
 */
int UsbdBulkIntrfGetTxData(UsbdBulkDevIntrf_t *pIntrf,
						   uint8_t *pBuffer, int BufferLen);

/**
 * @brief	Report completion of the current Bulk IN request.
 *
 * If more application data is queued, TxKick is called again. Otherwise
 * DEVINTRF_EVT_TX_FIFO_EMPTY is reported.
 *
 * @param	pIntrf	: Interface instance
 */
void UsbdBulkIntrfTxComplete(UsbdBulkDevIntrf_t *pIntrf);

/**
 * @brief	Check whether the application may queue a TX request.
 *
 * For a non-blocking FIFO this is always true for a positive request because
 * CFifo intentionally drops the oldest bytes when full. For a blocking FIFO,
 * TxKick is first given a chance to move queued bytes to the USB transfer
 * buffer, then the available space is checked.
 *
 * @param	pIntrf	: Interface instance
 * @param	NbBytes	: Number of bytes the caller wants to queue
 *
 * @return	true when the request can be queued according to CFifo policy
 */
bool UsbdBulkIntrfRequestToSend(UsbdBulkDevIntrf_t *pIntrf, int NbBytes);

/** @brief Return bytes currently queued for the application. */
int UsbdBulkIntrfRxUsed(UsbdBulkDevIntrf_t *pIntrf);

/** @brief Return bytes currently queued for Bulk IN. */
int UsbdBulkIntrfTxUsed(UsbdBulkDevIntrf_t *pIntrf);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * @brief	C++ wrapper for the generic USB device Bulk DeviceIntrf.
 *
 * USB classes such as CDC ACM may derive from this class and add their control
 * interface and descriptors while reusing this data plane.
 */
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
