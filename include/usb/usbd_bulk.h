/**-------------------------------------------------------------------------
@file	usbd_bulk.h

@brief	USB vendor bulk interface.

UsbdBulk is the public adapter for a vendor-defined interface with one bulk
OUT endpoint and one bulk IN endpoint sharing the same endpoint number. It
inherits the internal UsbIntrf data path, owns the controller transfer
buffers, and leaves the RX/TX CFifo storage to the application.

The interface can operate as a byte stream or as USB packets. Byte mode uses
a one-byte TX CFifo and lets UsbIntrf packetize queued data. Packet mode uses
one UsbPkt_t block per USB packet, including an explicit zero-length packet.

Interface and endpoint numbers are allocated internally when the function is
registered. Applications configure the vendor interface behaviour and storage,
not USB topology.

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
#ifndef __USBD_BULK_H__
#define __USBD_BULK_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "usb/usb.h"
#include "usb/usb_intrf.h"

/** @addtogroup USBD
  * @{
  */

#define USBD_BULK_CONFIG_VALUE			1U
#define USBD_BULK_FS_MPS				64U
#define USBD_BULK_HS_MPS				512U
#define USBD_BULK_MAX_MPS				USB_PKT_MAXLEN(0, BULK)
#define USBD_BULK_PKT_BLKSIZE			USB_INTRF_PKT_BLKSIZE(USBD_BULK_MAX_MPS)
#define USBD_BULK_RXMEM_SIZE(NbPkt)		USB_INTRF_RXMEM_SIZE((NbPkt), USBD_BULK_MAX_MPS)
#define USBD_BULK_TXMEM_SIZE(NbPkt)		CFIFO_TOTAL_MEMSIZE((NbPkt), USBD_BULK_PKT_BLKSIZE)

typedef enum __Usbd_Bulk_Mode {
	USBD_BULK_MODE_BYTE,			//!< Byte stream, UsbIntrf packetizes up to MPS
	USBD_BULK_MODE_PACKET			//!< One UsbPkt_t CFifo block per USB packet
} UsbdBulkMode_t;

#pragma pack(push, 1)

/// Descriptor fragment for one vendor interface and its endpoint pair.
typedef struct __Usbd_Bulk_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} UsbdBulkDesc_t;

#pragma pack(pop)

#pragma pack(push, 4)

typedef struct __Usbd_Bulk_Config {
	bool bBlocking;
	int RxFifoMemSize;
	uint8_t *pRxFifoMem;
	int TxFifoMemSize;
	uint8_t *pTxFifoMem;
	int DevNo;
	uint8_t SubClass;
	uint8_t Protocol;
	uint8_t InterfaceString;
	uint16_t FsMps;				//!< Zero selects USBD_BULK_FS_MPS
	uint16_t HsMps;				//!< Zero selects USBD_BULK_HS_MPS
	UsbdBulkMode_t Mode;
	UsbRequestHandler_t RequestHandler;	//!< Optional vendor request handler
	void *pRequestContext;
	DevIntrfEvtHandler_t EvtCB;
} UsbdBulkCfg_t;

typedef struct __Usbd_Bulk_Dev {
	UsbDevIntrf_t *pData;
	UsbRequestHandler_t RequestHandler;
	void *pRequestContext;
	int ItfNo;					//!< Internal allocation
	int DevNo;
	uint8_t EpNo;				//!< Internal allocation
	uint8_t SubClass;
	uint8_t Protocol;
	uint8_t InterfaceString;
	uint16_t FsMps;
	uint16_t HsMps;
	uint32_t RxTransfer[(USBD_BULK_MAX_MPS + sizeof(uint32_t) - 1U) /
						 sizeof(uint32_t)];
	uint32_t TxTransfer[(USBD_BULK_MAX_MPS + sizeof(uint32_t) - 1U) /
						 sizeof(uint32_t)];
} UsbdBulkDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool UsbdBulkInit(UsbdBulkDev_t * const pBulk,
				  UsbDevIntrf_t * const pData,
				  const UsbdBulkCfg_t *pCfg);

/** Build the interface plus OUT/IN endpoint descriptor fragment. */
bool UsbdBulkMakeDesc(UsbdBulkDesc_t *pDesc, const UsbdBulkDev_t *pBulk,
					  UsbSpeed_t Speed);

#ifdef __cplusplus
}

class UsbdBulk : public UsbIntrf {
public:
	UsbdBulk() = default;

	bool Init(const UsbdBulkCfg_t &Cfg);

	DevIntrf_t *Data(void) { return static_cast<DevIntrf_t *>(*this); }

	bool MakeDesc(UsbdBulkDesc_t *pDesc, UsbSpeed_t Speed) const;

private:
	UsbdBulkDev_t vUsbdBulk = {};
};
#endif

/** @} End of group USBD */

#endif	// __USBD_BULK_H__
