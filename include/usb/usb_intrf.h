/**-------------------------------------------------------------------------
@file	usb_intrf.h

@brief	Generic USB data-interface implementation of DeviceIntrf.

DeviceIntrf is the IOsonata data interface used by devices and applications.
UsbIntrf applies DeviceIntrf to a configured USB data endpoint. Endpoint
address/direction, transfer type and maximum packet size (MPS) are instance
state. DeviceIntrf DevAddr is not used to select an endpoint on each transfer.

The generic layer in usb.cpp handles endpoint zero, Chapter 9 requests,
descriptors, configuration and class/vendor dispatch. The port handles endpoint
registers, DMA/FIFO access and controller interrupts.

RX storage is a ring of fixed packet slots supplied by the application, not a
CFifo. Each slot contains UsbPktHdr_t followed by storage for one endpoint
packet, word aligned, sized with USB_INTRF_RXMEM_SIZE. The controller is armed
directly on the slot the ring will publish next and writes into it, so nothing
is copied on the receive path. The slot becomes visible to the reader only
after the transfer completes. UsbPktHdr_t.Length is the actual received data
length and may be from zero to MPS. DeviceIntrfRx() may consume across packet
slots and present a byte stream.

TX CFifo mode is selected by block size. BlkSize 1 provides byte-stream
accumulation and UsbIntrf packetizes queued bytes up to the endpoint MPS. In
packet mode, each CFifo block contains UsbPktHdr_t followed by storage for one
endpoint packet. The caller splits its data into USB packets and pushes one
CFifo block per packet. UsbPktHdr_t.Length is the actual packet data length and
may be from zero to MPS. UsbIntrf sends the stored packet length without
combining packet blocks. USB transfer type and CFifo block size are separate
settings.

RX ring memory and TX CFifo memory are both supplied by UsbIntrfCfg_t. Endpoint
MPS determines the RX slot size and the TX packet-mode block size. Port maximum
packet-size definitions may be used by static allocation macros so the
configuration reserves sufficient memory.

TX still uses a temporary packet-sized buffer, because CFifoGet releases a
block before the controller is done with it and byte mode has to gather across
the ring wrap.

Generic code must not assume a 64-byte packet, a specific USB speed, or a
specific controller.

There is no DTR style gating in this generic interface. A class adapter may
enable or disable RX acceptance with UsbIntrfRxEnable(). USB OUT flow control
is lossless: when receive storage is full the endpoint is left unarmed so the
host is backpressured by USB.

@author	Hoang Nguyen Hoan
@date	Sep. 1, 2026

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
#ifndef __USB_INTRF_H__
#define __USB_INTRF_H__

#include <stdbool.h>
#include <stdint.h>

#include "cfifo.h"
#include "device_intrf.h"
#include "usb/usb.h"

/** @addtogroup USBD
  * @{
  */

/// Bytes one stored packet occupies, header plus payload, word aligned.
#define USB_INTRF_PKT_BLKSIZE(Mps) \
	((uint32_t)((sizeof(UsbPktHdr_t) + (uint32_t)(Mps) + 3U) & ~3U))

/// RX memory holding NbPkt packet slots for an Mps byte endpoint.
#define USB_INTRF_RXMEM_SIZE(NbPkt, Mps) \
	((uint32_t)(NbPkt) * USB_INTRF_PKT_BLKSIZE(Mps))

#pragma pack(push, 4)

typedef struct __Usb_Packet_Header {
	uint16_t Length;				//!< Actual packet length, from zero through MPS
	uint16_t Reserved;			//!< Keeps the packet payload word aligned
} UsbPktHdr_t;

typedef struct __Usb_Interf_Config {
	int DevNo;					//!< USB controller number
	bool bBlocking;
	int RxFifoMemSize;
	uint8_t *pRxFifoMem;
	int TxFifoMemSize;
	uint8_t *pTxFifoMem;
	uint16_t TxFifoBlkSize;		//!< 1 for byte mode, header plus MPS for packet mode
	DevIntrfEvtHandler_t EvtCB;
} UsbIntrfCfg_t;

typedef struct __Usb_Dev_Interf {
	DevIntrf_t DevIntrf;
	hCFifo_t hTxFifo;
	int DevNo;					//!< USB controller number
	uint8_t *pRxMem;			//!< RX packet ring memory supplied by application
	uint32_t RxMemSize;			//!< RX packet ring memory size in bytes
	uint32_t RxPut;				//!< Completed RX packets published
	uint32_t RxGet;				//!< RX packets fully consumed
	uint32_t RxUsed;				//!< Published RX payload bytes not yet consumed
	uint32_t RxDropCnt;			//!< Controller/error drops, ring full uses backpressure
	uint16_t RxOffset;			//!< Byte-stream cursor in the head RX packet
	uint16_t RxSlotSize;		//!< Bytes one packet slot occupies
	uint16_t RxSlotCnt;			//!< Packet slots in pRxMem
	uint16_t RxMps;				//!< OUT endpoint maximum packet size
	uint8_t RxEpAddr;			//!< OUT endpoint owned by this instance
	uint8_t TxEpAddr;			//!< IN endpoint owned by this instance
	uint8_t *pTxBuffer;			//!< One packet staging buffer, TxMps bytes
	uint16_t TxMps;				//!< IN endpoint maximum packet size
	bool bEnabled;
	bool RxActive;				//!< OUT transfer currently registered with controller
	bool RxAccepting;			//!< Class/application currently accepts OUT traffic
	bool TxZlpRequired;			//!< Last packet was full and nothing followed
	bool TxTailArmed;			//!< Partial tail seen idle at previous SOF
} UsbDevIntrf_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool UsbIntrfInit(UsbDevIntrf_t *pIntrf, const UsbIntrfCfg_t *pCfg);

/**
 * Bind an OUT endpoint and build the RX packet ring for its maximum packet
 * size. Returns false when the supplied RX memory cannot hold one packet slot.
 *
 * There is no RX staging buffer. The controller writes straight into the slot
 * the ring is about to publish, and the slot becomes visible to the reader
 * only after the transfer completes.
 */
bool UsbIntrfConfigRx(UsbDevIntrf_t *pIntrf, uint8_t EpAddr, uint16_t Mps);

/** Drop RX endpoint/ring state on bus reset or unconfigure. */
void UsbIntrfResetRx(UsbDevIntrf_t *pIntrf);

/** Enable or disable acceptance of OUT data without changing endpoint config. */
void UsbIntrfRxEnable(UsbDevIntrf_t *pIntrf, bool Enable);

/** OUT endpoint transfer completion, called from the USB interrupt. */
void UsbIntrfRxXferComplete(UsbDevIntrf_t *pIntrf,
						 uint16_t Length, UsbCtrlrXferResult_t Result);

/**
 * Bind an IN endpoint to this instance after SET_CONFIGURATION.
 * pBuffer must hold Mps bytes and remain valid while configured.
 */
void UsbIntrfConfigTx(UsbDevIntrf_t *pIntrf, uint8_t EpAddr,
					   uint16_t Mps, uint8_t *pBuffer);

/** Drop endpoint transfer state and queued data on bus reset or unconfigure. */
void UsbIntrfResetTx(UsbDevIntrf_t *pIntrf);

/** IN endpoint transfer completion, called from the USB interrupt. */
void UsbIntrfTxXferComplete(UsbDevIntrf_t *pIntrf,
						 uint16_t Length, UsbCtrlrXferResult_t Result);

/** Start-of-frame callback used to flush an idle partial TX tail. */
void UsbIntrfSof(UsbDevIntrf_t *pIntrf);

bool UsbIntrfRequestToSend(UsbDevIntrf_t *pIntrf, int NbBytes);
int UsbIntrfRxUsed(UsbDevIntrf_t *pIntrf);
int UsbIntrfTxUsed(UsbDevIntrf_t *pIntrf);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class UsbIntrf : public DeviceIntrf {
public:
	bool Init(const UsbIntrfCfg_t &Cfg) {
		return UsbIntrfInit(&vUsbDevIntrf, &Cfg);
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
		return UsbIntrfRequestToSend(&vUsbDevIntrf, NbBytes);
	}

protected:
	UsbDevIntrf_t vUsbDevIntrf = {};
};

#endif

/** @} End of group USBD */

#endif	// __USB_INTRF_H__
