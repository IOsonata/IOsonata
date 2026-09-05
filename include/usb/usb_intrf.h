/**-------------------------------------------------------------------------
@file	usb_intrf.h

@brief	Internal USB endpoint-pair implementation of DeviceIntrf.

UsbIntrf is the reusable data path inherited by public USB classes such as
UsbdCdc. One instance is one bidirectional endpoint number: OUT is receive and
IN is transmit. Applications use the derived USB class, not UsbIntrf directly.
DeviceIntrf DevAddr is not used to select an endpoint on each transfer.

The generic layer in usb.cpp handles endpoint zero, Chapter 9 requests,
descriptors, configuration and class/vendor dispatch. The port handles endpoint
registers, DMA/FIFO access and controller interrupts.

The derived class supplies one RX and one TX controller buffer sized for its
transfer type. On OUT completion UsbIntrf copies the RX buffer into hRxFifo and
reuses the same controller buffer. On IN it copies queued TX data into the TX
buffer before submitting it. Controller buffers therefore never alias CFifo
storage that can be released while a transfer is active.

UsbPktHdr_t.Length is the actual received data length and may be from zero to
MPS. DeviceIntrfRx() may consume across packet blocks and present a byte
stream.

TX CFifo mode is selected by block size. BlkSize 1 provides byte-stream
accumulation and UsbIntrf packetizes queued bytes up to the endpoint MPS. In
packet mode, each CFifo block contains UsbPktHdr_t followed by storage for one
endpoint packet. The caller splits its data into USB packets and pushes one
CFifo block per packet. UsbPktHdr_t.Length is the actual packet data length and
may be from zero to MPS. UsbIntrf sends the stored packet length without
combining packet blocks. USB transfer type and CFifo block size are separate
settings.

The application supplies RX and TX CFifo memory through the public USB class.
The class supplies its controller buffers and endpoint number to UsbIntrf.
The actual MPS is applied after the bus speed is known and the class opens both
directions of the endpoint.

Generic code must not assume a 64-byte packet, a specific USB speed, or a
specific controller.

There is no DTR style gating in this layer. USB OUT flow control is lossless:
when receive storage is full the endpoint is left unarmed so the host is
backpressured by USB.

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

/// RX memory holding NbPkt packet slots for an Mps byte endpoint, including
/// the CFifo header that CFifoInit carves out of the same buffer.
#define USB_INTRF_RXMEM_SIZE(NbPkt, Mps) \
	CFIFO_TOTAL_MEMSIZE(NbPkt, USB_INTRF_PKT_BLKSIZE(Mps))

#pragma pack(push, 4)

typedef struct __Usb_Packet_Header {
	uint16_t Length;			//!< Actual packet length, from zero through MPS
	uint16_t Reserved;			//!< Keeps the packet payload word aligned
} UsbPktHdr_t;

// This structure must be cast to memory block, no allocate
typedef struct __Usb_Packet {
	UsbPktHdr_t Hdr;
	uint8_t Data[1];			//!< Variable length
} UsbPkt_t;

typedef struct __Usb_Interf_Config {
	int DevNo;					//!< USB controller number
	uint8_t EpNo;				//!< Bidirectional endpoint number, never an address
	bool bBlocking;
	int RxFifoMemSize;
	uint8_t *pRxFifoMem;
	int TxFifoMemSize;
	uint8_t *pTxFifoMem;
	uint16_t TxFifoBlkSize;		//!< 1 for byte mode, header plus MPS for packet mode
	uint16_t BufferSize;		//!< Capacity of each controller transfer buffer
	uint8_t *pRxBuffer;			//!< OUT transfer buffer supplied by derived class
	uint8_t *pTxBuffer;			//!< IN transfer buffer supplied by derived class
	DevIntrfEvtHandler_t EvtCB;
} UsbIntrfCfg_t;


typedef struct __Usb_Dev_Interf		UsbDevIntrf_t;

/// True when queued data was consumed, including a zero-length packet.
typedef bool (*EpSendFct_t)(UsbDevIntrf_t *pIntrf);

struct __Usb_Dev_Interf {
	int DevNo;					//!< USB controller number
	DevIntrf_t DevIntrf;
	hCFifo_t hTxFifo;
	hCFifo_t hRxFifo;			//!< Packet storage, one endpoint packet per block
	uint32_t RxDropCnt;			//!< Controller/error drops, FIFO full uses backpressure
	uint8_t *pRxBuffer;			//!< One OUT packet controller buffer
	uint8_t *pTxBuffer;			//!< One IN packet controller buffer
	uint16_t BufferSize;		//!< Capacity of pRxBuffer and pTxBuffer
	uint16_t Mps;				//!< Active packet size, zero while unconfigured
	uint8_t EpNo;				//!< Bidirectional endpoint number
	EpSendFct_t EpSend;
};


#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool UsbIntrfInit(UsbDevIntrf_t *pIntrf, const UsbIntrfCfg_t *pCfg);

/** Apply the negotiated MPS after both endpoint directions are open. */
bool UsbIntrfConfigure(UsbDevIntrf_t *pIntrf, uint16_t Mps);

/** Stop both directions and flush data on bus reset or unconfigure. */
void UsbIntrfUnconfigure(UsbDevIntrf_t *pIntrf);

/** Completion for either direction of EpNo, called from the USB interrupt. */
void UsbIntrfXferComplete(UsbDevIntrf_t *pIntrf, uint8_t EpAddr,
						  uint16_t Length, UsbCtrlrXferResult_t Result);

bool UsbIntrfRequestToSend(UsbDevIntrf_t *pIntrf, int NbBytes);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class UsbIntrf : public DeviceIntrf {
public:
	operator DevIntrf_t * () override {
		return &vUsbDevIntrf.DevIntrf;
	}

	/**
	 * Bring up the endpoint data path. A derived class supplies the endpoint
	 * number and its buffers; everything after this it does not manage.
	 */
	bool Init(const UsbIntrfCfg_t &Cfg) {
		return UsbIntrfInit(&vUsbDevIntrf, &Cfg);
	}

	uint32_t Rate(uint32_t DataRate) override {
		return DeviceIntrfSetRate(&vUsbDevIntrf.DevIntrf, DataRate);
	}

	uint32_t Rate(void) override {
		return DeviceIntrfGetRate(&vUsbDevIntrf.DevIntrf);
	}

	bool RequestToSend(int NbBytes) override {
		return UsbIntrfRequestToSend(&vUsbDevIntrf, NbBytes);
	}

	// The inherited versions pass *this, which calls the virtual conversion
	// operator to reach the same pointer this object already holds. On a byte
	// at a time stream that indirect call is paid per byte. Rate and
	// RequestToSend above already take the pointer directly; these do too.
	__attribute__((always_inline))
	int Tx(uint32_t DevAddr, const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTx(&vUsbDevIntrf.DevIntrf, DevAddr, pData, DataLen);
	}

	__attribute__((always_inline))
	int Rx(uint32_t DevAddr, uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRx(&vUsbDevIntrf.DevIntrf, DevAddr, pBuff, BuffLen);
	}

	__attribute__((always_inline))
	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vUsbDevIntrf.DevIntrf, pData, DataLen);
	}

	__attribute__((always_inline))
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vUsbDevIntrf.DevIntrf, pBuff, BuffLen);
	}

protected:
	UsbIntrf() = default;
	UsbIntrf(const UsbIntrf &) = delete;
	UsbIntrf &operator = (const UsbIntrf &) = delete;

	// The endpoint data path is owned here, not pointed at. A polymorphic
	// object cannot be reinterpreted as the C struct anyway, so there is
	// nothing to gain from placing it in the derived type and binding to it.
	UsbDevIntrf_t vUsbDevIntrf = {};
};

#endif

/** @} End of group USBD */

#endif	// __USB_INTRF_H__
