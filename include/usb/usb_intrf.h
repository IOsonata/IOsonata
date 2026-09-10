/**-------------------------------------------------------------------------
@file	usb_intrf.h

@brief	Internal USB endpoint-pair implementation of DeviceIntrf.

UsbIntrf is the reusable data path inherited by public USB classes such as
UsbdCdc. One instance is one bidirectional endpoint number: OUT is receive and
IN is transmit. Applications use the derived USB class, not UsbIntrf directly.
DeviceIntrf DevAddr is not used to select an endpoint on each transfer.

The generic layer in usb.cpp handles endpoint zero, Chapter 9 requests,
descriptors, configuration and class/vendor dispatch. The port handles endpoint
registers, DMA access and controller interrupts.

UsbIntrf supports three data policies. Byte and packet modes use CFifo storage
because their transfers may wait in software. Direct mode does not queue packets:
it owns one statically reserved RX slot and one TX slot. Each direct slot is a
UsbPkt_t whose Hdr.Flags bit USB_INTRF_SLOT_READY publishes whether the slot
contains a current packet; Hdr.Length remains the actual payload length and may
be zero.

The derived class supplies one fixed RX and one fixed TX controller buffer sized
for its transfer type. In byte and packet mode those are DMA staging buffers.
In direct mode the supplied buffers include UsbPktHdr_t followed by the payload;
UsbIntrf registers the Data portion with the controller and uses the header as
the single-slot ownership state.

RX is event driven. USB_CTRLR_EVT_DRDY means data is ready in the controller to
be retrieved. Byte and packet modes use the existing CFifo blocking/non-blocking
policy. A direct specialization selects whether the controller uses DRDY or
services OUT transfers directly. Completion publishes the single RX slot
instead of placing data into a FIFO.

For IN, byte and packet modes copy queued TX data into fixed staging before
submitting the endpoint transfer. Direct TxData copies one current packet into
the single TX slot and submits the endpoint transfer. The endpoint transfer type
and its scheduling remain properties of the specialization and controller.

UsbPktHdr_t.Length is the actual data length and may be from zero to MPS.
UsbPktHdr_t.Flags is zero in ordinary packet mode. Direct mode uses bit
USB_INTRF_SLOT_READY as the slot-ready flag. Reserved remains a source-compatible
alias for existing packet-mode code.

Generic code must not assume a 64-byte packet, a specific USB speed, or a
specific controller.

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

#define USB_INTRF_PKT_BLKSIZE(Mps) \
	((uint32_t)((sizeof(UsbPktHdr_t) + (uint32_t)(Mps) + 3U) & ~3U))

#define USB_INTRF_RXMEM_SIZE(NbPkt, Mps) \
	CFIFO_TOTAL_MEMSIZE(NbPkt, USB_INTRF_PKT_BLKSIZE(Mps))

#define USB_INTRF_SLOT_READY			1U

#pragma pack(push, 4)

typedef struct __Usb_Packet_Header {
	uint16_t Length;
	union {
		uint16_t Flags;
		uint16_t Reserved;
	};
} UsbPktHdr_t;

typedef struct __Usb_Packet {
	UsbPktHdr_t Hdr;
	uint8_t Data[1];
} UsbPkt_t;

typedef enum __Usb_Interf_Mode {
	USB_INTRF_MODE_AUTO = 0,
	USB_INTRF_MODE_BYTE,
	USB_INTRF_MODE_PACKET,
	USB_INTRF_MODE_DIRECT,
} UsbIntrfMode_t;

typedef struct __Usb_Interf_Config {
	int DevNo;
	uint8_t EpNo;
	bool bBlocking;				//!< CFifo/controller blocking policy
	bool bRxPrearm;				//!< Keep a DIRECT OUT transfer armed
	UsbIntrfMode_t Mode;
	int RxFifoMemSize;
	uint8_t *pRxFifoMem;
	int TxFifoMemSize;
	uint8_t *pTxFifoMem;
	uint16_t TxFifoBlkSize;
	uint16_t BufferSize;
	uint8_t *pRxBuffer;
	uint8_t *pTxBuffer;
	DevIntrfEvtHandler_t EvtCB;
} UsbIntrfCfg_t;

#pragma pack(pop)

typedef struct __Usb_Dev_Interf		UsbDevIntrf_t;
typedef int (*EpSendFct_t)(UsbDevIntrf_t *pIntrf);

struct __Usb_Dev_Interf {
	int DevNo;
	DevIntrf_t DevIntrf;
	hCFifo_t hTxFifo;
	hCFifo_t hRxFifo;
	uint32_t RxDropCnt;
	uint8_t *pRxBuffer;
	uint8_t *pTxBuffer;
	UsbPkt_t *pRxDirectBuffer;
	UsbPkt_t *pTxDirectBuffer;
	uint16_t BufferSize;
	uint16_t Mps;
	uint8_t EpNo;
	bool bBlocking;
	bool bRxPrearm;
	bool RxPending;
	UsbIntrfMode_t Mode;
	EpSendFct_t EpSend;
	void *pClassContext;
};

#ifdef __cplusplus
extern "C" {
#endif

bool UsbIntrfInit(UsbDevIntrf_t *pIntrf, const UsbIntrfCfg_t *pCfg);
bool UsbIntrfConfigure(UsbDevIntrf_t *pIntrf, uint16_t Mps);
void UsbIntrfUnconfigure(UsbDevIntrf_t *pIntrf);
bool UsbIntrfArmRx(UsbDevIntrf_t *pIntrf);
bool UsbIntrfRequestToSend(UsbDevIntrf_t *pIntrf, int NbBytes);

#ifdef __cplusplus
}
#endif


/** @} End of group USBD */

#endif	// __USB_INTRF_H__
