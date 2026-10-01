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

Byte and packet modes have no separate RX buffer: the RX CFifo is the DMA
destination. Init binds each endpoint's callback and context. On DRDY,
UsbIntrf reserves the next RX block (CFifoResv) and calls UsbCtrlrEpReceive
with its Data portion and capacity. The controller accepts one OUT transfer
and schedules DMA. At completion, UsbIntrf writes the packet header, publishes
the block with CFifoPut, then reports DEVINTRF_EVT_RX_DATA to the application.
Only this producer moves PutIdx; the reserved block remains owned until DMA
completes. BufferSize sizes the RX blocks and must cover the endpoint MPS.
Byte and packet modes use the TX CFifo as the transfer source.

Direct mode supplies RX and TX slots, each with UsbPktHdr_t followed by the
payload. The header records single-slot ownership. Direct RX completion
publishes the received slot and invokes the application callback.

USB_CTRLR_EVT_DRDY requests an OUT destination. A full blocking FIFO or occupied
blocking direct slot leaves the receive pending. RxData retries submission
when it releases space, including when consuming a zero-length packet.
Non-blocking FIFO mode gives up its oldest packet at DRDY when space is needed.
The controller owns DMA arbitration and prevents duplicate submissions for a
packet already queued or active. ISO retains the controller's interval scheduler.

For IN, byte and packet modes retain queued TX data until host consumption
completes the endpoint transfer. Direct TxData copies one current packet into
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
	bool bBlocking;
	UsbIntrfMode_t Mode;
	int RxFifoMemSize;
	uint8_t *pRxFifoMem;
	int TxFifoMemSize;
	uint8_t *pTxFifoMem;
	uint16_t TxFifoBlkSize;
	uint16_t BufferSize;
	uint8_t *pRxBuffer;		//!< Direct-mode RX slot; unused in byte/packet modes
	uint8_t *pTxBuffer;		//!< Direct-mode TX slot; unused in byte/packet modes
	DevIntrfEvtHandler_t EvtCB;
} UsbIntrfCfg_t;

#pragma pack(pop)

typedef struct __Usb_Dev_Interf		UsbDevIntrf_t;

struct __Usb_Dev_Interf {
	// Endpoint state ahead of the DevIntrf block so the transfer paths
	// address it with short load and store offsets.
	int DevNo;
	uint16_t BufferSize;
	uint16_t Mps;
	uint16_t RxPending;		//!< 1: DRDY waiting for RX space or controller admission
	uint8_t EpNo : 7;
	bool bBlocking : 1;
	UsbIntrfMode_t Mode;
	// Mode selects FIFO storage or a direct packet slot for each direction.
	union {
		hCFifo_t hTxFifo;
		UsbPkt_t *pTxDirectBuffer;
	};
	union {
		hCFifo_t hRxFifo;
		UsbPkt_t *pRxDirectBuffer;
	};
	uint32_t RxDropCnt;
	void *pClassContext;
	DevIntrf_t DevIntrf;
};

#ifdef __cplusplus
extern "C" {
#endif

bool UsbIntrfInit(UsbDevIntrf_t *pIntrf, const UsbIntrfCfg_t *pCfg);
bool UsbIntrfConfigure(UsbDevIntrf_t *pIntrf, uint16_t Mps);
void UsbIntrfUnconfigure(UsbDevIntrf_t *pIntrf);

// Internal buffer-less status notification shared by USB specializations.
void UsbIntrfNotify(UsbDevIntrf_t *pIntrf, DEVINTRF_EVT Event, int Length);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class UsbIntrf : public DeviceIntrf {
public:
	operator DevIntrf_t * () override {
		return &vUsbDevIntrf.DevIntrf;
	}

	DevIntrf_t *Data(void) { return &vUsbDevIntrf.DevIntrf; }

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

	// The USB implementation owns this DevIntrf directly. Avoid the generic
	// C++ wrappers converting through the virtual operator again.
	void Disable(void) override {
		DeviceIntrfDisable(&vUsbDevIntrf.DevIntrf);
	}

	void Enable(void) override {
		DeviceIntrfEnable(&vUsbDevIntrf.DevIntrf);
	}

	int Read(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen,
			 uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRead(&vUsbDevIntrf.DevIntrf, DevAddr,
			pAdCmd, AdCmdLen, pBuff, BuffLen);
	}

	int Write(uint32_t DevAddr, const uint8_t *pAdCmd, int AdCmdLen,
			  const uint8_t *pData, int DataLen) override {
		return DeviceIntrfWrite(&vUsbDevIntrf.DevIntrf, DevAddr,
			pAdCmd, AdCmdLen, pData, DataLen);
	}

	// Preserve the generic busy/hook contract while avoiding the base
	// class virtual handle conversion for this owned DevIntrf instance.
	bool StartRx(uint32_t DevAddr) override {
		return DeviceIntrfStartRx(&vUsbDevIntrf.DevIntrf, DevAddr);
	}

	void StopRx(void) override {
		DeviceIntrfStopRx(&vUsbDevIntrf.DevIntrf);
	}

	bool StartTx(uint32_t DevAddr) override {
		return DeviceIntrfStartTx(&vUsbDevIntrf.DevIntrf, DevAddr);
	}

	void StopTx(void) override {
		DeviceIntrfStopTx(&vUsbDevIntrf.DevIntrf);
	}

	// Use the owned data directly without a virtual conversion on each call.
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

	// One endpoint-pair data path, shared by all derived transport operations.
	UsbDevIntrf_t vUsbDevIntrf = {};
};

#endif

/** @} End of group USBD */

#endif	// __USB_INTRF_H__

