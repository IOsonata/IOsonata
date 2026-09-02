/**-------------------------------------------------------------------------
@file	usbd_intrf.h

@brief	Generic USB endpoint implementation of DeviceIntrf.

DeviceIntrf is the IOsonata data interface used by devices and applications.
UsbdIntrf applies DeviceIntrf to a configured USB data endpoint. Endpoint
address/direction, transfer type and maximum packet size (MPS) are instance
state. DeviceIntrf DevAddr is not used to select an endpoint on each transfer.

UsbdCore handles endpoint zero, Chapter 9 requests, descriptors,
configuration and class/vendor dispatch. UsbdCtrlr handles endpoint registers,
DMA/FIFO access and controller interrupts.

RX uses packet-mode CFifo storage. USB hardware reports one received packet at
a time and the complete packet is retrieved in one operation. Each RX CFifo
block contains UsbPktHdr_t followed by storage for one endpoint packet. Block
size is based on sizeof(UsbPktHdr_t) plus the endpoint MPS. UsbPktHdr_t.Length
is the actual received data length and may be from zero to MPS. DeviceIntrfRx()
may consume across packet blocks and present a byte stream.

TX CFifo mode is selected by block size. BlkSize 1 provides byte-stream
accumulation and UsbdIntrf packetizes queued bytes up to the endpoint MPS. In
packet mode, each CFifo block contains UsbPktHdr_t followed by storage for one
endpoint packet. The caller splits its data into USB packets and pushes one
CFifo block per packet. UsbPktHdr_t.Length is the actual packet data length and
may be from zero to MPS. UsbdIntrf sends the stored packet length without
combining packet blocks. USB transfer type and CFifo block size are separate
settings.

CFifo memory is supplied by UsbdIntrfCfg_t. Endpoint MPS determines the RX
packet block size and the TX packet-mode block size. Port maximum packet-size
definitions may be used by static allocation macros so the configuration
reserves sufficient memory.

Controller DMA storage and CFifo storage are separate. TX may require a
temporary packet-sized DMA buffer while an IN transfer is active. RX may DMA
directly into packet storage when the controller permits it.

Generic code must not assume a 64-byte packet, a specific USB speed, or a
specific controller.

There is no DTR style gating in this generic interface. A class adapter may
enable or disable RX acceptance with UsbdIntrfRxEnable(). USB OUT flow control
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
#ifndef __USBD_INTRF_H__
#define __USBD_INTRF_H__

#include <stdbool.h>
#include <stdint.h>

#include "cfifo.h"
#include "device_intrf.h"
#include "usb/usbd_ctrlr.h"

/** @addtogroup USBD
  * @{
  */

#pragma pack(push, 4)

typedef struct __Usbd_Interf_Config {
	bool bBlocking;
	int RxFifoMemSize;
	uint8_t *pRxFifoMem;
	int TxFifoMemSize;
	uint8_t *pTxFifoMem;
	DevIntrfEvtHandler_t EvtCB;
} UsbdIntrfCfg_t;

typedef struct __Usbd_Dev_Interf {
	DevIntrf_t DevIntrf;
	hCFifo_t hTxFifo;
	uint8_t *pRxMem;				//!< RX packet-ring memory supplied by application
	uint32_t RxMemSize;			//!< RX packet-ring memory size in bytes
	uint32_t RxPut;				//!< Number of completed RX packets published
	uint32_t RxGet;				//!< Number of RX packets fully consumed
	uint32_t RxUsed;				//!< Published RX payload bytes not yet consumed
	uint32_t RxDropCnt;			//!< Controller/error drops; ring-full uses backpressure
	uint16_t RxMps;				//!< OUT endpoint maximum packet size
	uint16_t RxSlotSize;			//!< Bytes occupied by one packet slot
	uint16_t RxSlotCnt;			//!< Number of packet slots in pRxMem
	uint8_t RxEpAddr;			//!< OUT endpoint owned by this instance
	uint8_t TxEpAddr;			//!< IN endpoint owned by this instance
	uint8_t *pTxBuffer;			//!< One packet staging buffer, TxMps bytes
	uint16_t TxMps;				//!< IN endpoint maximum packet size
	bool bEnabled;
	bool RxActive;				//!< OUT transfer currently registered with controller
	bool RxAccepting;			//!< Class/application currently accepts OUT traffic
	bool TxZlpRequired;			//!< Last packet was full and nothing followed
	bool TxTailArmed;			//!< Partial tail seen idle at previous SOF
} UsbdDevIntrf_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool UsbdIntrfInit(UsbdDevIntrf_t *pIntrf, const UsbdIntrfCfg_t *pCfg);

/**
 * Bind an OUT endpoint and build the RX packet ring for its maximum packet
 * size. Returns false when the supplied RX memory cannot hold one packet slot.
 */
bool UsbdIntrfConfigRx(UsbdDevIntrf_t *pIntrf, uint8_t EpAddr,
					   uint16_t Mps);

/** Drop RX endpoint/ring state on bus reset or unconfigure. */
void UsbdIntrfResetRx(UsbdDevIntrf_t *pIntrf);

/** Enable or disable acceptance of OUT data without changing endpoint config. */
void UsbdIntrfRxEnable(UsbdDevIntrf_t *pIntrf, bool Enable);

/** OUT endpoint transfer completion, called from the USB interrupt. */
void UsbdIntrfRxXferComplete(UsbdDevIntrf_t *pIntrf,
						 uint16_t Length, UsbdCtrlrXferResult_t Result);

/**
 * Bind an IN endpoint to this instance after SET_CONFIGURATION.
 * pBuffer must hold Mps bytes and remain valid while configured.
 */
void UsbdIntrfConfigTx(UsbdDevIntrf_t *pIntrf, uint8_t EpAddr,
					   uint16_t Mps, uint8_t *pBuffer);

/** Drop endpoint transfer state on bus reset or unconfigure. FIFO data stays. */
void UsbdIntrfResetTx(UsbdDevIntrf_t *pIntrf);

/** IN endpoint transfer completion, called from the USB interrupt. */
void UsbdIntrfTxXferComplete(UsbdDevIntrf_t *pIntrf,
						 uint16_t Length, UsbdCtrlrXferResult_t Result);

/** Start-of-frame callback used to flush an idle partial TX tail. */
void UsbdIntrfSof(UsbdDevIntrf_t *pIntrf);

bool UsbdIntrfRequestToSend(UsbdDevIntrf_t *pIntrf, int NbBytes);
int UsbdIntrfRxUsed(UsbdDevIntrf_t *pIntrf);
int UsbdIntrfTxUsed(UsbdDevIntrf_t *pIntrf);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class UsbdIntrf : public DeviceIntrf {
public:
	bool Init(const UsbdIntrfCfg_t &Cfg) {
		return UsbdIntrfInit(&vUsbDevIntrf, &Cfg);
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
		return UsbdIntrfRequestToSend(&vUsbDevIntrf, NbBytes);
	}

protected:
	UsbdDevIntrf_t vUsbDevIntrf = {};
};

#endif

/** @} End of group USBD */

#endif	// __USBD_INTRF_H__
