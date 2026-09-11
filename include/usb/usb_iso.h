/**-------------------------------------------------------------------------
@file	usb_iso.h

@brief	Reusable USB isochronous interface.

UsbIsoIntrf is the isochronous specialization of UsbIntrf. UsbIntrf remains the
single endpoint-pair DeviceIntrf implementation, but ISO selects a different
data policy from byte/packet traffic: one statically reserved RX slot and one
TX slot, no CFifo queue.

The layering is therefore:

    UsbIsoIntrf
        -> UsbIntrf (DIRECT mode)
            -> registered endpoint callback / controller ISO scheduling

Each ISO slot is one UsbPkt_t-sized block. Hdr.Reserved carries the
USB_INTRF_SLOT_READY flag and Hdr.Length carries the current payload length.
Length zero is therefore a valid ISO packet and is distinct from a free slot.

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

#define USB_ISO_INTRF_MAX_MPS		((uint16_t)USB_PKT_MAXLEN(0, ISO))
#define USB_ISO_INTRF_PKT_BLKSIZE \
	USB_INTRF_PKT_BLKSIZE(USB_ISO_INTRF_MAX_MPS)
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
	uint8_t EpNo;					//!< Internally allocated ISO endpoint number
	uint8_t Attributes;			//!< ISO sync/usage bits; zero = no-sync data
	UsbIsoIntrfRxHandler_t RxHandler;
	UsbIsoIntrfTxHandler_t TxHandler;
	void *pContext;
} UsbIsoIntrfCfg_t;

#pragma pack(pop)

struct __Usb_Iso_Interf {
	UsbDevIntrf_t IntrfData;		//!< Endpoint data path, owned by value
	void *pContext;
	UsbIsoIntrfRxHandler_t RxHandler;
	UsbIsoIntrfTxHandler_t TxHandler;
	uint32_t RxMissCnt;
	uint32_t TxMissCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
	uint16_t Mps;
	uint8_t EpNo;
	uint8_t Interval;
	uint8_t Attributes;
	bool Opened;
	bool Suspended;

	// One current packet per direction. UsbIntrf DIRECT mode uses the packet
	// header as ownership state and registers the Data portion for DMA.
	uint32_t RxBuffer[USB_ISO_INTRF_PACKET_WORDS];
	uint32_t TxBuffer[USB_ISO_INTRF_PACKET_WORDS];
};

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize an ISO interface using its embedded UsbIntrf data object. */
bool UsbIsoIntrfInit(UsbIsoIntrf_t *pIntrf, const UsbIsoIntrfCfg_t *pCfg);

/** Open the internally assigned endpoint pair as isochronous. */
bool UsbIsoIntrfOpen(UsbIsoIntrf_t *pIntrf, uint16_t Mps, uint8_t Interval);
void UsbIsoIntrfClose(UsbIsoIntrf_t *pIntrf);
void UsbIsoIntrfReset(UsbIsoIntrf_t *pIntrf);
void UsbIsoIntrfSuspend(UsbIsoIntrf_t *pIntrf);
bool UsbIsoIntrfResume(UsbIsoIntrf_t *pIntrf);

/** Publish one ISO IN frame into the current TX slot. */
bool UsbIsoIntrfSendFrame(UsbIsoIntrf_t *pIntrf, const uint8_t *pData,
						  uint16_t Length);

static inline bool UsbIsoIntrfTxReady(const UsbIsoIntrf_t *pIntrf)
{
	return pIntrf != NULL &&
		pIntrf->Opened && !pIntrf->Suspended &&
		atomic_load_explicit(&pIntrf->IntrfData.DevIntrf.bTxReady,
			memory_order_acquire);
}

#ifdef __cplusplus
}

class UsbIsoIntrf : public DeviceIntrf {
public:
	UsbIsoIntrf() = default;
	UsbIsoIntrf(const UsbIsoIntrf &) = delete;
	UsbIsoIntrf &operator = (const UsbIsoIntrf &) = delete;

	bool Init(const UsbIsoIntrfCfg_t &Cfg) {
		return UsbIsoIntrfInit(&vUsbIsoIntrf, &Cfg);
	}

	operator DevIntrf_t * () override {
		return &vUsbIsoIntrf.IntrfData.DevIntrf;
	}
	operator UsbIsoIntrf_t * () { return &vUsbIsoIntrf; }

	uint32_t Rate(uint32_t DataRate) override {
		return DeviceIntrfSetRate(&vUsbIsoIntrf.IntrfData.DevIntrf, DataRate);
	}

	uint32_t Rate(void) override {
		return DeviceIntrfGetRate(&vUsbIsoIntrf.IntrfData.DevIntrf);
	}

	bool RequestToSend(int NbBytes) override {
		return UsbIntrfRequestToSend(&vUsbIsoIntrf.IntrfData, NbBytes);
	}

	__attribute__((always_inline))
	int Tx(uint32_t DevAddr, const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTx(&vUsbIsoIntrf.IntrfData.DevIntrf, DevAddr, pData, DataLen);
	}

	__attribute__((always_inline))
	int Rx(uint32_t DevAddr, uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRx(&vUsbIsoIntrf.IntrfData.DevIntrf, DevAddr, pBuff, BuffLen);
	}

	__attribute__((always_inline))
	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vUsbIsoIntrf.IntrfData.DevIntrf, pData, DataLen);
	}

	__attribute__((always_inline))
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vUsbIsoIntrf.IntrfData.DevIntrf, pBuff, BuffLen);
	}

	bool Open(uint16_t Mps, uint8_t Interval) {
		return UsbIsoIntrfOpen(&vUsbIsoIntrf, Mps, Interval);
	}

	void Close(void) { UsbIsoIntrfClose(&vUsbIsoIntrf); }
	void Reset(void) override { UsbIsoIntrfReset(&vUsbIsoIntrf); }
	void Suspend(void) { UsbIsoIntrfSuspend(&vUsbIsoIntrf); }
	bool Resume(void) { return UsbIsoIntrfResume(&vUsbIsoIntrf); }

	bool SendFrame(const uint8_t *pData, uint16_t Length) {
		return UsbIsoIntrfSendFrame(&vUsbIsoIntrf, pData, Length);
	}

	bool TxReady(void) const { return UsbIsoIntrfTxReady(&vUsbIsoIntrf); }
	DevIntrf_t *Data(void) { return &vUsbIsoIntrf.IntrfData.DevIntrf; }

private:
	UsbIsoIntrf_t vUsbIsoIntrf = {};
};
#endif

/** @} End of group USBD */

#endif	// __USB_ISO_H__
