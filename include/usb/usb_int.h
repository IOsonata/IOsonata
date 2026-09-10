/**-------------------------------------------------------------------------
@file	usb_int.h

@brief	Reusable USB interrupt-transfer interface.

UsbIntIntrf is the interrupt-transfer specialization of UsbIntrf. It uses the
DIRECT data policy: one statically reserved RX slot and one TX slot, with no
CFifo. Endpoint transfer type and interval remain in this specialization.

@author	Hoang Nguyen Hoan
@date	Sep. 10, 2026

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
#ifndef __USB_INT_H__
#define __USB_INT_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "usb/usb_intrf.h"

/** @addtogroup USBD
  * @{
  */

#define USB_INT_INTRF_FS_MPS		64U
#define USB_INT_INTRF_MAX_MPS		((uint16_t)USB_PKT_MAXLEN(0, INT))
#define USB_INT_INTRF_PKT_BLKSIZE \
	USB_INTRF_PKT_BLKSIZE(USB_INT_INTRF_MAX_MPS)
#define USB_INT_INTRF_PACKET_WORDS \
	((USB_INT_INTRF_PKT_BLKSIZE + 3U) / 4U)

typedef struct __Usb_Int_Interf UsbIntIntrf_t;

typedef void (*UsbIntIntrfRxHandler_t)(UsbIntIntrf_t *pIntrf,
									 const uint8_t *pData, uint16_t Length,
									 UsbCtrlrXferResult_t Result,
									 void *pContext);
typedef void (*UsbIntIntrfTxHandler_t)(UsbIntIntrf_t *pIntrf,
									 uint16_t Length,
									 UsbCtrlrXferResult_t Result,
									 void *pContext);

#pragma pack(push, 4)

typedef struct __Usb_Int_Interf_Config {
	int DevNo;
	uint8_t EpNo;
	UsbIntIntrfRxHandler_t RxHandler;
	UsbIntIntrfTxHandler_t TxHandler;
	void *pContext;
} UsbIntIntrfCfg_t;

#pragma pack(pop)

struct __Usb_Int_Interf {
	UsbDevIntrf_t IntrfData;
	void *pContext;
	UsbIntIntrfRxHandler_t RxHandler;
	UsbIntIntrfTxHandler_t TxHandler;
	uint32_t RxErrorCnt;
	uint32_t TxErrorCnt;
	uint32_t RxEmptyCnt;
	uint32_t TxEmptyCnt;
	uint16_t Mps;
	uint8_t EpNo;
	uint8_t Interval;
	bool Opened;
	bool Suspended;
	uint32_t RxBuffer[USB_INT_INTRF_PACKET_WORDS];
	uint32_t TxBuffer[USB_INT_INTRF_PACKET_WORDS];
};

#ifdef __cplusplus
extern "C" {
#endif

bool UsbIntIntrfInit(UsbIntIntrf_t *pIntrf, const UsbIntIntrfCfg_t *pCfg);
bool UsbIntIntrfOpen(UsbIntIntrf_t *pIntrf, uint16_t Mps, uint8_t Interval);
void UsbIntIntrfClose(UsbIntIntrf_t *pIntrf);
void UsbIntIntrfReset(UsbIntIntrf_t *pIntrf);
void UsbIntIntrfSuspend(UsbIntIntrf_t *pIntrf);
bool UsbIntIntrfResume(UsbIntIntrf_t *pIntrf);
bool UsbIntIntrfSendPacket(UsbIntIntrf_t *pIntrf, const uint8_t *pData,
						   uint16_t Length);

static inline bool UsbIntIntrfTxReady(const UsbIntIntrf_t *pIntrf)
{
	return pIntrf != NULL && pIntrf->Opened && !pIntrf->Suspended &&
		atomic_load_explicit(&pIntrf->IntrfData.DevIntrf.bTxReady,
			memory_order_acquire);
}

#ifdef __cplusplus
}

class UsbIntIntrf : public DeviceIntrf {
public:
	UsbIntIntrf() = default;
	UsbIntIntrf(const UsbIntIntrf &) = delete;
	UsbIntIntrf &operator = (const UsbIntIntrf &) = delete;

	bool Init(const UsbIntIntrfCfg_t &Cfg) {
		return UsbIntIntrfInit(&vUsbIntIntrf, &Cfg);
	}

	operator DevIntrf_t * () override {
		return &vUsbIntIntrf.IntrfData.DevIntrf;
	}
	operator UsbIntIntrf_t * () { return &vUsbIntIntrf; }

	uint32_t Rate(uint32_t DataRate) override {
		return DeviceIntrfSetRate(&vUsbIntIntrf.IntrfData.DevIntrf, DataRate);
	}

	uint32_t Rate(void) override {
		return DeviceIntrfGetRate(&vUsbIntIntrf.IntrfData.DevIntrf);
	}

	bool RequestToSend(int NbBytes) override {
		return UsbIntrfRequestToSend(&vUsbIntIntrf.IntrfData, NbBytes);
	}

	__attribute__((always_inline))
	int Tx(uint32_t DevAddr, const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTx(&vUsbIntIntrf.IntrfData.DevIntrf,
			DevAddr, pData, DataLen);
	}

	__attribute__((always_inline))
	int Rx(uint32_t DevAddr, uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRx(&vUsbIntIntrf.IntrfData.DevIntrf,
			DevAddr, pBuff, BuffLen);
	}

	__attribute__((always_inline))
	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vUsbIntIntrf.IntrfData.DevIntrf,
			pData, DataLen);
	}

	__attribute__((always_inline))
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vUsbIntIntrf.IntrfData.DevIntrf,
			pBuff, BuffLen);
	}

	bool Open(uint16_t Mps, uint8_t Interval) {
		return UsbIntIntrfOpen(&vUsbIntIntrf, Mps, Interval);
	}

	void Close(void) { UsbIntIntrfClose(&vUsbIntIntrf); }
	void Reset(void) { UsbIntIntrfReset(&vUsbIntIntrf); }
	void Suspend(void) { UsbIntIntrfSuspend(&vUsbIntIntrf); }
	bool Resume(void) { return UsbIntIntrfResume(&vUsbIntIntrf); }

	bool SendPacket(const uint8_t *pData, uint16_t Length) {
		return UsbIntIntrfSendPacket(&vUsbIntIntrf, pData, Length);
	}

	bool TxReady(void) const { return UsbIntIntrfTxReady(&vUsbIntIntrf); }
	DevIntrf_t *Data(void) { return &vUsbIntIntrf.IntrfData.DevIntrf; }

private:
	UsbIntIntrf_t vUsbIntIntrf = {};
};
#endif

/** @} End of group USBD */

#endif	// __USB_INT_H__
