/**-------------------------------------------------------------------------
@file	usbd_cdc.h

@brief	USB CDC ACM class adapter.

CDC owns ACM control requests, line/control state, notifications and the
controller transfer buffers required by its bulk endpoints. UsbdCdc derives
from the internal UsbIntrf data path and presents the DeviceIntrf API.

Interface and endpoint numbers are allocated internally when the function is
registered. Applications configure CDC behaviour and storage only.

@author	Hoang Nguyen Hoan
@date	May 2, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

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
#ifndef __USBD_CDC_H__
#define __USBD_CDC_H__

#include <stdbool.h>
#include <stdint.h>

#include "cfifo.h"
#include "device_intrf.h"
#include "usb/usb_cdcdef.h"
#include "usb/usb.h"
#include "usb/usb_intrf.h"

/** @addtogroup USBD
  * @{
  */

#define USBD_CDC_FUNC_MAXCNT			7

// Built-in CDC descriptor topology. Registration uses the same lowest-free
// ordering, so ordinary CDC-only devices keep the established descriptor
// layout without exposing these numbers in UsbdCdcCfg_t.
#define USBD_CDC_CTRL_IF(n)			((uint8_t)((n) * 2U))
#define USBD_CDC_DATA_IF(n)			((uint8_t)(USBD_CDC_CTRL_IF(n) + 1U))
#define USBD_CDC_NOTIF_EP(n)			USB_ENDPADDR_DIRIN(1U + ((n) * 2U))
#define USBD_CDC_DATA_EP_NO(n)			((uint8_t)(2U + ((n) * 2U)))
#define USBD_CDC_DATA_OUT_EP(n)			USB_ENDPADDR_DIROUT(USBD_CDC_DATA_EP_NO(n))
#define USBD_CDC_DATA_IN_EP(n)			USB_ENDPADDR_DIRIN(USBD_CDC_DATA_EP_NO(n))

#define USBD_CDC_CONFIG_VALUE			1U

#define USBD_CDC_NOTIF_MPS				8U
#define USBD_CDC_BULK_FS_MPS			64U
#define USBD_CDC_BULK_HS_MPS			512U

#define USBD_CDC_NOTIFY_LEN				(sizeof(UsbCdcNotification_t) + 2U)
#define USBD_CDC_TRANS_WORDS \
	((USB_PKT_MAXLEN(0, BULK) + sizeof(uint32_t) - 1U) / sizeof(uint32_t))
#define USBD_CDC_NOTIFY_WORDS \
	((USBD_CDC_NOTIFY_LEN + sizeof(uint32_t) - 1U) / sizeof(uint32_t))

#pragma pack(push, 4)

typedef struct __Usbd_Cdc_Config {
	int DevNo;						//!< USB controller number
	bool bBlocking;
	int RxFifoMemSize;
	uint8_t *pRxFifoMem;
	int TxFifoMemSize;
	uint8_t *pTxFifoMem;
	DevIntrfEvtHandler_t EvtCB;
} UsbdCdcCfg_t;

#pragma pack(pop)

// Natural alignment: IntrfData embeds DevIntrf_t whose pointer and atomic
// members must stay naturally aligned on 64-bit host test builds.
typedef struct __Usbd_Cdc_Dev {
	UsbDevIntrf_t IntrfData;		//!< Endpoint data path, owned by value
	UsbCdcLineCoding_t LineCoding;
	UsbCdcLineCoding_t PendingLineCoding;
	uint16_t ControlLineState;
	uint16_t PendingControlLineState;
	uint16_t SerialState;
	uint8_t CtrlIfNo;				//!< Internal allocation
	uint8_t NotifyEpNo;			//!< Internal allocation
	uint8_t DataEpNo;				//!< Internal allocation
	int DevNo;
	bool SerialStatePending;
	uint32_t RxTransfer[USBD_CDC_TRANS_WORDS];
	uint32_t TxTransfer[USBD_CDC_TRANS_WORDS];
	uint32_t NotifTransfer[USBD_CDC_NOTIFY_WORDS];
} UsbdCdcDev_t;

#ifdef __cplusplus

extern "C" {
#endif

bool UsbdCdcInit(UsbdCdcDev_t * const pCdc, const UsbdCdcCfg_t *pCfg);

void UsbdCdcProcess(UsbdCdcDev_t * const pCdc);

bool UsbdCdcPortIsOpen(const UsbdCdcDev_t * const pCdc);

const UsbCdcLineCoding_t *UsbdCdcLineCoding(const UsbdCdcDev_t * const pCdc);

uint16_t UsbdCdcControlLineState(const UsbdCdcDev_t * const pCdc);

void UsbdCdcSetSerialState(UsbdCdcDev_t * const pCdc, uint16_t SerialState);

const uint8_t *UsbdCdcDescHandler(uint8_t DescType, uint8_t DescIndex,
								  uint16_t LangId, UsbSpeed_t Speed,
								  uint16_t *pLength, void *pContext);

static inline int UsbdCdcRx(UsbdCdcDev_t * const pCdc, uint8_t *pBuff, int BuffLen) {
	return DeviceIntrfRx(&pCdc->IntrfData.DevIntrf, 0, pBuff, BuffLen);
}

static inline int UsbdCdcTx(UsbdCdcDev_t * const pCdc, const uint8_t *pData, int DataLen) {
	return DeviceIntrfTx(&pCdc->IntrfData.DevIntrf, 0, pData, DataLen);
}

static inline void UsbdCdcEnable(UsbdCdcDev_t * const pCdc) {
	DeviceIntrfEnable(&pCdc->IntrfData.DevIntrf);
}

static inline void UsbdCdcDisable(UsbdCdcDev_t * const pCdc) {
	DeviceIntrfDisable(&pCdc->IntrfData.DevIntrf);
}

static inline UsbdCdcDev_t *UsbdCdcGetDevHandle(DevIntrf_t * const pDevIntrf) {
	return (UsbdCdcDev_t *)((UsbDevIntrf_t *)pDevIntrf->pDevData)->pClassContext;
}

#ifdef __cplusplus
}

class UsbdCdc : public DeviceIntrf {
public:
	UsbdCdc() = default;
	UsbdCdc(const UsbdCdc &) = delete;
	UsbdCdc &operator = (const UsbdCdc &) = delete;

	operator DevIntrf_t * () override { return &vUsbdCdc.IntrfData.DevIntrf; }
	operator UsbdCdcDev_t * () { return &vUsbdCdc; }
	DevIntrf_t *Data(void) { return &vUsbdCdc.IntrfData.DevIntrf; }

	bool Init(const UsbdCdcCfg_t &Cfg) { return UsbdCdcInit(&vUsbdCdc, &Cfg); }

	uint32_t Rate(uint32_t DataRate) override {
		return DeviceIntrfSetRate(&vUsbdCdc.IntrfData.DevIntrf, DataRate);
	}

	uint32_t Rate(void) override {
		return DeviceIntrfGetRate(&vUsbdCdc.IntrfData.DevIntrf);
	}

	bool RequestToSend(int NbBytes) override {
		return UsbIntrfRequestToSend(&vUsbdCdc.IntrfData, NbBytes);
	}

	__attribute__((always_inline))
	int Tx(uint32_t DevAddr, const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTx(&vUsbdCdc.IntrfData.DevIntrf, DevAddr, pData, DataLen);
	}

	__attribute__((always_inline))
	int Rx(uint32_t DevAddr, uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRx(&vUsbdCdc.IntrfData.DevIntrf, DevAddr, pBuff, BuffLen);
	}

	__attribute__((always_inline))
	int TxData(const uint8_t *pData, int DataLen) override {
		return DeviceIntrfTxData(&vUsbdCdc.IntrfData.DevIntrf, pData, DataLen);
	}

	__attribute__((always_inline))
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(&vUsbdCdc.IntrfData.DevIntrf, pBuff, BuffLen);
	}

	bool IsPortOpen(void);
	const UsbCdcLineCoding_t *LineCoding(void);
	uint16_t ControlLineState(void);
	void SetSerialState(uint16_t SerialState);

private:
	UsbdCdcDev_t vUsbdCdc = {};
};

#endif

/** @} End of group USBD */

#endif	// __USBD_CDC_H__
