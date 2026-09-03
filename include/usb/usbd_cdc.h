/**-------------------------------------------------------------------------
@file	usbd_cdc.h

@brief	USB CDC ACM class adapter.

CDC owns ACM control requests, line/control state and notifications.
Application data moves through the embedded generic UsbdIntrf data interface;
CDC itself is not a DeviceIntrf and does not own a separate RX data path.

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
#include "usb/usbd.h"
#include "usb/usb_intrf.h"
#include "usb/usbd_ctrlr.h"

/** @addtogroup USBD
  * @{
  */

#define USBD_CDC_FUNC_MAXCNT			7
#define USBD_CDC_CTRL_INTRF(n)			((uint8_t)((n) * 2U))
#define USBD_CDC_DATA_INTRF(n)			((uint8_t)(USBD_CDC_CTRL_INTRF(n) + 1U))
#define USBD_CDC_NOTIF_EP(n)			USB_ENDPADDR_DIRIN(1U + ((n) * 2U))
#define USBD_CDC_DATA_OUT_EP(n)			USB_ENDPADDR_DIROUT(2U + ((n) * 2U))
#define USBD_CDC_DATA_IN_EP(n)			USB_ENDPADDR_DIRIN(2U + ((n) * 2U))

#define USBD_CDC_CONFIG_VALUE			1U

#define USBD_CDC_NOTIF_MPS				8U
#define USBD_CDC_BULK_FS_MPS			64U
#define USBD_CDC_BULK_HS_MPS			512U

#define USBD_CDC_NOTIFY_LEN				(sizeof(UsbCdcNotification_t) + 2U)
#define USBD_CDC_TRANS_WORDS			(USBD_CDC_BULK_HS_MPS / sizeof(uint32_t))
#define USBD_CDC_NOTIFY_WORDS \
	((USBD_CDC_NOTIFY_LEN + sizeof(uint32_t) - 1U) / sizeof(uint32_t))

#pragma pack(push, 4)

typedef struct __Usbd_Cdc_Config {
	bool bBlocking;
	int RxFifoMemSize;
	uint8_t *pRxFifoMem;
	int TxFifoMemSize;
	uint8_t *pTxFifoMem;
	int ItfNo;
	DevIntrfEvtHandler_t EvtCB;
} UsbdCdcCfg_t;

typedef struct __Usbd_Cdc_Dev {
	UsbDevIntrf_t Data;
	UsbCdcLineCoding_t LineCoding;
	UsbCdcLineCoding_t PendingLineCoding;
	uint16_t ControlLineState;
	uint16_t PendingControlLineState;
	uint16_t SerialState;
	uint16_t DataMps;
	uint32_t RxDropCnt;
	uint32_t TxDropCnt;
	int ItfNo;
	bool Configured;
	bool NotifActive;
	bool SerialStatePending;
	bool ReportedOpen;
	uint32_t TxTransfer[USBD_CDC_TRANS_WORDS];
	uint32_t NotifTransfer[USBD_CDC_NOTIFY_WORDS];
} UsbdCdcDev_t;

#pragma pack(pop)

#ifdef __cplusplus

class UsbdCdc {
public:
	bool Init(const UsbdCdcCfg_t &Cfg);

	DevIntrf_t *Data(void) { return &vUsbdCdc.Data.DevIntrf; }

	bool IsPortOpen(void);
	const UsbCdcLineCoding_t *LineCoding(void);
	uint16_t ControlLineState(void);
	void SetSerialState(uint16_t SerialState);

private:
	UsbdCdcDev_t vUsbdCdc = {};
};

extern "C" {
#endif

bool UsbdCdcInit(UsbdCdcDev_t * const pCdc, const UsbdCdcCfg_t *pCfg);

void UsbdCdcProcess(UsbdCdcDev_t * const pCdc);

bool UsbdCdcPortIsOpen(const UsbdCdcDev_t * const pCdc);

const UsbCdcLineCoding_t *UsbdCdcLineCoding(const UsbdCdcDev_t * const pCdc);

uint16_t UsbdCdcControlLineState(const UsbdCdcDev_t * const pCdc);

void UsbdCdcSetSerialState(UsbdCdcDev_t * const pCdc, uint16_t SerialState);

const uint8_t *UsbdCdcDescHandler(uint8_t DescType,
								  uint8_t DescIndex,
								  uint16_t LangId,
								  UsbdSpeed_t Speed,
								  uint16_t *pLength,
								  void *pContext);

#ifdef __cplusplus
}
#endif

/** @} End of group USBD */

#endif	// __USBD_CDC_H__
