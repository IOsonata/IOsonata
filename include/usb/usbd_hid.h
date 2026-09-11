/**-------------------------------------------------------------------------
@file	usbd_hid.h

@brief	Generic USB HID device class.

UsbdHid owns HID descriptors and class requests. Interrupt report transport is
provided by the embedded UsbIntIntrf. The application supplies the report
descriptor and handles GET_REPORT and SET_REPORT payloads.

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
#ifndef __USBD_HID_H__
#define __USBD_HID_H__

#include <stdbool.h>
#include <stdint.h>

#include "usb/usb.h"
#include "usb/usb_hiddef.h"
#include "usb/usb_int.h"

/** @addtogroup USBD
  * @{
  */

#define USBD_HID_CONFIG_VALUE		1U
#define USBD_HID_BCD_VERSION		0x0111U
#define USBD_HID_FS_MPS			64U
#define USBD_HID_HS_MPS			64U
#define USBD_HID_FS_INTERVAL		1U
#define USBD_HID_HS_INTERVAL		4U
#define USBD_HID_PROTOCOL_BOOT		0U
#define USBD_HID_PROTOCOL_REPORT	1U

typedef struct __Usbd_Hid_Dev UsbdHidDev_t;

typedef void (*UsbdHidRxHandler_t)(UsbdHidDev_t *pHid,
								 const uint8_t *pData, uint16_t Length,
								 UsbCtrlrXferResult_t Result,
								 void *pContext);
typedef void (*UsbdHidTxHandler_t)(UsbdHidDev_t *pHid, uint16_t Length,
								 UsbCtrlrXferResult_t Result,
								 void *pContext);

#pragma pack(push, 1)

/// Descriptor fragment for one HID interface and its Interrupt endpoint pair.
typedef struct __Usbd_Hid_Descriptor {
	UsbIntrfDesc_t Interface;
	UsbHidDesc_t Hid;
	UsbEndPointDesc_t Out;
	UsbEndPointDesc_t In;
} UsbdHidDesc_t;

#pragma pack(pop)

#pragma pack(push, 4)

typedef struct __Usbd_Hid_Config {
	int DevNo;
	const uint8_t *pReportDesc;
	uint16_t ReportDescLength;
	uint16_t BcdHid;				//!< Zero selects USBD_HID_BCD_VERSION
	uint16_t FsMps;				//!< Zero selects USBD_HID_FS_MPS
	uint16_t HsMps;				//!< Zero selects USBD_HID_HS_MPS
	uint8_t FsInterval;			//!< Zero selects USBD_HID_FS_INTERVAL
	uint8_t HsInterval;			//!< Zero selects USBD_HID_HS_INTERVAL
	uint8_t SubClass;
	uint8_t Protocol;
	uint8_t CountryCode;
	uint8_t InterfaceString;
	UsbdHidRxHandler_t RxHandler;
	UsbdHidTxHandler_t TxHandler;
	void *pContext;
} UsbdHidCfg_t;

#pragma pack(pop)

struct __Usbd_Hid_Dev {
	int DevNo;
	int ItfNo;					//!< Internal allocation
	UsbIntIntrf_t IntIntrf;
	const uint8_t *pReportDesc;
	UsbdHidRxHandler_t RxHandler;
	UsbdHidTxHandler_t TxHandler;
	void *pContext;
	UsbHidDesc_t HidDesc;
	UsbdHidDesc_t FsDesc;
	UsbdHidDesc_t HsDesc;
	uint16_t ReportDescLength;
	uint16_t BcdHid;
	uint16_t FsMps;
	uint16_t HsMps;
	uint8_t EpNo;				//!< Internal allocation
	uint8_t FsInterval;
	uint8_t HsInterval;
	uint8_t SubClass;
	uint8_t Protocol;
	uint8_t CountryCode;
	uint8_t InterfaceString;
	uint8_t Idle;
	uint8_t ActiveProtocol;
	uint8_t PendingIdle;
	uint8_t PendingProtocol;
	uint8_t PendingRequest;
	uint8_t CtrlReply;
	bool Configured;
};

#ifdef __cplusplus
extern "C" {
#endif

bool UsbdHidSendReport(UsbdHidDev_t *pHid, const uint8_t *pData,
					   uint16_t Length);
void UsbdHidSuspend(UsbdHidDev_t *pHid);
bool UsbdHidResume(UsbdHidDev_t *pHid);

static inline bool UsbdHidTxReady(const UsbdHidDev_t *pHid)
{
	return pHid != NULL && UsbIntIntrfTxReady(&pHid->IntIntrf);
}

#ifdef __cplusplus
}

class UsbdHid : public UsbDeviceClass, public DeviceIntrf {
public:
	UsbdHid() = default;
	UsbdHid(const UsbdHid &) = delete;
	UsbdHid &operator = (const UsbdHid &) = delete;

	bool Init(const UsbdHidCfg_t &Cfg);
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
				 uint8_t **ppData, uint16_t *pLength) override;
	bool SelectConfig(uint8_t ConfigValue) override;
	void Reset(void) override;

	operator DevIntrf_t * () override {
		return &vUsbdHid.IntIntrf.IntrfData.DevIntrf;
	}
	operator UsbdHidDev_t * () { return &vUsbdHid; }
	DevIntrf_t *Data(void) { return &vUsbdHid.IntIntrf.IntrfData.DevIntrf; }

	uint32_t Rate(uint32_t DataRate) override {
		return DeviceIntrfSetRate(Data(), DataRate);
	}
	uint32_t Rate(void) override { return DeviceIntrfGetRate(Data()); }

	bool RequestToSend(int NbBytes) override {
		return NbBytes >= 0 && NbBytes <= (int)vUsbdHid.IntIntrf.Mps &&
			UsbdHidTxReady(&vUsbdHid);
	}

	int Tx(uint32_t DevAddr, const uint8_t *pData, int DataLen) override {
		(void)DevAddr;
		return TxData(pData, DataLen);
	}
	int Rx(uint32_t DevAddr, uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRx(Data(), DevAddr, pBuff, BuffLen);
	}
	int TxData(const uint8_t *pData, int DataLen) override {
		return DataLen >= 0 && DataLen <= UINT16_MAX &&
			UsbdHidSendReport(&vUsbdHid, pData, (uint16_t)DataLen) ?
			DataLen : 0;
	}
	int RxData(uint8_t *pBuff, int BuffLen) override {
		return DeviceIntrfRxData(Data(), pBuff, BuffLen);
	}

	bool SendReport(const uint8_t *pData, uint16_t Length) {
		return UsbdHidSendReport(&vUsbdHid, pData, Length);
	}
	bool TxReady(void) const { return UsbdHidTxReady(&vUsbdHid); }
	void Suspend(void) { UsbdHidSuspend(&vUsbdHid); }
	bool Resume(void) { return UsbdHidResume(&vUsbdHid); }

private:
	UsbdHidDev_t vUsbdHid = {};
};
#endif

/** @} End of group USBD */

#endif	// __USBD_HID_H__
