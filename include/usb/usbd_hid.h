/**-------------------------------------------------------------------------
@file	usbd_hid.h

@brief	Generic USB HID device class.

UsbdHid owns HID descriptors and class requests. Interrupt report transport is
provided by the inherited UsbIntIntrf. The application supplies the report
descriptor and optional callbacks for GET_REPORT and SET_REPORT payloads.

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
#define USBD_HID_CTRL_REPORT_SIZE	64U

typedef struct __Usbd_Hid_Dev UsbdHidDev_t;

/// Fill a snapshot of a report. Return its byte length, or -1 to reject it.
/// Type uses USB_HID_REPTYPE_* values; ReportId is zero for unnumbered reports.
/// Do not write beyond BufferSize. The core truncates replies to host wLength.
/// Called in USB processing context; do not block or retain pBuffer.
typedef int (*UsbdHidGetReport_t)(USB_HID_REPTYPE Type, uint8_t ReportId,
	uint8_t *pBuffer, uint16_t BufferSize, void *pContext);

/// Consume a complete SET_REPORT payload before the status stage. False stalls
/// the request. Called once, in USB processing context; do not block. The buffer
/// is valid only during the callback. Payload bytes include any report ID byte;
/// the library does not parse report layouts or insert/remove report IDs.
/// A later status-stage failure cannot undo application effects.
typedef bool (*UsbdHidSetReport_t)(USB_HID_REPTYPE Type, uint8_t ReportId,
	const uint8_t *pBuffer, uint16_t Length, void *pContext);

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
	DevIntrfEvtHandler_t EvtCB;	//!< Return RX length to consume; less retains the report
	void *pContext;
	// Separate caller-owned, 4-byte aligned slots, each at least
	// USB_INT_INTRF_PKT_BLKSIZE bytes; valid for the interface lifetime.
	uint8_t *pRxBuffer;
	uint8_t *pTxBuffer;
	UsbdHidGetReport_t GetReportCB;	//!< Optional; absent callback stalls GET_REPORT
	UsbdHidSetReport_t SetReportCB;	//!< Optional; absent callback stalls SET_REPORT
	// Optional separate, 4-byte aligned control storage for larger reports.
	// Null/zero selects the object's USBD_HID_CTRL_REPORT_SIZE byte buffer.
	// Must not overlap interrupt RX/TX slots; valid for the object lifetime.
	uint8_t *pControlBuffer;
	uint16_t ControlBufferSize;
} UsbdHidCfg_t;

#pragma pack(pop)

struct __Usbd_Hid_Dev {
	int DevNo;
	int ItfNo;					//!< Internal allocation
	UsbIntIntrf_t *pIntIntrf;
	uint16_t ReportDescLength;
	uint16_t BcdHid;
	uint16_t FsMps;
	uint16_t HsMps;
	// Keep the descriptor word-aligned; its trailing byte shares the
	// following byte-field group instead of adding end padding.
	UsbHidDesc_t HidDesc;
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
	const uint8_t *pReportDesc;
	void *pContext;
	UsbdHidGetReport_t GetReportCB;
	UsbdHidSetReport_t SetReportCB;
	uint8_t *pControlBuffer;
	uint16_t ControlBufferSize;
	uint16_t PendingReportValue;
	uint16_t PendingReportLength;
};

#ifdef __cplusplus
extern "C" {
#endif

// The application callback is registered with the INT transport directly.
// Recover HID state and the application pContext through this handle.
static inline UsbdHidDev_t *UsbdHidGetDevHandle(DevIntrf_t *pDev)
{
	return (UsbdHidDev_t *)UsbIntIntrfGetDevHandle(pDev)->pContext;
}

// Builds the HID configuration fragment at run time. Weak: an application
// building fully static descriptors may define a strong replacement, which the
// linker then substitutes and the default is removed. Pair with a strong
// UsbGetDescriptor so the assembled configuration matches the static fragment.
bool UsbdHidMakeDesc(UsbdHidDesc_t *pDesc, const UsbdHidDev_t *pHid,
					 UsbSpeed_t Speed);

#ifdef __cplusplus
}

class UsbdHid : public UsbDeviceClass, public UsbIntIntrf {
public:
	UsbdHid();
	UsbdHid(const UsbdHid &) = delete;
	UsbdHid &operator = (const UsbdHid &) = delete;

	bool Init(const UsbdHidCfg_t &Cfg);
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
				 uint8_t **ppData, uint16_t *pLength) override;
	bool SelectConfig(uint8_t ConfigValue) override;
	void Reset(void) override;

	using UsbIntIntrf::operator DevIntrf_t *;
	operator UsbdHidDev_t * () { return &vUsbdHid; }
	operator const UsbdHidDev_t * () const { return &vUsbdHid; }

private:
	UsbdHidDev_t vUsbdHid = {};
	alignas(4) uint8_t vControlBuffer[USBD_HID_CTRL_REPORT_SIZE] = {};
};
#endif

/** @} End of group USBD */

#endif

