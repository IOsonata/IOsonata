/**-------------------------------------------------------------------------
@file	usbd_hci.h

@brief	USB Bluetooth HCI device function.

UsbdHci owns the standard Bluetooth USB function topology: one HCI interface
with interrupt Event IN and bulk ACL OUT/IN endpoints, followed by the
zero-bandwidth synchronous interface required by the Bluetooth USB transport.
Interface and endpoint numbers are allocated internally.

This first layer owns function registration, descriptors, endpoint lifecycle,
and control-request routing. Packet transport is layered on the allocated
endpoints without exposing USB topology to the application.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

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
#ifndef __USBD_HCI_H__
#define __USBD_HCI_H__

#include <stdbool.h>
#include <stdint.h>

#include "usb/usb.h"

/** @addtogroup USBD
  * @{
  */

#define USBD_HCI_CONFIG_VALUE			1U
#define USBD_HCI_SUBCLASS_RF			0x01U
#define USBD_HCI_PROTOCOL_BT			0x01U
#define USBD_HCI_EVENT_FS_MPS			16U
#define USBD_HCI_EVENT_HS_MPS			16U
#define USBD_HCI_ACL_FS_MPS			64U
#define USBD_HCI_ACL_HS_MPS			512U
#define USBD_HCI_EVENT_FS_INTERVAL		1U
#define USBD_HCI_EVENT_HS_INTERVAL		1U

#pragma pack(push, 1)

/// Descriptor fragment for the legacy Bluetooth USB HCI function.
typedef struct __Usbd_Hci_Descriptor {
	UsbInrtfAssDesc_t Association;
	UsbIntrfDesc_t Hci;
	UsbEndPointDesc_t EventIn;
	UsbEndPointDesc_t AclOut;
	UsbEndPointDesc_t AclIn;
	UsbIntrfDesc_t Sync;
} UsbdHciDesc_t;

#pragma pack(pop)

#pragma pack(push, 4)

typedef struct __Usbd_Hci_Config {
	int DevNo;
	uint8_t InterfaceString;
	uint16_t EventFsMps;			//!< Zero selects USBD_HCI_EVENT_FS_MPS
	uint16_t EventHsMps;			//!< Zero selects USBD_HCI_EVENT_HS_MPS
	uint16_t AclFsMps;			//!< Zero selects USBD_HCI_ACL_FS_MPS
	uint16_t AclHsMps;			//!< Zero selects USBD_HCI_ACL_HS_MPS
	uint8_t EventFsInterval;		//!< Zero selects USBD_HCI_EVENT_FS_INTERVAL
	uint8_t EventHsInterval;		//!< Zero selects USBD_HCI_EVENT_HS_INTERVAL
	UsbRequestHandler_t RequestHandler;	//!< HCI class/control request handler
	void *pRequestContext;
} UsbdHciCfg_t;

typedef struct __Usbd_Hci_Dev {
	UsbRequestHandler_t RequestHandler;
	void *pRequestContext;
	int HciItfNo;				//!< Internal allocation
	int SyncItfNo;				//!< Internal allocation
	int DevNo;
	uint8_t EventEpNo;			//!< Internal IN endpoint allocation
	uint8_t AclEpNo;			//!< Internal bidirectional endpoint allocation
	uint8_t InterfaceString;
	uint16_t EventFsMps;
	uint16_t EventHsMps;
	uint16_t AclFsMps;
	uint16_t AclHsMps;
	uint8_t EventFsInterval;
	uint8_t EventHsInterval;
} UsbdHciDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool UsbdHciInit(UsbdHciDev_t * const pHci, const UsbdHciCfg_t *pCfg);

/** Build the Bluetooth IAD, HCI interface/endpoints and sync alt-0 fragment. */
bool UsbdHciMakeDesc(UsbdHciDesc_t *pDesc, const UsbdHciDev_t *pHci,
					 UsbSpeed_t Speed);

#ifdef __cplusplus
}

class UsbdHci {
public:
	UsbdHci() = default;

	bool Init(const UsbdHciCfg_t &Cfg) {
		return UsbdHciInit(&vUsbdHci, &Cfg);
	}

	bool MakeDesc(UsbdHciDesc_t *pDesc, UsbSpeed_t Speed) const {
		return UsbdHciMakeDesc(pDesc, &vUsbdHci, Speed);
	}

private:
	UsbdHciDev_t vUsbdHci = {};
};
#endif

/** @} End of group USBD */

#endif	// __USBD_HCI_H__
