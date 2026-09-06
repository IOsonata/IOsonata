/**-------------------------------------------------------------------------
@file	usb_ctrlr.h

@brief	USB controller description for Nordic parts.

Everything about the USB controllers that changes from one target to the next.
Every target that has a USB controller provides a usb_ctrlr.h of its own on its
include path, the same way it provides iopinctrl.h. Generic USB code includes
it by plain name and never switches on a vendor macro.

This file also declares the UsbCtrlr entry points usb_ctrlr_<target>.cpp
implements, so the port describes both what it is and what it provides.

The values here describe the silicon, not a class or a configuration. They are
compile-time constants so an application can size CFifo memory and DMA staging
buffers statically, before any endpoint is configured and without calling into
the stack.

Runtime APIs carry DevNo. The current supported ports expose one controller,
so the capability accessors select controller zero with normal constant
expressions. They do not construct identifiers with token-pasting macros.

Packet lengths are allocation bounds: the largest packet the controller can
move on that transfer type at the fastest speed it supports. The value an
endpoint actually negotiates comes from its descriptor and may be smaller.

@author	Hoang Nguyen Hoan
@date	Sep. 3, 2026

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
#ifndef __USB_CTRLR_H__
#define __USB_CTRLR_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_peripherals.h"
#include "usb/usb_def.h"

/** @addtogroup USB
  * @{
  */

typedef enum __Usb_Ctrlr_Trans_Type {
	CONTROL = USB_ENDPATT_TRANS_CONTROL,
	ISO = USB_ENDPATT_TRANS_ISO,
	BULK = USB_ENDPATT_TRANS_BULK,
	INT = USB_ENDPATT_TRANS_INT,
} UsbCtrlrTransType_t;

#if defined(USBD_PRESENT)

// nRF52840 and nRF5340 USBD. Full speed only. Endpoint numbers 0 through 7 in
// each direction. Endpoint 8 is isochronous only and is not counted because
// the current port does not drive it.
enum {
	USB_CTRLR_CNT = 1,
	USB_HIGHSPEED_CAPABLE_0 = 0,
	USB_EPIN_CNT_0 = 8,
	USB_EPOUT_CNT_0 = 8,
	USB_PKT_MAXLEN_0_CONTROL = 64,
	USB_PKT_MAXLEN_0_BULK = 64,
	USB_PKT_MAXLEN_0_INT = 64,
	USB_PKT_MAXLEN_0_ISO = 1023,
	USB_ISO_SUPPORTED_0 = 0,
};

#elif defined(USBHS_PRESENT)

// nRF54 USBHS. High speed capable, so the bulk and interrupt bounds are the
// high-speed maxima. A full-speed host negotiates smaller packets and buffers
// sized from these constants still fit.
enum {
	USB_CTRLR_CNT = 1,
	USB_HIGHSPEED_CAPABLE_0 = 1,
	USB_EPIN_CNT_0 = 16,
	USB_EPOUT_CNT_0 = 16,
	USB_PKT_MAXLEN_0_CONTROL = 64,
	USB_PKT_MAXLEN_0_BULK = 512,
	USB_PKT_MAXLEN_0_INT = 1024,
	USB_PKT_MAXLEN_0_ISO = 1024,
	USB_ISO_SUPPORTED_0 = 0,
};

#else
#error "usb_ctrlr: this part has no USB controller"
#endif

#define USB_EPIN_CNT(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_EPIN_CNT_0 : 0)
#define USB_EPOUT_CNT(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_EPOUT_CNT_0 : 0)
#define USB_HIGHSPEED_CAPABLE(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_HIGHSPEED_CAPABLE_0 : 0)
#define USB_ISO_SUPPORTED(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_ISO_SUPPORTED_0 : 0)

#define USB_PKT_MAXLEN(CtrlrNo, TransType) \
	((CtrlrNo) != 0 ? 0 : \
	 (TransType) == CONTROL ? USB_PKT_MAXLEN_0_CONTROL : \
	 (TransType) == ISO ? USB_PKT_MAXLEN_0_ISO : \
	 (TransType) == BULK ? USB_PKT_MAXLEN_0_BULK : \
	 (TransType) == INT ? USB_PKT_MAXLEN_0_INT : 0)


//////////////////////////////////////////////////////////////////////////////
// Public. What the generic USB layer sees and what usb_ctrlr_<target>.cpp
// must implement. Helpers private to the port stay in the port source, or in
// a private section below when several port files share them.
//////////////////////////////////////////////////////////////////////////////

//
// Controller layer. Events come from the USB interrupt.
//

typedef enum __Usb_Ctrlr_Xfer_Result {
	USB_CTRLR_XFER_SUCCESS,
	USB_CTRLR_XFER_FAILED,
	USB_CTRLR_XFER_CANCELLED,
} UsbCtrlrXferResult_t;

typedef enum __Usb_Ctrlr_Evt_Type {
	USB_CTRLR_EVT_RESET,		//!< USB bus reset
	USB_CTRLR_EVT_SETUP,		//!< New EP0 SETUP request
	USB_CTRLR_EVT_XFER_CMPL,	//!< Endpoint transfer completed
	USB_CTRLR_EVT_SUSPEND,		//!< Bus entered suspend
	USB_CTRLR_EVT_RESUME,		//!< Bus resumed
	USB_CTRLR_EVT_SOF,			//!< Start of frame
	USB_CTRLR_EVT_ADDRESS,		//!< Hardware accepted SET_ADDRESS itself
} UsbCtrlrEvtType_t;

#pragma pack(push, 4)

typedef struct __Usb_Ctrlr_Xfer_Evt {
	uint8_t EpAddr;
	uint16_t Length;
	UsbCtrlrXferResult_t Result;
} UsbCtrlrXferEvt_t;

typedef struct __Usb_Ctrlr_Evt {
	UsbCtrlrEvtType_t Type;
	union {
		UsbSetupData_t Setup;
		UsbCtrlrXferEvt_t Xfer;
		uint16_t FrameNo;
		uint8_t Address;
	};
} UsbCtrlrEvt_t;

#pragma pack(pop)

/**
 * @brief	Controller event callback, called from the USB interrupt.
 *
 * Must stay bounded and must not retain pEvt after it returns.
 */
typedef void (*UsbCtrlrEvtHandler_t)(int DevNo, const UsbCtrlrEvt_t *pEvt,
									 void *pContext);

/**
 * @brief	Non-control endpoint completion callback.
 *
 * Registered once with the endpoint DMA buffer. It is called directly from
 * the controller interrupt, avoiding a function-table search per packet.
 */
typedef void (*UsbCtrlrEpHandler_t)(uint8_t EpAddr, uint16_t Length,
									UsbCtrlrXferResult_t Result, void *pContext);

/// What the generic layer hands the port at UsbCtrlrInit. Interrupt priority
/// and suspend behaviour reach the hardware only through here, so the port
/// needs them alongside the event callback.
typedef struct __Usb_Ctrlr_Config {
	int IntPrio;					//!< Interrupt priority of the USB peripheral
	bool bLowPowerSuspend;			//!< true - Sit in USB low power while suspended
	UsbCtrlrEvtHandler_t EvtHandler;
	void *pContext;
} UsbCtrlrCfg_t;

#ifdef __cplusplus
extern "C" {
#endif

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg);
bool UsbCtrlrStart(int DevNo);
void UsbCtrlrStop(int DevNo);
void UsbCtrlrProcess(int DevNo);
bool UsbCtrlrVbusDetected(int DevNo);
bool UsbCtrlrHighSpeed(int DevNo);
void UsbCtrlrIntEnable(int DevNo);
void UsbCtrlrIntDisable(int DevNo);
void UsbCtrlrConnect(int DevNo);
void UsbCtrlrDisconnect(int DevNo);
void UsbCtrlrRemoteWakeup(int DevNo);
void UsbCtrlrSofEnable(int DevNo, bool Enable);
void UsbCtrlrSetAddress(int DevNo, uint8_t Address);
bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc);
void UsbCtrlrEpClose(int DevNo, uint8_t EpAddr);
void UsbCtrlrEpCloseAll(int DevNo);
bool UsbCtrlrEpRegister(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
						UsbCtrlrEpHandler_t Handler, void *pContext);
bool UsbCtrlrEpRxArm(int DevNo, uint8_t EpNo);
bool UsbCtrlrEpSend(int DevNo, uint8_t EpNo, uint16_t Length);
bool UsbCtrlrEp0Xfer(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
						 uint16_t Length);
void UsbCtrlrEpStall(int DevNo, uint8_t EpAddr);
void UsbCtrlrEpClearStall(int DevNo, uint8_t EpAddr);
size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen);

#ifdef __cplusplus
}
#endif

/** @} End of group USB */

#endif	// __USB_CTRLR_H__
