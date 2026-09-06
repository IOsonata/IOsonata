/**-------------------------------------------------------------------------
@file	usb_ctrlr.h

@brief	Host test stand in for a port supplied usb_ctrlr.h.

Describes one full speed controller with eight endpoints in each direction.
The host tests link against a fake controller, so these numbers only have to
be a valid target shape.

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
#include <stddef.h>
#include <stdint.h>

#include "usb/usb_def.h"

#define USB_CTRLR_CNT					1

#define USB_PASTE3_(a, b, c)			a##b##c
#define USB_PASTE3(a, b, c)				USB_PASTE3_(a, b, c)
#define USB_PASTE2_(a, b)				a##b
#define USB_PASTE2(a, b)				USB_PASTE2_(a, b)

#define USB_PKT_MAXLEN_0_CONTROL		64
#define USB_PKT_MAXLEN_0_BULK			64
#define USB_PKT_MAXLEN_0_INT			64
#define USB_PKT_MAXLEN_0_ISO			1023

#define USB_PKT_MAXLEN(CtrlrNo, TransType) \
	USB_PASTE3(USB_PKT_MAXLEN_, CtrlrNo, USB_PASTE2(_, TransType))

#define USB_EPIN_CNT(CtrlrNo)			8
#define USB_EPOUT_CNT(CtrlrNo)			8
#define USB_HIGHSPEED_CAPABLE(CtrlrNo)	0
#define USB_ISO_SUPPORTED(CtrlrNo)		1


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

// Every entry point below is implemented by usb_ctrlr_<target>.cpp for this
// target. The generic USB layer calls only these; an application never does.
//

/**
 * @brief	Initialize controller software state.
 *
 * Must not touch controller registers, the peripheral may still be unpowered.
 */
bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg);

/**
 * @brief	Power, clock and PHY up, then prepare endpoint zero.
 *
 * One call. The split into a power stage and a register stage was an artifact
 * of the old usbd.h and usbd_ctrlr.h boundary.
 */
bool UsbCtrlrStart(int DevNo);

/** @brief Stop the controller and drop its power and clock. */
void UsbCtrlrStop(int DevNo);

/** @brief Bus power and housekeeping pass, called from UsbProcess. */
void UsbCtrlrProcess(int DevNo);

/** @brief True while bus power is present. */
bool UsbCtrlrVbusDetected(int DevNo);

/** @brief True when the active connection negotiated high speed. */
bool UsbCtrlrHighSpeed(int DevNo);

void UsbCtrlrIntEnable(int DevNo);
void UsbCtrlrIntDisable(int DevNo);
void UsbCtrlrConnect(int DevNo);
void UsbCtrlrDisconnect(int DevNo);

/** @brief Request remote wakeup when the controller permits it. */
void UsbCtrlrRemoteWakeup(int DevNo);

/** @brief Enable or disable SOF events. */
void UsbCtrlrSofEnable(int DevNo, bool Enable);

/**
 * @brief	Apply a device address when software owns address programming.
 *
 * Controllers that implement SET_ADDRESS in hardware leave this a no-op and
 * report USB_CTRLR_EVT_ADDRESS instead.
 */
void UsbCtrlrSetAddress(int DevNo, uint8_t Address);

/** @brief Open one non-control endpoint. Endpoint zero is done by Start. */
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

/** @brief Clear an endpoint stall and reset its data toggle to DATA0. */
void UsbCtrlrEpClearStall(int DevNo, uint8_t EpAddr);

/**
 * @brief	MCU unique id as a printable serial string.
 *
 * Not a USB property. It sits here because UsbInit needs it when pSerial is
 * NULL and only the port can read it.
 */
size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen);

#ifdef __cplusplus
}
#endif

#endif
