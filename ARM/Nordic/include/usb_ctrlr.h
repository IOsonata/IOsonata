/**-------------------------------------------------------------------------
@file	usb_ctrlr.h

@brief	USB controller description for Nordic parts.

Everything about the USB controllers that changes from one target to the next.
Every target that has a USB controller provides a usb_ctrlr.h of its own on its
include path, the same way it provides iopinctrl.h. Generic USB code includes
it by plain name and never switches on a vendor macro.

The UsbCtrlr entry points do not change across targets, so they are declared
once in usb.h rather than repeated here. This file is the part that differs.

The values here describe the silicon, not a class or a configuration. They are
constants so an application can size CFifo memory and DMA staging buffers
statically, before any endpoint is configured and without calling into the
stack.

Controller numbering starts at zero and matches the CtrlrNo argument of the
USB API. Because the accessor macros paste their arguments, CtrlrNo and
TransType must be literal tokens, not variables.

Packet lengths are allocation bounds: the largest packet the controller can
move on that transfer type at the fastest speed it supports. The value an
endpoint actually negotiates comes from its descriptor and may be smaller. A
buffer sized from these macros is large enough at every speed.

TransType tokens match the transfer types in usb_def.h: CONTROL, ISO, BULK,
INT.

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

// Each accessor pastes through a second macro so a named constant expands
// before it is pasted. Without that, USB_PKT_MAXLEN(USB_DEVNO, BULK) would
// build the token USB_PKT_MAXLEN_USB_DEVNO_BULK and fail to resolve.
#define USB_PASTE3_(a, b, c)				a##b##c
#define USB_PASTE3(a, b, c)					USB_PASTE3_(a, b, c)
#define USB_PASTE2_(a, b)					a##b
#define USB_PASTE2(a, b)					USB_PASTE2_(a, b)

/// Maximum packet length in bytes for one controller and transfer type.
#define USB_PKT_MAXLEN(CtrlrNo, TransType) \
	USB_PASTE3(USB_PKT_MAXLEN_, CtrlrNo, USB_PASTE2(_, TransType))

/// IN endpoint numbers, including endpoint zero.
#define USB_EPIN_CNT(CtrlrNo)				USB_PASTE2(USB_EPIN_CNT_, CtrlrNo)

/// OUT endpoint numbers, including endpoint zero.
#define USB_EPOUT_CNT(CtrlrNo)				USB_PASTE2(USB_EPOUT_CNT_, CtrlrNo)

/// Nonzero when the controller can enumerate at high speed.
#define USB_HIGHSPEED_CAPABLE(CtrlrNo) \
	USB_PASTE2(USB_HIGHSPEED_CAPABLE_, CtrlrNo)

/// Nonzero when this port drives the controller's isochronous endpoints.
#define USB_ISO_SUPPORTED(CtrlrNo)			USB_PASTE2(USB_ISO_SUPPORTED_, CtrlrNo)

#if defined(USBD_PRESENT)

/// Number of USB controllers on this part.
#define USB_CTRLR_CNT						1

// nRF52840 and nRF5340 USBD. Full speed only. Endpoint numbers 0 through 7 in
// each direction. Endpoint 8 is isochronous only and is not counted here
// because usbd_ctrlr_nrf52 does not drive it.
#define USB_HIGHSPEED_CAPABLE_0				0
#define USB_EPIN_CNT_0						8
#define USB_EPOUT_CNT_0						8

#define USB_PKT_MAXLEN_0_CONTROL			64
#define USB_PKT_MAXLEN_0_BULK				64
#define USB_PKT_MAXLEN_0_INT				64

// The hardware isochronous endpoint carries a full-speed maximum of 1023
// bytes. usbd_ctrlr_nrf52 leaves isochronous out, so allocating against this
// value buys nothing until that changes. USB_ISO_SUPPORTED says which is true.
#define USB_PKT_MAXLEN_0_ISO				1023
#define USB_ISO_SUPPORTED_0					0

#elif defined(USBHS_PRESENT)

/// Number of USB controllers on this part.
#define USB_CTRLR_CNT						1

// nRF54 USBHS. High speed capable, so the bulk and interrupt bounds are the
// high-speed maxima. A full-speed host negotiates smaller packets at
// enumeration and a buffer sized from these still fits.
#define USB_HIGHSPEED_CAPABLE_0				1
#define USB_EPIN_CNT_0						16
#define USB_EPOUT_CNT_0						16

#define USB_PKT_MAXLEN_0_CONTROL			64
#define USB_PKT_MAXLEN_0_BULK				512
#define USB_PKT_MAXLEN_0_INT				1024

#define USB_PKT_MAXLEN_0_ISO				1024
#define USB_ISO_SUPPORTED_0					0

#else
#error "usb_ctrlr: this part has no USB controller"
#endif


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

/**
 * @brief	Start one endpoint transfer.
 *
 * TotalBytes is the logical transfer length, not a DMA or FIFO limit. The
 * backend splits it as the hardware requires. pBuffer is controller transfer
 * storage owned by the caller and must stay valid until completion is
 * reported. It is never CFifo storage, which can be released sooner. A zero
 * length transfer is valid. One transfer per endpoint direction at a time.
 */
bool UsbCtrlrEpXfer(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
					uint16_t TotalBytes);

/** @brief True while the endpoint direction has a transfer in flight. */
bool UsbCtrlrEpBusy(int DevNo, uint8_t EpAddr);

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

/** @} End of group USB */

#endif	// __USB_CTRLR_H__
