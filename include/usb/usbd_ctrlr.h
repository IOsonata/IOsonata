/**-------------------------------------------------------------------------
@file	usbd_ctrlr.h

@brief	USB device controller interface.

This is the hardware-facing boundary of the IOsonata USB device stack. The
USB device core owns Chapter 9, descriptors and class dispatch. Controller
implementations own endpoint registers, DMA and the USB interrupt.

No controller-specific type is exposed here. A Nordic USBD peripheral, a DWC2
core or another USB device controller implements the same interface.

@author	Hoang Nguyen Hoan
@date	Aug. 29, 2026

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
#ifndef __USBD_CTRLR_H__
#define __USBD_CTRLR_H__

#include <stdbool.h>
#include <stdint.h>

#include "usb/usb_def.h"

/** @addtogroup USBD
  * @{
  */

// Compatibility aliases for the initial native device-controller code.
// Endpoint address encoding is defined once in usb_def.h and is common to
// both USB host and device implementations.
#define USBD_EP_NUM(addr)		USB_ENDPADDR_NUM(addr)
#define USBD_EP_IS_IN(addr)		USB_ENDPADDR_IS_IN(addr)
#define USBD_EP_DIR_OUT			USB_ENDPADDR_DIR_OUT
#define USBD_EP_DIR_IN			USB_ENDPADDR_DIR_IN

typedef enum __Usbd_Ctrlr_Xfer_Result {
	USBD_CTRLR_XFER_SUCCESS,
	USBD_CTRLR_XFER_FAILED,
	USBD_CTRLR_XFER_CANCELLED,
} UsbdCtrlrXferResult_t;

typedef enum __Usbd_Ctrlr_Evt_Type {
	USBD_CTRLR_EVT_RESET,			//!< USB bus reset
	USBD_CTRLR_EVT_SETUP,			//!< New EP0 SETUP request
	USBD_CTRLR_EVT_XFER_CMPL,		//!< Endpoint transfer completed
	USBD_CTRLR_EVT_SUSPEND,			//!< Bus entered suspend
	USBD_CTRLR_EVT_RESUME,			//!< Bus resumed
	USBD_CTRLR_EVT_SOF,				//!< Start of frame
	USBD_CTRLR_EVT_ADDRESS,			//!< Hardware accepted SET_ADDRESS itself
} UsbdCtrlrEvtType_t;

#pragma pack(push, 4)

typedef struct __Usbd_Ctrlr_Xfer_Evt {
	uint8_t EpAddr;
	uint16_t Length;
	UsbdCtrlrXferResult_t Result;
} UsbdCtrlrXferEvt_t;

typedef struct __Usbd_Ctrlr_Evt {
	UsbdCtrlrEvtType_t Type;
	union {
		UsbSetupData_t Setup;
		UsbdCtrlrXferEvt_t Xfer;
		uint16_t FrameNo;
		uint8_t Address;
	};
} UsbdCtrlrEvt_t;

#pragma pack(pop)

/**
 * @brief	Controller event callback.
 *
 * Called from the USB interrupt. The device core must keep the callback
 * bounded and must not retain pEvt after it returns.
 */
typedef void (*UsbdCtrlrEvtHandler_t)(const UsbdCtrlrEvt_t *pEvt,
									 void *pContext);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize controller software state.
 *
 * The MCU USB power/clock wrapper is initialized separately through UsbdInit().
 * This call only initializes endpoint and interrupt-facing controller state.
 */
bool UsbdCtrlrInit(UsbdCtrlrEvtHandler_t EvtHandler, void *pContext);

/** @brief Enable the USB controller interrupt. */
void UsbdCtrlrIntEnable(void);

/** @brief Disable the USB controller interrupt. */
void UsbdCtrlrIntDisable(void);

/** @brief Connect the device to the bus by enabling its pull-up. */
void UsbdCtrlrConnect(void);

/** @brief Disconnect the device from the bus. */
void UsbdCtrlrDisconnect(void);

/** @brief Request remote wakeup when the controller permits it. */
void UsbdCtrlrRemoteWakeup(void);

/** @brief Enable or disable SOF events. */
void UsbdCtrlrSofEnable(bool Enable);

/**
 * @brief	Apply a USB device address when software owns address programming.
 *
 * Controllers that implement SET_ADDRESS in hardware may leave this as a
 * no-op and report USBD_CTRLR_EVT_ADDRESS instead.
 */
void UsbdCtrlrSetAddress(uint8_t Address);

/**
 * @brief	Open one non-control endpoint.
 *
 * The endpoint descriptor supplies address, transfer type and maximum packet
 * size. Endpoint zero is prepared by UsbdCtrlrInit().
 */
bool UsbdCtrlrEpOpen(const UsbEndPointDesc_t *pDesc);

/** @brief Close one non-control endpoint. */
void UsbdCtrlrEpClose(uint8_t EpAddr);

/** @brief Close all non-control endpoints. */
void UsbdCtrlrEpCloseAll(void);

/**
 * @brief	Start one endpoint transfer.
 *
 * The transfer buffer must remain valid until USBD_CTRLR_EVT_XFER_CMPL. A
 * zero length transfer is valid. Only one transfer may be outstanding on an
 * endpoint direction at a time.
 */
bool UsbdCtrlrEpXfer(uint8_t EpAddr, uint8_t *pBuffer, uint16_t TotalBytes);

/** @brief Return true while the endpoint direction has an active transfer. */
bool UsbdCtrlrEpBusy(uint8_t EpAddr);

/** @brief Stall an endpoint. */
void UsbdCtrlrEpStall(uint8_t EpAddr);

/** @brief Clear endpoint stall and reset its data toggle to DATA0. */
void UsbdCtrlrEpClearStall(uint8_t EpAddr);

#ifdef __cplusplus
}
#endif

/** @} End of group USBD */

#endif	// __USBD_CTRLR_H__
