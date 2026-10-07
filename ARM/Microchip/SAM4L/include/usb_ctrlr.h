/**-------------------------------------------------------------------------
@file	usb_ctrlr.h

@brief	SAM4L USBC device controller contract.

Full-speed control, isochronous, bulk and interrupt transfers. Logical
endpoints 1-7 are redirected to seven physical endpoints shared between IN
and OUT. Opening more than seven non-control directions fails. Isochronous
support uses those same independent endpoint banks; host mode is not enabled
by this port.

@author	Hoang Nguyen Hoan
@date	Oct. 5, 2026

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
#include "coredev/iopincfg.h"
#include "usb/usb_def.h"

typedef enum __Usb_Ctrlr_Trans_Type {
	CONTROL = USB_ENDPATT_TRANS_CONTROL,
	ISO = USB_ENDPATT_TRANS_ISO,
	BULK = USB_ENDPATT_TRANS_BULK,
	INT = USB_ENDPATT_TRANS_INT,
} UsbCtrlrTransType_t;

// Counts include EP0; IN and OUT share seven physical data endpoints.
enum {
	USB_CTRLR_CNT = 1,
	USB_HIGHSPEED_CAPABLE_0 = 0,
	USB_EPIN_CNT_0 = 8,
	USB_EPOUT_CNT_0 = 8,
	USB_CTRLR0_CONTROL_PKT_LEN_MAX = 64,
	USB_CTRLR0_BULK_PKT_LEN_MAX = 64,
	USB_CTRLR0_INT_PKT_LEN_MAX = 64,
	USB_CTRLR0_ISO_PKT_LEN_MAX = 1023,
	USB_ISO_SUPPORTED_0 = 1,
	USB_ISO_EPIN_MASK_0 = 0x00FEU,
	USB_ISO_EPOUT_MASK_0 = 0x00FEU,
};

#ifndef USB_CONFIG_DESC_MAXLEN
#define USB_CONFIG_DESC_MAXLEN		768U
#endif

#define USB_EPIN_CNT(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_EPIN_CNT_0 : 0)
#define USB_EPOUT_CNT(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_EPOUT_CNT_0 : 0)
#define USB_HIGHSPEED_CAPABLE(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_HIGHSPEED_CAPABLE_0 : 0)
#define USB_ISO_SUPPORTED(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_ISO_SUPPORTED_0 : 0)
#define USB_ISO_EPIN_MASK(CtrlrNo) \
	((CtrlrNo) == 0 ? (uint32_t)USB_ISO_EPIN_MASK_0 : 0U)
#define USB_ISO_EPOUT_MASK(CtrlrNo) \
	((CtrlrNo) == 0 ? (uint32_t)USB_ISO_EPOUT_MASK_0 : 0U)
#define USB_CTRLR_PKT_LEN_MAX(CtrlrNo, TransType) \
	((CtrlrNo) != 0 ? 0 : \
	 (TransType) == CONTROL ? USB_CTRLR0_CONTROL_PKT_LEN_MAX : \
	 (TransType) == ISO ? USB_CTRLR0_ISO_PKT_LEN_MAX : \
	 (TransType) == BULK ? USB_CTRLR0_BULK_PKT_LEN_MAX : \
	 (TransType) == INT ? USB_CTRLR0_INT_PKT_LEN_MAX : 0)

#define USB_CTRLR_ISO_INIT(DevNo) UsbCtrlrIsoInit(DevNo)


typedef enum __Usb_Ctrlr_Xfer_Result {
	USB_CTRLR_XFER_SUCCESS,
	USB_CTRLR_XFER_FAILED,
	USB_CTRLR_XFER_CANCELLED,
} UsbCtrlrXferResult_t;

typedef enum __Usb_Ctrlr_Evt_Type {
	USB_CTRLR_EVT_RESET,		//!< USB bus reset
	USB_CTRLR_EVT_SETUP,		//!< New EP0 SETUP request
	USB_CTRLR_EVT_DRDY,			//!< Data is ready in the device to be retrieved
	USB_CTRLR_EVT_XFER_CMPL,	//!< Endpoint transfer completed successfully
	USB_CTRLR_EVT_CANCEL,		//!< Endpoint transfer cancelled
	USB_CTRLR_EVT_SUSPEND,		//!< Bus entered suspend
	USB_CTRLR_EVT_RESUME,		//!< Bus resumed
	USB_CTRLR_EVT_SOF,			//!< Start of frame
	USB_CTRLR_EVT_ADDRESS,		//!< Hardware accepted SET_ADDRESS itself
	USB_CTRLR_EVT_XFER_FAILED,	//!< Endpoint transfer failed
} UsbCtrlrEvtType_t;

#pragma pack(push, 4)

typedef struct __Usb_Ctrlr_Xfer_Evt {
	uint8_t EpAddr;
	uint16_t Length;
	UsbCtrlrXferResult_t Result;
	const uint8_t *pBuffer;		//!< EP0 OUT bytes, valid during the callback only
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
 * @brief	Non-control endpoint event callback.
 *
 * Registered with the endpoint DMA buffer. Called from interrupt or deferred
 * event processing. A NULL OUT buffer withholds reception; controller processing
 * delivers DRDY so the handler can retry pending work and restore the buffer.
 * XFER_CMPL reports success; failure and cancellation use their own events.
 */
typedef void (*UsbCtrlrEpHandler_t)(UsbCtrlrEvtType_t Event,
									uint16_t Length, void *pContext);

/// What the generic layer hands the port at UsbCtrlrInit.
typedef struct __Usb_Ctrlr_Config {
	int IntPrio;					//!< Interrupt priority of the USB peripheral
	const IOPinCfg_t *pIOPinMap;	//!< Optional board USB pins
	int NbIOPins;					//!< Number of entries in pIOPinMap
	bool bLowPowerSuspend;			//!< true - Sit in USB low power while suspended
} UsbCtrlrCfg_t;

#ifdef __cplusplus
extern "C" {
#endif

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg);
bool UsbCtrlrStart(int DevNo);
void UsbCtrlrStop(int DevNo);
void UsbCtrlrProcess(int DevNo);
// Return the current VBUS level. SAM4L device mode uses the optional board
// GPIO passed in UsbCtrlrCfg_t; the port queues UsbProcessQue on GPIO edges.
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
bool UsbCtrlrEpOpenData(int DevNo, uint8_t EpNo, bool bIn, uint8_t Type,
						 uint16_t MaxPacketSize);
void UsbCtrlrEpClose(int DevNo, uint8_t EpNo, bool bIn);
void UsbCtrlrEpCloseAll(int DevNo);
void UsbCtrlrEpBind(int DevNo, uint8_t EpNo, bool bIn, bool bBlocking,
					 UsbCtrlrEpHandler_t Handler, void *pContext);
bool UsbCtrlrEpReceive(int DevNo, uint8_t EpNo, uint8_t *pBuffer,
						  uint16_t Capacity);
void UsbCtrlrEpProcessEvent(int DevNo, uint8_t EpNo, bool bIn,
						 UsbCtrlrEvtType_t Event, uint16_t Value);
// The source stays owned until completion or cancellation. Zero length
// submits a ZLP. Only EP0 copies its source before returning.
bool UsbCtrlrEpSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length);
int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length);
bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr);
void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool bIn);
void UsbCtrlrEpClearStall(int DevNo, uint8_t EpNo, bool bIn);
size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen);
// UsbCtrlrIsoInit lives in the optional SAM4L ISO archive member. Referencing
// UsbIsoIntrf pulls that member in; non-ISO applications keep it out.
bool UsbCtrlrIsoInit(int DevNo);
bool UsbCtrlrIsoOpen(int DevNo, uint8_t EpNo, bool bIn, uint16_t MaxPacketSize);
bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length);
uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData);

#ifdef __cplusplus
}
#endif

#endif // __USB_CTRLR_H__
