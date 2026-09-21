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

#if defined(USBD_PRESENT)
#include "cfifo.h"
#endif

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

// nRF52840 and nRF5340 USBD. Full speed only. Endpoint numbers 0 through 7
// are control, bulk or interrupt. Endpoint 8 is the dedicated isochronous
// endpoint in both directions.
enum {
	USB_CTRLR_CNT = 1,
	USB_HIGHSPEED_CAPABLE_0 = 0,
	USB_EPIN_CNT_0 = 8,
	USB_EPOUT_CNT_0 = 8,
	USB_CTRLR0_CONTROL_PKT_LEN_MAX = 64,
	USB_CTRLR0_BULK_PKT_LEN_MAX = 64,
	USB_CTRLR0_INT_PKT_LEN_MAX = 64,
	USB_CTRLR0_ISO_PKT_LEN_MAX = 512,
	USB_ISO_SUPPORTED_0 = 1,
	USB_ISO_EPIN_MASK_0 = (1U << 8),
	USB_ISO_EPOUT_MASK_0 = (1U << 8),
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
	USB_CTRLR0_CONTROL_PKT_LEN_MAX = 64,
	USB_CTRLR0_BULK_PKT_LEN_MAX = 512,
	USB_CTRLR0_INT_PKT_LEN_MAX = 1024,
	USB_CTRLR0_ISO_PKT_LEN_MAX = 1024,
	USB_ISO_SUPPORTED_0 = 0,
	USB_ISO_EPIN_MASK_0 = 0,
	USB_ISO_EPOUT_MASK_0 = 0,
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
#define USB_ISO_EPIN_MASK(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_ISO_EPIN_MASK_0 : 0U)
#define USB_ISO_EPOUT_MASK(CtrlrNo) \
	((CtrlrNo) == 0 ? USB_ISO_EPOUT_MASK_0 : 0U)
#define USB_CTRLR_PKT_LEN_MAX(CtrlrNo, TransType) \
	((CtrlrNo) != 0 ? 0 : \
	 (TransType) == CONTROL ? USB_CTRLR0_CONTROL_PKT_LEN_MAX : \
	 (TransType) == ISO ? USB_CTRLR0_ISO_PKT_LEN_MAX : \
	 (TransType) == BULK ? USB_CTRLR0_BULK_PKT_LEN_MAX : \
	 (TransType) == INT ? USB_CTRLR0_INT_PKT_LEN_MAX : 0)

#if defined(USBD_PRESENT)
#define USB_CTRLR_ISO_INIT(DevNo) UsbCtrlrIsoInit(DevNo)
#else
#define USB_CTRLR_ISO_INIT(DevNo) false
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
typedef void (*UsbCtrlrEpHandler_t)(uint8_t EpAddr, UsbCtrlrEvtType_t Event,
									uint16_t Length, void *pContext);

/// What the generic layer hands the port at UsbCtrlrInit.
typedef struct __Usb_Ctrlr_Config {
	int IntPrio;					//!< Interrupt priority of the USB peripheral
	bool bLowPowerSuspend;			//!< true - Sit in USB low power while suspended
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
#if defined(USBD_PRESENT)
bool UsbCtrlrIsoInit(int DevNo);
#endif
void UsbCtrlrIntEnable(int DevNo);
void UsbCtrlrIntDisable(int DevNo);
void UsbCtrlrConnect(int DevNo);
void UsbCtrlrDisconnect(int DevNo);
void UsbCtrlrRemoteWakeup(int DevNo);
void UsbCtrlrSofEnable(int DevNo, bool Enable);
void UsbCtrlrSetAddress(int DevNo, uint8_t Address);
bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc);
bool UsbCtrlrEpOpenData(int DevNo, uint8_t EpAddr, uint8_t Type, uint16_t MaxPacketSize);
void UsbCtrlrEpClose(int DevNo, uint8_t EpAddr);
void UsbCtrlrEpCloseAll(int DevNo);
void UsbCtrlrEpAlloc(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
					 bool bBlocking,
					 UsbCtrlrEpHandler_t Handler, void *pContext);
// EpNum is an endpoint number: device IN, host OUT. The controller schedules RX.
// pBuffer supplies the DMA source. On nRF52, NULL selects the registered
// TX byte CFifo. The source remains owned until the completion callback.
bool UsbCtrlrEpSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length);
// IN returns bytes copied into the queue; completion notifies that it drained.
// A zero-length send queues a data ZLP; negative means it was not accepted.
int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length);
bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr);
void UsbCtrlrEpStall(int DevNo, uint8_t EpAddr);
void UsbCtrlrEpClearStall(int DevNo, uint8_t EpAddr);
size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen);

#ifdef __cplusplus
}
#endif

#if defined(USBD_PRESENT)

// Shared nRF52 USBD implementation state used by the base and optional ISO
// archive members. This is a target-family header; nRF54 USBHS does not see it.
enum
{
	NRF_USB_EP_COUNT = 9,
	NRFX_USBD_DATA_EP_COUNT = 8,
	NRFX_USBD_EP_COUNT = 9,
	NRFX_USBD_ISO_EP_NO = 8,
	NRFX_USBD_MAX_PACKET_SIZE = 64,
	NRFX_USBD_ISO_MAX_PACKET_SIZE = 512,
};

typedef struct __nRF_Usb_Ep_Registration
{
	uint8_t *pBuffer;
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
	uint16_t MaxPacketSize;
	bool bBlocking;
} nRFUsbEpReg_t;

enum
{
	USBD_FLAG_SUSPENDED     = 0x0001U,
	USBD_FLAG_SUSPEND_PEND  = 0x0002U,
	USBD_FLAG_REMOTE_WAKE   = 0x0004U,
	USBD_FLAG_HOST_RESUME   = 0x0008U,
	USBD_FLAG_MAC_AWAKE     = 0x0010U,
	// Suspend clears READY with the wake flags; keep that mask byte-sized.
	USBD_FLAG_ISO_OUT_READY = 0x0020U,
	USBD_FLAG_ISO_IN_READY  = 0x0040U,
	USBD_FLAG_ISO_OUT_OPEN  = 0x0400U,
	USBD_FLAG_ISO_IN_OPEN   = 0x0800U,
	USBD_FLAG_ISO_OUT_BUSY  = 0x1000U,
	USBD_FLAG_ISO_IN_BUSY   = 0x2000U,
	USBD_FLAG_ISO_OUT_CMPL  = 0x4000U,
	USBD_FLAG_ISO_IN_CMPL   = 0x8000U,
};

typedef struct __nRF_Usbd_State
{
	// Keep queue metadata at small offsets for Thumb loads/stores.
	uint8_t IntPrio;
	bool LowPowerSuspend;
	bool SofEnabled;
	// One pending DMA packet per ISO direction; -1 means none, 0 is a ZLP.
	int16_t IsoDmaLen[2];
	uint16_t IsoOutSize;
	volatile uint32_t Flags;
	hCFifo_t hQue;
	hCFifo_t hEp0Que;
	uint32_t IsoGeneration[2];
	// Non-control endpoints 1-8.
	nRFUsbEpReg_t EpReg[NRF_USB_EP_COUNT - 1][2];
	alignas(4) uint8_t Ep0Bounce[NRFX_USBD_MAX_PACKET_SIZE];
} nRFUsbdState_t;

extern nRFUsbdState_t s_Usbd;

void nRFUsbEpRegisteredEvent(uint8_t EpNum, uint8_t Dir,
							 UsbCtrlrEvtType_t Event, uint16_t Length);
void nRFUsbdDmaUnlock(void);
void nRFUsbdSofAcquire(void);
void nRFUsbdSofRelease(void);
void nRFUsbdDmaWait(void);
void nRFUsbdResumeQueuedDmaLocked(void);

/** Start EasyDMA with the channel already locked by the caller. */
static inline __attribute__((always_inline))
void nRFUsbdDmaStartLocked(volatile uint32_t *pTask,
	volatile uint32_t *pEnd)
{
	*pEnd = 0;
	__DSB();

	*pTask = 1;
	__DSB();
}

#endif // USBD_PRESENT

/** @} End of group USB */

#endif	// __USB_CTRLR_H__
