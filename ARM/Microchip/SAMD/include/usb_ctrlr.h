/**-------------------------------------------------------------------------
@file	usb_ctrlr.h

@brief	SAM D11/D21 USB full speed device controller interface.

Control, isochronous, bulk and interrupt transfers. Endpoints 1 to 7 have an
OUT bank and an IN bank each, so an endpoint number can be open in both
directions. Device mode only, the SAM D21 host mode is not used by this
port.

The 48 MHz USB clock is generator 0: g_McuOsc.bUSBClk on, see
system_samd.c. The optional VBUS pin is entry USB_VBUS_PIN_IDX of the pin
map. Without it the port reports bus power present, a bus powered board.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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

#include "samdxxx.h"
#include "coredev/iopincfg.h"
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

// Counts include EP0
enum {
	USB_CTRLR_CNT = 1,
	USB_HIGHSPEED_CAPABLE_0 = 0,
	USB_EPIN_CNT_0 = USB_EPT_NUM,
	USB_EPOUT_CNT_0 = USB_EPT_NUM,
	USB_CTRLR0_CONTROL_PKT_LEN_MAX = 64,
	USB_CTRLR0_BULK_PKT_LEN_MAX = 64,
	USB_CTRLR0_INT_PKT_LEN_MAX = 64,
	USB_CTRLR0_ISO_PKT_LEN_MAX = 1023,
	USB_ISO_SUPPORTED_0 = 1,
	USB_ISO_EPIN_MASK_0 = 0x00FEU,
	USB_ISO_EPOUT_MASK_0 = 0x00FEU,
};

// The configuration descriptor buffer is in RAM: 256 bytes on the 4 KB of
// the SAM D11, enough for one class.
#ifndef USB_CONFIG_DESC_MAXLEN
#define USB_CONFIG_DESC_MAXLEN		(HMCRAMC0_SIZE <= 0x1000U ? 256U : 768U)
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

/**
 * @brief	Bytes the OUT bank writes for packets of up to Mps bytes.
 *
 * The bank size is Mps rounded up to a power of 2 from 8 to 512, or 1023. A
 * packet two bytes or more below the bank size is written with its two CRC
 * bytes: Mps + 2 bytes, at most the bank size.
 */
static inline uint16_t SamdUsbRxCapacity(uint16_t Mps)
{
	uint16_t bank = 8U;

	while (bank < 512U && bank < Mps)
	{
		bank = (uint16_t)(bank << 1);
	}
	if (bank < Mps)
	{
		bank = 1023U;
	}

	return (uint16_t)(Mps + 2U) < bank ? (uint16_t)(Mps + 2U) : bank;
}

#define USB_CTRLR_RX_CAPACITY(CtrlrNo, Mps)	SamdUsbRxCapacity(Mps)

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
 * Registered with UsbCtrlrEpBind. Called from the USB interrupt, or from the
 * controller call that changed the endpoint state. DRDY asks the owner for an
 * OUT destination; an owner without room supplies none and submits one with
 * UsbCtrlrEpReceive when space returns. XFER_CMPL reports success; failure and
 * cancellation use their own events.
 */
typedef void (*UsbCtrlrEpHandler_t)(UsbCtrlrEvtType_t Event,
									uint16_t Length, void *pContext);

/// What the generic layer hands the port at UsbCtrlrInit.
typedef struct __Usb_Ctrlr_Config {
	int IntPrio;					//!< Interrupt priority of the USB peripheral
	const IOPinCfg_t *pIOPinMap;	//!< Optional board USB pins
	int NbIOPins;					//!< Number of entries in pIOPinMap
	bool bLowPowerSuspend;			//!< true - Sit in USB low power while suspended, not supported here
} UsbCtrlrCfg_t;

#ifdef __cplusplus
extern "C" {
#endif

bool UsbCtrlrInit(int DevNo, const UsbCtrlrCfg_t *pCfg);
bool UsbCtrlrStart(int DevNo);
void UsbCtrlrStop(int DevNo);
void UsbCtrlrProcess(int DevNo);
// VBUS pin level, or true without a VBUS pin. The port queues UsbProcessQue
// on the edges of the pin.
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
// OUT destination, word aligned, Capacity at least USB_CTRLR_RX_CAPACITY
// of the endpoint packet size.
bool UsbCtrlrEpReceive(int DevNo, uint8_t EpNo, uint8_t *pBuffer,
						  uint16_t Capacity);
void UsbCtrlrEpProcessEvent(int DevNo, uint8_t EpNo, bool bIn,
						 UsbCtrlrEvtType_t Event, uint16_t Value);
// The source stays owned until completion or cancellation. Zero length
// submits a ZLP. Only EP0 copies its source before returning. A source not
// word aligned sends the bytes up to the next word boundary first, the
// completion length tells how many.
bool UsbCtrlrEpSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length);
int UsbCtrlrEp0Send(int DevNo, uint8_t *pBuffer, int Length);
bool UsbCtrlrEp0Status(int DevNo, uint8_t EpAddr);
void UsbCtrlrEpStall(int DevNo, uint8_t EpNo, bool bIn);
void UsbCtrlrEpClearStall(int DevNo, uint8_t EpNo, bool bIn);
size_t UsbCtrlrGetSerial(int DevNo, char *pBuff, size_t BuffLen);
// UsbCtrlrIsoInit is in the ISO archive member, usb_ctrlr_samd_iso.cpp.
// Referencing UsbIsoIntrf pulls it in, applications without ISO keep it out.
bool UsbCtrlrIsoInit(int DevNo);
bool UsbCtrlrIsoOpen(int DevNo, uint8_t EpNo, bool bIn, uint16_t MaxPacketSize);
bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length);
uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData);

#ifdef __cplusplus
}
#endif

/** @} End of group USB */

#endif // __USB_CTRLR_H__
