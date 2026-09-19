/**-------------------------------------------------------------------------
@file	usb_ctrlr_nrf52_priv.h

@brief	Shared private state for the nRF52 USBD base and optional ISO object.
----------------------------------------------------------------------------*/
#ifndef __USB_CTRLR_NRF52_PRIV_H__
#define __USB_CTRLR_NRF52_PRIV_H__

#include <stdint.h>

#include "cfifo.h"
#include "usb/usb.h"

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
	uint16_t Mps;
	bool bBlocking;
} nRFUsbEpReg_t;

#pragma pack(push, 4)
typedef struct __nRF_Usbd_Xfer
{
	uint8_t *pBuffer;
	uint16_t TotalLen;
	volatile uint16_t ActualLen;
} nRFUsbdXfer_t;

typedef struct __nRF_Usbd_Ctrlr
{
	nRFUsbdXfer_t Ep0[2];
	nRFUsbdXfer_t Iso[2];
	bool SofEnabled;
} nRFUsbdCtrlr_t;
#pragma pack(pop)

enum
{
	USBD_FLAG_SUSPENDED     = 0x0001U,
	USBD_FLAG_SUSPEND_PEND  = 0x0002U,
	USBD_FLAG_REMOTE_WAKE   = 0x0004U,
	USBD_FLAG_HOST_RESUME   = 0x0008U,
	USBD_FLAG_MAC_AWAKE     = 0x0010U,
	USBD_FLAG_ISO_OUT_READY = 0x0100U,
	USBD_FLAG_ISO_IN_READY  = 0x0200U,
	USBD_FLAG_ISO_OUT_OPEN  = 0x0400U,
	USBD_FLAG_ISO_IN_OPEN   = 0x0800U,
	USBD_FLAG_ISO_OUT_BUSY  = 0x1000U,
	USBD_FLAG_ISO_IN_BUSY   = 0x2000U,
	USBD_FLAG_ISO_OUT_CMPL  = 0x4000U,
	USBD_FLAG_ISO_IN_CMPL   = 0x8000U,
};

typedef struct __nRF_Usbd_State
{
	volatile uint32_t Flags;
	hCFifo_t hQue;
	hCFifo_t hEp0Que;
	uint32_t IsoGeneration[2];
	uint16_t IsoOutSize;
	uint8_t IntPrio;
	bool LowPowerSuspend;
	bool Initialized;
	bool Started;
	nRFUsbdCtrlr_t Ctrlr;
	nRFUsbEpReg_t EpReg[NRF_USB_EP_COUNT][2];
	alignas(4) uint8_t Ep0Bounce[NRFX_USBD_MAX_PACKET_SIZE];
} nRFUsbdState_t;

extern nRFUsbdState_t s_Usbd;
void nRFUsbdDmaWait(void);
void nRFUsbdResumeQueuedDmaLocked(void);

#endif
