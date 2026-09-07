/**-------------------------------------------------------------------------
@file	usb_iso_intrf_duplex_test.cpp

@brief	Simultaneous IN/OUT host test for UsbIsoIntrf.

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved
----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "usb/usb_iso.h"

typedef struct {
	uint8_t EpAddr;
	uint8_t *pBuffer;
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
} Reg_t;

static Reg_t s_Reg[2];
static int s_RegCnt;
static bool s_RxArmed;
static bool s_TxBusy;
static uint16_t s_TxLen;
static uint8_t s_TxData[USB_ISO_INTRF_MAX_MPS];

extern "C" {
bool UsbCtrlrEpRegister(int, uint8_t EpAddr, uint8_t *pBuffer,
	UsbCtrlrEpHandler_t Handler, void *pContext)
{
	if (s_RegCnt >= 2 || pBuffer == nullptr || Handler == nullptr)
	{
		return false;
	}
	s_Reg[s_RegCnt++] = { EpAddr, pBuffer, Handler, pContext };
	return true;
}

bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *) { return true; }

void UsbCtrlrEpClose(int, uint8_t EpAddr)
{
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		s_TxBusy = false;
	}
	else
	{
		s_RxArmed = false;
	}
}

bool UsbCtrlrEpRxArm(int, uint8_t EpNo)
{
	if (EpNo != 8U || s_RxArmed)
	{
		return false;
	}
	s_RxArmed = true;
	return true;
}

bool UsbCtrlrEpSend(int, uint8_t EpNo, uint16_t Length)
{
	if (EpNo != 8U || s_TxBusy || Length > sizeof(s_TxData))
	{
		return false;
	}
	Reg_t *pIn = nullptr;
	for (int i = 0; i < s_RegCnt; i++)
	{
		if (s_Reg[i].EpAddr == USB_ENDPADDR_DIRIN(8U))
		{
			pIn = &s_Reg[i];
		}
	}
	if (pIn == nullptr)
	{
		return false;
	}
	s_TxLen = Length;
	memcpy(s_TxData, pIn->pBuffer, Length);
	s_TxBusy = true;
	return true;
}
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

static int s_RxCnt;
static int s_TxCnt;
static uint8_t s_RxData[USB_ISO_INTRF_MAX_MPS];
static uint16_t s_RxLen;

static void RxCb(UsbIsoIntrf_t *, const uint8_t *pData, uint16_t Length,
	UsbCtrlrXferResult_t Result, void *)
{
	CHECK(Result == USB_CTRLR_XFER_SUCCESS);
	s_RxCnt++;
	s_RxLen = Length;
	memcpy(s_RxData, pData, Length);
}

static void TxCb(UsbIsoIntrf_t *, uint16_t Length,
	UsbCtrlrXferResult_t Result, void *)
{
	CHECK(Result == USB_CTRLR_XFER_SUCCESS);
	CHECK(Length == s_TxLen);
	s_TxCnt++;
}

static Reg_t *Find(uint8_t EpAddr)
{
	for (int i = 0; i < s_RegCnt; i++)
	{
		if (s_Reg[i].EpAddr == EpAddr)
		{
			return &s_Reg[i];
		}
	}
	return nullptr;
}

int main(void)
{
	UsbIsoIntrf_t iso = {};
	UsbIsoIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = 8U;
	cfg.RxHandler = RxCb;
	cfg.TxHandler = TxCb;

	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(UsbIsoIntrfOpen(&iso, 63U, 1U));
	CHECK(s_RxArmed);

	uint8_t tx[63];
	uint8_t rx[63];
	for (unsigned i = 0; i < sizeof(tx); i++)
	{
		tx[i] = (uint8_t)(0x80U + i);
		rx[i] = (uint8_t)i;
	}

	// Keep IN active while a full OUT frame completes. Both directions must
	// remain independent even though the controller may arbitrate one DMA engine.
	CHECK(UsbIsoIntrfSendFrame(&iso, tx, sizeof(tx)));
	CHECK(s_TxBusy);
	CHECK(s_RxArmed);
	CHECK(memcmp(s_TxData, tx, sizeof(tx)) == 0);

	Reg_t *pOut = Find(USB_ENDPADDR_DIROUT(8U));
	CHECK(pOut != nullptr);
	if (pOut != nullptr)
	{
		memcpy(pOut->pBuffer, rx, sizeof(rx));
		s_RxArmed = false;
		pOut->Handler(USB_ENDPADDR_DIROUT(8U), sizeof(rx),
			USB_CTRLR_XFER_SUCCESS, pOut->pContext);
	}
	CHECK(s_RxCnt == 1);
	CHECK(s_RxLen == sizeof(rx));
	CHECK(memcmp(s_RxData, rx, sizeof(rx)) == 0);
	CHECK(s_RxArmed);
	CHECK(s_TxBusy);

	Reg_t *pIn = Find(USB_ENDPADDR_DIRIN(8U));
	CHECK(pIn != nullptr);
	if (pIn != nullptr)
	{
		s_TxBusy = false;
		pIn->Handler(USB_ENDPADDR_DIRIN(8U), sizeof(tx),
			USB_CTRLR_XFER_SUCCESS, pIn->pContext);
	}
	CHECK(s_TxCnt == 1);
	CHECK(!iso.TxActive);
	CHECK(s_RxArmed);

	if (s_Fail != 0)
	{
		printf("usb_iso_intrf_duplex_test: %d failure(s)\n", s_Fail);
		return 1;
	}

	printf("usb_iso_intrf_duplex_test: PASS\n");
	return 0;
}
