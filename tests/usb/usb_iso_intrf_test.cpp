/**-------------------------------------------------------------------------
@file	usb_iso_intrf_test.cpp

@brief	Host tests for the generic USB isochronous endpoint interface.

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2026

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
#include <stdio.h>
#include <string.h>

#include "usb/usb_iso_intrf.h"

typedef struct {
	uint8_t EpAddr;
	uint8_t *pBuffer;
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
} RegisteredEp_t;

static RegisteredEp_t s_Reg[2];
static int s_RegCount;
static UsbEndPointDesc_t s_Open[4];
static int s_OpenCount;
static int s_CloseCount;
static bool s_OutArmed;
static bool s_InBusy;
static uint16_t s_SendLength;
static uint8_t s_SendData[USB_ISO_INTRF_MAX_MPS];
static bool s_ArmFail;
static bool s_SendFail;

extern "C" {
bool UsbCtrlrEpRegister(int DevNo, uint8_t EpAddr, uint8_t *pBuffer,
						UsbCtrlrEpHandler_t Handler, void *pContext)
{
	if (DevNo != 0 || pBuffer == nullptr || Handler == nullptr ||
		s_RegCount >= 2)
	{
		return false;
	}
	s_Reg[s_RegCount++] = { EpAddr, pBuffer, Handler, pContext };
	return true;
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	if (DevNo != 0 || pDesc == nullptr || s_OpenCount >= 4)
	{
		return false;
	}
	s_Open[s_OpenCount++] = *pDesc;
	return true;
}

void UsbCtrlrEpClose(int, uint8_t EpAddr)
{
	s_CloseCount++;
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		s_InBusy = false;
	}
	else
	{
		s_OutArmed = false;
	}
}

bool UsbCtrlrEpRxArm(int DevNo, uint8_t EpNo)
{
	if (DevNo != 0 || EpNo != 8U || s_OutArmed || s_ArmFail)
	{
		return false;
	}
	s_OutArmed = true;
	return true;
}

bool UsbCtrlrEpSend(int DevNo, uint8_t EpNo, uint16_t Length)
{
	if (DevNo != 0 || EpNo != 8U || s_InBusy || s_SendFail ||
		Length > sizeof(s_SendData))
	{
		return false;
	}
	RegisteredEp_t *pIn = nullptr;
	for (int i = 0; i < s_RegCount; i++)
	{
		if (s_Reg[i].EpAddr == USB_ENDPADDR_DIRIN(EpNo))
		{
			pIn = &s_Reg[i];
			break;
		}
	}
	if (pIn == nullptr)
	{
		return false;
	}
	s_SendLength = Length;
	if (Length != 0U)
	{
		memcpy(s_SendData, pIn->pBuffer, Length);
	}
	s_InBusy = true;
	return true;
}
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

static int s_RxCount;
static int s_TxCount;
static uint16_t s_LastRxLength;
static uint16_t s_LastTxLength;
static UsbCtrlrXferResult_t s_LastRxResult;
static UsbCtrlrXferResult_t s_LastTxResult;
static uint8_t s_LastRx[USB_ISO_INTRF_MAX_MPS];

static void RxFrame(UsbIsoIntrf_t *, const uint8_t *pData, uint16_t Length,
					UsbCtrlrXferResult_t Result, void *)
{
	s_RxCount++;
	s_LastRxLength = Length;
	s_LastRxResult = Result;
	if (Result == USB_CTRLR_XFER_SUCCESS && Length != 0U)
	{
		memcpy(s_LastRx, pData, Length);
	}
}

static void TxFrame(UsbIsoIntrf_t *, uint16_t Length,
					UsbCtrlrXferResult_t Result, void *)
{
	s_TxCount++;
	s_LastTxLength = Length;
	s_LastTxResult = Result;
}

static RegisteredEp_t *FindReg(uint8_t EpAddr)
{
	for (int i = 0; i < s_RegCount; i++)
	{
		if (s_Reg[i].EpAddr == EpAddr)
		{
			return &s_Reg[i];
		}
	}
	return nullptr;
}

static void Receive(const uint8_t *pData, uint16_t Length,
					UsbCtrlrXferResult_t Result)
{
	RegisteredEp_t *pOut = FindReg(USB_ENDPADDR_DIROUT(8U));
	CHECK(pOut != nullptr);
	CHECK(s_OutArmed);
	if (pOut == nullptr || !s_OutArmed)
	{
		return;
	}
	if (pData != nullptr && Length != 0U)
	{
		memcpy(pOut->pBuffer, pData, Length);
	}
	s_OutArmed = false;
	pOut->Handler(USB_ENDPADDR_DIROUT(8U), Length, Result, pOut->pContext);
}

static void CompleteIn(uint16_t Length, UsbCtrlrXferResult_t Result)
{
	RegisteredEp_t *pIn = FindReg(USB_ENDPADDR_DIRIN(8U));
	CHECK(pIn != nullptr);
	CHECK(s_InBusy);
	if (pIn == nullptr || !s_InBusy)
	{
		return;
	}
	s_InBusy = false;
	pIn->Handler(USB_ENDPADDR_DIRIN(8U), Length, Result, pIn->pContext);
}

static void ResetFake(void)
{
	memset(s_Reg, 0, sizeof(s_Reg));
	memset(s_Open, 0, sizeof(s_Open));
	memset(s_SendData, 0, sizeof(s_SendData));
	memset(s_LastRx, 0, sizeof(s_LastRx));
	s_RegCount = 0;
	s_OpenCount = 0;
	s_CloseCount = 0;
	s_OutArmed = false;
	s_InBusy = false;
	s_SendLength = 0U;
	s_ArmFail = false;
	s_SendFail = false;
	s_RxCount = 0;
	s_TxCount = 0;
	s_LastRxLength = 0U;
	s_LastTxLength = 0U;
	s_LastRxResult = USB_CTRLR_XFER_SUCCESS;
	s_LastTxResult = USB_CTRLR_XFER_SUCCESS;
}

static UsbIsoIntrfCfg_t MakeCfg(void)
{
	UsbIsoIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = 8U;
	cfg.RxHandler = RxFrame;
	cfg.TxHandler = TxFrame;
	return cfg;
}

static void TestLifecycle(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	UsbIsoIntrfCfg_t cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(s_RegCount == 2);
	CHECK(FindReg(USB_ENDPADDR_DIROUT(8U))->pBuffer ==
		reinterpret_cast<uint8_t *>(iso.RxBuffer));
	CHECK(FindReg(USB_ENDPADDR_DIRIN(8U))->pBuffer ==
		reinterpret_cast<uint8_t *>(iso.TxBuffer));

	CHECK(UsbIsoIntrfOpen(&iso, 25U, 1U));
	CHECK(iso.Opened);
	CHECK(iso.Mps == 25U);
	CHECK(iso.Interval == 1U);
	CHECK(s_OpenCount == 2);
	CHECK(s_Open[0].bEndpointAddress == USB_ENDPADDR_DIROUT(8U));
	CHECK(s_Open[1].bEndpointAddress == USB_ENDPADDR_DIRIN(8U));
	CHECK(s_Open[0].bmAttributes == USB_ENDPATT_TRANS_ISO);
	CHECK(s_Open[0].wMaxPacketSize == 25U);
	CHECK(s_OutArmed);

	UsbIsoIntrfClose(&iso);
	CHECK(!iso.Opened);
	CHECK(s_CloseCount == 2);
	CHECK(!s_OutArmed);
	CHECK(!s_InBusy);
}

static void TestRxFrames(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	UsbIsoIntrfCfg_t cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(UsbIsoIntrfOpen(&iso, 17U, 1U));

	const uint8_t data[] = { 1U, 2U, 3U, 4U, 5U };
	Receive(data, sizeof(data), USB_CTRLR_XFER_SUCCESS);
	CHECK(s_RxCount == 1);
	CHECK(s_LastRxLength == sizeof(data));
	CHECK(s_LastRxResult == USB_CTRLR_XFER_SUCCESS);
	CHECK(memcmp(s_LastRx, data, sizeof(data)) == 0);
	CHECK(s_OutArmed);

	Receive(nullptr, 0U, USB_CTRLR_XFER_SUCCESS);
	CHECK(s_RxCount == 2);
	CHECK(iso.RxEmptyCnt == 1U);
	CHECK(s_OutArmed);

	Receive(nullptr, 0U, USB_CTRLR_XFER_FAILED);
	CHECK(s_RxCount == 3);
	CHECK(s_LastRxResult == USB_CTRLR_XFER_FAILED);
	CHECK(iso.RxMissCnt == 1U);
	CHECK(s_OutArmed);
}

static void TestTxFrames(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	UsbIsoIntrfCfg_t cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(UsbIsoIntrfOpen(&iso, 9U, 1U));

	const uint8_t frame[] = { 0x11U, 0x22U, 0x33U, 0x44U };
	CHECK(UsbIsoIntrfSendFrame(&iso, frame, sizeof(frame)));
	CHECK(iso.TxActive);
	CHECK(s_SendLength == sizeof(frame));
	CHECK(memcmp(s_SendData, frame, sizeof(frame)) == 0);
	CHECK(!UsbIsoIntrfSendFrame(&iso, frame, sizeof(frame)));
	CompleteIn(sizeof(frame), USB_CTRLR_XFER_SUCCESS);
	CHECK(!iso.TxActive);
	CHECK(s_TxCount == 1);
	CHECK(s_LastTxLength == sizeof(frame));
	CHECK(s_LastTxResult == USB_CTRLR_XFER_SUCCESS);

	CHECK(UsbIsoIntrfSendFrame(&iso, nullptr, 0U));
	CompleteIn(0U, USB_CTRLR_XFER_SUCCESS);
	CHECK(iso.TxEmptyCnt == 1U);

	CHECK(UsbIsoIntrfSendFrame(&iso, frame, 3U));
	CompleteIn(2U, USB_CTRLR_XFER_SUCCESS);
	CHECK(s_LastTxResult == USB_CTRLR_XFER_FAILED);
	CHECK(iso.TxMissCnt == 1U);

	s_SendFail = true;
	CHECK(!UsbIsoIntrfSendFrame(&iso, frame, 1U));
	CHECK(iso.TxMissCnt == 2U);
	CHECK(!iso.TxActive);
}

static void TestSuspendResume(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	UsbIsoIntrfCfg_t cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(UsbIsoIntrfOpen(&iso, 33U, 1U));

	UsbIsoIntrfSuspend(&iso);
	CHECK(iso.Suspended);
	const uint8_t byte = 0x5AU;
	CHECK(!UsbIsoIntrfSendFrame(&iso, &byte, 1U));

	Receive(nullptr, 0U, USB_CTRLR_XFER_CANCELLED);
	CHECK(!s_OutArmed);
	CHECK(iso.RxMissCnt == 1U);
	CHECK(UsbIsoIntrfResume(&iso));
	CHECK(!iso.Suspended);
	CHECK(s_OutArmed);

	UsbIsoIntrfReset(&iso);
	CHECK(!iso.Opened);
	CHECK(iso.RxMissCnt == 0U);
	CHECK(iso.TxMissCnt == 0U);
	CHECK(iso.RxEmptyCnt == 0U);
	CHECK(iso.TxEmptyCnt == 0U);
}

static void TestValidation(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	UsbIsoIntrfCfg_t cfg = MakeCfg();
	cfg.EpNo = 7U;
	CHECK(!UsbIsoIntrfInit(&iso, &cfg));

	ResetFake();
	cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(!UsbIsoIntrfOpen(&iso, 0U, 1U));
	CHECK(!UsbIsoIntrfOpen(&iso, USB_ISO_INTRF_MAX_MPS + 1U, 1U));
	CHECK(!UsbIsoIntrfOpen(&iso, 9U, 0U));
}

int main(void)
{
	TestLifecycle();
	TestRxFrames();
	TestTxFrames();
	TestSuspendResume();
	TestValidation();

	if (s_Fail != 0)
	{
		printf("usb_iso_intrf_test: %d failure(s)\n", s_Fail);
		return 1;
	}

	printf("usb_iso_intrf_test: PASS\n");
	return 0;
}
