/**-------------------------------------------------------------------------
@file	usbd_bulk_test.cpp

@brief	Host tests for the public vendor bulk USB class.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

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

#include "usb/usbd_bulk.h"

#define EP_NO		3U
#define ITF_NO		2
#define RX_SLOTS	4U
#define TX_SLOTS	4U

static UsbCfg_t s_UsbCfg;
static UsbFuncCfg_t s_FuncCfg;
static bool s_FuncRegistered;
static UsbEndPointDesc_t s_OpenDesc[2];
static int s_OpenCount;
static int s_CloseCount;
static uint8_t *s_OutBuffer;
static uint8_t *s_InBuffer;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static bool s_OutBusy;
static bool s_InBusy;
static uint16_t s_InLength;
static int s_ArmCount;
static int s_SendCount;
static void *s_RequestContext;
static int s_RequestCount;

extern "C" {
const UsbCfg_t *UsbGetCfg(int DevNo)
{
	return DevNo == 0 ? &s_UsbCfg : nullptr;
}

bool UsbRegisterFunc(int DevNo, const UsbFuncCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == nullptr)
	{
		return false;
	}
	s_FuncCfg = *pCfg;
	s_FuncRegistered = true;
	return true;
}

bool UsbCtrlrHighSpeed(int) { return false; }

bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == nullptr || s_OpenCount >= 2)
	{
		return false;
	}
	s_OpenDesc[s_OpenCount++] = *pDesc;
	return true;
}

void UsbCtrlrEpClose(int, uint8_t) { s_CloseCount++; }

bool UsbCtrlrEpRegister(int, uint8_t EpAddr, uint8_t *pBuffer,
						UsbCtrlrEpHandler_t Handler, void *pContext)
{
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		s_InBuffer = pBuffer;
		s_InHandler = Handler;
		s_InContext = pContext;
	}
	else
	{
		s_OutBuffer = pBuffer;
		s_OutHandler = Handler;
		s_OutContext = pContext;
	}
	return true;
}

bool UsbCtrlrEpRxArm(int, uint8_t EpNo)
{
	if (EpNo != EP_NO || s_OutBusy)
	{
		return false;
	}
	s_OutBusy = true;
	s_ArmCount++;
	return true;
}

bool UsbCtrlrEpSend(int, uint8_t EpNo, uint16_t Length)
{
	if (EpNo != EP_NO || s_InBusy)
	{
		return false;
	}
	s_InBusy = true;
	s_InLength = Length;
	s_SendCount++;
	return true;
}
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

alignas(4) static uint8_t s_RxMem[USBD_BULK_RXMEM_SIZE(RX_SLOTS)];
alignas(4) static uint8_t s_TxByteMem[CFIFO_MEMSIZE(256)];
alignas(4) static uint8_t s_TxPacketMem[USBD_BULK_TXMEM_SIZE(TX_SLOTS)];

static bool VendorRequest(const UsbSetupData_t *, UsbCtrlStage_t,
						  uint8_t **, uint16_t *, void *pContext)
{
	s_RequestContext = pContext;
	s_RequestCount++;
	return true;
}

static UsbdBulkCfg_t MakeCfg(UsbdBulkMode_t Mode)
{
	UsbdBulkCfg_t cfg = {};
	cfg.bBlocking = true;
	cfg.RxFifoMemSize = sizeof(s_RxMem);
	cfg.pRxFifoMem = s_RxMem;
	cfg.TxFifoMemSize = Mode == USBD_BULK_MODE_PACKET ?
		sizeof(s_TxPacketMem) : sizeof(s_TxByteMem);
	cfg.pTxFifoMem = Mode == USBD_BULK_MODE_PACKET ?
		s_TxPacketMem : s_TxByteMem;
	cfg.ItfNo = ITF_NO;
	cfg.DevNo = 0;
	cfg.EpNo = EP_NO;
	cfg.SubClass = 0x12U;
	cfg.Protocol = 0x34U;
	cfg.InterfaceString = 5U;
	cfg.Mode = Mode;
	cfg.RequestHandler = VendorRequest;
	cfg.pRequestContext = &s_RequestContext;
	return cfg;
}

static void ResetFake(void)
{
	memset(&s_UsbCfg, 0, sizeof(s_UsbCfg));
	memset(&s_FuncCfg, 0, sizeof(s_FuncCfg));
	memset(s_OpenDesc, 0, sizeof(s_OpenDesc));
	s_FuncRegistered = false;
	s_OpenCount = 0;
	s_CloseCount = 0;
	s_OutBuffer = nullptr;
	s_InBuffer = nullptr;
	s_OutHandler = nullptr;
	s_InHandler = nullptr;
	s_OutContext = nullptr;
	s_InContext = nullptr;
	s_OutBusy = false;
	s_InBusy = false;
	s_InLength = 0U;
	s_ArmCount = 0;
	s_SendCount = 0;
	s_RequestContext = nullptr;
	s_RequestCount = 0;
	s_UsbCfg.DevNo = 0;
}

static void TestDescriptor(void)
{
	ResetFake();
	const UsbdBulkCfg_t cfg = MakeCfg(USBD_BULK_MODE_BYTE);
	UsbdBulkDesc_t desc = {};

	CHECK(sizeof(desc) == sizeof(UsbIntrfDesc_t) + 2U * sizeof(UsbEndPointDesc_t));
	CHECK(UsbdBulkMakeDesc(&desc, &cfg, USB_SPEED_FULL));
	CHECK(desc.Interface.bLength == sizeof(UsbIntrfDesc_t));
	CHECK(desc.Interface.bInterfaceNumber == ITF_NO);
	CHECK(desc.Interface.bNumEndpoints == 2U);
	CHECK(desc.Interface.bInterfaceClass == USB_INTRFCLASS_VENDOR);
	CHECK(desc.Interface.bInterfaceSubClass == cfg.SubClass);
	CHECK(desc.Interface.bInterfaceProtocol == cfg.Protocol);
	CHECK(desc.Interface.iInterface == cfg.InterfaceString);
	CHECK(desc.Out.bEndpointAddress == USB_ENDPADDR_DIROUT(EP_NO));
	CHECK(desc.In.bEndpointAddress == USB_ENDPADDR_DIRIN(EP_NO));
	CHECK(desc.Out.bmAttributes == USB_ENDPATT_TRANS_BULK);
	CHECK(desc.In.bmAttributes == USB_ENDPATT_TRANS_BULK);
	CHECK(desc.Out.wMaxPacketSize == USBD_BULK_FS_MPS);
	CHECK(desc.In.wMaxPacketSize == USBD_BULK_FS_MPS);
}

static void TestByteMode(void)
{
	ResetFake();
	UsbdBulk bulk;
	const UsbdBulkCfg_t cfg = MakeCfg(USBD_BULK_MODE_BYTE);

	CHECK(bulk.Init(cfg));
	CHECK(s_FuncRegistered);
	CHECK(s_FuncCfg.FirstInterface == ITF_NO);
	CHECK(s_FuncCfg.InterfaceCount == 1U);
	CHECK(s_FuncCfg.EpInMask == (1U << EP_NO));
	CHECK(s_FuncCfg.EpOutMask == (1U << EP_NO));
	CHECK(s_OutBuffer != nullptr && s_InBuffer != nullptr);
	CHECK(s_OutBuffer != s_InBuffer);

	CHECK(s_FuncCfg.ConfigHandler(1U, s_FuncCfg.pContext));
	CHECK(s_OpenCount == 2);
	CHECK(s_OpenDesc[0].bEndpointAddress == USB_ENDPADDR_DIROUT(EP_NO));
	CHECK(s_OpenDesc[1].bEndpointAddress == USB_ENDPADDR_DIRIN(EP_NO));
	CHECK(s_OpenDesc[0].wMaxPacketSize == USBD_BULK_FS_MPS);
	CHECK(s_OutBusy && s_ArmCount == 1);

	const uint8_t rx[] = { 1U, 2U, 3U, 4U };
	memcpy(s_OutBuffer, rx, sizeof(rx));
	s_OutBusy = false;
	s_OutHandler(USB_ENDPADDR_DIROUT(EP_NO), sizeof(rx),
				 USB_CTRLR_XFER_SUCCESS, s_OutContext);
	uint8_t received[sizeof(rx)] = {};
	CHECK(bulk.RxData(received, sizeof(received)) == (int)sizeof(received));
	CHECK(memcmp(received, rx, sizeof(rx)) == 0);

	const uint8_t tx[] = { 5U, 6U, 7U };
	CHECK(bulk.TxData(tx, sizeof(tx)) == (int)sizeof(tx));
	CHECK(s_InBusy && s_InLength == sizeof(tx));
	CHECK(memcmp(s_InBuffer, tx, sizeof(tx)) == 0);
	s_InBusy = false;
	s_InHandler(USB_ENDPADDR_DIRIN(EP_NO), sizeof(tx),
				USB_CTRLR_XFER_SUCCESS, s_InContext);

	UsbSetupData_t setup = {};
	setup.bmRequestType = USB_REQTYPE_VEND | USB_REQTYPE_INTERFACE;
	setup.wIndex = ITF_NO;
	uint16_t length = 0U;
	CHECK(s_FuncCfg.RequestHandler(&setup, USB_CTRL_SETUP, nullptr, &length,
								   s_FuncCfg.pContext));
	CHECK(s_RequestCount == 1);
	CHECK(s_RequestContext == &s_RequestContext);

	CHECK(s_FuncCfg.ConfigHandler(0U, s_FuncCfg.pContext));
	CHECK(DeviceIntrfGetRate(bulk.Data()) == 0U);
}

static void TestPacketMode(void)
{
	ResetFake();
	UsbdBulk bulk;
	const UsbdBulkCfg_t cfg = MakeCfg(USBD_BULK_MODE_PACKET);

	CHECK(bulk.Init(cfg));
	CHECK(s_FuncCfg.ConfigHandler(1U, s_FuncCfg.pContext));

	alignas(4) uint8_t block[USBD_BULK_PKT_BLKSIZE] = {};
	UsbPkt_t *pPacket = reinterpret_cast<UsbPkt_t *>(block);
	pPacket->Hdr.Length = 5U;
	memcpy(pPacket->Data, "bulk!", 5U);

	CHECK(bulk.TxData(block, sizeof(block)) == (int)sizeof(block));
	CHECK(s_InBusy && s_InLength == 5U);
	CHECK(memcmp(s_InBuffer, "bulk!", 5U) == 0);
}

int main(void)
{
	TestDescriptor();
	TestByteMode();
	TestPacketMode();

	if (s_Fail != 0)
	{
		printf("%d failed\n", s_Fail);
		return 1;
	}

	printf("all pass\n");
	return 0;
}
