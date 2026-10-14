/**-------------------------------------------------------------------------
@file	classes17_test.cpp

@brief	LPC1769 real class allocation and data integration checks.

Uses the real generic core, classes and LPC1769 public data driver.
FIFO, SIE and EP0 handshakes are supplied by integration17_model.c.

@author	Hoang Nguyen Hoan
@date	Sep. 10, 2026

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
#include "app_evt_handler.h"
#include "usb/usbd_cdc.h"
#include "usb/usbd_hid.h"
#include "usb/usbd_bulk.h"
#include <cassert>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include "usb/usbd_epalloc.h"


class AllocationClass : public UsbDeviceClass {};

static AllocationClass s_Allocation[7];
static UsbdCdc s_Cdc;
static UsbdHid s_Hid;
static UsbdBulk s_Bulk;
alignas(4) static uint8_t s_BulkRx[USBD_BULK_RXMEM_SIZE(2)];
alignas(4) static uint8_t s_BulkTx[CFIFO_MEMSIZE(128)];
static const uint8_t s_ReportDesc[] = {
	0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01,
	0x75, 0x08, 0x95, 0x40, 0x09, 0x01, 0x81, 0x02,
	0x09, 0x01, 0x91, 0x02, 0xC0
};
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];
alignas(4) static uint8_t s_CdcRx[USB_INTRF_RXMEM_SIZE(2, 64)];
alignas(4) static uint8_t s_CdcTx[CFIFO_MEMSIZE(128)];
alignas(4) static uint8_t s_HidRx[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_HidTx[USB_INT_INTRF_PKT_BLKSIZE];

static UsbCfg_t s_UsbCfg = {
	.DevNo = 0,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0005,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "LPC CDC/HID/bulk link fixture",
	.IntPrio = 6,
	.DeviceClass = USB_DEVCLASS_MISC,
	.DeviceSubClass = 2,
	.DeviceProtocol = 1,
	.MaxPower = 100,
};
static const UsbdCdcCfg_t s_CdcCfg = {
	.DevNo = 0,
	.bBlocking = true,
	.RxFifoMemSize = sizeof(s_CdcRx),
	.pRxFifoMem = s_CdcRx,
	.TxFifoMemSize = sizeof(s_CdcTx),
	.pTxFifoMem = s_CdcTx,
};
static const UsbdBulkCfg_t s_BulkCfg = {
	.DevNo = 0,
	.bBlocking = true,
	.RxFifoMemSize = sizeof(s_BulkRx),
	.pRxFifoMem = s_BulkRx,
	.TxFifoMemSize = sizeof(s_BulkTx),
	.pTxFifoMem = s_BulkTx,
};
static const UsbdHidCfg_t s_HidCfg = {
	.DevNo = 0,
	.pReportDesc = s_ReportDesc,
	.ReportDescLength = sizeof(s_ReportDesc),
	.pRxBuffer = s_HidRx,
	.pTxBuffer = s_HidTx,
};


extern "C" void Test17Init(UsbCfg_t *pConfig);
extern "C" void Test17Setup(const UsbSetupData_t *pSetup);
extern "C" void Test17Out(unsigned EpNo, const uint8_t *pData, unsigned Length);
extern "C" void Test17InDrain(unsigned EpNo, const uint8_t *pExpected, unsigned Length);

static void CheckDescriptors(void)
{
	uint16_t length = 0;
	const uint8_t *desc = UsbGetDescriptor(0, USB_DESCTYPE_CONFIGURATION, 0, 0,
		USB_SPEED_FULL, &length);
	assert(desc != nullptr && length >= 9);
	unsigned endpoints = 0;
	for (unsigned offset = 0; offset < length; offset += desc[offset])
	{
		assert(desc[offset] >= 2 && offset + desc[offset] <= length);
		if (desc[offset + 1] != USB_DESCTYPE_ENDPOINT)
			continue;
		const unsigned ep = desc[offset + 2] & 15;
		const unsigned type = desc[offset + 3] & 3;
		assert((USB_EPIN_TYPE_MASK(0, type) & (1U << ep)) != 0);
		++endpoints;
	}
	assert(endpoints == 7);
}

static void Transfer(DeviceIntrf &Intrf, unsigned EpNo, unsigned Length)
{
	uint8_t payload[129];
	uint8_t received[129];
	for (unsigned i = 0; i < Length; ++i)
		payload[i] = i ^ EpNo;
	unsigned transferred = 0;
	while (transferred < Length)
	{
		unsigned packet = Length - transferred;
		if (packet > 64)
			packet = 64;
		Test17Out(EpNo, payload + transferred, packet);
		assert(Intrf.Rx(0, received + transferred, packet) == (int)packet);
		transferred += packet;
	}
	assert(std::memcmp(payload, received, Length) == 0);
	transferred = 0;
	while (transferred < Length)
	{
		const int sent = Intrf.Tx(0, payload + transferred, Length - transferred);
		assert(sent > 0 && sent <= (int)(Length - transferred));
		Test17InDrain(EpNo, payload + transferred, sent);
		transferred += sent;
	}
}

static void AllocationLimits(unsigned Type)
{
	UsbdEpAllocReq_t request = {};
	request.InterfaceCount = 1;
	request.BidirectionalCount = 1;
	request.BidirectionalType = USB_ENDPATT_TRANS_ISO;
	UsbdEpAllocRes_t result;
	std::memset(&result, 0xCC, sizeof(result));
	assert(!UsbdEpAlloc(0, &request, &s_Allocation[0], &result));
	assert(result.FirstInterface == 0xCC);
	request.BidirectionalType = Type;
	const unsigned mask = USB_EPIN_TYPE_MASK(0, Type);
	unsigned allocated = 0;
	unsigned index = 0;
	for (unsigned ep = 1; ep < 16; ++ep)
	{
		if ((mask & (1U << ep)) == 0)
			continue;
		assert(UsbdEpAlloc(0, &request, &s_Allocation[index], &result));
		assert(result.Bidirectional[0] == ep && result.FirstInterface == index);
		allocated |= 1U << ep;
		++index;
	}
	assert(allocated == mask);
	std::memset(&result, 0xCC, sizeof(result));
	assert(!UsbdEpAlloc(0, &request, &s_Allocation[index], &result));
	assert(result.FirstInterface == 0xCC && result.Bidirectional[0] == 0xCC);
	std::printf("LPC1769 endpoint type %u exhaustion and unsupported ISO: PASS\n", Type);
}

int main(int argc, char **argv)
{
	assert(argc == 2);
	unsigned order = std::strtoul(argv[1], nullptr, 10);
	assert(order < 8);
	const unsigned permutations[6][3] = {
		{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}
	};
	Test17Init(&s_UsbCfg);
	assert(AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)));
	assert(UsbInit(&s_UsbCfg));
	if (order >= 6)
	{
		AllocationLimits(order == 6 ? BULK : INT);
		return 0;
	}
	for (unsigned i = 0; i < 3; ++i)
	{
		switch (permutations[order][i])
		{
			case 0:
				assert(s_Cdc.Init(s_CdcCfg));
				break;
			case 1:
				assert(s_Hid.Init(s_HidCfg));
				break;
			case 2:
				assert(s_Bulk.Init(s_BulkCfg));
				break;
		}
	}
	CheckDescriptors();
	const UsbdCdcDev_t *cdc = s_Cdc;
	const UsbdHidDev_t *hid = s_Hid;
	const UsbdBulkDev_t *bulk = s_Bulk;
	assert(cdc->DataEpNo != bulk->EpNo);
	assert(cdc->NotifyEpNo != hid->EpNo);
	assert(UsbEnable(0));
	UsbProcess(0);
	UsbSetupData_t setup = {0, 5, 7, 0, 0};
	Test17Setup(&setup);
	assert(UsbGetAddress(0) == 7);
	for (unsigned cycle = 0; cycle < 3; ++cycle)
	{
		setup = {0, 9, 1, 0, 0};
		Test17Setup(&setup);
		assert(UsbConfigured(0));
		setup = {0x21, 0x22, 3, (uint16_t)cdc->CtrlIfNo, 0};
		Test17Setup(&setup);
		assert(s_Cdc.IsPortOpen());
		const unsigned lengths[] = {1, 7, 63, 64, 65, 127, 128, 129};
		for (unsigned length : lengths)
		{
			Transfer(s_Cdc, cdc->DataEpNo, length);
			Transfer(s_Bulk, bulk->EpNo, length);
		}
		Transfer(s_Hid, hid->EpNo, 64);
		assert(UsbEpSetHalt(0, bulk->EpNo, true, true));
		assert(UsbEpHalted(0, bulk->EpNo, true));
		assert(UsbEpSetHalt(0, bulk->EpNo, true, false));
		Transfer(s_Bulk, bulk->EpNo, 65);
		setup = {0, 9, 0, 0, 0};
		Test17Setup(&setup);
		assert(!UsbConfigured(0));
	}
	UsbDisable(0);
	std::printf("LPC1769 real CDC/HID/bulk allocation order %u, transfers and reconfiguration: PASS\n", order);
}
