/**-------------------------------------------------------------------------
@file	usbd_hci_test.cpp

@brief	Host tests for the Bluetooth HCI USB function.

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

#include "usb/usbd_hci.h"

#define RX_SLOTS		8U
#define TX_SLOTS		20U

typedef struct {
	uint8_t EpAddr;
	uint8_t *pBuffer;
	UsbCtrlrEpHandler_t Handler;
	void *pContext;
} RegisteredEp_t;

typedef struct {
	uint8_t EpAddr;
	uint16_t Length;
	uint8_t Data[USBD_HCI_ACL_MAX_MPS];
} SentPacket_t;

static UsbCfg_t s_UsbCfg;
static UsbFuncCfg_t s_FuncCfg;
static bool s_FuncRegistered;
static uint8_t s_ReservedFirst;
static uint8_t s_ReservedCount;
static uint16_t s_ReservedIn;
static uint16_t s_ReservedOut;
static UsbEndPointDesc_t s_OpenDesc[3];
static int s_OpenCount;
static int s_OpenFailAt;
static int s_CloseCount;
static RegisteredEp_t s_Registered[3];
static int s_RegisteredCount;
static bool s_OutArmed;
static bool s_InBusy[16];
static SentPacket_t s_Sent[32];
static int s_SendCount;
static int s_RxEventCount;
static int s_TxEventCount;
static DEVINTRF_EVT s_LastEvent;
static DevIntrf_t *s_LastEventDev;

extern "C" {
const UsbCfg_t *UsbGetCfg(int DevNo)
{
	return DevNo == 0 ? &s_UsbCfg : nullptr;
}

bool UsbRegisterFunc(int DevNo, const UsbFuncCfg_t *pCfg)
{
	if (DevNo != 0 || pCfg == nullptr ||
		(((pCfg->EpInMask | pCfg->EpOutMask) & 1U) != 0U) ||
		((pCfg->EpInMask | pCfg->EpOutMask) != 0U &&
		 pCfg->XferHandler == nullptr))
	{
		return false;
	}

	if (pCfg->InterfaceCount != 0U && s_ReservedCount != 0U)
	{
		const uint16_t firstA = pCfg->FirstInterface;
		const uint16_t lastA = firstA + pCfg->InterfaceCount;
		const uint16_t firstB = s_ReservedFirst;
		const uint16_t lastB = firstB + s_ReservedCount;

		if (firstA < lastB && firstB < lastA)
		{
			return false;
		}
	}

	if ((pCfg->EpInMask & s_ReservedIn) != 0U ||
		(pCfg->EpOutMask & s_ReservedOut) != 0U)
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
	if (pDesc == nullptr || s_OpenCount >= 3 ||
		s_OpenCount == s_OpenFailAt)
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
	if (pBuffer == nullptr || Handler == nullptr || s_RegisteredCount >= 3)
	{
		return false;
	}
	s_Registered[s_RegisteredCount++] = { EpAddr, pBuffer, Handler, pContext };
	return true;
}

bool UsbCtrlrEpRxArm(int, uint8_t EpNo)
{
	if (EpNo != 2U || s_OutArmed)
	{
		return false;
	}
	s_OutArmed = true;
	return true;
}

bool UsbCtrlrEpSend(int, uint8_t EpNo, uint16_t Length)
{
	if (EpNo >= 16U || s_InBusy[EpNo] || s_SendCount >= 32)
	{
		return false;
	}

	RegisteredEp_t *pReg = nullptr;
	for (int i = 0; i < s_RegisteredCount; i++)
	{
		if (s_Registered[i].EpAddr == USB_ENDPADDR_DIRIN(EpNo))
		{
			pReg = &s_Registered[i];
			break;
		}
	}
	if (pReg == nullptr || Length > sizeof(s_Sent[0].Data))
	{
		return false;
	}

	SentPacket_t *pSent = &s_Sent[s_SendCount++];
	pSent->EpAddr = USB_ENDPADDR_DIRIN(EpNo);
	pSent->Length = Length;
	if (Length > 0U)
	{
		memcpy(pSent->Data, pReg->pBuffer, Length);
	}
	s_InBusy[EpNo] = true;
	return true;
}
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

alignas(4) static uint8_t s_RxMem[USBD_HCI_ACL_RXMEM_SIZE(RX_SLOTS)];
alignas(4) static uint8_t s_TxMem[USBD_HCI_ACL_TXMEM_SIZE(TX_SLOTS)];

static int HciEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Evt,
					uint8_t *, int Length)
{
	s_LastEvent = Evt;
	s_LastEventDev = pDev;
	if (Evt == DEVINTRF_EVT_RX_DATA)
	{
		s_RxEventCount++;
	}
	if (Evt == DEVINTRF_EVT_TX_READY ||
		Evt == DEVINTRF_EVT_TX_FIFO_EMPTY)
	{
		s_TxEventCount++;
	}
	return Length;
}

static UsbdHciCfg_t MakeCfg(void)
{
	UsbdHciCfg_t cfg = {};
	cfg.bBlocking = true;
	cfg.RxFifoMemSize = sizeof(s_RxMem);
	cfg.pRxFifoMem = s_RxMem;
	cfg.TxFifoMemSize = sizeof(s_TxMem);
	cfg.pTxFifoMem = s_TxMem;
	cfg.DevNo = 0;
	cfg.InterfaceString = 4U;
	cfg.EvtCB = HciEvent;
	return cfg;
}

static RegisteredEp_t *FindRegistered(uint8_t EpAddr)
{
	for (int i = 0; i < s_RegisteredCount; i++)
	{
		if (s_Registered[i].EpAddr == EpAddr)
		{
			return &s_Registered[i];
		}
	}
	return nullptr;
}

static void CompleteIn(uint8_t EpNo)
{
	RegisteredEp_t *pReg = FindRegistered(USB_ENDPADDR_DIRIN(EpNo));
	CHECK(pReg != nullptr);
	CHECK(s_InBusy[EpNo]);
	if (pReg == nullptr || !s_InBusy[EpNo])
	{
		return;
	}
	s_InBusy[EpNo] = false;
	const uint16_t length = s_Sent[s_SendCount - 1].Length;
	pReg->Handler(USB_ENDPADDR_DIRIN(EpNo), length,
				  USB_CTRLR_XFER_SUCCESS, pReg->pContext);
}

static void ReceiveOut(const uint8_t *pData, uint16_t Length)
{
	RegisteredEp_t *pReg = FindRegistered(USB_ENDPADDR_DIROUT(2U));
	CHECK(pReg != nullptr);
	CHECK(s_OutArmed);
	if (pReg == nullptr || !s_OutArmed)
	{
		return;
	}
	memcpy(pReg->pBuffer, pData, Length);
	s_OutArmed = false;
	pReg->Handler(USB_ENDPADDR_DIROUT(2U), Length,
				  USB_CTRLR_XFER_SUCCESS, pReg->pContext);
}

static void ResetFake(void)
{
	memset(&s_UsbCfg, 0, sizeof(s_UsbCfg));
	memset(&s_FuncCfg, 0, sizeof(s_FuncCfg));
	memset(s_OpenDesc, 0, sizeof(s_OpenDesc));
	memset(s_Registered, 0, sizeof(s_Registered));
	memset(s_InBusy, 0, sizeof(s_InBusy));
	memset(s_Sent, 0, sizeof(s_Sent));
	s_FuncRegistered = false;
	s_ReservedFirst = 0U;
	s_ReservedCount = 0U;
	s_ReservedIn = 0U;
	s_ReservedOut = 0U;
	s_OpenCount = 0;
	s_OpenFailAt = -1;
	s_CloseCount = 0;
	s_RegisteredCount = 0;
	s_OutArmed = false;
	s_SendCount = 0;
	s_RxEventCount = 0;
	s_TxEventCount = 0;
	s_LastEvent = DEVINTRF_EVT_RX_TIMEOUT;
	s_LastEventDev = nullptr;
	s_UsbCfg.DevNo = 0;
}

static void TestDescriptor(void)
{
	ResetFake();
	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();
	UsbdHciDesc_t desc = {};

	CHECK(hci.Init(cfg));
	CHECK(hci.MakeDesc(&desc, USB_SPEED_FULL));
	CHECK(sizeof(desc) == sizeof(UsbInrtfAssDesc_t) +
		2U * sizeof(UsbIntrfDesc_t) + 3U * sizeof(UsbEndPointDesc_t));
	CHECK(desc.Association.bDescriptorType == USB_DESCTYPE_IA);
	CHECK(desc.Association.bFirstInterface == 0U);
	CHECK(desc.Association.bInterfaceCount == 2U);
	CHECK(desc.Association.bFunctionClass == USB_INTRFCLASS_WIRELESS);
	CHECK(desc.Association.bFunctionSubClass == USBD_HCI_SUBCLASS_RF);
	CHECK(desc.Association.bFunctionProtocol == USBD_HCI_PROTOCOL_BT);
	CHECK(desc.Hci.bInterfaceNumber == 0U);
	CHECK(desc.Hci.bNumEndpoints == 3U);
	CHECK(desc.Hci.bInterfaceClass == USB_INTRFCLASS_WIRELESS);
	CHECK(desc.EventIn.bEndpointAddress == USB_ENDPADDR_DIRIN(1U));
	CHECK(desc.EventIn.bmAttributes == USB_ENDPATT_TRANS_INT);
	CHECK(desc.EventIn.wMaxPacketSize == USBD_HCI_EVENT_FS_MPS);
	CHECK(desc.AclOut.bEndpointAddress == USB_ENDPADDR_DIROUT(2U));
	CHECK(desc.AclIn.bEndpointAddress == USB_ENDPADDR_DIRIN(2U));
	CHECK(desc.AclOut.bmAttributes == USB_ENDPATT_TRANS_BULK);
	CHECK(desc.AclOut.wMaxPacketSize == USBD_HCI_ACL_FS_MPS);
	CHECK(desc.Sync.bInterfaceNumber == 1U);
	CHECK(desc.Sync.bNumEndpoints == 0U);
}

static void TestAutoPlacement(void)
{
	ResetFake();
	s_ReservedFirst = 0U;
	s_ReservedCount = 2U;
	s_ReservedIn = (uint16_t)(1U << 1);
	s_ReservedOut = (uint16_t)(1U << 1);

	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();
	UsbdHciDesc_t desc = {};

	CHECK(hci.Init(cfg));
	CHECK(s_FuncRegistered);
	CHECK(s_FuncCfg.XferHandler != nullptr);
	CHECK(s_FuncCfg.FirstInterface == 2U);
	CHECK(s_FuncCfg.InterfaceCount == 2U);
	CHECK(s_FuncCfg.EpInMask == ((1U << 2) | (1U << 3)));
	CHECK(s_FuncCfg.EpOutMask == (1U << 3));
	CHECK(hci.MakeDesc(&desc, USB_SPEED_FULL));
	CHECK(desc.Association.bFirstInterface == 2U);
	CHECK(desc.Hci.bInterfaceNumber == 2U);
	CHECK(desc.Sync.bInterfaceNumber == 3U);
	CHECK(desc.EventIn.bEndpointAddress == USB_ENDPADDR_DIRIN(2U));
	CHECK(desc.AclOut.bEndpointAddress == USB_ENDPADDR_DIROUT(3U));
	CHECK(desc.AclIn.bEndpointAddress == USB_ENDPADDR_DIRIN(3U));
}

static void TestConfiguration(void)
{
	ResetFake();
	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();

	CHECK(hci.Init(cfg));
	CHECK(s_RegisteredCount == 3);
	CHECK(s_Registered[0].EpAddr == USB_ENDPADDR_DIROUT(2U));
	CHECK(s_Registered[1].EpAddr == USB_ENDPADDR_DIRIN(2U));
	CHECK(s_Registered[2].EpAddr == USB_ENDPADDR_DIRIN(1U));
	CHECK(s_FuncCfg.ConfigHandler(USBD_HCI_CONFIG_VALUE, s_FuncCfg.pContext));
	CHECK(s_OpenCount == 3);
	CHECK(s_OpenDesc[0].bEndpointAddress == USB_ENDPADDR_DIRIN(1U));
	CHECK(s_OpenDesc[1].bEndpointAddress == USB_ENDPADDR_DIROUT(2U));
	CHECK(s_OpenDesc[2].bEndpointAddress == USB_ENDPADDR_DIRIN(2U));
	CHECK(s_OutArmed);
	CHECK(DeviceIntrfGetRate(hci.Data()) == USB_LINK_RATE_FULL);
	CHECK(s_FuncCfg.SetInterfaceHandler(0U, 0U, s_FuncCfg.pContext));
	CHECK(s_FuncCfg.SetInterfaceHandler(1U, 0U, s_FuncCfg.pContext));
	CHECK(!s_FuncCfg.SetInterfaceHandler(0U, 1U, s_FuncCfg.pContext));
	CHECK(s_FuncCfg.ConfigHandler(0U, s_FuncCfg.pContext));
	CHECK(DeviceIntrfGetRate(hci.Data()) == 0U);
	CHECK(!s_FuncCfg.SetInterfaceHandler(0U, 0U, s_FuncCfg.pContext));

	s_FuncCfg.ResetHandler(s_FuncCfg.pContext);
	CHECK(DeviceIntrfGetRate(hci.Data()) == 0U);
}

static void TestConfigurationFailure(void)
{
	ResetFake();
	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();
	CHECK(hci.Init(cfg));
	s_OpenFailAt = 1;
	CHECK(!s_FuncCfg.ConfigHandler(USBD_HCI_CONFIG_VALUE,
								   s_FuncCfg.pContext));
	CHECK(s_CloseCount == 3);
	CHECK(DeviceIntrfGetRate(hci.Data()) == 0U);
}

static void TestCommand(void)
{
	ResetFake();
	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();
	CHECK(hci.Init(cfg));
	CHECK(s_FuncCfg.ConfigHandler(1U, s_FuncCfg.pContext));

	UsbSetupData_t setup = {};
	setup.bmRequestType = USB_REQTYPE_CLASS | USB_REQTYPE_INTERFACE;
	setup.bRequest = 0U;
	setup.wValue = 0U;
	setup.wIndex = 0U;
	setup.wLength = 3U;
	uint8_t *pData = nullptr;
	uint16_t length = 0U;
	CHECK(s_FuncCfg.RequestHandler(&setup, USB_CTRL_SETUP, &pData, &length,
								   s_FuncCfg.pContext));
	CHECK(pData != nullptr && length == setup.wLength);
	const uint8_t command[] = { 0x03U, 0x0CU, 0x00U };
	memcpy(pData, command, sizeof(command));
	CHECK(s_FuncCfg.RequestHandler(&setup, USB_CTRL_DATA, &pData, &length,
								   s_FuncCfg.pContext));
	CHECK(s_FuncCfg.RequestHandler(&setup, USB_CTRL_COMPLETE, &pData, &length,
								   s_FuncCfg.pContext));
	CHECK(s_RxEventCount == 1);
	CHECK(s_LastEventDev == hci.Data());
	CHECK(!s_FuncCfg.RequestHandler(&setup, USB_CTRL_SETUP, &pData, &length,
									s_FuncCfg.pContext));

	uint8_t received[8] = {};
	CHECK(DeviceIntrfRx(hci.Data(), USBD_HCI_PACKET_ACL,
						received, sizeof(received)) == 0);
	CHECK(DeviceIntrfRx(hci.Data(), USBD_HCI_PACKET_COMMAND,
						received, sizeof(received)) == 3);
	CHECK(memcmp(received, command, sizeof(command)) == 0);

	setup.wIndex = 1U;
	CHECK(!s_FuncCfg.RequestHandler(&setup, USB_CTRL_SETUP, &pData, &length,
									s_FuncCfg.pContext));
	setup.wIndex = 0U;
	setup.bRequest = 1U;
	CHECK(!s_FuncCfg.RequestHandler(&setup, USB_CTRL_SETUP, &pData, &length,
									s_FuncCfg.pContext));
	setup.bRequest = 0U;
	setup.wValue = 1U;
	CHECK(!s_FuncCfg.RequestHandler(&setup, USB_CTRL_SETUP, &pData, &length,
									s_FuncCfg.pContext));
	setup.wValue = 0U;
	setup.bmRequestType |= USB_REQTYPE_DIRHOST;
	CHECK(!s_FuncCfg.RequestHandler(&setup, USB_CTRL_SETUP, &pData, &length,
									s_FuncCfg.pContext));
}

static void TestAclReceive(void)
{
	ResetFake();
	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();
	CHECK(hci.Init(cfg));
	CHECK(s_FuncCfg.ConfigHandler(1U, s_FuncCfg.pContext));

	uint8_t packet[70] = {};
	packet[0] = 0x01U;
	packet[1] = 0x20U;
	packet[2] = 66U;
	packet[3] = 0U;
	for (unsigned i = 4U; i < sizeof(packet); i++)
	{
		packet[i] = (uint8_t)i;
	}

	ReceiveOut(packet, 64U);
	uint8_t received[sizeof(packet)] = {};
	CHECK(DeviceIntrfRx(hci.Data(), USBD_HCI_PACKET_ACL,
						received, sizeof(received)) == 0);
	ReceiveOut(&packet[64], 6U);
	CHECK(DeviceIntrfRx(hci.Data(), USBD_HCI_PACKET_ACL,
						received, sizeof(received)) == (int)sizeof(packet));
	CHECK(memcmp(received, packet, sizeof(packet)) == 0);
}

static void TestAclTransmit(void)
{
	ResetFake();
	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();
	CHECK(hci.Init(cfg));
	CHECK(s_FuncCfg.ConfigHandler(1U, s_FuncCfg.pContext));

	uint8_t packet[64] = {};
	packet[0] = 0x01U;
	packet[1] = 0x20U;
	packet[2] = 60U;
	packet[3] = 0U;
	for (unsigned i = 4U; i < sizeof(packet); i++)
	{
		packet[i] = (uint8_t)(0x80U + i);
	}

	CHECK(DeviceIntrfTx(hci.Data(), USBD_HCI_PACKET_ACL,
						packet, sizeof(packet)) == (int)sizeof(packet));
	CHECK(s_SendCount == 1);
	CHECK(s_Sent[0].EpAddr == USB_ENDPADDR_DIRIN(2U));
	CHECK(s_Sent[0].Length == 64U);
	CHECK(memcmp(s_Sent[0].Data, packet, sizeof(packet)) == 0);
	CompleteIn(2U);
	CHECK(s_SendCount == 2);
	CHECK(s_Sent[1].Length == 0U);
	CompleteIn(2U);
	CHECK(s_TxEventCount == 1);
}

static void TestEventTransmit(void)
{
	ResetFake();
	UsbdHci hci;
	const UsbdHciCfg_t cfg = MakeCfg();
	CHECK(hci.Init(cfg));
	CHECK(s_FuncCfg.ConfigHandler(1U, s_FuncCfg.pContext));

	uint8_t event[16] = {};
	event[0] = 0x0EU;
	event[1] = 14U;
	for (unsigned i = 2U; i < sizeof(event); i++)
	{
		event[i] = (uint8_t)i;
	}

	CHECK(DeviceIntrfTx(hci.Data(), USBD_HCI_PACKET_EVENT,
						event, sizeof(event)) == (int)sizeof(event));
	CHECK(s_SendCount == 1);
	CHECK(s_Sent[0].EpAddr == USB_ENDPADDR_DIRIN(1U));
	CHECK(s_Sent[0].Length == sizeof(event));
	CHECK(memcmp(s_Sent[0].Data, event, sizeof(event)) == 0);
	CompleteIn(1U);
	CHECK(s_SendCount == 2);
	CHECK(s_Sent[1].Length == 0U);
	CompleteIn(1U);
	CHECK(s_TxEventCount == 1);
	CHECK(s_LastEvent == DEVINTRF_EVT_TX_READY);

	uint8_t longEvent[18] = {};
	longEvent[0] = 0x3EU;
	longEvent[1] = 16U;
	CHECK(hci.Tx(USBD_HCI_PACKET_EVENT, longEvent, sizeof(longEvent)) ==
		(int)sizeof(longEvent));
	CHECK(s_SendCount == 3);
	CHECK(s_Sent[2].Length == USBD_HCI_EVENT_FS_MPS);
	CHECK(memcmp(s_Sent[2].Data, longEvent, USBD_HCI_EVENT_FS_MPS) == 0);
	CompleteIn(1U);
	CHECK(s_SendCount == 4);
	CHECK(s_Sent[3].Length == sizeof(longEvent) - USBD_HCI_EVENT_FS_MPS);
	CHECK(memcmp(s_Sent[3].Data, &longEvent[USBD_HCI_EVENT_FS_MPS],
			s_Sent[3].Length) == 0);
	CompleteIn(1U);
	CHECK(s_SendCount == 4);
	CHECK(s_TxEventCount == 2);

	event[1] = 13U;
	CHECK(DeviceIntrfTx(hci.Data(), USBD_HCI_PACKET_EVENT,
						event, sizeof(event)) == 0);
	CHECK(DeviceIntrfTx(hci.Data(), USBD_HCI_PACKET_SCO,
						event, sizeof(event)) == 0);
	CHECK(DeviceIntrfTx(hci.Data(), USBD_HCI_PACKET_ISO,
						event, sizeof(event)) == 0);
}

int main(void)
{
	TestDescriptor();
	TestAutoPlacement();
	TestConfiguration();
	TestConfigurationFailure();
	TestCommand();
	TestAclReceive();
	TestAclTransmit();
	TestEventTransmit();

	if (s_Fail != 0)
	{
		printf("%d failed\n", s_Fail);
		return 1;
	}

	printf("all pass\n");
	return 0;
}
