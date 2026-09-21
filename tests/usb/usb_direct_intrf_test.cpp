#include <stdio.h>
#include <string.h>
#include <initializer_list>

#include "usb/usb_intrf.h"
#include "app_evt_handler.h"

static uint8_t *s_OutBuffer;
static uint8_t *s_InBuffer;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static bool s_XferOk = true;
static uint16_t s_InLength;
static int s_OutXferCount;

extern "C" {
bool UsbCtrlrInit(int, const UsbCtrlrCfg_t *) { return true; }
bool UsbCtrlrStart(int) { return true; }
void UsbCtrlrStop(int) {}
void UsbCtrlrProcess(int)
{
	AppEvtHandlerExec();
	if (s_OutBuffer == nullptr && s_OutHandler != nullptr)
		s_OutHandler(USB_CTRLR_EVT_DRDY, 0U, s_OutContext);
}
bool UsbCtrlrVbusDetected(int) { return true; }
bool UsbCtrlrHighSpeed(int) { return false; }
void UsbCtrlrIntEnable(int) {}
void UsbCtrlrIntDisable(int) {}
void UsbCtrlrConnect(int) {}
void UsbCtrlrDisconnect(int) {}
void UsbCtrlrRemoteWakeup(int) {}
void UsbCtrlrSofEnable(int, bool) {}
void UsbCtrlrSetAddress(int, uint8_t) {}
void UsbCtrlrEpStall(int, uint8_t, bool) {}
void UsbCtrlrEpClearStall(int, uint8_t, bool) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }
bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *) { return true; }
void UsbCtrlrEpClose(int, uint8_t, bool) {}
void UsbCtrlrEpCloseAll(int) {}

void UsbCtrlrEpAlloc(int, uint8_t, bool bIn, uint8_t *pBuffer, bool,
						UsbCtrlrEpHandler_t Handler, void *pContext)
{
	if (bIn)
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
	return;
}

bool UsbCtrlrEpSend(int, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	if ((EpNum & 0x80U) != 0U) return false;
	if (!s_XferOk) return false;
	s_InBuffer = pBuffer;
	s_InLength = Length;
	return true;
}
}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

static int KeepRx(DevIntrf_t *, DEVINTRF_EVT Event, uint8_t *, int)
{
	return Event == DEVINTRF_EVT_RX_DATA ? -1 : 0;
}

static void ResetFake(void)
{
	CHECK(AppEvtHandlerInit(nullptr, 0));
	s_OutBuffer = nullptr;
	s_InBuffer = nullptr;
	s_OutHandler = nullptr;
	s_InHandler = nullptr;
	s_OutContext = nullptr;
	s_InContext = nullptr;
	s_XferOk = true;
	s_InLength = 0U;
	s_OutXferCount = 0;
}

static bool Init(UsbDevIntrf_t *pIntrf, uint32_t *pRx, uint32_t *pTx,
				 bool Blocking = false)
{
	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = 3U;
	cfg.bBlocking = Blocking;
	cfg.Mode = USB_INTRF_MODE_DIRECT;
	cfg.BufferSize = 16U;
	cfg.pRxBuffer = reinterpret_cast<uint8_t *>(pRx);
	cfg.pTxBuffer = reinterpret_cast<uint8_t *>(pTx);
	cfg.EvtCB = KeepRx;
	return UsbIntrfInit(pIntrf, &cfg);
}

static void RxComplete(const uint8_t *pData, uint16_t Length,
					   UsbCtrlrEvtType_t Event = USB_CTRLR_EVT_XFER_CMPL)
{
	if (Length > 0U)
	{
		memcpy(s_OutBuffer, pData, Length);
	}
	s_OutHandler(Event,
		Length, s_OutContext);
}

static void TestValidation(void)
{
	ResetFake();
	UsbDevIntrf_t intrf = {};
	alignas(4) uint32_t rx[5] = {};
	alignas(4) uint32_t tx[5] = {};
	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = 3U;
	cfg.Mode = USB_INTRF_MODE_DIRECT;
	cfg.BufferSize = 16U;
	cfg.pRxBuffer = reinterpret_cast<uint8_t *>(rx);
	cfg.pTxBuffer = reinterpret_cast<uint8_t *>(tx);
	CHECK(UsbIntrfInit(&intrf, &cfg));
	CHECK(intrf.hRxFifo == nullptr && intrf.hTxFifo == nullptr);
	CHECK(intrf.pRxDirectBuffer != nullptr && intrf.pTxDirectBuffer != nullptr);
	CHECK(!UsbIntrfConfigure(&intrf, 0U));
	CHECK(!UsbIntrfConfigure(&intrf, 17U));
	CHECK(UsbIntrfConfigure(&intrf, 16U));
}

static void TestRxOwnershipAndDrop(void)
{
	ResetFake();
	UsbDevIntrf_t intrf = {};
	alignas(4) uint32_t rx[5] = {};
	alignas(4) uint32_t tx[5] = {};
	CHECK(Init(&intrf, rx, tx));
	CHECK(UsbIntrfConfigure(&intrf, 16U));

	const uint8_t first[] = {1U, 2U};
	const uint8_t second[] = {3U, 4U, 5U};
	RxComplete(first, sizeof(first));
	CHECK((intrf.pRxDirectBuffer->Hdr.Flags & USB_INTRF_SLOT_READY) != 0U);
	RxComplete(second, sizeof(second));
	CHECK(intrf.RxDropCnt == 1U);

	uint8_t out[3] = {};
	CHECK(DeviceIntrfRxData(&intrf.DevIntrf, out, sizeof(out)) == 3);
	CHECK(memcmp(out, second, sizeof(second)) == 0);
	CHECK((intrf.pRxDirectBuffer->Hdr.Flags & USB_INTRF_SLOT_READY) == 0U);

	RxComplete(nullptr, 0U);
	CHECK((intrf.pRxDirectBuffer->Hdr.Flags & USB_INTRF_SLOT_READY) != 0U);
	CHECK(DeviceIntrfRxData(&intrf.DevIntrf, nullptr, 0) == 0);
	CHECK((intrf.pRxDirectBuffer->Hdr.Flags & USB_INTRF_SLOT_READY) == 0U);

	RxComplete(nullptr, 0U, USB_CTRLR_EVT_XFER_FAILED);
	CHECK(intrf.RxDropCnt == 2U);
}

static void TestDrdyPolicy(void)
{
	ResetFake();
	UsbDevIntrf_t intrf = {};
	alignas(4) uint32_t rx[5] = {};
	alignas(4) uint32_t tx[5] = {};
	CHECK(Init(&intrf, rx, tx, true));
	CHECK(UsbIntrfConfigure(&intrf, 8U));
	s_OutHandler(USB_CTRLR_EVT_DRDY, 0U, s_OutContext);
	CHECK(s_OutBuffer != nullptr && s_OutXferCount == 0);
	for (bool fullEvents : {false, true})
	{
		const uint8_t packet[] = {1, 2, 3};
		RxComplete(packet, sizeof(packet));
		if (fullEvents)
			while (AppEvtHandlerQue(0U, nullptr, [](uint32_t, void *) {})) {}
		s_OutHandler(USB_CTRLR_EVT_DRDY, 0U, s_OutContext);
		CHECK(s_OutBuffer == nullptr && intrf.RxPending);
		const int submits = s_OutXferCount;
		uint8_t output[3];
		CHECK(DeviceIntrfRxData(&intrf.DevIntrf, output, sizeof(output)) == 3);
		CHECK(memcmp(output, packet, sizeof(packet)) == 0);
		CHECK(s_OutBuffer == nullptr && intrf.RxPending);
		CHECK(s_OutXferCount == submits);
		UsbCtrlrProcess(0);
		CHECK(!intrf.RxPending && s_OutBuffer == intrf.pRxBuffer);
		CHECK(s_OutXferCount == submits);
		UsbCtrlrProcess(0);
		CHECK(s_OutXferCount == submits);
	}

	ResetFake();
	UsbDevIntrf_t nonblocking = {};
	CHECK(Init(&nonblocking, rx, tx, false));
	CHECK(UsbIntrfConfigure(&nonblocking, 8U));
	s_OutHandler(USB_CTRLR_EVT_DRDY, 0U, s_OutContext);
	CHECK(s_OutXferCount == 0);
}

static void TestTx(void)
{
	ResetFake();
	UsbDevIntrf_t intrf = {};
	alignas(4) uint32_t rx[5] = {};
	alignas(4) uint32_t tx[5] = {};
	CHECK(Init(&intrf, rx, tx));
	CHECK(UsbIntrfConfigure(&intrf, 8U));
	const uint8_t data[] = {9U, 8U, 7U};
	CHECK(DeviceIntrfTxData(&intrf.DevIntrf, data, sizeof(data)) == 3);
	CHECK(s_InLength == 3U && memcmp(s_InBuffer, data, sizeof(data)) == 0);
	CHECK(!UsbIntrfRequestToSend(&intrf, 1));
	s_InHandler(USB_CTRLR_EVT_XFER_CMPL, 3U, s_InContext);
	CHECK(UsbIntrfRequestToSend(&intrf, 0));
	CHECK(DeviceIntrfTxData(&intrf.DevIntrf, nullptr, 0) == 0);
	CHECK(!atomic_load(&intrf.DevIntrf.bTxReady));
	s_InHandler(USB_CTRLR_EVT_CANCEL, 0U, s_InContext);
	CHECK(atomic_load(&intrf.DevIntrf.bTxReady));

	s_XferOk = false;
	CHECK(DeviceIntrfTxData(&intrf.DevIntrf, data, 1) == 0);
	CHECK(atomic_load(&intrf.DevIntrf.bTxReady));
}

int main(void)
{
	TestValidation();
	TestRxOwnershipAndDrop();
	TestDrdyPolicy();
	TestTx();
	printf("%s\n", s_Fail == 0 ? "usb_direct_intrf_test: PASS" :
		"usb_direct_intrf_test: FAIL");
	return s_Fail == 0 ? 0 : 1;
}
