#include <stdio.h>
#include <string.h>

#include "usb/usb_intrf.h"

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
void UsbCtrlrProcess(int) {}
bool UsbCtrlrVbusDetected(int) { return true; }
bool UsbCtrlrHighSpeed(int) { return false; }
void UsbCtrlrIntEnable(int) {}
void UsbCtrlrIntDisable(int) {}
void UsbCtrlrConnect(int) {}
void UsbCtrlrDisconnect(int) {}
void UsbCtrlrRemoteWakeup(int) {}
void UsbCtrlrSofEnable(int, bool) {}
void UsbCtrlrSetAddress(int, uint8_t) {}
void UsbCtrlrEpStall(int, uint8_t) {}
void UsbCtrlrEpClearStall(int, uint8_t) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }
bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *) { return true; }
void UsbCtrlrEpClose(int, uint8_t) {}
void UsbCtrlrEpCloseAll(int) {}

bool UsbCtrlrEpRegister(int, uint8_t EpAddr, uint8_t *pBuffer, bool,
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

bool UsbCtrlrEpXfer(int, uint8_t EpAddr, uint16_t Length)
{
	if (!s_XferOk)
	{
		return false;
	}
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		s_InLength = Length;
	}
	else
	{
		s_OutXferCount++;
	}
	return true;
}
bool UsbCtrlrEp0Xfer(int, uint8_t, uint8_t *, uint16_t) { return true; }
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
				 bool Blocking = false, bool Prearm = false)
{
	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = 3U;
	cfg.bBlocking = Blocking;
	cfg.bRxPrearm = Prearm;
	cfg.Mode = USB_INTRF_MODE_DIRECT;
	cfg.BufferSize = 16U;
	cfg.pRxBuffer = reinterpret_cast<uint8_t *>(pRx);
	cfg.pTxBuffer = reinterpret_cast<uint8_t *>(pTx);
	cfg.EvtCB = KeepRx;
	return UsbIntrfInit(pIntrf, &cfg);
}

static void RxComplete(const uint8_t *pData, uint16_t Length,
					   UsbCtrlrXferResult_t Result = USB_CTRLR_XFER_SUCCESS)
{
	if (Length > 0U)
	{
		memcpy(s_OutBuffer, pData, Length);
	}
	s_OutHandler(USB_ENDPADDR_DIROUT(3U), USB_CTRLR_EVT_XFER_CMPL,
		Length, Result, s_OutContext);
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

	RxComplete(nullptr, 0U, USB_CTRLR_XFER_FAILED);
	CHECK(intrf.RxDropCnt == 2U);
}

static void TestDrdyAndPrearm(void)
{
	ResetFake();
	UsbDevIntrf_t intrf = {};
	alignas(4) uint32_t rx[5] = {};
	alignas(4) uint32_t tx[5] = {};
	CHECK(Init(&intrf, rx, tx, true));
	CHECK(UsbIntrfConfigure(&intrf, 8U));
	s_OutHandler(USB_ENDPADDR_DIROUT(3U), USB_CTRLR_EVT_DRDY, 0U,
		USB_CTRLR_XFER_SUCCESS, s_OutContext);
	CHECK(s_OutXferCount == 1);

	ResetFake();
	UsbDevIntrf_t prearm = {};
	CHECK(Init(&prearm, rx, tx, false, true));
	CHECK(UsbIntrfConfigure(&prearm, 8U));
	CHECK(UsbIntrfArmRx(&prearm));
	CHECK(s_OutXferCount == 1);
	const uint8_t data = 7U;
	RxComplete(&data, 1U);
	CHECK(prearm.RxPending);
	uint8_t out = 0U;
	CHECK(DeviceIntrfRxData(&prearm.DevIntrf, &out, 1) == 1);
	CHECK(out == data && s_OutXferCount == 2 && !prearm.RxPending);
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
	s_InHandler(USB_ENDPADDR_DIRIN(3U), USB_CTRLR_EVT_XFER_CMPL, 3U,
		USB_CTRLR_XFER_SUCCESS, s_InContext);
	CHECK(UsbIntrfRequestToSend(&intrf, 0));
	CHECK(DeviceIntrfTxData(&intrf.DevIntrf, nullptr, 0) == 0);
	CHECK(!atomic_load(&intrf.DevIntrf.bTxReady));
	s_InHandler(USB_ENDPADDR_DIRIN(3U), USB_CTRLR_EVT_CANCEL, 0U,
		USB_CTRLR_XFER_CANCELLED, s_InContext);
	CHECK(atomic_load(&intrf.DevIntrf.bTxReady));

	s_XferOk = false;
	CHECK(DeviceIntrfTxData(&intrf.DevIntrf, data, 1) == 0);
	CHECK(atomic_load(&intrf.DevIntrf.bTxReady));
}

int main(void)
{
	TestValidation();
	TestRxOwnershipAndDrop();
	TestDrdyAndPrearm();
	TestTx();
	printf("%s\n", s_Fail == 0 ? "usb_direct_intrf_test: PASS" :
		"usb_direct_intrf_test: FAIL");
	return s_Fail == 0 ? 0 : 1;
}
