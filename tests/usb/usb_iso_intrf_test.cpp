#include <stdio.h>
#include <string.h>

#include "usb/usb_iso.h"

static uint8_t *s_OutBuffer;
static uint8_t *s_InBuffer;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static bool s_OutBlocking;
static bool s_InBusy;
static uint16_t s_InLength;
static uint8_t s_InData[USB_ISO_INTRF_MAX_MPS];
static UsbEndPointDesc_t s_Open[4];
static int s_OpenCount;
static int s_CloseCount;
static int s_OutXferCount;
static int s_InXferCount;
static bool s_XferOk = true;

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

bool UsbCtrlrEpRegister(int, uint8_t EpAddr, uint8_t *pBuffer, bool Blocking,
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
		s_OutBlocking = Blocking;
	}
	return pBuffer != nullptr && Handler != nullptr;
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	if (DevNo != 0 || pDesc == nullptr || s_OpenCount >= 4)
		return false;
	s_Open[s_OpenCount++] = *pDesc;
	return true;
}

void UsbCtrlrEpClose(int, uint8_t EpAddr)
{
	s_CloseCount++;
	if (USB_ENDPADDR_IS_IN(EpAddr))
		s_InBusy = false;
}
void UsbCtrlrEpCloseAll(int) {}

bool UsbCtrlrEpXfer(int, uint8_t EpAddr, uint16_t Length)
{
	if (!s_XferOk)
		return false;
	if (!USB_ENDPADDR_IS_IN(EpAddr))
	{
		s_OutXferCount++;
		return false;
	}
	if (s_InBusy || Length > sizeof(s_InData))
		return false;
	s_InBusy = true;
	s_InLength = Length;
	s_InXferCount++;
	if (Length > 0U)
		memcpy(s_InData, s_InBuffer, Length);
	return true;
}

bool UsbCtrlrEp0Xfer(int, uint8_t, uint8_t *, uint16_t) { return true; }
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
	if (Result == USB_CTRLR_XFER_SUCCESS && Length > 0U)
		memcpy(s_LastRx, pData, Length);
}

static void TxFrame(UsbIsoIntrf_t *, uint16_t Length,
					UsbCtrlrXferResult_t Result, void *)
{
	s_TxCount++;
	s_LastTxLength = Length;
	s_LastTxResult = Result;
}

static void ResetFake(void)
{
	s_OutBuffer = nullptr;
	s_InBuffer = nullptr;
	s_OutHandler = nullptr;
	s_InHandler = nullptr;
	s_OutContext = nullptr;
	s_InContext = nullptr;
	s_OutBlocking = true;
	s_InBusy = false;
	s_InLength = 0U;
	memset(s_InData, 0, sizeof(s_InData));
	memset(s_Open, 0, sizeof(s_Open));
	memset(s_LastRx, 0, sizeof(s_LastRx));
	s_OpenCount = 0;
	s_CloseCount = 0;
	s_OutXferCount = 0;
	s_InXferCount = 0;
	s_XferOk = true;
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

static void Receive(const uint8_t *pData, uint16_t Length,
					UsbCtrlrXferResult_t Result = USB_CTRLR_XFER_SUCCESS)
{
	CHECK(s_OutHandler != nullptr);
	CHECK(!s_OutBlocking);
	CHECK(Length <= USB_ISO_INTRF_MAX_MPS);
	if (Length > 0U)
		memcpy(s_OutBuffer, pData, Length);
	s_OutHandler(USB_ENDPADDR_DIROUT(8U), USB_CTRLR_EVT_XFER_CMPL,
		Length, Result, s_OutContext);
}

static void CompleteIn(UsbCtrlrXferResult_t Result = USB_CTRLR_XFER_SUCCESS)
{
	CHECK(s_InHandler != nullptr);
	CHECK(s_InBusy);
	if (!s_InBusy)
		return;
	const uint16_t len = s_InLength;
	s_InBusy = false;
	s_InHandler(USB_ENDPADDR_DIRIN(8U), USB_CTRLR_EVT_XFER_CMPL,
		len, Result, s_InContext);
}

static void TestLifecycle(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	auto cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(iso.pIntrfData == &iso.LocalData);
	CHECK(iso.pIntrfData->hRxFifo != nullptr);
	CHECK(iso.pIntrfData->hTxFifo != nullptr);
	CHECK(!s_OutBlocking);
	CHECK(s_OutXferCount == 0);

	CHECK(UsbIsoIntrfOpen(&iso, 25U, 1U));
	CHECK(iso.Opened && iso.Mps == 25U && iso.Interval == 1U);
	CHECK(iso.pIntrfData->Mps == 25U);
	CHECK(s_OpenCount == 2);
	CHECK(s_Open[0].bEndpointAddress == USB_ENDPADDR_DIRIN(8U));
	CHECK(s_Open[1].bEndpointAddress == USB_ENDPADDR_DIROUT(8U));
	CHECK(s_Open[0].bmAttributes == USB_ENDPATT_TRANS_ISO);
	CHECK(s_OutXferCount == 0);

	UsbIsoIntrfClose(&iso);
	CHECK(!iso.Opened && iso.pIntrfData->Mps == 0U);
	CHECK(s_CloseCount == 2);
}

static void TestRx(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	auto cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(UsbIsoIntrfOpen(&iso, 17U, 1U));

	const uint8_t data[] = {1,2,3,4,5};
	Receive(data, sizeof(data));
	CHECK(s_RxCount == 1);
	CHECK(s_LastRxLength == sizeof(data));
	CHECK(s_LastRxResult == USB_CTRLR_XFER_SUCCESS);
	CHECK(memcmp(s_LastRx, data, sizeof(data)) == 0);
	CHECK(CFifoUsed(iso.pIntrfData->hRxFifo) == 0);
	CHECK(s_OutXferCount == 0);

	Receive(nullptr, 0U);
	CHECK(s_RxCount == 2 && iso.RxEmptyCnt == 1U);

	Receive(nullptr, 0U, USB_CTRLR_XFER_FAILED);
	CHECK(s_RxCount == 3);
	CHECK(s_LastRxResult == USB_CTRLR_XFER_FAILED);
	CHECK(iso.RxMissCnt == 1U);
}

static void TestTx(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	auto cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(UsbIsoIntrfOpen(&iso, 9U, 1U));

	const uint8_t frame[] = {0x11,0x22,0x33,0x44};
	CHECK(UsbIsoIntrfSendFrame(&iso, frame, sizeof(frame)));
	CHECK(iso.TxActive && s_InBusy && s_InLength == sizeof(frame));
	CHECK(memcmp(s_InData, frame, sizeof(frame)) == 0);
	CHECK(!UsbIsoIntrfSendFrame(&iso, frame, sizeof(frame)));
	CompleteIn();
	CHECK(!iso.TxActive && s_TxCount == 1);
	CHECK(s_LastTxLength == sizeof(frame));
	CHECK(s_LastTxResult == USB_CTRLR_XFER_SUCCESS);

	CHECK(UsbIsoIntrfSendFrame(&iso, nullptr, 0U));
	CompleteIn();
	CHECK(iso.TxEmptyCnt == 1U);

	CHECK(UsbIsoIntrfSendFrame(&iso, frame, 1U));
	CompleteIn(USB_CTRLR_XFER_FAILED);
	CHECK(!iso.TxActive && iso.TxMissCnt == 1U);
	CHECK(s_LastTxResult == USB_CTRLR_XFER_FAILED);
	CHECK(UsbIsoIntrfTxReady(&iso));
}

static void TestSuspendResume(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	auto cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(UsbIsoIntrfOpen(&iso, 33U, 1U));
	UsbIsoIntrfSuspend(&iso);
	CHECK(iso.Suspended);
	const uint8_t b = 0x5A;
	CHECK(!UsbIsoIntrfSendFrame(&iso, &b, 1U));
	CHECK(UsbIsoIntrfResume(&iso));
	CHECK(!iso.Suspended);
	CHECK(UsbIsoIntrfSendFrame(&iso, &b, 1U));
	CompleteIn();
}

static void TestValidation(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	auto cfg = MakeCfg();
	cfg.EpNo = 7U;
	CHECK(!UsbIsoIntrfInit(&iso, &cfg));
	cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(!UsbIsoIntrfOpen(&iso, 0U, 1U));
	CHECK(!UsbIsoIntrfOpen(&iso, USB_ISO_INTRF_MAX_MPS + 1U, 1U));
	CHECK(!UsbIsoIntrfOpen(&iso, 9U, 0U));
}

int main(void)
{
	TestLifecycle();
	TestRx();
	TestTx();
	TestSuspendResume();
	TestValidation();
	printf("%s\n", s_Fail == 0 ? "usb_iso_intrf_test: PASS" :
		"usb_iso_intrf_test: FAIL");
	return s_Fail == 0 ? 0 : 1;
}
