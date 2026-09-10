#include <stdio.h>
#include <string.h>
#include <type_traits>

#include "usb/usb_int.h"

static_assert(std::is_base_of<DeviceIntrf, UsbIntIntrf>::value,
	"UsbIntIntrf must present the DeviceIntrf API");

static uint8_t *s_OutBuffer;
static uint8_t *s_InBuffer;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static UsbEndPointDesc_t s_Open[4];
static int s_OpenCount;
static int s_CloseCount;
static int s_OutXferCount;
static bool s_XferOk = true;
static bool s_HighSpeed;
static uint16_t s_InLength;

extern "C" {
bool UsbCtrlrInit(int, const UsbCtrlrCfg_t *) { return true; }
bool UsbCtrlrStart(int) { return true; }
void UsbCtrlrStop(int) {}
void UsbCtrlrProcess(int) {}
bool UsbCtrlrVbusDetected(int) { return true; }
bool UsbCtrlrHighSpeed(int) { return s_HighSpeed; }
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
	return pBuffer != nullptr && Handler != nullptr;
}

bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *pDesc)
{
	if (pDesc == nullptr || s_OpenCount >= 4)
	{
		return false;
	}
	s_Open[s_OpenCount++] = *pDesc;
	return true;
}

void UsbCtrlrEpClose(int, uint8_t) { s_CloseCount++; }
void UsbCtrlrEpCloseAll(int) {}

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

static int s_RxCount;
static int s_TxCount;
static uint16_t s_LastRxLength;
static uint16_t s_LastTxLength;
static UsbCtrlrXferResult_t s_LastRxResult;
static UsbCtrlrXferResult_t s_LastTxResult;
static uint8_t s_LastRx[USB_INT_INTRF_MAX_MPS];

static void RxPacket(UsbIntIntrf_t *, const uint8_t *pData, uint16_t Length,
					 UsbCtrlrXferResult_t Result, void *)
{
	s_RxCount++;
	s_LastRxLength = Length;
	s_LastRxResult = Result;
	if (Result == USB_CTRLR_XFER_SUCCESS && Length > 0U)
	{
		memcpy(s_LastRx, pData, Length);
	}
}

static void TxPacket(UsbIntIntrf_t *, uint16_t Length,
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
	memset(s_Open, 0, sizeof(s_Open));
	memset(s_LastRx, 0, sizeof(s_LastRx));
	s_OpenCount = 0;
	s_CloseCount = 0;
	s_OutXferCount = 0;
	s_XferOk = true;
	s_HighSpeed = false;
	s_InLength = 0U;
	s_RxCount = 0;
	s_TxCount = 0;
	s_LastRxLength = 0U;
	s_LastTxLength = 0U;
	s_LastRxResult = USB_CTRLR_XFER_SUCCESS;
	s_LastTxResult = USB_CTRLR_XFER_SUCCESS;
}

static UsbIntIntrfCfg_t MakeCfg(void)
{
	UsbIntIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = 3U;
	cfg.RxHandler = RxPacket;
	cfg.TxHandler = TxPacket;
	return cfg;
}

static void Drdy(void)
{
	s_OutHandler(USB_ENDPADDR_DIROUT(3U), USB_CTRLR_EVT_DRDY, 0U,
		USB_CTRLR_XFER_SUCCESS, s_OutContext);
}

static void Receive(const uint8_t *pData, uint16_t Length,
					UsbCtrlrXferResult_t Result = USB_CTRLR_XFER_SUCCESS)
{
	Drdy();
	if (Length > 0U)
	{
		memcpy(s_OutBuffer, pData, Length);
	}
	s_OutHandler(USB_ENDPADDR_DIROUT(3U), USB_CTRLR_EVT_XFER_CMPL,
		Length, Result, s_OutContext);
}

static void CompleteIn(UsbCtrlrXferResult_t Result = USB_CTRLR_XFER_SUCCESS)
{
	s_InHandler(USB_ENDPADDR_DIRIN(3U), USB_CTRLR_EVT_XFER_CMPL,
		s_InLength, Result, s_InContext);
}

static void TestLifecycleAndValidation(void)
{
	ResetFake();
	UsbIntIntrf_t intrf = {};
	auto cfg = MakeCfg();
	CHECK(UsbIntIntrfInit(&intrf, &cfg));
	CHECK(intrf.IntrfData.Mode == USB_INTRF_MODE_DIRECT);
	CHECK(intrf.IntrfData.hRxFifo == nullptr);
	CHECK(intrf.IntrfData.hTxFifo == nullptr);
	CHECK(!UsbIntIntrfOpen(&intrf, 0U, 1U));
	CHECK(!UsbIntIntrfOpen(&intrf, 65U, 1U));
	CHECK(!UsbIntIntrfOpen(&intrf, 8U, 0U));
	s_HighSpeed = true;
	CHECK(!UsbIntIntrfOpen(&intrf, 8U, 17U));
	s_HighSpeed = false;
	CHECK(UsbIntIntrfOpen(&intrf, 16U, 4U));
	CHECK(intrf.Opened && intrf.Interval == 4U && intrf.Mps == 16U);
	CHECK(s_OpenCount == 2);
	CHECK(s_Open[0].bmAttributes == USB_ENDPATT_TRANS_INT);
	CHECK(s_Open[1].bmAttributes == USB_ENDPATT_TRANS_INT);
	CHECK(s_Open[0].bInterval == 4U && s_Open[1].bInterval == 4U);
	CHECK(s_Open[0].bEndpointAddress == USB_ENDPADDR_DIRIN(3U));
	CHECK(s_Open[1].bEndpointAddress == USB_ENDPADDR_DIROUT(3U));
	UsbIntIntrfClose(&intrf);
	CHECK(!intrf.Opened && intrf.IntrfData.Mps == 0U && s_CloseCount == 2);

	cfg.EpNo = 0U;
	CHECK(!UsbIntIntrfInit(&intrf, &cfg));
	cfg.EpNo = 8U;
	CHECK(!UsbIntIntrfInit(&intrf, &cfg));
}

static void TestDuplexAndZeroLength(void)
{
	ResetFake();
	UsbIntIntrf_t intrf = {};
	auto cfg = MakeCfg();
	CHECK(UsbIntIntrfInit(&intrf, &cfg));
	CHECK(UsbIntIntrfOpen(&intrf, 16U, 1U));

	const uint8_t tx[] = {1U, 2U, 3U};
	const uint8_t rx[] = {4U, 5U};
	CHECK(UsbIntIntrfSendPacket(&intrf, tx, sizeof(tx)));
	CHECK(memcmp(s_InBuffer, tx, sizeof(tx)) == 0);
	CHECK(!UsbIntIntrfSendPacket(&intrf, tx, sizeof(tx)));
	Receive(rx, sizeof(rx));
	CHECK(s_RxCount == 1 && s_LastRxLength == sizeof(rx));
	CHECK(memcmp(s_LastRx, rx, sizeof(rx)) == 0);
	CHECK(s_OutXferCount == 1);
	CompleteIn();
	CHECK(s_TxCount == 1 && s_LastTxLength == sizeof(tx));
	CHECK(UsbIntIntrfTxReady(&intrf));

	CHECK(UsbIntIntrfSendPacket(&intrf, nullptr, 0U));
	CompleteIn();
	CHECK(intrf.TxEmptyCnt == 1U);
	Receive(nullptr, 0U);
	CHECK(intrf.RxEmptyCnt == 1U && s_LastRxLength == 0U);
}

static void TestErrorsSuspendAndReset(void)
{
	ResetFake();
	UsbIntIntrf_t intrf = {};
	auto cfg = MakeCfg();
	CHECK(UsbIntIntrfInit(&intrf, &cfg));
	CHECK(UsbIntIntrfOpen(&intrf, 8U, 1U));
	UsbIntIntrfSuspend(&intrf);
	const uint8_t data = 9U;
	CHECK(!UsbIntIntrfSendPacket(&intrf, &data, 1U));
	CHECK(UsbIntIntrfResume(&intrf));
	CHECK(UsbIntIntrfSendPacket(&intrf, &data, 1U));
	CompleteIn(USB_CTRLR_XFER_FAILED);
	CHECK(intrf.TxErrorCnt == 1U);
	CHECK(s_LastTxResult == USB_CTRLR_XFER_FAILED);
	Receive(nullptr, 0U, USB_CTRLR_XFER_FAILED);
	CHECK(intrf.RxErrorCnt == 1U);
	CHECK(s_LastRxResult == USB_CTRLR_XFER_FAILED);
	UsbIntIntrfReset(&intrf);
	CHECK(!intrf.Opened && intrf.RxErrorCnt == 0U && intrf.TxErrorCnt == 0U);
}

static void TestPolledRxOwnership(void)
{
	ResetFake();
	UsbIntIntrf_t intrf = {};
	auto cfg = MakeCfg();
	cfg.RxHandler = nullptr;
	CHECK(UsbIntIntrfInit(&intrf, &cfg));
	CHECK(UsbIntIntrfOpen(&intrf, 8U, 1U));
	const uint8_t first[] = {1U, 2U};
	const uint8_t second[] = {7U, 8U, 9U};
	Receive(first, sizeof(first));
	CHECK((intrf.IntrfData.pRxDirectBuffer->Hdr.Flags &
		USB_INTRF_SLOT_READY) != 0U);
	Receive(second, sizeof(second));
	CHECK(intrf.IntrfData.RxDropCnt == 1U);
	uint8_t out[3] = {};
	CHECK(DeviceIntrfRxData(&intrf.IntrfData.DevIntrf, out, sizeof(out)) == 3);
	CHECK(memcmp(out, second, sizeof(second)) == 0);
}

int main(void)
{
	TestLifecycleAndValidation();
	TestDuplexAndZeroLength();
	TestErrorsSuspendAndReset();
	TestPolledRxOwnership();
	printf("%s\n", s_Fail == 0 ? "usb_int_intrf_test: PASS" :
		"usb_int_intrf_test: FAIL");
	return s_Fail == 0 ? 0 : 1;
}
