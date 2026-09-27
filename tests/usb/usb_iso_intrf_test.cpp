#include <stdio.h>
#include <string.h>

#include "usb/usb_iso.h"

// Fake controller: one registration per endpoint direction, as the real
// controller keeps it, so the ISO class taking over the IN endpoint after
// UsbIntrfInit replaces the UsbIntrf handler.
static uint8_t *s_OutBuffer;
static uint8_t *s_InBuffer;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static bool s_OutBlocking;
static bool s_InBlocking;
static bool s_OutBusy;
static bool s_InBusy;
static uint16_t s_InLength;
static uint8_t s_InData[USB_ISO_INTRF_MAX_MPS];
static UsbEndPointDesc_t s_Open[4];
static int s_OpenCount;
static int s_CloseCount;
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
void UsbCtrlrEpStall(int, uint8_t, bool) {}
void UsbCtrlrEpClearStall(int, uint8_t, bool) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }

void UsbCtrlrEpAlloc(int, uint8_t, bool bIn, uint8_t *pBuffer, bool Blocking,
						UsbCtrlrEpHandler_t Handler, void *pContext)
{
	if (bIn)
	{
		s_InBuffer = pBuffer;
		s_InHandler = Handler;
		s_InContext = pContext;
		s_InBlocking = Blocking;
	}
	else
	{
		s_OutBuffer = pBuffer;
		s_OutHandler = Handler;
		s_OutContext = pContext;
		s_OutBlocking = Blocking;
	}
}

bool UsbCtrlrEpOpen(int DevNo, const UsbEndPointDesc_t *pDesc)
{
	if (DevNo != 0 || pDesc == nullptr || s_OpenCount >= 4)
		return false;
	s_Open[s_OpenCount++] = *pDesc;
	return true;
}

void UsbCtrlrEpClose(int, uint8_t, bool bIn)
{
	s_CloseCount++;
	if (bIn)
		s_InBusy = false;
	else
		s_OutBusy = false;
}
void UsbCtrlrEpCloseAll(int) {}

bool UsbCtrlrEpSend(int, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	if ((EpNum & 0x80U) != 0U) return false;
	if (!s_XferOk) return false;
	if (s_InBusy || Length > sizeof(s_InData)) return false;
	s_InBuffer = pBuffer;
	s_InBusy = true;
	s_InLength = Length;
	if (Length > 0U) memcpy(s_InData, pBuffer, Length);
	return true;
}
static uint32_t s_IsoRxDma[(USB_CTRLR0_ISO_PKT_LEN_MAX + 3U) / 4U];
uint8_t *UsbCtrlrIsoRxBuffer(int) { return (uint8_t *)s_IsoRxDma; }
bool UsbCtrlrIsoSend(int, uint8_t, uint8_t *pBuffer, uint16_t Length)
{
	s_InXferCount++;
	if (pBuffer == nullptr) return true;
	if (!s_XferOk || s_InBusy || Length > sizeof(s_InData)) return false;
	s_InBuffer = pBuffer;
	s_InBusy = true;
	s_InLength = Length;
	if (Length > 0U) memcpy(s_InData, pBuffer, Length);
	return true;
}

}

static int s_Fail;
#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

// Application side: the DeviceIntrf event callback. RX_DATA is drained
// with RxData unless the test wants frames left in the FIFO.
static bool s_Drain = true;
static int s_RxEventCount;
static int s_RxCount;
static int s_RxTimeoutCount;
static int s_TxCount;
static int s_TxEmptyCount;
static int s_TxTimeoutCount;
static uint16_t s_LastRxLength;
static uint16_t s_LastTxLength;
static uint8_t s_LastRx[USB_ISO_INTRF_MAX_MPS];
static void *s_LastContext;

static int IsoEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event,
					uint8_t *, int Length)
{
	s_LastContext = UsbIsoIntrfContext(pDev);
	switch (Event)
	{
		case DEVINTRF_EVT_RX_DATA:
		{
			s_RxEventCount++;
			if (!s_Drain)
				return 0;
			int total = 0;
			int len;
			while ((len = DeviceIntrfRxData(pDev, s_LastRx, sizeof(s_LastRx))) > 0)
			{
				s_RxCount++;
				s_LastRxLength = (uint16_t)len;
				total += len;
			}
			return total;
		}
		case DEVINTRF_EVT_RX_TIMEOUT:
			s_RxTimeoutCount++;
			return 0;
		case DEVINTRF_EVT_TX_READY:
			s_TxCount++;
			s_LastTxLength = (uint16_t)Length;
			return Length;
		case DEVINTRF_EVT_TX_FIFO_EMPTY:
			s_TxCount++;
			s_TxEmptyCount++;
			s_LastTxLength = (uint16_t)Length;
			return Length;
		case DEVINTRF_EVT_TX_TIMEOUT:
			s_TxTimeoutCount++;
			s_LastTxLength = (uint16_t)Length;
			return 0;
		default:
			return 0;
	}
}

static void ResetFake(void)
{
	s_OutBuffer = nullptr;
	s_InBuffer = nullptr;
	s_OutHandler = nullptr;
	s_InHandler = nullptr;
	s_OutContext = nullptr;
	s_InContext = nullptr;
	s_OutBlocking = false;
	s_InBlocking = false;
	s_OutBusy = false;
	s_InBusy = false;
	s_InLength = 0U;
	memset(s_InData, 0, sizeof(s_InData));
	memset(s_Open, 0, sizeof(s_Open));
	memset(s_LastRx, 0, sizeof(s_LastRx));
	s_OpenCount = 0;
	s_CloseCount = 0;
	s_InXferCount = 0;
	s_XferOk = true;
	s_Drain = true;
	s_RxEventCount = 0;
	s_RxCount = 0;
	s_RxTimeoutCount = 0;
	s_TxCount = 0;
	s_TxEmptyCount = 0;
	s_TxTimeoutCount = 0;
	s_LastRxLength = 0U;
	s_LastTxLength = 0U;
	s_LastContext = nullptr;
}

static int s_Context;

static UsbIsoIntrfCfg_t MakeCfg(void)
{
	alignas(4) static uint8_t rx[USB_ISO_INTRF_FIFO_MEMSIZE(USB_ISO_INTRF_MAX_MPS)];
	alignas(4) static uint8_t tx[USB_ISO_INTRF_FIFO_MEMSIZE(USB_ISO_INTRF_MAX_MPS)];
	UsbIsoIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = 8U;
	cfg.BufferSize = USB_ISO_INTRF_MAX_MPS;
	cfg.pRxFifoMem = rx;
	cfg.pTxFifoMem = tx;
	cfg.EvtCB = IsoEvent;
	cfg.pContext = &s_Context;
	return cfg;
}

// The controller DMAs ISO OUT into its own buffer, the one it registered
// through UsbIntrfInit, and reports the completion.
static void Receive(const uint8_t *pData, uint16_t Length,
					UsbCtrlrEvtType_t Event = USB_CTRLR_EVT_XFER_CMPL)
{
	CHECK(s_OutHandler != nullptr);
	CHECK(s_OutBuffer == UsbCtrlrIsoRxBuffer(0));
	CHECK(!s_OutBusy);
	if (Length > 0U)
		memcpy(s_OutBuffer, pData, Length);
	s_OutHandler(Event, Length, s_OutContext);
}

static void Sof(uint16_t Frame = 0U)
{
	CHECK(s_InHandler != nullptr);
	s_InHandler(USB_CTRLR_EVT_SOF, Frame, s_InContext);
}

static void CompleteIn(UsbCtrlrEvtType_t Event = USB_CTRLR_EVT_XFER_CMPL)
{
	CHECK(s_InHandler != nullptr);
	CHECK(s_InBusy);
	if (!s_InBusy)
		return;
	const uint16_t len = s_InLength;
	s_InBusy = false;
	s_InHandler(Event, len, s_InContext);
}

static void TestLifecycle(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	UsbDevIntrf_t isoData = {};
	auto cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &isoData, &cfg));
	CHECK(iso.pData->Mode == USB_INTRF_MODE_PACKET);
	CHECK(iso.pData->hRxFifo != nullptr && iso.pData->hTxFifo != nullptr);
	// TX blocks (the controller reads its head in place); RX drops the
	// oldest frame (no back-pressure on an isochronous endpoint).
	CHECK(CFifoIsBlocking(iso.pData->hTxFifo));
	CHECK(!CFifoIsBlocking(iso.pData->hRxFifo));
	CHECK(CFifoAvail(iso.pData->hTxFifo) == (int)USB_ISO_INTRF_FIFO_PKTCNT);
	CHECK(CFifoAvail(iso.pData->hRxFifo) == (int)USB_ISO_INTRF_FIFO_PKTCNT);
	// The application's callback is the DeviceIntrf callback, untouched.
	CHECK(iso.pData->DevIntrf.EvtCB == IsoEvent);
	CHECK(iso.pContext == &s_Context);
	// IN endpoint is the ISO class's; OUT stays with UsbIntrf.
	CHECK(s_InContext == &iso && s_OutContext == iso.pData);
	CHECK(s_OutBlocking);

	CHECK(UsbIsoIntrfOpen(&iso, 25U, 1U));
	CHECK(iso.Opened && iso.Mps == 25U && iso.Interval == 1U);
	CHECK(iso.pData->Mps == 25U);
	CHECK(s_OpenCount == 2);
	CHECK(s_Open[0].bEndpointAddress == USB_ENDPADDR_DIRIN(8U));
	CHECK(s_Open[1].bEndpointAddress == USB_ENDPADDR_DIROUT(8U));
	CHECK(s_Open[0].bmAttributes == USB_ENDPATT_TRANS_ISO);

	UsbIsoIntrfClose(&iso);
	CHECK(!iso.Opened && iso.pData->Mps == 0U);
	CHECK(s_CloseCount == 2);
}

static void TestRx(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	UsbDevIntrf_t isoData = {};
	auto cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &isoData, &cfg));
	CHECK(UsbIsoIntrfOpen(&iso, 17U, 1U));

	// A frame is reported through RX_DATA and pulled with RxData.
	const uint8_t data[] = {1,2,3,4,5};
	Receive(data, sizeof(data));
	CHECK(s_RxEventCount == 1 && s_RxCount == 1);
	CHECK(s_LastRxLength == sizeof(data));
	CHECK(memcmp(s_LastRx, data, sizeof(data)) == 0);
	CHECK(s_LastContext == &s_Context);
	CHECK(CFifoUsed(iso.pData->hRxFifo) == 0);

	// Zero-length frames are counted and skipped by RxData.
	Receive(nullptr, 0U);
	CHECK(s_RxEventCount == 2 && s_RxCount == 1);
	CHECK(iso.RxEmptyCnt == 1U);
	CHECK(CFifoUsed(iso.pData->hRxFifo) == 0);

	// A frame above the packet size is counted and skipped.
	uint8_t big[20];
	memset(big, 0xEE, sizeof(big));
	Receive(big, sizeof(big));
	CHECK(s_RxCount == 1 && iso.RxMissCnt == 1U);

	// A failed transfer is UsbIntrf's RX_TIMEOUT.
	Receive(nullptr, 0U, USB_CTRLR_EVT_XFER_FAILED);
	CHECK(s_RxTimeoutCount == 1);
	CHECK(iso.pData->RxDropCnt == 1U);

	// Frames the application leaves in the FIFO: the newest two survive,
	// the oldest is dropped, and RxData returns one frame per call.
	s_Drain = false;
	const uint8_t f1[] = {0x11}, f2[] = {0x22, 0x22}, f3[] = {0x33, 0x33, 0x33};
	Receive(f1, sizeof(f1));
	Receive(f2, sizeof(f2));
	Receive(f3, sizeof(f3));
	CHECK(CFifoUsed(iso.pData->hRxFifo) == 2);
	uint8_t out[USB_ISO_INTRF_MAX_MPS];
	CHECK(DeviceIntrfRxData(&iso.pData->DevIntrf, out, sizeof(out)) == 2);
	CHECK(out[0] == 0x22 && out[1] == 0x22);
	// Too small a buffer leaves the frame queued.
	CHECK(DeviceIntrfRxData(&iso.pData->DevIntrf, out, 2) == 0);
	CHECK(CFifoUsed(iso.pData->hRxFifo) == 1);
	CHECK(DeviceIntrfRxData(&iso.pData->DevIntrf, out, sizeof(out)) == 3);
	CHECK(out[2] == 0x33);
	CHECK(DeviceIntrfRxData(&iso.pData->DevIntrf, out, sizeof(out)) == 0);
	s_Drain = true;
}

static void TestTx(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	UsbDevIntrf_t isoData = {};
	auto cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &isoData, &cfg));
	CHECK(UsbIsoIntrfOpen(&iso, 9U, 1U));

	// Two frames fit, the third is refused; the head goes out at SOF and
	// stays queued until its completion pops it.
	const uint8_t frame[] = {0x11,0x22,0x33,0x44};
	const uint8_t second[] = {0x55,0x66};
	CHECK(DeviceIntrfTxData(&iso.pData->DevIntrf, frame, sizeof(frame)) == (int)sizeof(frame));
	CHECK(UsbIsoIntrfSendFrame(&iso, second, sizeof(second)));
	CHECK(!UsbIsoIntrfSendFrame(&iso, frame, sizeof(frame)));
	CHECK(!UsbIsoIntrfTxReady(&iso));
	Sof();
	CHECK(s_InBusy && s_InLength == sizeof(frame));
	CHECK(memcmp(s_InData, frame, sizeof(frame)) == 0);
	CHECK(CFifoUsed(iso.pData->hTxFifo) == 2);

	// While the first is in flight, SOF offers the same head again and the
	// controller refuses it; nothing is popped.
	Sof(1U);
	CHECK(s_InXferCount == 2 && CFifoUsed(iso.pData->hTxFifo) == 2);

	// Completion pops the head and reports TX_READY, a frame still waits.
	CompleteIn();
	CHECK(s_TxCount == 1 && s_TxEmptyCount == 0);
	CHECK(s_LastTxLength == sizeof(frame));
	CHECK(CFifoUsed(iso.pData->hTxFifo) == 1);
	CHECK(UsbIsoIntrfTxReady(&iso));

	// Second frame: TX_FIFO_EMPTY when the queue drains.
	Sof(2U);
	CHECK(s_InBusy && s_InLength == sizeof(second));
	CompleteIn();
	CHECK(s_TxCount == 2 && s_TxEmptyCount == 1);
	CHECK(s_LastTxLength == sizeof(second));
	CHECK(CFifoUsed(iso.pData->hTxFifo) == 0);

	// A zero-length frame is a frame.
	CHECK(UsbIsoIntrfSendFrame(&iso, nullptr, 0U));
	Sof(3U);
	CHECK(s_InBusy && s_InLength == 0U);
	CompleteIn();
	CHECK(iso.TxEmptyCnt == 1U && s_TxCount == 3);

	// A failed frame is TX_TIMEOUT and is still popped.
	CHECK(UsbIsoIntrfSendFrame(&iso, frame, 1U));
	Sof(4U);
	CompleteIn(USB_CTRLR_EVT_XFER_FAILED);
	CHECK(iso.TxMissCnt == 1U && s_TxTimeoutCount == 1);
	CHECK(s_LastTxLength == 1U);
	CHECK(CFifoUsed(iso.pData->hTxFifo) == 0);
	CHECK(UsbIsoIntrfTxReady(&iso));

	// Interval 2: only every other frame offers.
	CHECK(UsbIsoIntrfOpen(&iso, 9U, 2U));
	CHECK(UsbIsoIntrfSendFrame(&iso, frame, 2U));
	const int before = s_InXferCount;
	Sof(1U);
	CHECK(s_InXferCount == before && !s_InBusy);
	Sof(2U);
	CHECK(s_InXferCount == before + 1 && s_InBusy);
	CompleteIn();
}

static void TestSuspendResume(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	UsbDevIntrf_t isoData = {};
	auto cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &isoData, &cfg));
	CHECK(UsbIsoIntrfOpen(&iso, 33U, 1U));
	UsbIsoIntrfSuspend(&iso);
	CHECK(iso.Suspended);
	const uint8_t b = 0x5A;
	CHECK(!UsbIsoIntrfSendFrame(&iso, &b, 1U));
	CHECK(!UsbIsoIntrfTxReady(&iso));
	Sof();
	CHECK(s_InXferCount == 0);
	CHECK(UsbIsoIntrfResume(&iso));
	CHECK(!iso.Suspended);
	CHECK(UsbIsoIntrfSendFrame(&iso, &b, 1U));
	Sof();
	CHECK(s_InBusy);
	CompleteIn();
	CHECK(s_TxCount == 1);
}

static void TestValidation(void)
{
	ResetFake();
	UsbIsoIntrf_t iso = {};
	UsbDevIntrf_t isoData = {};
	auto cfg = MakeCfg();
	cfg.EpNo = 7U;
	CHECK(!UsbIsoIntrfInit(&iso, &isoData, &cfg));
	cfg = MakeCfg();
	cfg.BufferSize = USB_ISO_INTRF_MAX_MPS + 1U;
	CHECK(!UsbIsoIntrfInit(&iso, &isoData, &cfg));
	cfg = MakeCfg();
	cfg.pRxFifoMem = nullptr;
	CHECK(!UsbIsoIntrfInit(&iso, &isoData, &cfg));
	cfg = MakeCfg();
	CHECK(UsbIsoIntrfInit(&iso, &isoData, &cfg));
	CHECK(!UsbIsoIntrfOpen(&iso, 0U, 1U));
	CHECK(!UsbIsoIntrfOpen(&iso, USB_ISO_INTRF_MAX_MPS + 1U, 1U));
	CHECK(!UsbIsoIntrfOpen(&iso, 9U, 0U));
	CHECK(!UsbIsoIntrfSendFrame(&iso, nullptr, 1U));
}

static void TestSharedTransport(void)
{
	ResetFake();
	UsbIsoIntrf intrf;
	auto cfg = MakeCfg();
	cfg.pRxFifoMem = nullptr;
	cfg.pTxFifoMem = nullptr;
	CHECK(intrf.Init(cfg));
	UsbIsoIntrf_t *pState = intrf;
	UsbIntrf *pTransport = &intrf;
	DeviceIntrf *pDevice = pTransport;
	CHECK(pTransport->Data() == &pState->pData->DevIntrf);
	CHECK(static_cast<DevIntrf_t *>(*pDevice) == intrf.Data());
	CHECK(s_InContext == pState && s_OutContext == pState->pData);
	CHECK(intrf.Open(9U, 1U));

	// Transmit and receive through the DeviceIntrf API only.
	const uint8_t data[] = {2U, 5U, 8U};
	CHECK(pTransport->TxData(data, sizeof(data)) == (int)sizeof(data));
	Sof();
	CHECK(memcmp(s_InBuffer, data, sizeof(data)) == 0);
	CompleteIn();
	CHECK(s_TxCount == 1 && s_LastTxLength == sizeof(data));

	s_Drain = false;
	Receive(data, sizeof(data));
	CHECK(s_RxEventCount == 1 && s_RxCount == 0);
	uint8_t received[USB_ISO_INTRF_MAX_MPS] = {};
	CHECK(pTransport->RxData(received, sizeof(received)) == (int)sizeof(data));
	CHECK(memcmp(received, data, sizeof(data)) == 0);
	CHECK(pTransport->RxData(received, sizeof(received)) == 0);
	s_Drain = true;
	intrf.Close();
}

int main(void)
{
	TestSharedTransport();
	TestLifecycle();
	TestRx();
	TestTx();
	TestSuspendResume();
	TestValidation();
	printf("%s\n", s_Fail == 0 ? "usb_iso_intrf_test: PASS" :
		"usb_iso_intrf_test: FAIL");
	return s_Fail == 0 ? 0 : 1;
}
