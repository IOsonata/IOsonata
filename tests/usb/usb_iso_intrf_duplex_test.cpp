#include <stdio.h>
#include <string.h>
#include <type_traits>

#include "usb/usb_iso.h"

static_assert(std::is_base_of<DeviceIntrf, UsbIsoIntrf>::value,
	"UsbIsoIntrf must present the DeviceIntrf API");

static uint8_t *s_OutBuffer;
static uint8_t *s_InBuffer;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static bool s_OutBusy;
static bool s_InBusy;
static uint16_t s_InLength;
static uint8_t s_InData[USB_ISO_INTRF_MAX_MPS];
static int s_OpenCount;
static int s_CloseCount;

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

bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *) { s_OpenCount++; return true; }
void UsbCtrlrEpClose(int, uint8_t, bool) { s_CloseCount++; }
void UsbCtrlrEpCloseAll(int) {}

bool UsbCtrlrEpSend(int, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	if ((EpNum & 0x80U) != 0U) return false;
	if (s_InBusy || Length > sizeof(s_InData)) return false;
	s_InBuffer = pBuffer;
	s_InBusy = true;
	s_InLength = Length;
	if (Length > 0U) memcpy(s_InData, pBuffer, Length);
	return true;
}
bool UsbCtrlrIsoSend(int, uint8_t, uint8_t *pBuffer, uint16_t Length)
{
	if (pBuffer == nullptr) return true;
	if (s_InBusy || Length > sizeof(s_InData)) return false;
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

static int s_RxCount;
static int s_TxCount;
static int s_TxEmptyEvents;
static uint8_t s_LastRx[USB_ISO_INTRF_MAX_MPS];
static uint16_t s_LastRxLen;

// Application callback, the DeviceIntrf model: pull frames with RxData on
// RX_DATA, count sent frames on TX_READY / TX_FIFO_EMPTY.
static int IsoEvent(DevIntrf_t * const pDev, DEVINTRF_EVT Event,
					uint8_t *, int Length)
{
	switch (Event)
	{
		case DEVINTRF_EVT_RX_DATA:
		{
			int len;
			int total = 0;
			while ((len = DeviceIntrfRxData(pDev, s_LastRx, sizeof(s_LastRx))) > 0)
			{
				s_RxCount++;
				s_LastRxLen = (uint16_t)len;
				total += len;
			}
			return total;
		}
		case DEVINTRF_EVT_TX_FIFO_EMPTY:
			s_TxEmptyEvents++;
			s_TxCount++;
			return Length;
		case DEVINTRF_EVT_TX_READY:
			s_TxCount++;
			return Length;
		default:
			return 0;
	}
}

static void Receive(const uint8_t *pData, uint16_t Length)
{
	CHECK(s_OutHandler != nullptr);
	CHECK(!s_OutBusy);
	CHECK(s_OutBuffer != nullptr);
	if (s_OutBuffer == nullptr) return;
	if (Length > 0U) memcpy(s_OutBuffer, pData, Length);

	// ISO OUT DMA lands directly in the registered buffer, the RX FIFO block
	// UsbIntrf reserved. UsbIntrf receives only the transfer-complete
	// notification.
	s_OutHandler(USB_CTRLR_EVT_XFER_CMPL,
		Length, s_OutContext);
}

static void Sof(uint16_t Frame = 0U)
{
	CHECK(s_InHandler != nullptr);
	s_InHandler(USB_CTRLR_EVT_SOF, Frame, s_InContext);
}

static void CompleteIn(void)
{
	CHECK(s_InBusy);
	const uint16_t len = s_InLength;
	s_InBusy = false;
	s_InHandler(USB_CTRLR_EVT_XFER_CMPL,
		len, s_InContext);
}

int main(void)
{
	UsbIsoIntrf_t iso = {};
	UsbDevIntrf_t isoData = {};
	alignas(4) uint8_t rxFifo[USB_ISO_INTRF_FIFO_MEMSIZE(49U)] = {};
	alignas(4) uint8_t txFifo[USB_ISO_INTRF_FIFO_MEMSIZE(49U)] = {};
	UsbIsoIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = 8U;
	cfg.BufferSize = 49U;
	cfg.pRxFifoMem = rxFifo;
	cfg.pTxFifoMem = txFifo;
	cfg.EvtCB = IsoEvent;

	CHECK(UsbIsoIntrfInit(&iso, &isoData, &cfg));
	CHECK(iso.pData->Mode == USB_INTRF_MODE_PACKET);
	CHECK(iso.pData->hRxFifo != nullptr && iso.pData->hTxFifo != nullptr);
	// The OUT buffer is a block of the application's RX FIFO memory.
	CHECK(s_OutBuffer > rxFifo && s_OutBuffer < rxFifo + sizeof(rxFifo));
	CHECK(UsbIsoIntrfOpen(&iso, 49U, 1U));
	CHECK(s_OpenCount == 2);

	const uint8_t tx[] = {0x10,0x20,0x30,0x40,0x50};
	const uint8_t rx[] = {0xA1,0xA2,0xA3};
	CHECK(UsbIsoIntrfSendFrame(&iso, tx, sizeof(tx)));
	Sof();
	CHECK(s_InBusy);
	CHECK(memcmp(s_InData, tx, sizeof(tx)) == 0);

	// USB is serial, but the logical IN and OUT slots are independent. An OUT
	// service can complete while the IN slot remains owned by its transfer.
	Receive(rx, sizeof(rx));
	CHECK(s_RxCount == 1);
	CHECK(s_LastRxLen == sizeof(rx));
	CHECK(memcmp(s_LastRx, rx, sizeof(rx)) == 0);
	CHECK(s_InBusy);

	CompleteIn();
	CHECK(s_TxCount == 1);
	CHECK(UsbIsoIntrfTxReady(&iso));

	// Zero-length traffic is a valid frame on the wire. Sent, it is
	// offered and completed like any other; received, RxData skips it and
	// counts it.
	CHECK(UsbIsoIntrfSendFrame(&iso, nullptr, 0U));
	Sof();
	CHECK(s_InBusy);
	CHECK(s_InLength == 0U);
	Receive(nullptr, 0U);
	CHECK(s_RxCount == 1);
	CHECK(iso.RxEmptyCnt == 1U);
	CompleteIn();
	CHECK(s_TxCount == 2 && s_TxEmptyEvents == 2);
	CHECK(iso.TxEmptyCnt == 1U);
	CHECK(UsbIsoIntrfTxReady(&iso));

	UsbIsoIntrfClose(&iso);
	CHECK(s_CloseCount == 2);

	printf("%s\n", s_Fail == 0 ? "usb_iso_intrf_duplex_test: PASS" :
		"usb_iso_intrf_duplex_test: FAIL");
	return s_Fail == 0 ? 0 : 1;
}
