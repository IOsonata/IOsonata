#include <stdio.h>
#include <string.h>
#include <type_traits>

#include "usb/usb_iso.h"

static_assert(std::is_base_of<UsbIntrf, UsbIsoIntrf>::value,
	"UsbIsoIntrf must derive from UsbIntrf");

static uint8_t *s_OutBuffer;
static uint8_t *s_InBuffer;
static UsbCtrlrEpHandler_t s_OutHandler;
static UsbCtrlrEpHandler_t s_InHandler;
static void *s_OutContext;
static void *s_InContext;
static bool s_InBusy;
static uint16_t s_InLength;
static uint8_t s_InData[USB_ISO_INTRF_MAX_MPS];
static int s_OpenCount;
static int s_CloseCount;
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

bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *) { s_OpenCount++; return true; }
void UsbCtrlrEpClose(int, uint8_t) { s_CloseCount++; }
void UsbCtrlrEpCloseAll(int) {}

bool UsbCtrlrEpXfer(int, uint8_t EpAddr, uint16_t Length)
{
	if (!USB_ENDPADDR_IS_IN(EpAddr))
	{
		s_OutXferCount++;
		return false;
	}
	if (s_InBusy || Length > sizeof(s_InData))
		return false;
	s_InBusy = true;
	s_InLength = Length;
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
static uint8_t s_LastRx[USB_ISO_INTRF_MAX_MPS];
static uint16_t s_LastRxLen;

static void RxFrame(UsbIsoIntrf_t *, const uint8_t *pData, uint16_t Length,
					UsbCtrlrXferResult_t Result, void *)
{
	if (Result == USB_CTRLR_XFER_SUCCESS)
	{
		s_RxCount++;
		s_LastRxLen = Length;
		if (Length > 0U) memcpy(s_LastRx, pData, Length);
	}
}

static void TxFrame(UsbIsoIntrf_t *, uint16_t,
					UsbCtrlrXferResult_t Result, void *)
{
	if (Result == USB_CTRLR_XFER_SUCCESS) s_TxCount++;
}

static void Receive(const uint8_t *pData, uint16_t Length)
{
	if (Length > 0U) memcpy(s_OutBuffer, pData, Length);
	s_OutHandler(USB_ENDPADDR_DIROUT(8U), USB_CTRLR_EVT_XFER_CMPL,
		Length, USB_CTRLR_XFER_SUCCESS, s_OutContext);
}

static void CompleteIn(void)
{
	CHECK(s_InBusy);
	const uint16_t len = s_InLength;
	s_InBusy = false;
	s_InHandler(USB_ENDPADDR_DIRIN(8U), USB_CTRLR_EVT_XFER_CMPL,
		len, USB_CTRLR_XFER_SUCCESS, s_InContext);
}

int main(void)
{
	UsbIsoIntrf_t iso = {};
	UsbIsoIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = 8U;
	cfg.RxHandler = RxFrame;
	cfg.TxHandler = TxFrame;

	CHECK(UsbIsoIntrfInit(&iso, &cfg));
	CHECK(UsbIsoIntrfOpen(&iso, 49U, 1U));
	CHECK(s_OpenCount == 2);

	const uint8_t tx[] = {0x10,0x20,0x30,0x40,0x50};
	const uint8_t rx[] = {0xA1,0xA2,0xA3};
	CHECK(UsbIsoIntrfSendFrame(&iso, tx, sizeof(tx)));
	CHECK(s_InBusy);
	CHECK(memcmp(s_InData, tx, sizeof(tx)) == 0);

	// OUT is controller-direct and must remain independent while IN is active.
	Receive(rx, sizeof(rx));
	CHECK(s_RxCount == 1);
	CHECK(s_LastRxLen == sizeof(rx));
	CHECK(memcmp(s_LastRx, rx, sizeof(rx)) == 0);
	CHECK(s_InBusy);
	CHECK(s_OutXferCount == 0);

	CompleteIn();
	CHECK(s_TxCount == 1);
	CHECK(!iso.TxActive);

	UsbIsoIntrfClose(&iso);
	CHECK(s_CloseCount == 2);

	printf("%s\n", s_Fail == 0 ? "usb_iso_intrf_duplex_test: PASS" :
		"usb_iso_intrf_duplex_test: FAIL");
	return s_Fail == 0 ? 0 : 1;
}
