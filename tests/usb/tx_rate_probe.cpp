// Throughput probe for the byte mode TX path.
//
// Models the PRBS example: the application pushes one byte per iteration and
// the controller completes an IN transfer some number of iterations after it
// was submitted. Reports how many bytes each USB packet carried, which is what
// decides bulk throughput once the bus is not the limit.
//
// Not a regression test. Built and run by hand against two revisions.
#include <stdio.h>
#include <string.h>

#include "usb/usb_intrf.h"

#define MPS			64U
#define BUFFER_SIZE	64U
#define SLOTS		4U
#define EP_NO		1U

static uint8_t *s_InBuf;
static uint16_t s_InLen;
static bool s_InBusy;
static bool s_OutBusy;

static long s_Packets;
static long s_Bytes;
static long s_Zlp;

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
bool UsbCtrlrEpOpen(int, const UsbEndPointDesc_t *) { return true; }
void UsbCtrlrEpClose(int, uint8_t) {}
void UsbCtrlrEpCloseAll(int) {}
bool UsbCtrlrEpBusy(int, uint8_t EpAddr)
{
	return USB_ENDPADDR_IS_IN(EpAddr) ? s_InBusy : s_OutBusy;
}
void UsbCtrlrEpStall(int, uint8_t) {}
void UsbCtrlrEpClearStall(int, uint8_t) {}
size_t UsbCtrlrGetSerial(int, char *p, size_t n) { if (n) p[0] = 0; return 0; }

bool UsbCtrlrEpXfer(int, uint8_t EpAddr, uint8_t *pBuf, uint16_t Len)
{
	if (USB_ENDPADDR_IS_IN(EpAddr))
	{
		s_InBuf = pBuf;
		s_InLen = Len;
		s_InBusy = true;
		s_Packets++;
		s_Bytes += Len;
		if (Len == 0U) { s_Zlp++; }
	}
	else
	{
		s_OutBusy = true;
	}
	return true;
}
}

alignas(4) static uint8_t s_RxMem[USB_INTRF_RXMEM_SIZE(SLOTS, BUFFER_SIZE)];
alignas(4) static uint8_t s_TxMem[CFIFO_MEMSIZE(2048)];
alignas(4) static uint8_t s_RxTransfer[BUFFER_SIZE];
alignas(4) static uint8_t s_TxTransfer[BUFFER_SIZE];
static UsbDevIntrf_t s_Intrf;

static bool Setup(void)
{
	UsbIntrfCfg_t cfg = {};
	cfg.DevNo = 0;
	cfg.EpNo = EP_NO;
	cfg.pRxFifoMem = s_RxMem;
	cfg.RxFifoMemSize = (int)sizeof(s_RxMem);
	cfg.pTxFifoMem = s_TxMem;
	cfg.TxFifoMemSize = (int)sizeof(s_TxMem);
	cfg.TxFifoBlkSize = 1U;
	cfg.BufferSize = BUFFER_SIZE;
	cfg.pRxBuffer = s_RxTransfer;
	cfg.pTxBuffer = s_TxTransfer;
	memset(static_cast<void *>(&s_Intrf), 0, sizeof(s_Intrf));
	s_InBusy = false;
	s_OutBusy = false;
	s_Packets = 0;
	s_Bytes = 0;
	s_Zlp = 0;
	return UsbIntrfInit(&s_Intrf, &cfg) && UsbIntrfConfigure(&s_Intrf, MPS);
}

// BusTicks is how many producer iterations pass before an in flight packet
// completes. Small means the bus is fast relative to the application, which is
// the PRBS case: a tight one byte fill loop against full speed bulk.
static void Run(int BusTicks, long Iterations)
{
	if (!Setup()) { printf("setup failed\n"); return; }

	uint8_t d = 0xFF;
	int due = -1;

	for (long i = 0; i < Iterations; i++)
	{
		if (DeviceIntrfTxData(&s_Intrf.DevIntrf, &d, 1) > 0)
		{
			d = (uint8_t)(d + 1U);
		}

		if (s_InBusy && due < 0) { due = BusTicks; }

		if (due == 0)
		{
			const uint16_t len = s_InLen;
			s_InBusy = false;
			due = -1;
			UsbIntrfXferComplete(&s_Intrf, USB_ENDPADDR_DIRIN(EP_NO), len,
								 USB_CTRLR_XFER_SUCCESS);
			if (s_InBusy) { due = BusTicks; }
		}
		else if (due > 0)
		{
			due--;
		}

	}

	printf("bus ticks %4d   packets %8ld   bytes %9ld   avg pkt %6.1f   zlp %ld\n",
		   BusTicks, s_Packets, s_Bytes,
		   s_Packets ? (double)s_Bytes / (double)s_Packets : 0.0, s_Zlp);
}

int main(void)
{
	const int ticks[] = { 1, 4, 16, 64, 256 };

	for (unsigned i = 0; i < sizeof(ticks) / sizeof(ticks[0]); i++)
	{
		Run(ticks[i], 200000);
	}

	return 0;
}
