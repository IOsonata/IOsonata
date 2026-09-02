#include <stdio.h>
#include <string.h>

#include "device_intrf.h"
#include "usb/usb_intrf.h"

#define CHECK(v) do { if (!(v)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #v); return false; } } while (0)

typedef struct {
	uint8_t EpAddr;
	uint8_t *pBuffer;
	uint16_t Length;
} Xfer_t;

static Xfer_t s_Xfer[16];
static int s_XferCnt;

extern "C" bool UsbdCtrlrEpXfer(uint8_t EpAddr, uint8_t *pBuffer,
								 uint16_t Length)
{
	if (s_XferCnt >= (int)(sizeof(s_Xfer) / sizeof(s_Xfer[0])))
	{
		return false;
	}
	s_Xfer[s_XferCnt++] = { EpAddr, pBuffer, Length };
	return true;
}

static bool Init(UsbDevIntrf_t *pIntrf, uint8_t *pRxMem, int RxMemSize,
				 uint8_t *pTxMem, int TxMemSize, uint16_t TxBlkSize)
{
	UsbIntrfCfg_t cfg = {};
	cfg.bBlocking = true;
	cfg.pRxFifoMem = pRxMem;
	cfg.RxFifoMemSize = RxMemSize;
	cfg.pTxFifoMem = pTxMem;
	cfg.TxFifoMemSize = TxMemSize;
	cfg.TxFifoBlkSize = TxBlkSize;
	return UsbIntrfInit(pIntrf, &cfg);
}

static bool TestRx(void)
{
	alignas(4) uint8_t rxMem[CFIFO_TOTAL_MEMSIZE(4, sizeof(UsbPktHdr_t) + 8)];
	alignas(4) uint8_t txMem[CFIFO_MEMSIZE(32)];
	alignas(4) uint8_t rxDma[8] = {};
	UsbDevIntrf_t intrf = {};
	s_XferCnt = 0;
	CHECK(Init(&intrf, rxMem, sizeof(rxMem), txMem, sizeof(txMem), 1));
	CHECK(UsbIntrfConfigRx(&intrf, 1, sizeof(rxDma), rxDma));
	UsbIntrfRxEnable(&intrf, true);
	CHECK(s_XferCnt == 1 && s_Xfer[0].pBuffer == rxDma);

	const uint8_t first[] = { 1, 2, 3, 4, 5 };
	memcpy(rxDma, first, sizeof(first));
	UsbIntrfRxXferComplete(&intrf, sizeof(first), USBD_CTRLR_XFER_SUCCESS);
	CHECK(CFifoUsed(intrf.hRxFifo) == 1 && UsbIntrfRxUsed(&intrf) == 5);

	uint8_t out[8] = {};
	CHECK(DeviceIntrfRxData(&intrf.DevIntrf, out, 2) == 2);
	CHECK(out[0] == 1 && out[1] == 2 && intrf.RxOffset == 2);
	CHECK(DeviceIntrfRxData(&intrf.DevIntrf, out + 2, 6) == 3);
	CHECK(memcmp(out, first, sizeof(first)) == 0);
	CHECK(CFifoUsed(intrf.hRxFifo) == 0 && intrf.RxOffset == 0);

	UsbIntrfRxXferComplete(&intrf, 0, USBD_CTRLR_XFER_SUCCESS);
	UsbPktHdr_t *pZlp = reinterpret_cast<UsbPktHdr_t *>(CFifoPeek(intrf.hRxFifo));
	CHECK(pZlp != nullptr && pZlp->Length == 0);
	CHECK(DeviceIntrfRxData(&intrf.DevIntrf, out, sizeof(out)) == 0);
	CHECK(CFifoUsed(intrf.hRxFifo) == 0);
	return true;
}

static bool TestByteTx(void)
{
	alignas(4) uint8_t rxMem[CFIFO_TOTAL_MEMSIZE(2, sizeof(UsbPktHdr_t) + 8)];
	alignas(4) uint8_t txMem[CFIFO_MEMSIZE(32)];
	alignas(4) uint8_t txDma[8] = {};
	UsbDevIntrf_t intrf = {};
	s_XferCnt = 0;
	CHECK(Init(&intrf, rxMem, sizeof(rxMem), txMem, sizeof(txMem), 1));
	UsbIntrfConfigTx(&intrf, USB_ENDPADDR_DIRIN(1), sizeof(txDma), txDma);
	const uint8_t data[] = { 9, 8, 7 };
	CHECK(DeviceIntrfTxData(&intrf.DevIntrf, data, sizeof(data)) == 3);
	CHECK(s_XferCnt == 1 && s_Xfer[0].Length == 3);
	CHECK(memcmp(txDma, data, sizeof(data)) == 0);
	return true;
}

static bool TestPacketTx(void)
{
	constexpr uint16_t mps = 8;
	constexpr uint16_t blockSize = sizeof(UsbPktHdr_t) + mps;
	alignas(4) uint8_t rxMem[CFIFO_TOTAL_MEMSIZE(2, blockSize)];
	alignas(4) uint8_t txMem[CFIFO_TOTAL_MEMSIZE(4, blockSize)];
	alignas(4) uint8_t txDma[mps] = {};
	UsbDevIntrf_t intrf = {};
	s_XferCnt = 0;
	CHECK(Init(&intrf, rxMem, sizeof(rxMem), txMem, sizeof(txMem), blockSize));
	UsbIntrfConfigTx(&intrf, USB_ENDPADDR_DIRIN(1), mps, txDma);

	alignas(4) uint8_t packets[blockSize * 3] = {};
	UsbPktHdr_t *p0 = reinterpret_cast<UsbPktHdr_t *>(packets);
	UsbPktHdr_t *p1 = reinterpret_cast<UsbPktHdr_t *>(packets + blockSize);
	UsbPktHdr_t *p2 = reinterpret_cast<UsbPktHdr_t *>(packets + blockSize * 2);
	p0->Length = 3;
	memcpy(p0 + 1, "abc", 3);
	p1->Length = 0;
	p2->Length = mps;
	memcpy(p2 + 1, "12345678", mps);

	CHECK(DeviceIntrfTxData(&intrf.DevIntrf, packets, sizeof(packets)) ==
		(int)sizeof(packets));
	CHECK(s_XferCnt == 1 && s_Xfer[0].Length == 3);
	CHECK(memcmp(txDma, "abc", 3) == 0);
	UsbIntrfTxXferComplete(&intrf, 3, USBD_CTRLR_XFER_SUCCESS);
	CHECK(s_XferCnt == 2 && s_Xfer[1].Length == 0);
	UsbIntrfTxXferComplete(&intrf, 0, USBD_CTRLR_XFER_SUCCESS);
	CHECK(s_XferCnt == 3 && s_Xfer[2].Length == mps);
	CHECK(memcmp(txDma, "12345678", mps) == 0);
	return true;
}

int main(void)
{
	CHECK(TestRx());
	CHECK(TestByteTx());
	CHECK(TestPacketTx());
	printf("Result: PASS (3/3)\n");
	return 0;
}
