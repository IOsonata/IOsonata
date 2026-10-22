// Host checks for caller-buffer lifetime. run_lifetime.py supplies unchanged
// production function bodies; these registers model tasks and delayed IRQs.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include "device_intrf.h"

using std::min;
using std::max;

#define TWIM_PRESENT 1
#define TWI_PRESENT 1
#define SPIM_PRESENT 1
#define SPI_PRESENT 1
#define I2C_TIMEOUT_CNT 8
#define NRFSPI_TIMEOUT 8
#ifndef NRFX_I2C_TXBUF_SIZE
#define NRFX_I2C_TXBUF_SIZE 64
#endif
#define NRFX_SPI_TXBUF_SIZE 256
#define NRFX_I2C_DMA_MAXCNT TEST_DMA_MAX
#define NRFX_SPI_DMA_MAXCNT TEST_DMA_MAX
#define NRFX_I2C_ASYNC_IDLE 0
#define NRFX_I2C_ASYNC_TX 1
#define NRFX_I2C_ASYNC_RX 2
#define NRFX_SPI_RX(r) ((r)->RxDma)
#define NRFX_SPI_TX(r) ((r)->TxDma)
#define NRFX_SPI_ENDRX(r) ((r)->EVENTS_ENDRX)
#define NRFX_SPI_ENDTX(r) ((r)->EVENTS_ENDTX)
#define NRFX_SPI_TX_LIST 0
#define NRFX_SPI_RX_LIST 0
#define NRFX_TWI_RX(r) ((r)->RxDma)
#define NRFX_TWI_TX(r) ((r)->TxDma)
#define NRFX_TWI_STARTRX(r) ((r)->TASKS_STARTRX)
#define NRFX_TWI_STARTTX(r) ((r)->TASKS_STARTTX)
#define NRFX_TWI_RXSTARTED(r) ((r)->EVENTS_RXSTARTED)
#define NRFX_TWI_TXSTARTED(r) ((r)->EVENTS_TXSTARTED)
#define TWIM_SHORTS_LASTRX_STOP_Msk 1
#define TWIM_SHORTS_LASTTX_STOP_Msk 2
#define TWIM_SHORTS_LASTTX_SUSPEND_Msk 4
#define TWIM_SHORTS_LASTTX_SUSPEND_Enabled 1
#define TWIM_SHORTS_LASTTX_SUSPEND_Pos 2
#define TWI_SHORTS_BB_SUSPEND_Msk 8
#define TWI_SHORTS_BB_STOP_Msk 16
#define SPIM_INTENSET_STOPPED_Msk 32
#define SPIM_INTENCLR_STOPPED_Msk 32
#define SPIMODE_SLAVE 1
#define SPICSEL_AUTO 0
#define SPICSEL_MAN 1
#define SPIPHY_3WIRE 1
#define SPI_CS_IOPIN_IDX 3
#define SPI_MISO_IOPIN_IDX 1
#define SPI_MOSI_IOPIN_IDX 2

static int checks, failures, tasks, txBytes, csLow, csHigh;
#define CHECK(x) \
	do \
	{ \
		checks++; \
		if (!(x)) \
		{ \
			failures++; \
			printf("line %d: %s\n", __LINE__, #x); \
		} \
	} while (0)

struct Task
{
	uint32_t Value = 0;
	Task &operator=(uint32_t v)
	{
		Value = v;
		if (v)
		{
			tasks++;
		}
		return *this;
	}
	operator uint32_t() const
	{
		return Value;
	}
};
struct TxByte
{
	uint32_t Value = 0;
	TxByte &operator=(uint32_t v)
	{
		Value = v;
		txBytes++;
		return *this;
	}
	operator uint32_t() const
	{
		return Value;
	}
};
struct Dma
{
	uint32_t PTR = 0, MAXCNT = 0, AMOUNT = 0, LIST = 0;
};
struct Reg
{
	Task TASKS_START, TASKS_STARTTX, TASKS_STARTRX, TASKS_STOP, TASKS_RESUME;
	Task TASKS_ACQUIRE, TASKS_RELEASE;
	uint32_t EVENTS_STOPPED = 0, EVENTS_SUSPENDED = 0, EVENTS_ERROR = 0;
	uint32_t EVENTS_LASTTX = 0, EVENTS_LASTRX = 0, EVENTS_TXSTARTED = 0, EVENTS_RXSTARTED = 0;
	uint32_t EVENTS_END = 0, EVENTS_ENDRX = 0, EVENTS_ENDTX = 0, EVENTS_STARTED = 0;
	uint32_t EVENTS_TXDSENT = 0, EVENTS_RXDREADY = 0, EVENTS_READY = 0, EVENTS_ACQUIRED = 0;
	uint32_t INTENSET = 0, INTENCLR = 0, ENABLE = 0, SHORTS = 0, ERRORSRC = 0;
	uint32_t ADDRESS = 0, STATUS = 0, RXD = 0;
	TxByte TXD;
	Dma RxDma, TxDma;
	struct
	{
		uint32_t MOSI = 0, MISO = 0;
	} PSEL;
};
using NRF_TWIM_Type = Reg;
using NRF_TWI_Type = Reg;
using NRF_SPIM_Type = Reg;
using NRF_SPI_Type = Reg;
struct Pin
{
	int PortNo, PinNo;
};
struct Config
{
	uint32_t Rate = 1000000;
	int Mode = 0, ChipSel = SPICSEL_AUTO, Phy = 0, NbIOPins = 4;
	Pin *pIOPinMap;
	uint8_t DummyByte = 0xA5;
};
struct Port
{
	DevIntrf_t DevIntrf;
	Config Cfg;
	int CurDevCs = 0;
	uint8_t *pRxBuff[1];
	const uint8_t *pTxData[1];
	int RxBuffLen[1], TxDataLen[1];
};
struct nRFTwiDev_t
{
	Port *pI2cDev;
	Reg *pReg, *pDmaReg;
	int RxBufflen = 0, RxIdx = 0;
	uint8_t *pRxBuff = NULL;
	int TxDatalen = 0, TxIdx = 0;
	const uint8_t *pTxData = NULL;
	volatile int Async = 0;
	volatile bool bAsyncErr = false, bStopped = false;
	bool bTxRejected = false;
	alignas(4) uint8_t TxBuf[NRFX_I2C_TXBUF_SIZE];
};
struct nRFSpiDev_t
{
	Port *pSpiDev;
	Reg *pReg, *pDmaReg, *pDmaSReg;
	int RxBufflen = 0, RxIdx = 0;
	uint8_t *pRxBuff = NULL;
	int TxDatalen = 0, TxIdx = 0;
	const uint8_t *pTxData = NULL;
	volatile bool *pTxWait = NULL;
	volatile bool bTxAbort = false;
	bool bCmdFailed = false;
	alignas(4) uint8_t TxBuf[NRFX_SPI_TXBUF_SIZE];
};
static nRFSpiDev_t g_nRFxSPIDev[1];
static nRFTwiDev_t i2cDev;
static Port i2c, spi;
static Reg i2cReg, spiReg;
static Pin pins[4] = {};
static uint8_t source[70000], receive[32];
static uint32_t primask;
static void (*beforeMask)(), (*afterMask)();
static uint32_t __get_PRIMASK()
{
	return primask;
}
static void __disable_irq()
{
	if (beforeMask)
	{
		auto f = beforeMask;
		beforeMask = NULL;
		f();
	}
	primask = 1;
}
static void __set_PRIMASK(uint32_t mask)
{
	primask = mask;
	if (mask == 0 && afterMask)
	{
		auto f = afterMask;
		afterMask = NULL;
		f();
	}
}
static uint32_t SystemCoreClock = 1;
static void IOPinSet(int, int)
{
	csHigh++;
}
static void IOPinClear(int, int)
{
	csLow++;
}
static bool nRFxI2CWaitStop(nRFTwiDev_t *, int)
{
	CHECK(false);
	return false;
}
static bool nRFxI2CWaitRxComplete(nRFTwiDev_t *, int)
{
	CHECK(false);
	return false;
}
static bool nRFxI2CWaitTxComplete(nRFTwiDev_t *, int)
{
	CHECK(false);
	return false;
}
static bool nRFxSPIWaitReady(nRFSpiDev_t *, uint32_t)
{
	CHECK(false);
	return false;
}
static bool nRFxSPIWaitDMA(nRFSpiDev_t *, uint32_t)
{
	CHECK(false);
	return false;
}
static bool nRFxSPIWaitRX(nRFSpiDev_t *, uint32_t)
{
	CHECK(false);
	return false;
}

#include "driver_lifetime.inc"

static int completed, timedout, lastCount, txAttempts, replacementResult;
static bool replaceTx, replaceTimeout, stopBeforeRx, stopOnRxUnlock;
static int Event(DevIntrf_t *p, DEVINTRF_EVT evt, uint8_t *, int count)
{
	if (evt == DEVINTRF_EVT_COMPLETED)
	{
		completed++;
	}
	if (evt == DEVINTRF_EVT_TX_TIMEOUT)
	{
		timedout++;
	}
	lastCount = count;
	if (replaceTx && evt == DEVINTRF_EVT_COMPLETED)
	{
		replaceTx = false;
		CHECK(DeviceIntrfTx(p, 0, source, 8) == -1);
	}
	if (replaceTimeout && evt == DEVINTRF_EVT_TX_TIMEOUT)
	{
		replaceTimeout = false;
		replacementResult = DeviceIntrfTx(p, 0, source, 8);
	}
	return 0;
}
static void SpiStop();
static bool I2cStartTx(DevIntrf_t *p, uint32_t addr)
{
	txAttempts++;
	return nRFxI2CStartTx(p, addr);
}
static bool SpiStartRx(DevIntrf_t *p, uint32_t addr)
{
	if (stopBeforeRx)
	{
		stopBeforeRx = false;
		SpiStop();
	}
	if (stopOnRxUnlock)
	{
		stopOnRxUnlock = false;
		afterMask = SpiStop;
	}
	return nRFxSPIStartRx(p, addr);
}
static bool Busy(DevIntrf_t *p)
{
	bool was = atomic_flag_test_and_set(&p->bBusy);
	if (!was)
	{
		atomic_flag_clear(&p->bBusy);
	}
	return was;
}
static void Init(bool dma)
{
	i2cReg = Reg{};
	spiReg = Reg{};
	i2cDev = nRFTwiDev_t{};
	g_nRFxSPIDev[0] = nRFSpiDev_t{};
	i2cDev.pI2cDev = &i2c;
	i2cDev.pReg = i2cDev.pDmaReg = &i2cReg;
	auto &d = g_nRFxSPIDev[0];
	d.pSpiDev = &spi;
	d.pReg = d.pDmaReg = d.pDmaSReg = &spiReg;
	for (auto p : { &i2c, &spi })
	{
		p->Cfg.pIOPinMap = pins;
		auto &v = p->DevIntrf;
		v.bDma = dma;
		v.bIntEn = true;
		v.bNoStop = false;
		v.bTxReady = true;
		v.MaxRetry = 2;
		v.EvtCB = Event;
		v.TxSrData = NULL;
		atomic_flag_clear(&v.bBusy);
	}
	auto &i = i2c.DevIntrf;
	auto &s = spi.DevIntrf;
	i.pDevData = &i2cDev;
	s.pDevData = &d;
	i.StartTx = I2cStartTx;
	i.StopTx = nRFxI2CStopTx;
	i.StartRx = nRFxI2CStartRx;
	i.StopRx = nRFxI2CStopRx;
	i.TxData = dma ? nRFxI2CTxDataDMA : nRFxI2CTxData;
	i.RxData = dma ? nRFxI2CRxDataDMA : nRFxI2CRxData;
	s.StartTx = nRFxSPIStartTx;
	s.StopTx = nRFxSPIStopTx;
	s.StartRx = SpiStartRx;
	s.StopRx = nRFxSPIStopRx;
	s.TxData = dma ? nRFxSPITxDataDma : nRFxSPITxData;
	s.RxData = dma ? nRFxSPIRxDataDma : nRFxSPIRxData;
	completed = timedout = tasks = txBytes = csLow = csHigh = lastCount = 0;
	primask = 0;
	beforeMask = afterMask = NULL;
	replaceTx = false;
	txAttempts = 0;
	replacementResult = -2;
	replaceTimeout = stopBeforeRx = stopOnRxUnlock = false;
	for (unsigned n = 0; n < sizeof(source); n++)
	{
		source[n] = (uint8_t)n;
	}
}
static void I2cFinish(int n, bool dma)
{
	if (dma)
	{
		i2cReg.TxDma.AMOUNT = n;
		i2cReg.EVENTS_STOPPED = 1;
		nRFxI2CTwimIrq(&i2c.DevIntrf, &i2cDev);
	}
	else
	{
		for (int k = 0; k < n; k++)
		{
			i2cReg.EVENTS_TXDSENT = 1;
			nRFxI2CTwiIrq(&i2c.DevIntrf, &i2cDev);
		}
		i2cReg.EVENTS_STOPPED = 1;
		nRFxI2CTwiIrq(&i2c.DevIntrf, &i2cDev);
	}
}
static void SpiComplete()
{
	auto &d = g_nRFxSPIDev[0];
	while (spi.DevIntrf.bDma ? d.TxDatalen > 0 : d.TxIdx > 0)
	{
		if (spi.DevIntrf.bDma)
		{
			CHECK(spiReg.TxDma.PTR == (uintptr_t)d.TxBuf);
			CHECK(spiReg.TxDma.MAXCNT <= min(NRFX_SPI_TXBUF_SIZE, NRFX_SPI_DMA_MAXCNT));
			CHECK(memcmp(d.TxBuf, source + d.TxIdx, spiReg.TxDma.MAXCNT) == 0);
			spiReg.TxDma.AMOUNT = spiReg.TxDma.MAXCNT;
			spiReg.EVENTS_END = 1;
		}
		else
		{
			spiReg.EVENTS_READY = 1;
		}
		SPI_IRQHandler(0, &spi.DevIntrf);
		if (completed)
		{
			break;
		}
	}
}
static void PendingSpiIrq()
{
	int starts = tasks;
	spiReg.EVENTS_END = 1;
	SPI_IRQHandler(0, &spi.DevIntrf);
	CHECK(tasks == starts && completed == 0 && timedout == 0);
}
static void SpiStop()
{
	if (spi.DevIntrf.bDma)
	{
		spiReg.EVENTS_STOPPED = 1;
	}
	else
	{
		spiReg.EVENTS_READY = 1;
	}
	SPI_IRQHandler(0, &spi.DevIntrf);
}
static void SpiReceive()
{
	completed = 0;
	CHECK(DeviceIntrfRx(&spi.DevIntrf, 0, receive, sizeof(receive)) == -1);
	CHECK(Busy(&spi.DevIntrf));
	if (spi.DevIntrf.bDma)
	{
		CHECK(spiReg.RxDma.PTR == (uintptr_t)receive);
		spiReg.RxDma.AMOUNT = sizeof(receive);
		spiReg.EVENTS_END = 1;
		SPI_IRQHandler(0, &spi.DevIntrf);
	}
	else
	{
		for (unsigned i = 0; i < sizeof(receive); i++)
		{
			spiReg.EVENTS_READY = 1;
			SPI_IRQHandler(0, &spi.DevIntrf);
		}
	}
	CHECK(completed == 1 && !Busy(&spi.DevIntrf));
}
int main()
{
	for (bool dma : {false, true})
	{
		int limit = dma ? min(NRFX_I2C_TXBUF_SIZE, NRFX_I2C_DMA_MAXCNT) : NRFX_I2C_TXBUF_SIZE;
		for (int operation = 0; operation < 3; operation++)
		{
			Init(dma);
			auto p = &i2c.DevIntrf;
			int r = operation == 0 ? DeviceIntrfTx(p, 0x22, source, limit + 1) :
				operation == 1 ? DeviceIntrfWrite(p, 0x22, source, 1, source + 1, limit) :
				DeviceIntrfRead(p, 0x22, source, limit + 1, receive, sizeof(receive));
			CHECK(r == 0 && tasks == 0 && txBytes == 0);
			CHECK(txAttempts == 3);
			CHECK(!Busy(p) && p->bTxReady && completed == 0 && timedout == 0);
			CHECK(!i2cDev.bTxRejected);
			CHECK(DeviceIntrfTx(p, 0x22, source, limit) == -1);
			CHECK(memcmp(i2cDev.TxBuf, source, limit) == 0);
			memset(source, 0x55, limit);
			I2cFinish(limit, dma);
			CHECK(completed == 1 && lastCount == limit && !Busy(p));
		}
		for (int length : {NRFX_SPI_TXBUF_SIZE, NRFX_SPI_TXBUF_SIZE + 9, 66000})
		{
			Init(dma);
			if (length > NRFX_SPI_TXBUF_SIZE)
			{
				beforeMask = SpiComplete;
			}
			CHECK(DeviceIntrfTx(&spi.DevIntrf, 0, source, length) == -1);
			if (length <= NRFX_SPI_TXBUF_SIZE)
			{
				SpiComplete();
			}
			CHECK(completed == 1 && lastCount == length && timedout == 0 && !Busy(&spi.DevIntrf));
			CHECK(csLow == 1 && csHigh == 1);
		}

		Init(dma);
		replaceTx = true;
		beforeMask = SpiComplete;
		CHECK(DeviceIntrfTx(&spi.DevIntrf, 0, source, NRFX_SPI_TXBUF_SIZE + 9) == -1);
		CHECK(completed == 1 && timedout == 0 && Busy(&spi.DevIntrf));
		CHECK(!g_nRFxSPIDev[0].bTxAbort && g_nRFxSPIDev[0].pTxWait == NULL);
		completed = 0;
		SpiComplete();
		CHECK(completed == 1 && !Busy(&spi.DevIntrf));

		Init(dma);
		CHECK(DeviceIntrfWrite(&spi.DevIntrf, 0, source, 1,
			source + 1, NRFX_SPI_TXBUF_SIZE + 8) == 0);
		CHECK(g_nRFxSPIDev[0].bTxAbort && g_nRFxSPIDev[0].pTxData == NULL);
		CHECK(g_nRFxSPIDev[0].pTxWait == NULL && Busy(&spi.DevIntrf));
		SpiStop();
		CHECK(timedout == 1 && completed == 0 && !Busy(&spi.DevIntrf));

		Init(dma);
		if (dma)
		{
			afterMask = PendingSpiIrq;
		}
		CHECK(DeviceIntrfTx(&spi.DevIntrf, 0, source, NRFX_SPI_TXBUF_SIZE + 9) == -1);
		auto &d = g_nRFxSPIDev[0];
		CHECK(d.bTxAbort && d.pTxData == NULL && d.pTxWait == NULL);
		CHECK(Busy(&spi.DevIntrf) && !spi.DevIntrf.bTxReady && timedout == 0);
		CHECK(csHigh == 0 && completed == 0);
		int before = tasks;
		CHECK(DeviceIntrfTx(&spi.DevIntrf, 0, source, 8) == 0 && tasks == before);
		memset(source, 0x55, sizeof(source));
		if (dma)
		{
			CHECK(spiReg.TxDma.PTR == (uintptr_t)d.TxBuf);
		}
		SpiStop();
		CHECK(timedout == 1 && completed == 0 && !Busy(&spi.DevIntrf));
		CHECK(!d.bTxAbort && spi.DevIntrf.bTxReady && csHigh == 1);
		SpiStop();
		CHECK(timedout == 1 && completed == 0);
		CHECK(DeviceIntrfTx(&spi.DevIntrf, 0, source, 8) == -1);
		SpiComplete();
		CHECK(completed == 1 && !Busy(&spi.DevIntrf));

		Init(dma);
		spi.DevIntrf.MaxRetry = 0;
		afterMask = SpiStop;
		CHECK(DeviceIntrfRead(&spi.DevIntrf, 0, source, NRFX_SPI_TXBUF_SIZE + 9,
			receive, sizeof(receive)) == 0);
		CHECK(timedout == 1 && completed == 0 && !Busy(&spi.DevIntrf));
		CHECK(spiReg.RxDma.MAXCNT == 0 && !d.bCmdFailed);
		SpiReceive();

		// STOPPED after bNoStop clears must not let the timeout callback
		// replace the failed command before the old read rejects its RX.
		Init(dma);
		spi.DevIntrf.MaxRetry = 0;
		stopBeforeRx = replaceTimeout = true;
		CHECK(DeviceIntrfRead(&spi.DevIntrf, 0, source, NRFX_SPI_TXBUF_SIZE + 9,
			receive, sizeof(receive)) == 0);
		CHECK(timedout == 1 && replacementResult == 0 && !Busy(&spi.DevIntrf));
		CHECK(spiReg.RxDma.MAXCNT == 0 && !d.bCmdFailed && !d.bTxAbort);
		SpiReceive();

		// Once RxData consumes the failed command, a late stop can release
		// the bus and start another transmit without the old read stopping it.
		Init(dma);
		spi.DevIntrf.MaxRetry = 0;
		stopOnRxUnlock = replaceTimeout = true;
		CHECK(DeviceIntrfRead(&spi.DevIntrf, 0, source, NRFX_SPI_TXBUF_SIZE + 9,
			receive, sizeof(receive)) == -1);
		CHECK(timedout == 1 && replacementResult == -1 && Busy(&spi.DevIntrf));
		CHECK(spiReg.RxDma.MAXCNT == 0 && !d.bCmdFailed && !d.bTxAbort);
		SpiComplete();
		CHECK(completed == 1 && !Busy(&spi.DevIntrf));
		SpiReceive();

		Init(dma);
		spi.DevIntrf.MaxRetry = 0;
		CHECK(DeviceIntrfRead(&spi.DevIntrf, 0, source, NRFX_SPI_TXBUF_SIZE + 9,
			receive, sizeof(receive)) == -1);
		CHECK(timedout == 0 && d.bTxAbort && !d.bCmdFailed && Busy(&spi.DevIntrf));
		SpiStop();
		CHECK(timedout == 1 && !Busy(&spi.DevIntrf));
		SpiReceive();
	}
	printf("Nordic transfer lifetime: %d checks, %d failures\n", checks, failures);
	return failures != 0;
}
