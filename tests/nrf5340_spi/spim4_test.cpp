/**-------------------------------------------------------------------------
@file	spim4_test.cpp

@brief	Host test support: spim4 test.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
----------------------------------------------------------------------------*/

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "coredev/spi.h"
#include "spi_nrfx.h"
#include "coredev/shared_intrf.h"

NRF_SPIM_Type g_Regs[5] = {};
NRF_QSPI_Type g_Qspi = {};
NRF_GPIO_Type g_Gpio = {};
uint32_t SystemCoreClock = 128000000;
bool g_IrqEnabled[6], g_CsHigh = true;
const int g_SharedIntrfMaxCnt = 4;
SharedIntrf_t g_SharedIntrf[4] = {};
extern nRFSpiDev_t g_nRFxSPIDev[];
extern "C" void SPIM4_IRQHandler(void);
static bool s_Poll, s_Timeout, s_QspiCalled, s_AutoIrq, s_StopFail;
static int s_Events, s_Count, s_Blocks;
static DEVINTRF_EVT s_Event;
static uint8_t *s_Buffer;
static uint32_t s_LastPtr;
static uint8_t s_Rx[70000], s_Tx[800];
static SPI s_Spi;

void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
}
bool nRFxQSPIInit(SPIDev_t *)
{
	s_QspiCalled = true;
	return true;
}

static void Complete()
{
	auto &r = g_Regs[4];
	r.RXD.AMOUNT = r.RXD.MAXCNT;
	r.TXD.AMOUNT = r.TXD.MAXCNT;
	if (r.RXD.MAXCNT)
		memset((void *)(uintptr_t)r.RXD.PTR, 0x5a, r.RXD.MAXCNT);
	r.EVENTS_ENDRX = r.EVENTS_ENDTX = r.EVENTS_END = 1;
}

void TestTask::operator=(uint32_t value)
{
	if (!value)
		return;
	auto &r = g_Regs[4];
	if (this == &r.TASKS_STOP)
	{
		r.EVENTS_STOPPED = s_StopFail ? 0 : 1;
		return;
	}
	if (this != &r.TASKS_START)
		return;
	++s_Blocks;
	s_LastPtr = r.RXD.MAXCNT ? r.RXD.PTR : r.TXD.PTR;
	r.RXD.AMOUNT = r.TXD.AMOUNT = 0;
	if ((s_Poll || s_AutoIrq) && !s_Timeout)
	{
		Complete();
		if (s_AutoIrq)
			SPIM4_IRQHandler();
	}
}

static int Event(DevIntrf_t *dev, DEVINTRF_EVT event, uint8_t *buffer, int count)
{
	++s_Events; s_Event = event; s_Buffer = buffer; s_Count = count;
	assert(g_CsHigh);
	// Generic completion must release serialization before calling the client.
	assert(DeviceIntrfStartTx(dev, 0));
	DeviceIntrfStopTx(dev);
	return 0;
}

static IOPinCfg_t s_Pins[] = {
	{0, 8, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 10, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 9, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{0, 11, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}
};
static SPICfg_t Config(bool irq)
{
	SPICfg_t c = {};
	c.DevNo = 4; c.Mode = SPIMODE_MASTER; c.Phy = SPIPHY_NORMAL;
	c.pIOPinMap = s_Pins; c.NbIOPins = 4; c.Rate = 8000000; c.DataSize = 8;
	c.ChipSel = SPICSEL_AUTO; c.bIntEn = irq; c.bDmaEn = false;
	c.DummyByte = 0x37; c.EvtCB = Event;
	return c;
}

int main()
{
	SPIM4_IRQHandler(); // Safe before initialization.
	auto cfg = Config(true);
	assert(s_Spi.Init(cfg));
	DevIntrf_t *intrf = s_Spi;
	assert(intrf->bDma && g_IrqEnabled[SPIM4_IRQn]);
	assert(g_Regs[4].ORC == 0x37 && g_Regs[4].PSEL.MISO == 10);
	for (auto &entry : g_SharedIntrf)
		assert(entry.pIntrf == nullptr);
	assert(s_Spi.Rate(0) == 125000);
	assert(s_Spi.Rate(700000) == 500000);
	assert(s_Spi.Rate(32000000) == 16000000); // RX cannot operate at 32 Mbps.
	assert(s_Spi.Rate(8000000) == 8000000);
	for (unsigned i = 0; i < sizeof(s_Tx); ++i)
		s_Tx[i] = i;

	// Async TX owns a copy and completes through the actual dedicated vector.
	assert(s_Spi.Tx(0, s_Tx, 64) == -1);
	assert(!g_CsHigh && g_Regs[4].TXD.MAXCNT == 64);
	assert(memcmp((void *)(uintptr_t)g_Regs[4].TXD.PTR, s_Tx, 64) == 0);
	memset(s_Tx, 0xff, 64);
	assert(*(uint8_t *)(uintptr_t)g_Regs[4].TXD.PTR == 0);
	Complete(); SPIM4_IRQHandler();
	assert(s_Events == 1 && s_Count == 64 && s_Event == DEVINTRF_EVT_COMPLETED);
	SPIM4_IRQHandler(); assert(s_Events == 1);

	// More than one 16-bit EasyDMA block; buffer address advances explicitly.
	assert(s_Spi.Rx(0, s_Rx, sizeof(s_Rx)) == -1);
	assert(g_Regs[4].RXD.MAXCNT == 65535 && g_Regs[4].RXD.LIST == 0);
	Complete(); SPIM4_IRQHandler();
	assert(s_Events == 1 && g_Regs[4].RXD.PTR == (uintptr_t)s_Rx + 65535);
	assert(g_Regs[4].RXD.MAXCNT == sizeof(s_Rx) - 65535);
	Complete(); SPIM4_IRQHandler();
	assert(s_Events == 2 && s_Buffer == s_Rx && s_Count == sizeof(s_Rx));
	assert(s_Rx[0] == 0x5a && s_Rx[69999] == 0x5a);

	// Command-phase completion retains CS and the generic transaction lock.
	assert(s_Spi.StartTx(0)); intrf->bNoStop = true;
	assert(s_Spi.TxData(s_Tx, 2) == -1);
	Complete(); SPIM4_IRQHandler();
	assert(!g_CsHigh && intrf->bTxReady && s_Events == 2);
	intrf->bNoStop = false;
	assert(intrf->StartRx(intrf, 0));
	assert(s_Spi.RxData(s_Rx, 3) == -1);
	Complete(); SPIM4_IRQHandler(); assert(s_Events == 3);

	s_AutoIrq = true;
	int events = s_Events;
	assert(s_Spi.Tx(0, s_Tx, sizeof(s_Tx)) == -1);
	assert(s_Events == events + 1 && s_Count == sizeof(s_Tx));
	s_AutoIrq = false;

	// Long async TX cancellation never retains the caller's pointer.
	assert(s_Spi.Tx(0, s_Tx, sizeof(s_Tx)) == -1);
	assert(g_nRFxSPIDev[4].bTxAbort && g_nRFxSPIDev[4].pTxData == nullptr);
	SPIM4_IRQHandler();
	assert(s_Event == DEVINTRF_EVT_TX_TIMEOUT && !g_nRFxSPIDev[4].bTxAbort);

	cfg = Config(false); assert(s_Spi.Init(cfg)); s_Poll = true;
	s_Blocks = 0;
	assert(s_Spi.Tx(0, s_Tx, sizeof(s_Tx)) == sizeof(s_Tx));
	assert(s_Blocks == 4 && g_CsHigh);
	assert(s_LastPtr == (uintptr_t)g_nRFxSPIDev[4].TxBuf);
	s_Blocks = 0;
	assert(s_Spi.Rx(0, s_Rx, sizeof(s_Rx)) == sizeof(s_Rx));
	assert(s_Blocks == 2 && s_LastPtr == (uintptr_t)s_Rx + 65535);
	s_Timeout = true;
	assert(s_Spi.Rx(0, s_Rx, 16) == 0 && g_CsHigh);
	assert(g_Regs[4].EVENTS_STOPPED); s_Timeout = false;
	assert(s_Spi.Rx(0, s_Rx, 16) == 16);

	s_Timeout = s_StopFail = true;
	assert(s_Spi.Tx(0, s_Tx, 16) == 0 && g_Regs[4].ENABLE == 0);
	s_Timeout = s_StopFail = false;
	assert(s_Spi.Init(cfg));
	assert(s_Spi.Tx(0, s_Tx, 16) == 16);
	s_Spi.Disable(); assert(g_Regs[4].ENABLE == 0);
	s_Spi.Enable(); assert(g_Regs[4].ENABLE == SPIM_ENABLE_ENABLE_Enabled);

	// TX-only 32 Mbps requires the fixed pins and 128 MHz CPU clock.
	s_Pins[1].PinNo = -1;
	cfg.Rate = 32000000; assert(s_Spi.Init(cfg));
	assert(s_Spi.Rate() == 32000000 && g_Regs[4].PSEL.MISO == UINT32_MAX);
	assert(s_Spi.Rx(0, s_Rx, 1) == 0);
	assert(s_Spi.Tx(0, s_Tx, 1) == 1);
	assert((g_Gpio.PIN_CNF[8] & GPIO_PIN_CNF_MCUSEL_Msk) != 0);
	SystemCoreClock = 64000000; assert(s_Spi.Rate(32000000) == 16000000);
	assert((g_Gpio.PIN_CNF[8] & GPIO_PIN_CNF_MCUSEL_Msk) == 0);
	SystemCoreClock = 128000000; s_Pins[1].PinNo = 10;
	cfg = Config(false); cfg.Phy = SPIPHY_3WIRE; cfg.ChipSel = SPICSEL_MAN;
	s_Pins[1].PinNo = 9;
	assert(s_Spi.Init(cfg));
	assert(s_Spi.StartTx(0));
	assert(g_Regs[4].PSEL.MOSI == 9 && g_Regs[4].PSEL.MISO == UINT32_MAX);
	s_Spi.StopTx();
	assert(s_Spi.StartRx(0));
	assert(g_Regs[4].PSEL.MISO == 9 && g_Regs[4].PSEL.MOSI == UINT32_MAX);
	s_Spi.StopRx(); s_Pins[1].PinNo = 10;
	cfg = Config(false); cfg.Mode = SPIMODE_MMASTER; assert(!s_Spi.Init(cfg));
	cfg = Config(false); cfg.Mode = SPIMODE_SLAVE; assert(!s_Spi.Init(cfg));
	cfg = Config(false); cfg.pIOPinMap = nullptr; assert(!s_Spi.Init(cfg));
	cfg = Config(false); cfg.DataSize = 16; assert(!s_Spi.Init(cfg));
	cfg = Config(false); cfg.NbIOPins = 3; assert(!s_Spi.Init(cfg));
	cfg = Config(false); cfg.DevNo = 5; cfg.Phy = SPIPHY_QUAD_SDR;
	assert(s_Spi.Init(cfg) && s_QspiCalled);
	puts("nRF5340 SPIM4 production driver: PASS");
}
