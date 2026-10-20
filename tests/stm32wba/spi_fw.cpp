/**-------------------------------------------------------------------------
@file	spi_fw.cpp

@brief	STM32WBA SPI master test firmware

Runs on the instruction set emulator of run_spi.py, which models the SPI
registers of the STM32WBA65, the chip select pins and a slave that returns
a known byte sequence. Each test calls the driver through the generic SPI
interface of the library and records its result in g_SpiTestResult.
SpiTestMark tells the emulator which test starts, so that it can check the
transfers of each test.

Not hardware validation.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdint.h>
#include <string.h>

#include "coredev/spi.h"
#include "coredev/iopincfg.h"

#define SPI_TEST_MAX_RESULT		64

extern "C" volatile int32_t g_SpiTestResult[SPI_TEST_MAX_RESULT];
extern "C" volatile uint8_t g_SpiTestRx[512];
extern "C" volatile uint32_t g_SpiTestDone;
extern "C" volatile uint8_t g_SpiTestRx2[4][320];
extern "C" volatile int32_t g_SpiTestSlvEvt[64];
extern "C" volatile int32_t g_SpiTestSlvCnt;
extern "C" volatile uint8_t g_SpiTestSlvRx[24];
extern "C" volatile uint8_t g_SpiTestSlvSnap[4][24];

extern "C" void SPI1_IRQHandler(void);
extern "C" void SPI3_IRQHandler(void);
extern "C" void SpiTestMark(int Id) __attribute__((noinline));
extern "C" void SpiTestSlaveStep(void) __attribute__((noinline));
extern "C" void SpiTestDoneHook(void) __attribute__((noinline));

volatile int32_t g_SpiTestResult[SPI_TEST_MAX_RESULT];
volatile uint8_t g_SpiTestRx[512];
volatile uint32_t g_SpiTestDone;
volatile uint8_t g_SpiTestRx2[4][320];
volatile int32_t g_SpiTestSlvEvt[64];
volatile int32_t g_SpiTestSlvCnt;
volatile uint8_t g_SpiTestSlvRx[24];
volatile uint8_t g_SpiTestSlvSnap[4][24];

extern "C" void SpiTestMark(int Id)
{
	__asm volatile ("" : : "r" (Id) : "memory");
}

extern "C" void SpiTestSlaveStep(void)
{
	__asm volatile ("" ::: "memory");
}

extern "C" void SpiTestDoneHook(void)
{
	__asm volatile ("" ::: "memory");
}

// SPI1 on the DK1 header: SCK PB4, MISO PB3, MOSI PA15, AF5. Two chip
// selects on PA12 and PA11.
static const IOPinCfg_t s_Spi1Pins[] = {
	{IOPORTB, 4, IOPINOP_FUNC5, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTB, 3, IOPINOP_FUNC5, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTA, 15, IOPINOP_FUNC5, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTA, 12, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTA, 11, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

// SPI3 without a chip select pin; the fourth entry is the NSS pin of the slave
static const IOPinCfg_t s_Spi3Pins[] = {
	{IOPORTA, 0, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTA, 1, IOPINOP_FUNC6, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTB, 8, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{IOPORTA, 4, IOPINOP_FUNC6, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static SPIDev_t s_Spi1;
static SPIDev_t s_Spi3;
static uint8_t s_Tx[512];
static uint8_t s_Big[70000];
static uint8_t s_SlvTx[8];

static int s_SlvDone;
static volatile int s_MasterDone;
static volatile int s_MasterCount;

static int MasterCb(DevIntrf_t * const pDev, DEVINTRF_EVT Evt, uint8_t *pBuff, int Len)
{
	if (Evt == DEVINTRF_EVT_COMPLETED)
	{
		s_MasterCount = Len;
		s_MasterDone = 1;
	}

	return 0;
}

// Interrupt mode: the vector is called from here, the emulator has no NVIC
static int WaitMaster(void)
{
	for (int i = 0; i < 200000 && !s_MasterDone; i++)
	{
		SPI1_IRQHandler();
	}

	return s_MasterDone ? s_MasterCount : -2;
}

// Slave events recorded by the callback: 1 arm, 2 completed, and the length
static int SlaveCb(DevIntrf_t * const pDev, DEVINTRF_EVT Evt, uint8_t *pBuff, int Len)
{
	int n = g_SpiTestSlvCnt;

	if (Evt == DEVINTRF_EVT_STATECHG)
	{
		SPISetSlaveRxBuffer(SPIGetHandle(pDev), 0, (uint8_t*)g_SpiTestSlvRx, 20);
		SPISetSlaveTxData(SPIGetHandle(pDev), 0, s_SlvTx, sizeof(s_SlvTx));
	}
	if (Evt == DEVINTRF_EVT_COMPLETED && pBuff != NULL && s_SlvDone < 4)
	{
		memcpy((void*)g_SpiTestSlvSnap[s_SlvDone], pBuff, 20);
		s_SlvDone++;
	}
	if ((Evt == DEVINTRF_EVT_STATECHG || Evt == DEVINTRF_EVT_COMPLETED) && n < 31)
	{
		g_SpiTestSlvEvt[n * 2] = Evt == DEVINTRF_EVT_STATECHG ? 1 : 2;
		g_SpiTestSlvEvt[n * 2 + 1] = Len;
		g_SpiTestSlvCnt = n + 1;
	}

	return 0;
}

static SPICfg_t MakeCfg(int DevNo, const IOPinCfg_t *pPins, int NbPins, uint32_t Rate)
{
	SPICfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.DevNo = DevNo;
	cfg.Phy = SPIPHY_NORMAL;
	cfg.Mode = SPIMODE_MASTER;
	cfg.pIOPinMap = pPins;
	cfg.NbIOPins = NbPins;
	cfg.Rate = Rate;
	cfg.DataSize = 8;
	cfg.MaxRetry = 0;
	cfg.BitOrder = SPIDATABIT_MSB;
	cfg.DataPhase = SPIDATAPHASE_FIRST_CLK;
	cfg.ClkPol = SPICLKPOL_HIGH;
	cfg.ChipSel = SPICSEL_AUTO;
	cfg.DummyByte = 0xA5;

	return cfg;
}

extern "C" int main(void)
{
	int r = 0;
	SPICfg_t cfg = MakeCfg(0, s_Spi1Pins, 5, 1000000);

	for (int i = 0; i < (int)sizeof(s_Tx); i++)
	{
		s_Tx[i] = (uint8_t)(i * 7 + 3);
	}

	// 0 : init at 1 MHz, rate reported
	SpiTestMark(0);
	g_SpiTestResult[r++] = SPIInit(&s_Spi1, &cfg) ? 1 : 0;
	g_SpiTestResult[r++] = (int32_t)SPIGetRate(&s_Spi1);

	// 1 : transmit 5 bytes on chip select 0
	SpiTestMark(1);
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 0, s_Tx, 5);

	// 2 : receive 6 bytes, the dummy byte is clocked out
	SpiTestMark(2);
	g_SpiTestResult[r++] = SPIRx(&s_Spi1, 0, (uint8_t*)g_SpiTestRx, 6);

	// 3 : command then read, chip select stays low
	SpiTestMark(3);
	uint8_t cmd = 0x9F;
	g_SpiTestResult[r++] = DeviceIntrfRead(&s_Spi1.DevIntrf, 0, &cmd, 1, (uint8_t*)&g_SpiTestRx[16], 4);

	// 4 : command and data in one transmit
	SpiTestMark(4);
	uint8_t wc[2] = { 0x02, 0x10 };
	g_SpiTestResult[r++] = DeviceIntrfWrite(&s_Spi1.DevIntrf, 0, wc, 2, s_Tx, 3);

	// 5 : 70000 bytes, two hardware transfers
	SpiTestMark(5);
	for (int i = 0; i < (int)sizeof(s_Big); i++)
	{
		s_Big[i] = (uint8_t)i;
	}
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 0, s_Big, sizeof(s_Big));

	// 6 : second chip select, and one that does not exist
	SpiTestMark(6);
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 1, s_Tx, 2);
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 2, s_Tx, 2);

	// 7 : rate request: 100 kHz, 50 MHz, then 0
	SpiTestMark(7);
	g_SpiTestResult[r++] = (int32_t)SPISetRate(&s_Spi1, 100000);
	g_SpiTestResult[r++] = (int32_t)SPISetRate(&s_Spi1, 50000000);
	g_SpiTestResult[r++] = (int32_t)SPISetRate(&s_Spi1, 0);

	// 8 : mode 3, LSB first, 12 bit data
	SpiTestMark(8);
	SPICfg_t c12 = MakeCfg(0, s_Spi1Pins, 5, 4000000);
	c12.ClkPol = SPICLKPOL_LOW;
	c12.DataPhase = SPIDATAPHASE_SECOND_CLK;
	c12.BitOrder = SPIDATABIT_LSB;
	c12.DataSize = 12;
	c12.DummyByte = 0x5A;
	g_SpiTestResult[r++] = SPIInit(&s_Spi1, &c12) ? 1 : 0;
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 0, s_Tx, 6);
	g_SpiTestResult[r++] = SPIRx(&s_Spi1, 0, (uint8_t*)&g_SpiTestRx[32], 4);

	// 9 : chip select left to the application
	SpiTestMark(9);
	SPICfg_t cman = MakeCfg(0, s_Spi1Pins, 5, 1000000);
	cman.ChipSel = SPICSEL_MAN;
	g_SpiTestResult[r++] = SPIInit(&s_Spi1, &cman) ? 1 : 0;
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 0, s_Tx, 3);

	// 10 : unsupported configurations are refused
	SpiTestMark(10);
	SPICfg_t bad = MakeCfg(1, s_Spi1Pins, 5, 1000000);
	SPIDev_t baddev;
	bad.Mode = SPIMODE_SLAVE;
	g_SpiTestResult[r++] = SPIInit(&baddev, &bad) ? 1 : 0;
	bad.Mode = SPIMODE_MASTER;
	bad.Phy = SPIPHY_3WIRE;
	g_SpiTestResult[r++] = SPIInit(&baddev, &bad) ? 1 : 0;
	bad.Phy = SPIPHY_NORMAL;
	bad.bDmaEn = true;
	g_SpiTestResult[r++] = SPIInit(&baddev, &bad) ? 1 : 0;
	bad.bDmaEn = false;
	bad.bIntEn = true;
	g_SpiTestResult[r++] = SPIInit(&baddev, &bad) ? 1 : 0;
	bad.bIntEn = false;
	bad.DataSize = 3;
	g_SpiTestResult[r++] = SPIInit(&baddev, &bad) ? 1 : 0;
	bad.DataSize = 17;
	g_SpiTestResult[r++] = SPIInit(&baddev, &bad) ? 1 : 0;
	bad.DataSize = 8;
	bad.DevNo = 3;
	g_SpiTestResult[r++] = SPIInit(&baddev, &bad) ? 1 : 0;

	// 11 : SPI3 on APB7, no chip select pin
	SpiTestMark(11);
	SPICfg_t cfg3 = MakeCfg(2, s_Spi3Pins, 3, 8000000);
	g_SpiTestResult[r++] = SPIInit(&s_Spi3, &cfg3) ? 1 : 0;
	g_SpiTestResult[r++] = (int32_t)SPIGetRate(&s_Spi3);
	g_SpiTestResult[r++] = SPITx(&s_Spi3, 0, s_Tx, 3);
	g_SpiTestResult[r++] = SPIRx(&s_Spi3, 0, (uint8_t*)&g_SpiTestRx[48], 3);

	// 12 : interrupt master on SPI1, the end is the completed event
	SpiTestMark(12);
	SPICfg_t cfgi = MakeCfg(0, s_Spi1Pins, 5, 1000000);
	cfgi.bIntEn = true;
	cfgi.EvtCB = MasterCb;
	g_SpiTestResult[r++] = SPIInit(&s_Spi1, &cfgi) ? 1 : 0;
	s_MasterDone = 0;
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 0, s_Tx, 5);
	g_SpiTestResult[r++] = WaitMaster();
	s_MasterDone = 0;
	g_SpiTestResult[r++] = SPIRx(&s_Spi1, 0, (uint8_t*)&g_SpiTestRx[64], 6);
	g_SpiTestResult[r++] = WaitMaster();
	s_MasterDone = 0;
	g_SpiTestResult[r++] = SPIRx(&s_Spi1, 0, (uint8_t*)g_SpiTestRx2[0], 300);
	g_SpiTestResult[r++] = WaitMaster();
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 0, s_Tx, 300);

	// 13 : DMA master, polling wait, then 12 bit data
	SpiTestMark(13);
	SPICfg_t cfgd = MakeCfg(0, s_Spi1Pins, 5, 1000000);
	cfgd.bDmaEn = true;
	g_SpiTestResult[r++] = SPIInit(&s_Spi1, &cfgd) ? 1 : 0;
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 0, s_Tx, 5);
	g_SpiTestResult[r++] = SPIRx(&s_Spi1, 0, (uint8_t*)g_SpiTestRx2[1], 300);
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 0, s_Tx, 300);
	cfgd.DataSize = 12;
	g_SpiTestResult[r++] = SPIInit(&s_Spi1, &cfgd) ? 1 : 0;
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 0, s_Tx, 6);
	g_SpiTestResult[r++] = SPIRx(&s_Spi1, 0, (uint8_t*)&g_SpiTestRx[96], 4);

	// 14 : DMA master, interrupt
	SpiTestMark(14);
	cfgd.DataSize = 8;
	cfgd.bIntEn = true;
	cfgd.EvtCB = MasterCb;
	g_SpiTestResult[r++] = SPIInit(&s_Spi1, &cfgd) ? 1 : 0;
	s_MasterDone = 0;
	g_SpiTestResult[r++] = SPITx(&s_Spi1, 0, s_Tx, 129);
	g_SpiTestResult[r++] = WaitMaster();
	s_MasterDone = 0;
	g_SpiTestResult[r++] = SPIRx(&s_Spi1, 0, (uint8_t*)g_SpiTestRx2[2], 129);
	g_SpiTestResult[r++] = WaitMaster();

	// 15 : slave on SPI3, polling transfers of the interrupt, a scripted master
	SpiTestMark(15);
	for (int i = 0; i < (int)sizeof(s_SlvTx); i++)
	{
		s_SlvTx[i] = (uint8_t)(0xC0 + i);
	}
	SPICfg_t cfgs = MakeCfg(2, s_Spi3Pins, 4, 1000000);
	cfgs.Mode = SPIMODE_SLAVE;
	cfgs.EvtCB = SlaveCb;
	cfgs.DummyByte = 0x3C;
	g_SpiTestResult[r++] = SPIInit(&s_Spi3, &cfgs) ? 1 : 0;
	for (int i = 0; i < 6000; i++)
	{
		SpiTestSlaveStep();
		SPI3_IRQHandler();
	}

	// 16 : slave with DMA
	SpiTestMark(16);
	g_SpiTestSlvCnt = 0;
	s_SlvDone = 0;
	cfgs.bDmaEn = true;
	g_SpiTestResult[r++] = SPIInit(&s_Spi3, &cfgs) ? 1 : 0;
	for (int i = 0; i < 6000; i++)
	{
		SpiTestSlaveStep();
		SPI3_IRQHandler();
	}

	SpiTestMark(17);
	g_SpiTestResult[SPI_TEST_MAX_RESULT - 1] = r;
	g_SpiTestDone = 1;
	SpiTestDoneHook();

	while (1)
	{
	}

	return 0;
}

// Minimal start: stack, data and bss, then main. SystemInit is not needed.
extern "C" uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

extern "C" void ResetHandler(void)
{
	uint32_t *src = &_sidata;

	for (uint32_t *dst = &_sdata; dst < &_edata; )
	{
		*dst++ = *src++;
	}
	for (uint32_t *dst = &_sbss; dst < &_ebss; )
	{
		*dst++ = 0;
	}
	main();
	while (1)
	{
	}
}

extern "C" __attribute__((section(".isr_vector"), used))
void (* const __Vectors[16 + 128])(void) = {
	(void (*)(void))&_estack,
	ResetHandler,
};
