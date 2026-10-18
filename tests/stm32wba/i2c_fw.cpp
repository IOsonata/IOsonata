/**-------------------------------------------------------------------------
@file	i2c_fw.cpp

@brief	STM32WBA I2C test firmware, master and slave

Runs on the instruction set emulator of run_i2c.py, which models the I2C
registers of the STM32WBA65 and the slave devices. Each test calls the
driver through the generic I2C interface of the library and records its
result in g_I2cTestResult. I2cTestMark tells the emulator which test
starts, so that it can check the bus transactions of each test.

Slave devices modeled by the emulator:
- 0x50 : memory, the first byte of a write is the pointer
- 0x51 : memory that refuses the third byte of a write
I2C3 is a slave at 0x30 and 0x31 driven by a scripted bus master.

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

#include "coredev/i2c.h"
#include "coredev/iopincfg.h"

#define I2C_TEST_MAX_RESULT		64

extern "C" volatile int32_t g_I2cTestResult[I2C_TEST_MAX_RESULT];
extern "C" volatile uint8_t g_I2cTestRx[512];
extern "C" volatile uint8_t g_I2cTestRx2[3][320];
extern "C" volatile uint32_t g_I2cTestDone;
extern "C" uint32_t SystemCoreClock;

extern "C" void I2C1_EV_IRQHandler(void);
extern "C" void I2C3_EV_IRQHandler(void);
extern "C" void I2cTestMark(int Id) __attribute__((noinline));
extern "C" void I2cTestSlaveStep(void) __attribute__((noinline));
extern "C" void I2cTestDoneHook(void) __attribute__((noinline));

volatile int32_t g_I2cTestResult[I2C_TEST_MAX_RESULT];
volatile uint8_t g_I2cTestRx[512];
volatile uint8_t g_I2cTestRx2[3][320];
volatile uint32_t g_I2cTestDone;

extern "C" void I2cTestMark(int Id)
{
	__asm volatile ("" : : "r" (Id) : "memory");
}

extern "C" void I2cTestSlaveStep(void)
{
	__asm volatile ("" ::: "memory");
}

extern "C" void I2cTestDoneHook(void)
{
	__asm volatile ("" ::: "memory");
}

static const IOPinCfg_t s_I2c1Pins[] = {
	{IOPORTB, 1, IOPINOP_FUNC4, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{IOPORTB, 2, IOPINOP_FUNC4, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
};

static const IOPinCfg_t s_I2c3Pins[] = {
	{IOPORTA, 7, IOPINOP_FUNC4, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{IOPORTA, 6, IOPINOP_FUNC4, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
};

// Slave events recorded by the callback: 1 read request, 2 write request,
// 3 completed, and the length
#define SLV_BUF_SIZE		16
volatile int32_t g_I2cTestSlvEvt[64];
volatile int32_t g_I2cTestSlvCnt;
volatile uint8_t g_I2cTestSlvRx[2][SLV_BUF_SIZE];
volatile uint8_t g_I2cTestSlvRx2[2][SLV_BUF_SIZE];
volatile int32_t g_I2cTestSlvDmaInit;
static volatile uint8_t (*s_pSlvRx)[SLV_BUF_SIZE] = g_I2cTestSlvRx;
static uint8_t s_SlvTx[2][8];
static I2CDev_t s_I2c1;
static I2CDev_t s_I2c3;
static uint8_t s_Tx[512];

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
	for (int i = 0; i < 20000 && !s_MasterDone; i++)
	{
		I2C1_EV_IRQHandler();
	}

	return s_MasterDone ? s_MasterCount : -2;
}

static int SlaveCb(DevIntrf_t * const pDev, DEVINTRF_EVT Evt, uint8_t *pBuff, int Len)
{
	int code = Evt == DEVINTRF_EVT_READ_RQST ? 1 : (Evt == DEVINTRF_EVT_WRITE_RQST ? 2 : 3);
	int n = g_I2cTestSlvCnt;

	if (n < 31)
	{
		g_I2cTestSlvEvt[n * 2] = code;
		g_I2cTestSlvEvt[n * 2 + 1] = Len;
		g_I2cTestSlvCnt = n + 1;
	}
	if (code == 1)
	{
		for (int i = 0; i < 2; i++)
		{
			I2CSetReadRqstData(&s_I2c3, i, s_SlvTx[i], sizeof(s_SlvTx[i]));
		}
	}
	else if (code == 2)
	{
		for (int i = 0; i < 2; i++)
		{
			I2CSetWriteRqstBuffer(&s_I2c3, i, (uint8_t*)s_pSlvRx[i], SLV_BUF_SIZE);
		}
	}

	return 0;
}

static I2CCfg_t MakeCfg(int DevNo, const IOPinCfg_t *pPins, uint32_t Rate)
{
	I2CCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.DevNo = DevNo;
	cfg.Type = I2CTYPE_STANDARD;
	cfg.Mode = I2CMODE_MASTER;
	cfg.pIOPinMap = pPins;
	cfg.NbIOPins = 2;
	cfg.Rate = Rate;
	cfg.MaxRetry = 0;
	cfg.AddrType = I2CADDR_TYPE_NORMAL;

	return cfg;
}

extern "C" int main(void)
{
	int r = 0;
	I2CCfg_t cfg = MakeCfg(0, s_I2c1Pins, 100000);

	for (int i = 0; i < (int)sizeof(s_Tx); i++)
	{
		s_Tx[i] = (uint8_t)(i * 7 + 3);
	}

	// 0 : init at 100 kHz, rate reported
	I2cTestMark(0);
	g_I2cTestResult[r++] = I2CInit(&s_I2c1, &cfg) ? 1 : 0;
	g_I2cTestResult[r++] = (int32_t)I2CGetRate(&s_I2c1);

	// 1 : address probe, a device and none
	I2cTestMark(1);
	g_I2cTestResult[r++] = I2CStartTx(&s_I2c1, 0x50) ? 1 : 0;
	I2CStopTx(&s_I2c1);
	g_I2cTestResult[r++] = I2CStartTx(&s_I2c1, 0x23) ? 1 : 0;

	// 2 : write pointer and 3 data bytes
	I2cTestMark(2);
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x50, s_Tx, 4);

	// 3 : write 300 bytes, two chunks with reload
	I2cTestMark(3);
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x50, s_Tx, 300);

	// 4 : command then read, repeated start
	I2cTestMark(4);
	uint8_t cmd = 2;
	g_I2cTestResult[r++] = I2CRead(&s_I2c1, 0x50, &cmd, 1, (uint8_t*)g_I2cTestRx, 5);

	// 5 : read 300 bytes, two chunks with reload
	I2cTestMark(5);
	cmd = 0;
	g_I2cTestResult[r++] = I2CRead(&s_I2c1, 0x50, &cmd, 1, (uint8_t*)&g_I2cTestRx[8], 300);

	// 6 : write to a device that refuses the third byte
	I2cTestMark(6);
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x51, s_Tx, 5);

	// 7 : write to an absent device, then the bus is usable again
	I2cTestMark(7);
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x23, s_Tx, 3);
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x50, s_Tx, 2);

	// 8 : plain read without a command
	I2cTestMark(8);
	g_I2cTestResult[r++] = I2CRx(&s_I2c1, 0x50, (uint8_t*)&g_I2cTestRx[320], 3);

	// 9 : rate request, 400 kHz then 1 MHz then 0
	I2cTestMark(9);
	g_I2cTestResult[r++] = (int32_t)I2CSetRate(&s_I2c1, 400000);
	g_I2cTestResult[r++] = (int32_t)I2CSetRate(&s_I2c1, 1000000);
	g_I2cTestResult[r++] = (int32_t)I2CSetRate(&s_I2c1, 0);

	// 10 : slave mode without an address is refused, interrupt master is accepted
	I2cTestMark(10);
	I2CCfg_t bad = MakeCfg(1, s_I2c1Pins, 100000);
	I2CDev_t baddev;
	bad.Mode = I2CMODE_SLAVE;
	g_I2cTestResult[r++] = I2CInit(&baddev, &bad) ? 1 : 0;
	bad.Mode = I2CMODE_MASTER;
	bad.bIntEn = true;
	g_I2cTestResult[r++] = I2CInit(&baddev, &bad) ? 1 : 0;

	// 11 : I2C3 on APB7
	I2cTestMark(11);
	I2CCfg_t cfg3 = MakeCfg(2, s_I2c3Pins, 400000);
	g_I2cTestResult[r++] = I2CInit(&s_I2c3, &cfg3) ? 1 : 0;
	g_I2cTestResult[r++] = (int32_t)I2CGetRate(&s_I2c3);
	g_I2cTestResult[r++] = I2CTx(&s_I2c3, 0x50, s_Tx, 3);

	// 12 : 100 MHz bus clock, the clock of the PLL setup
	I2cTestMark(12);
	SystemCoreClock = 100000000;
	I2CCfg_t cfg100 = MakeCfg(0, s_I2c1Pins, 100000);
	g_I2cTestResult[r++] = I2CInit(&s_I2c1, &cfg100) ? (int32_t)I2CGetRate(&s_I2c1) : -1;
	g_I2cTestResult[r++] = (int32_t)I2CSetRate(&s_I2c1, 400000);
	g_I2cTestResult[r++] = (int32_t)I2CSetRate(&s_I2c1, 1000000);
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x50, s_Tx, 2);

	// 13 : interrupt master on I2C1, the end is the completed event
	I2cTestMark(13);
	SystemCoreClock = 16000000;
	I2CCfg_t cfgi = MakeCfg(0, s_I2c1Pins, 400000);
	cfgi.bIntEn = true;
	cfgi.EvtCB = MasterCb;
	g_I2cTestResult[r++] = I2CInit(&s_I2c1, &cfgi) ? 1 : 0;
	s_MasterDone = 0;
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x50, s_Tx, 5);
	g_I2cTestResult[r++] = WaitMaster();
	s_MasterDone = 0;
	g_I2cTestResult[r++] = I2CRx(&s_I2c1, 0x50, (uint8_t*)&g_I2cTestRx[400], 4);
	g_I2cTestResult[r++] = WaitMaster();
	s_MasterDone = 0;
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x51, s_Tx, 5);
	g_I2cTestResult[r++] = WaitMaster();
	s_MasterDone = 0;
	g_I2cTestResult[r++] = I2CRx(&s_I2c1, 0x50, (uint8_t*)g_I2cTestRx2[0], 300);
	g_I2cTestResult[r++] = WaitMaster();
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x50, s_Tx, 100);
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x23, s_Tx, 3);

	// 14 : slave on I2C3 with two addresses, the emulator plays the bus master
	I2cTestMark(14);
	for (int i = 0; i < 8; i++)
	{
		s_SlvTx[0][i] = (uint8_t)(0xA0 + i);
		s_SlvTx[1][i] = (uint8_t)(0xB0 + i);
	}
	I2CCfg_t cfgs = MakeCfg(2, s_I2c3Pins, 100000);
	cfgs.Mode = I2CMODE_SLAVE;
	cfgs.NbSlaveAddr = 2;
	cfgs.SlaveAddr[0] = 0x30;
	cfgs.SlaveAddr[1] = 0x31;
	cfgs.bIntEn = true;
	cfgs.EvtCB = SlaveCb;
	g_I2cTestResult[r++] = I2CInit(&s_I2c3, &cfgs) ? 1 : 0;
	for (int i = 0; i < 4000; i++)
	{
		I2cTestSlaveStep();
		I2C3_EV_IRQHandler();
	}

	// 15 : DMA master, polling wait
	I2cTestMark(15);
	I2CCfg_t cfgd = MakeCfg(0, s_I2c1Pins, 400000);
	cfgd.bDmaEn = true;
	g_I2cTestResult[r++] = I2CInit(&s_I2c1, &cfgd) ? 1 : 0;
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x50, s_Tx, 5);
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x50, s_Tx, 300);
	g_I2cTestResult[r++] = I2CRx(&s_I2c1, 0x50, (uint8_t*)&g_I2cTestRx[420], 4);
	g_I2cTestResult[r++] = I2CRx(&s_I2c1, 0x50, (uint8_t*)g_I2cTestRx2[1], 300);
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x51, s_Tx, 5);
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x23, s_Tx, 3);
	cmd = 7;
	g_I2cTestResult[r++] = I2CRead(&s_I2c1, 0x50, &cmd, 1, (uint8_t*)&g_I2cTestRx[430], 6);

	// 16 : DMA master, interrupt, the end is the completed event
	I2cTestMark(16);
	cfgd.bIntEn = true;
	cfgd.EvtCB = MasterCb;
	g_I2cTestResult[r++] = I2CInit(&s_I2c1, &cfgd) ? 1 : 0;
	s_MasterDone = 0;
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x50, s_Tx, 5);
	g_I2cTestResult[r++] = WaitMaster();
	s_MasterDone = 0;
	g_I2cTestResult[r++] = I2CRx(&s_I2c1, 0x50, (uint8_t*)&g_I2cTestRx[440], 4);
	g_I2cTestResult[r++] = WaitMaster();
	s_MasterDone = 0;
	g_I2cTestResult[r++] = I2CRx(&s_I2c1, 0x50, (uint8_t*)g_I2cTestRx2[2], 300);
	g_I2cTestResult[r++] = WaitMaster();
	s_MasterDone = 0;
	g_I2cTestResult[r++] = I2CTx(&s_I2c1, 0x51, s_Tx, 5);
	g_I2cTestResult[r++] = WaitMaster();

	I2cTestMark(17);

	// 18 : the same slave requests with the DMA channels
	I2cTestMark(18);
	s_pSlvRx = g_I2cTestSlvRx2;
	cfgs.bDmaEn = true;
	g_I2cTestSlvDmaInit = I2CInit(&s_I2c3, &cfgs) ? 1 : 0;
	for (int i = 0; i < 4000; i++)
	{
		I2cTestSlaveStep();
		I2C3_EV_IRQHandler();
	}

	g_I2cTestResult[I2C_TEST_MAX_RESULT - 1] = r;
	g_I2cTestDone = 1;
	I2cTestDoneHook();

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
