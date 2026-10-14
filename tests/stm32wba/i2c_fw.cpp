/**-------------------------------------------------------------------------
@file	i2c_fw.cpp

@brief	STM32WBA I2C master test firmware

Runs on the instruction set emulator of run_i2c.py, which models the I2C
registers of the STM32WBA65 and the slave devices. Each test calls the
driver through the generic I2C interface of the library and records its
result in g_I2cTestResult. I2cTestMark tells the emulator which test
starts, so that it can check the bus transactions of each test.

Slave devices modeled by the emulator:
- 0x50 : memory, the first byte of a write is the pointer
- 0x51 : memory that refuses the third byte of a write

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
extern "C" volatile uint32_t g_I2cTestDone;
extern "C" uint32_t SystemCoreClock;

extern "C" void I2cTestMark(int Id) __attribute__((noinline));
extern "C" void I2cTestDoneHook(void) __attribute__((noinline));

volatile int32_t g_I2cTestResult[I2C_TEST_MAX_RESULT];
volatile uint8_t g_I2cTestRx[512];
volatile uint32_t g_I2cTestDone;

extern "C" void I2cTestMark(int Id)
{
	__asm volatile ("" : : "r" (Id) : "memory");
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

static I2CDev_t s_I2c1;
static I2CDev_t s_I2c3;
static uint8_t s_Tx[512];

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

	// 10 : unsupported modes are refused
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

	I2cTestMark(13);
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
