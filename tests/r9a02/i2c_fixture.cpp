/**-------------------------------------------------------------------------
@file	i2c_fixture.cpp

@brief	R9A02G021 I2C master, emulator firmware

IICA1 (DevNo 1, SDAA1 P010, SCLA1 P011, the pin map of the I2CScan example)
and the simplified I2C IIC20 (DevNo 6, SDA20 P102, SCL20 P101). i2c_emu.py
loads it, calls the Emu functions, plays the slave devices on the bus and
delivers the interrupts through R9a02IrqDispatch. main only waits.

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
#include <string.h>
#include "coredev/iopincfg.h"
#include "coredev/i2c.h"

I2C g_I2c;
uint8_t g_TxBuf[64];
uint8_t g_RxBuf[64];
volatile int g_EvtCnt;
volatile int g_EvtLen;

// IICA1 as the I2CScan example: SDA bidirectional, SCL output
static const IOPinCfg_t s_IicaPins[2] = {
	{0, 10, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{0, 11, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
};

// IIC20 as the comment of the I2CScan board.h
static const IOPinCfg_t s_SauPins[2] = {
	{1, 2, IOPINOP_FUNC16, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
	{1, 1, IOPINOP_FUNC16, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
};

static int Evt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	if (EvtId == DEVINTRF_EVT_COMPLETED)
	{
		g_EvtLen = Len;
		g_EvtCnt = g_EvtCnt + 1;
	}

	return 0;
}

// DevNo 1 (IICA1) or 6 (IIC20). Returns the rate set, 0 when I2CInit fails.
extern "C" int EmuI2cInit(int DevNo, int Rate, int bIntEn)
{
	I2CCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.DevNo = DevNo;
	cfg.Type = I2CTYPE_STANDARD;
	cfg.Mode = I2CMODE_MASTER;
	cfg.pIOPinMap = DevNo == 1 ? s_IicaPins : s_SauPins;
	cfg.NbIOPins = 2;
	cfg.Rate = (uint32_t)Rate;
	cfg.MaxRetry = 0;
	cfg.AddrType = I2CADDR_TYPE_NORMAL;
	cfg.bDmaEn = false;
	cfg.bIntEn = bIntEn != 0;
	cfg.IntPrio = 3;
	cfg.EvtCB = Evt;

	return g_I2c.Init(cfg) ? (int)g_I2c.Rate() : 0;
}

// The scan of the I2CScan example: 1 when the address is acknowledged
extern "C" int EmuI2cProbe(int Addr)
{
	if (g_I2c.StartTx((uint32_t)Addr))
	{
		g_I2c.StopTx();

		return 1;
	}

	return 0;
}

// Start in the read direction then stop, without data: 1 when the address
// is acknowledged
extern "C" int EmuI2cProbeRx(int Addr)
{
	if (g_I2c.StartRx((uint32_t)Addr))
	{
		g_I2c.StopRx();

		return 1;
	}

	return 0;
}

// Register Reg then Len bytes of g_TxBuf. Returns what Write returns.
extern "C" int EmuI2cWrite(int Addr, int Reg, int Len)
{
	uint8_t reg = (uint8_t)Reg;

	return g_I2c.Write((uint32_t)Addr, &reg, 1, g_TxBuf, Len);
}

// Register Reg, restart, Len bytes to g_RxBuf. Returns what Read returns.
extern "C" int EmuI2cRead(int Addr, int Reg, int Len)
{
	uint8_t reg = (uint8_t)Reg;

	return g_I2c.Read((uint32_t)Addr, &reg, 1, g_RxBuf, Len);
}

// Len bytes to g_RxBuf without a command. Returns what Rx returns.
extern "C" int EmuI2cRx(int Addr, int Len)
{
	return g_I2c.Rx((uint32_t)Addr, g_RxBuf, Len);
}

// Len bytes of g_TxBuf without a command. Returns what Tx returns.
extern "C" int EmuI2cTx(int Addr, int Len)
{
	return g_I2c.Tx((uint32_t)Addr, g_TxBuf, Len);
}

extern "C" void EmuI2cReset(void)
{
	g_I2c.Reset();
}

extern "C" int EmuI2cRate(int Rate)
{
	return (int)g_I2c.Rate((uint32_t)Rate);
}

extern "C" void EmuI2cPowerOff(void)
{
	g_I2c.PowerOff();
}

int main()
{
	while (1)
	{
	}

	return 0;
}
