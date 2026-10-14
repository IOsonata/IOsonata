/**-------------------------------------------------------------------------
@file	i2c_fixture.cpp

@brief	R9A02G021 I2C master and slave, emulator firmware

IICA1 (DevNo 1, SDAA1 P010, SCLA1 P011, the pin map of the I2CScan example)
and the simplified I2C IIC20 (DevNo 6, SDA20 P102, SCL20 P101) as master.
IICA0 (DevNo 0, SDAA0_B P103, SCLA0_B P102) as slave, wired to IICA1 as in
the I2CMasterSlave example; its event handler is the one of
exemples/i2c/i2c_master_slave.cpp: a write gives the offset in the memory
then the data, a read continues at the offset. i2c_emu.py loads it, calls
the Emu functions, plays the other slave devices on the bus and delivers the
interrupts through R9a02IrqDispatch. main only waits.

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

#define SLAVE_MEM_SIZE		32
#define SLAVE_EVT_MAX		16

I2C g_I2c;
uint8_t g_TxBuf[64];
uint8_t g_RxBuf[64];
volatile int g_EvtCnt;
volatile int g_EvtLen;

// Slave: memory, write buffer, lengths given at the requests (-1: none),
// events as (EvtId << 16) | Len
I2C g_I2cSlave;
uint8_t g_SlvMem[SLAVE_MEM_SIZE];
uint8_t g_SlvWr[SLAVE_MEM_SIZE];
volatile int g_SlvWrLen = SLAVE_MEM_SIZE;
volatile int g_SlvRdLen = SLAVE_MEM_SIZE;
volatile int g_SlvOffset;
volatile int g_SlvEvt[SLAVE_EVT_MAX];
volatile int g_SlvEvtCnt;
static bool s_bSlvWrite;

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

// IICA0 slave as the I2CMasterSlave example: SDAA0_B P103, SCLA0_B P102
static const IOPinCfg_t s_SlavePins[2] = {
	{1, 3, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{1, 2, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
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

// The slave handler of i2c_master_slave.cpp, the lengths given set by the
// test
static int SlaveEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	int n = g_SlvEvtCnt;

	if (n < SLAVE_EVT_MAX)
	{
		g_SlvEvt[n] = ((int)EvtId << 16) | (Len & 0xFFFF);
		g_SlvEvtCnt = n + 1;
	}

	switch (EvtId)
	{
		case DEVINTRF_EVT_READ_RQST:
			if (s_bSlvWrite && Len > 0 && g_SlvWr[0] < SLAVE_MEM_SIZE)
			{
				g_SlvOffset = g_SlvWr[0];
			}
			s_bSlvWrite = false;
			if (g_SlvRdLen >= 0)
			{
				int l = SLAVE_MEM_SIZE - g_SlvOffset;

				g_I2cSlave.SetReadRqstData(0, &g_SlvMem[g_SlvOffset], l < g_SlvRdLen ? l : g_SlvRdLen);
			}
			break;

		case DEVINTRF_EVT_WRITE_RQST:
			s_bSlvWrite = true;
			if (g_SlvWrLen >= 0)
			{
				g_I2cSlave.SetWriteRqstBuffer(0, g_SlvWr, g_SlvWrLen);
			}
			break;

		case DEVINTRF_EVT_COMPLETED:
			if (!s_bSlvWrite)
			{
				// A read without command continues after the bytes sent
				g_SlvOffset = (g_SlvOffset + Len) % SLAVE_MEM_SIZE;
			}
			else if (Len > 1 && g_SlvWr[0] < SLAVE_MEM_SIZE)
			{
				int l = Len - 1;

				if (l > SLAVE_MEM_SIZE - g_SlvWr[0])
				{
					l = SLAVE_MEM_SIZE - g_SlvWr[0];
				}
				memcpy(&g_SlvMem[g_SlvWr[0]], &g_SlvWr[1], l);
			}
			s_bSlvWrite = false;
			break;

		default:
			break;
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

// Start with Addr, the register byte, restart with Other in the read
// direction, one byte if it answers, stop. Returns the register bytes sent
// x 10 + 1 when Other answered. With bIntEn the byte and the stop come in
// the background.
extern "C" int EmuI2cRestartOther(int Addr, int Other)
{
	DevIntrf_t *intrf = g_I2c;
	uint8_t reg = 0x01;
	int n = 0;
	bool ok = false;

	if (DeviceIntrfStartTx(intrf, (uint32_t)Addr))
	{
		intrf->bNoStop = true;
		n = intrf->TxSrData(intrf, &reg, 1);
		intrf->bNoStop = false;
		ok = intrf->StartRx(intrf, (uint32_t)Other);
		if (!ok || intrf->RxData(intrf, g_RxBuf, 1) >= 0)
		{
			DeviceIntrfStopRx(intrf);
		}
	}

	return n * 10 + (ok ? 1 : 0);
}

// IICA0 slave at Addr. NbAddr, bIntEn and the bus are checked by I2CInit.
// Returns the rate kept, 0 when I2CInit fails.
extern "C" int EmuI2cSlaveInit(int Addr, int Rate, int NbAddr, int bIntEn)
{
	I2CCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.DevNo = 0;
	cfg.Type = I2CTYPE_STANDARD;
	cfg.Mode = I2CMODE_SLAVE;
	cfg.pIOPinMap = s_SlavePins;
	cfg.NbIOPins = 2;
	cfg.Rate = (uint32_t)Rate;
	cfg.MaxRetry = 0;
	cfg.AddrType = I2CADDR_TYPE_NORMAL;
	cfg.NbSlaveAddr = NbAddr;
	cfg.SlaveAddr[0] = (uint8_t)Addr;
	cfg.SlaveAddr[1] = (uint8_t)(Addr + 1);
	cfg.bDmaEn = false;
	cfg.bIntEn = bIntEn != 0;
	cfg.IntPrio = 7;
	cfg.EvtCB = SlaveEvt;
	g_SlvEvtCnt = 0;
	g_SlvOffset = 0;
	s_bSlvWrite = false;

	return g_I2cSlave.Init(cfg) ? (int)g_I2cSlave.Rate() : 0;
}

extern "C" int EmuI2cSlaveRate(int Rate)
{
	return (int)g_I2cSlave.Rate((uint32_t)Rate);
}

extern "C" void EmuI2cSlaveReset(void)
{
	g_I2cSlave.Reset();
}

extern "C" void EmuI2cSlaveDisable(void)
{
	g_I2cSlave.Disable();
}

extern "C" void EmuI2cSlaveEnable(void)
{
	g_I2cSlave.Enable();
}

extern "C" void EmuI2cSlavePowerOff(void)
{
	g_I2cSlave.PowerOff();
}

// A master call on the slave object: 0, nothing is sent
extern "C" int EmuI2cSlaveAsMaster(int Addr)
{
	return g_I2cSlave.StartTx((uint32_t)Addr) ? 1 : 0;
}

int main()
{
	while (1)
	{
	}

	return 0;
}
