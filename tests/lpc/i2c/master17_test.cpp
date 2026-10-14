/**-------------------------------------------------------------------------
@file	master17_test.cpp

@brief	LPC17xx I2C master register-model regression.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
// Register-model regression of the actual LPC17xx polling master driver.
#include <cassert>
#include <cstdio>
#include <deque>
#include "LPC17xx.h"
#include "coredev/i2c.h"

struct Step {
	int Status;
	uint8_t Data;
	int Ack;
};

struct Control {
	uint32_t Value = 0;
	operator uint32_t() const;
	Control &operator=(uint32_t Bits);
};

struct Clear {
	Clear &operator=(uint32_t Bits);
};

struct Regs {
	Control I2CONSET;
	Clear I2CONCLR;
	uint32_t I2STAT = 0xF8;
	uint32_t I2DAT = 0;
	uint32_t I2SCLL = 0;
	uint32_t I2SCLH = 0;
	uint32_t MMCTRL = UINT32_MAX;
};

struct Syscon {
	uint32_t PCONP = 0;
	uint32_t PCLKSEL0 = UINT32_MAX;
	uint32_t PCLKSEL1 = UINT32_MAX;
	uint32_t I2CPADCFG = UINT32_MAX;
};

static Regs s_Regs;
static Syscon s_Syscon;
static std::deque<Step> s_Steps;
static int s_Stops = 0;
static int s_Starts = 0;
static int s_PinConfigs = 0;
static int s_Resets = 0;
static bool s_StopStuck = false;
static uint8_t s_Writes[32];
static int s_WriteCount = 0;
uint32_t SystemCoreClock = 120000000;
uint32_t g_IrqMasked = 0;

Control::operator uint32_t() const
{
	return Value;
}

Control &Control::operator=(uint32_t Bits)
{
	Value |= Bits;
	if (Bits & 16U)
	{
		s_Stops++;
	}
	return *this;
}

Clear &Clear::operator=(uint32_t Bits)
{
	assert((Bits & ~0x6CU) == 0);
	bool start = (s_Regs.I2CONSET.Value & 32U) != 0;
	s_Regs.I2CONSET.Value &= ~Bits;
	if (Bits & 64U)
	{
		s_Regs.I2CONSET.Value &= ~16U;
		s_Resets++;
	}
	if ((Bits & 8U) && (s_Regs.I2CONSET.Value & 64U))
	{
		if (s_Regs.I2CONSET.Value & 16U)
		{
			if (!s_StopStuck)
			{
				s_Regs.I2CONSET.Value &= ~16U;
			}
			return *this;
		}
		if (start && !(Bits & 32U))
		{
			s_Starts++;
		}
		if (s_Steps.empty())
		{
			return *this;
		}
		Step step = s_Steps.front();
		s_Steps.pop_front();
		if (step.Status < 0)
		{
			return *this;
		}
		if (step.Ack >= 0)
		{
			assert(((s_Regs.I2CONSET.Value & 4U) != 0) == (step.Ack != 0));
		}
		if (step.Status == 0x18 || step.Status == 0x40 || step.Status == 0x28 || step.Status == 0x30)
		{
			s_Writes[s_WriteCount++] = s_Regs.I2DAT;
		}
		s_Regs.I2STAT = step.Status;
		s_Regs.I2DAT = step.Data;
		s_Regs.I2CONSET.Value |= 8U;
	}
	return *this;
}

void NVIC_DisableIRQ(IRQn_Type Irq)
{
	assert(Irq == I2C0_IRQn + TEST_DEVNO);
}

void NVIC_ClearPendingIRQ(IRQn_Type Irq)
{
	assert(Irq == I2C0_IRQn + TEST_DEVNO);
}

void IOPinConfig(int Port, int Pin, int Op, IOPINDIR, IOPINRES, IOPINTYPE)
{
	assert(Port == 0);
	assert(Pin >= 0 && Op >= IOPINOP_FUNC1);
	s_PinConfigs++;
}

#undef LPC_I2C0
#undef LPC_I2C1
#undef LPC_I2C2
#undef LPC_SC
#undef LPC_PINCON
#define LPC_I2C_TypeDef Regs
// Only the selected instance is exercised in each process.
#define LPC_I2C0 (&s_Regs)
#define LPC_I2C1 (&s_Regs)
#define LPC_I2C2 (&s_Regs)
#define LPC_SC (&s_Syscon)
#define LPC_PINCON (&s_Syscon)
#include "../../../ARM/NXP/LPC17xx/src/i2c_lpc17xx.c"
#include "../../../ARM/NXP/src/i2c_lpc_master.cpp"
#include "../../../src/device_intrf.cpp"

int main()
{
	const IOPinCfg_t pins[] = {
		{0, TEST_DEVNO == 0 ? 27 : (TEST_DEVNO == 1 ? 0 : 10), TEST_DEVNO == 0 ? IOPINOP_FUNC1 : (TEST_DEVNO == 1 ? IOPINOP_FUNC3 : IOPINOP_FUNC2), IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
		{0, TEST_DEVNO == 0 ? 28 : (TEST_DEVNO == 1 ? 1 : 11), TEST_DEVNO == 0 ? IOPINOP_FUNC1 : (TEST_DEVNO == 1 ? IOPINOP_FUNC3 : IOPINOP_FUNC2), IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
	};
	I2CDev_t dev = {}, other = {};
	I2CCfg_t cfg = {};
	cfg.DevNo = TEST_DEVNO;
	cfg.pIOPinMap = pins;
	cfg.NbIOPins = 2;
	cfg.Rate = 100000;
	assert(I2CInit(&dev, &cfg));
	DevIntrf_t *d = &dev.DevIntrf;
	assert(dev.Cfg.pIOPinMap == pins && I2CGetDevHandle(d) == &dev);
	assert(I2CGetRate(&dev) == 100000 && s_Regs.I2SCLL + s_Regs.I2SCLH == 300);
	assert(d->EnCnt == 1 && (s_Syscon.PCONP & s_ClockMask[TEST_DEVNO]));
	// Rejected initialization must preserve the current owner's clock.
	uint32_t savedCoreClock = SystemCoreClock;
	uint32_t savedClock = s_I2C[TEST_DEVNO].Clock;

	SystemCoreClock /= 2U;
	assert(!I2CInit(&other, &cfg));
	assert(s_I2C[TEST_DEVNO].Clock == savedClock);
	SystemCoreClock = savedCoreClock;
	assert(s_Regs.MMCTRL == 0);
	if (TEST_DEVNO == 0)
	{
		assert((s_Syscon.PCLKSEL0 & (3U << 14)) == 0);
		assert((s_Syscon.I2CPADCFG & 15U) == 0);
	}
	else
	{
		assert((s_Syscon.PCLKSEL1 & (3U << (TEST_DEVNO == 1 ? 6 : 20))) == 0);
	}
	// Even with valid pins a second instance cannot steal the same object.
	IOPinCfg_t alternate[] = {
		{0, TEST_DEVNO == 0 ? 10 : 27, TEST_DEVNO == 0 ? IOPINOP_FUNC2 : IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
		{0, TEST_DEVNO == 0 ? 11 : 28, TEST_DEVNO == 0 ? IOPINOP_FUNC2 : IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
	};
	I2CCfg_t alt = cfg;
	alt.DevNo = TEST_DEVNO == 0 ? 2 : 0;
	alt.pIOPinMap = alternate;
	assert(!I2CInit(&dev, &alt));
	cfg.DevNo = 3;
	assert(!I2CInit(&dev, &cfg));
	cfg.DevNo = TEST_DEVNO;
	cfg.bIntEn = true;
	assert(!I2CInit(&dev, &cfg));
	cfg.bIntEn = false;
	cfg.bDmaEn = true;
	assert(!I2CInit(&dev, &cfg));
	cfg.bDmaEn = false;
	cfg.Mode = I2CMODE_SLAVE;
	assert(!I2CInit(&dev, &cfg));
	cfg.Mode = I2CMODE_MASTER;
	cfg.AddrType = I2CADDR_TYPE_EXT;
	assert(!I2CInit(&dev, &cfg));
	cfg.AddrType = I2CADDR_TYPE_NORMAL;
	cfg.Rate = 0;
	assert(!I2CInit(&dev, &cfg));
	cfg.Rate = 100000;
	assert(I2CSetRate(&dev, 400000) == 400000);
	assert(s_Regs.I2SCLL >= 39 && s_Regs.I2SCLH >= 18);
	assert(I2CSetRate(&dev, 0) == 400000);
	assert(d->SetRate(d, UINT32_MAX) == 400000);
	assert(I2CSetRate(&dev, 1) == 228 && s_Regs.I2SCLL == 65535 && s_Regs.I2SCLH == 65535);
	assert(I2CSetRate(&dev, 100000) == 100000);
	uint8_t tx[] = {0x12, 0x34};
	uint8_t rx[] = {0xEE, 0xEE};
	// Complete write and STOP; the acknowledged bytes alone count.
	s_Steps = {{8, 0, -1}, {0x18, 0, -1}, {0x28, 0, -1}, {0x28, 0, -1}};
	assert(I2CTx(&dev, 0x22, tx, 2) == 2);
	assert(s_Steps.empty() && s_Stops == 1);
	assert(s_Writes[0] == 0x44 && s_Writes[1] == 0x12 && s_Writes[2] == 0x34);
	// Write command, repeated START, read with ACK then final NACK.
	s_Steps = {{8, 0, -1}, {0x18, 0, -1}, {0x28, 0, -1}, {0x10, 0, -1},
		{0x40, 0, -1}, {0x50, 0xA5, 1}, {0x58, 0x5A, 0}};
	assert(I2CRead(&dev, 0x22, tx, 1, rx, 2) == 2);
	assert(rx[0] == 0xA5 && rx[1] == 0x5A && s_Stops == 2 && s_Steps.empty());
	// Address NACK returns failure and releases the bus without a Stop caller.
	s_Steps = {{8, 0, -1}, {0x20, 0, -1}};
	assert(I2CTx(&dev, 0x22, tx, 2) == 0 && s_Stops == 3);
	// Data NACK is not counted as a sent byte.
	s_Steps = {{8, 0, -1}, {0x18, 0, -1}, {0x28, 0, -1}, {0x30, 0, -1}};
	assert(I2CTx(&dev, 0x22, tx, 2) == 1 && s_Stops == 4);
	// A failed command must not be followed by a read, despite generic Read's retry sequence.
	int starts = s_Starts;
	s_Steps = {{8, 0, -1}, {0x18, 0, -1}, {0x30, 0, -1}};
	assert(I2CRead(&dev, 0x22, tx, 1, rx, 2) == 0 && s_Starts == starts + 1);
	// Failed repeated address must never expose stale I2DAT to the receive buffer.
	rx[0] = 0xEE;
	s_Steps = {{8, 0, -1}, {0x18, 0, -1}, {0x28, 0, -1}, {0x10, 0, -1}, {0x48, 0xBA, -1}};
	assert(I2CRead(&dev, 0x22, tx, 1, rx, 1) == 0 && rx[0] == 0xEE);
	// Arbitration loss never sends STOP into the winning master's transfer.
	int stops = s_Stops;
	s_Steps = {{8, 0, -1}, {0x18, 0, -1}, {0x38, 0, -1}};
	assert(I2CTx(&dev, 0x22, tx, 1) == 0 && s_Stops == stops);
	// Bounded START timeout leaves the next operation usable.
	s_Steps = {{-1, 0, -1}};
	assert(I2CTx(&dev, 0x22, tx, 1) == 0);
	s_Steps = {{8, 0, -1}, {0x40, 0, -1}, {0x58, 0x77, 0}};
	assert(I2CRx(&dev, 0x22, rx, 1) == 1 && rx[0] == 0x77);
	// STOP stuck on the wire is bounded and resets the peripheral.
	s_StopStuck = true;
	int resets = s_Resets;
	s_Steps = {{8, 0, -1}, {0x18, 0, -1}, {0x28, 0, -1}};
	assert(I2CTx(&dev, 0x22, tx, 1) == 1 && s_Resets > resets);
	s_StopStuck = false;
	assert(!I2CStartTx(&dev, 0x80));
	// Reference-counted disable, clock restoration and retained rate.
	I2CEnable(&dev);
	I2CDisable(&dev);
	assert(s_I2C[TEST_DEVNO].bEnabled && d->EnCnt == 1);
	I2CDisable(&dev);
	assert(!s_I2C[TEST_DEVNO].bEnabled && !(s_Syscon.PCONP & s_ClockMask[TEST_DEVNO]));
	assert(!I2CStartTx(&dev, 0x22));
	assert(I2CSetRate(&dev, 400000) == 400000);
	I2CEnable(&dev);
	assert(s_I2C[TEST_DEVNO].bEnabled && s_Regs.I2SCLL + s_Regs.I2SCLH == 75);
	assert(s_PinConfigs >= 2);
	// The same object also cannot change the clock during a transfer.
	s_Steps = {{8, 0, -1}, {0x18, 0, -1}};
	assert(I2CStartTx(&dev, 0x22));
	SystemCoreClock /= 2U;
	assert(!I2CInit(&dev, &cfg));
	assert(s_I2C[TEST_DEVNO].Clock == savedClock);
	assert(s_I2C[TEST_DEVNO].bTransfer && s_I2C[TEST_DEVNO].bOwned);
	SystemCoreClock = savedCoreClock;
	I2CStopTx(&dev);
	DeviceIntrfReset(d);
	assert(I2CGetRate(&dev) == 400000);
	I2CPowerOff(&dev);
	assert(d->EnCnt == 0 && !s_I2C[TEST_DEVNO].bEnabled);
#if TEST_DEVNO == 1
	IOPinCfg_t alternateRoute[2] = {pins[0], pins[1]};

	alternateRoute[0].PinNo = 19;
	alternateRoute[1].PinNo = 20;
	cfg.pIOPinMap = alternateRoute;
	assert(I2CInit(&dev, &cfg));
	I2CPowerOff(&dev);
#endif
	puts("LPC17xx I2C master: PASS");
}
