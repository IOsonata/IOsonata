/**-------------------------------------------------------------------------
@file	spi_slave_fixture.cpp

@brief	R9A02G021 SPI slave emulator firmware

SPI21 (DevNo 5) slave on P204 (SCK21), P206 (SO21, MISO), P205 (SI21, MOSI),
chip select P202 (IRQ2). spi_slave_emu.py loads it, calls EmuInit and the
other Emu functions, acts as the master and delivers the interrupts through
R9a02IrqDispatch. main only waits.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#include "coredev/spi.h"
#include "iopinctrl.h"
#include "sau_r9a02.h"
#include "system_r9a02.h"

SPI g_Slave;
uint8_t g_Rx[256];
uint8_t g_Tx[256];
volatile int g_RxLen = 64;
volatile int g_TxLen = 64;
volatile int g_Done;
volatile int g_Count;
volatile int g_Arms;
uint8_t * volatile g_LastBuf;

// SCK21 P204, MISO SO21 P206, MOSI SI21 P205, CS P202 (IRQ2). The fifth
// pin is for the check that SPIInit refuses five pins.
// SPI20 for the reception in Snooze: SCK20 P101, MISO SO20 P103, MOSI SI20
// P102, CS P202 (IRQ2)
static const IOPinCfg_t s_Pins20[4] = {
	{1, 1, IOPINOP_FUNC16, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{1, 3, IOPINOP_FUNC16, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{1, 2, IOPINOP_FUNC16, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 2, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static IOPinCfg_t s_Pins[5] = {
	{2, 4, IOPINOP_FUNC16, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 6, IOPINOP_FUNC16, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 5, IOPINOP_FUNC16, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{2, 2, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{1, 1, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static int Evt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	if (EvtId == DEVINTRF_EVT_STATECHG)
	{
		g_Arms = g_Arms + 1;
		g_Slave.SetSlaveRxBuffer(0, g_Rx, g_RxLen);
		g_Slave.SetSlaveTxData(0, g_Tx, g_TxLen);
	}
	else if (EvtId == DEVINTRF_EVT_COMPLETED)
	{
		g_Count = Len;
		g_LastBuf = pBuffer;
		g_Done = g_Done + 1;
	}
	return 0;
}

// Flags: bit 0 DMA, bit 1 7-bit frames, bit 2 no interrupt, bit 3 five pins,
// bit 5 chip select on P104 (no IRQ line), bit 6 SPICSEL_MAN. Mode 0 to 3,
// Rate in Hz. Returns 1 when SPIInit succeeds.
extern "C" int EmuInit(int Flags, int Mode, int Rate)
{
	SPICfg_t cfg = {
		5, SPIPHY_NORMAL, SPIMODE_SLAVE, s_Pins, 4, (uint32_t)Rate, 8, 0, SPIDATABIT_MSB,
		SPIDATAPHASE_FIRST_CLK, SPICLKPOL_HIGH, SPICSEL_AUTO, false, true, 6, 0xFF, Evt
	};

	cfg.bDmaEn = (Flags & 1) != 0;
	cfg.DataSize = (Flags & 2) ? 7 : 8;
	cfg.bIntEn = (Flags & 4) == 0;
	cfg.NbIOPins = (Flags & 8) ? 5 : 4;
	s_Pins[3].PortNo = 2;
	s_Pins[3].PinNo = 2;
	if (Flags & 0x20)
	{
		s_Pins[3].PortNo = 1;
		s_Pins[3].PinNo = 4;
	}
	if (Flags & 0x40)
	{
		cfg.ChipSel = SPICSEL_MAN;
	}
	cfg.ClkPol = (Mode & 2) ? SPICLKPOL_LOW : SPICLKPOL_HIGH;
	cfg.DataPhase = (Mode & 1) ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK;

	return g_Slave.Init(cfg) ? 1 : 0;
}

extern "C" int EmuReset(void)
{
	g_Slave.Reset();
	return 1;
}

extern "C" int EmuDisable(void)
{
	g_Slave.Disable();
	return 1;
}

extern "C" int EmuEnable(void)
{
	g_Slave.Enable();
	return 1;
}

extern "C" int EmuPowerOff(void)
{
	g_Slave.PowerOff();
	return 1;
}

extern "C" int EmuRate(int Rate)
{
	return (int)g_Slave.Rate((uint32_t)Rate);
}

// SPI20 slave, DevNo 4. Flags bit 0: DMA. Returns 1 when SPIInit succeeds.
extern "C" int EmuInit20(int Flags, int Rate, int Mode)
{
	SPICfg_t cfg = {
		4, SPIPHY_NORMAL, SPIMODE_SLAVE, s_Pins20, 4, (uint32_t)Rate, 8, 0, SPIDATABIT_MSB,
		SPIDATAPHASE_FIRST_CLK, SPICLKPOL_HIGH, SPICSEL_AUTO, (Flags & 1) != 0, true, 6, 0xFF, Evt
	};

	cfg.DataPhase = (Mode & 1) ? SPIDATAPHASE_SECOND_CLK : SPIDATAPHASE_FIRST_CLK;
	cfg.ClkPol = (Mode & 2) ? SPICLKPOL_LOW : SPICLKPOL_HIGH;

	return g_Slave.Init(cfg) ? 1 : 0;
}

// Reception in Snooze on or off: returns 1 on success
extern "C" int EmuSnooze(int bEnable)
{
	return R9a02SpiSnooze(g_Slave, bEnable != 0) ? 1 : 0;
}

// Software Standby, the harness plays the time in WFI
// Another Snooze set up (IRQ3 request) on or off
extern "C" int EmuOtherSnooze(int bEnable)
{
	static const R9a02SnoozeCfg_t cfg = {
		.Request = R9A02_SNZREQ_IRQ(3),
		.End = 0,
		.WakeEvt = 0,
		.bDtc = false,
		.Handler = NULL,
		.pCtx = NULL,
		.IntPrio = 0,
	};

	return R9a02SnoozeSet(bEnable ? &cfg : NULL) ? 1 : 0;
}

extern "C" int EmuStandby(void)
{
	return R9a02LowPowerEnter(true) ? 1 : 0;
}

// A master transfer on a slave, refused: returns 0
extern "C" int EmuMasterTx(void)
{
	uint8_t d[2] = {1, 2};
	return g_Slave.Tx(0, d, 2);
}

int main()
{
	while (1)
	{
		__asm volatile ("nop");
	}
	return 0;
}
