/**-------------------------------------------------------------------------
@file	joystick_display_fixture.h

@brief	Shared fixture of the joystick menu, stopwatch and countdown display tests.

@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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

#ifndef __JOYSTICK_DISPLAY_FIXTURE_H__
#define __JOYSTICK_DISPLAY_FIXTURE_H__

// Exercise the shared menu with scripted ADC voltages and the real SSD1315 driver.
// Inspect OLED RAM at the SPI boundary using the WBA65 board configuration.
#include <cassert>
#include <cstdio>
#include <cstring>
#include "converters/adc_device.h"
#include "board.h"
static float voltage = 3.3f;
static bool readOk = true, ledHigh;

static bool selected, dataMode;
static uint8_t ram[8][128];
static unsigned page, column;
static bool pageMode, displayOn;
static unsigned transfers, resetLow, resetHigh;

class FakeAdc {
public:
	bool Init(const AdcCfg_t &c)
	{
		assert(!c.bInterrupt && c.Mode == ADC_CONV_MODE_SINGLE);
		return true;
	}

	bool OpenChannel(const AdcChanCfg_t *c, int n)
	{
		assert(n == 1 && c->Chan == 0 && c->PinP.PinNo == 6 && c->Gain == 0x100);
		return true;
	}

	bool Read(int c, AdcData_t *p)
	{
		assert(c == 0);
		p->Data = voltage;
		return readOk;
	}
};

#undef ADC_DRIVER
#undef ADC_DRIVER_HEADER
#define ADC_DRIVER FakeAdc
#define ADC_DRIVER_HEADER "converters/adc_device.h"
#define main DisplayDemoMain
#ifdef COUNTDOWN_TIMER_TEST
#include "../../exemples/display/countdown_timer_demo.cpp"
#else
#include "../../exemples/display/joystick_menu_demo.cpp"
#endif
#undef main

#if defined(JOYSTICK_STOPWATCH) || defined(COUNTDOWN_TIMER_TEST)
static uint64_t ticks;

static uint64_t TickCount(TimerDev_t *)
{
	return ticks;
}

extern "C" bool TimerInit(TimerDev_t *t, const TimerCfg_t *c)
{
	assert(c->DevNo == 2 && c->Freq == 1000000 && !c->bTickInt);
	t->Freq = c->Freq;
	t->GetTickCount = TickCount;
	return true;
}
#endif

// Unused SPI virtual methods must never be called by a display master.
extern "C" void SPISetSlaveRxBuffer(SPIDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" void SPISetSlaveTxData(SPIDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" SPIPHY SPISetPhy(SPIDev_t *, SPIPHY Phy)
{
	assert(false);
	return Phy;
}

extern "C" void IOPinConfig(int Port, int Pin, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
	assert((Port == IOPORTE && (Pin == 0 || Pin == 3)) || (Port == IOPORTD && Pin == 8));
}

extern "C" void IOPinSet(int Port, int Pin)
{
	assert((Port == IOPORTE && (Pin == 0 || Pin == 3)) || (Port == IOPORTD && Pin == 8));
	if (Port == IOPORTD)
	{
		ledHigh = true;
		return;
	}
	if (Pin == 3)
	{
		resetHigh++;
	}
	else
	{
		dataMode = true;
	}
}

extern "C" void IOPinClear(int Port, int Pin)
{
	assert((Port == IOPORTE && (Pin == 0 || Pin == 3)) || (Port == IOPORTD && Pin == 8));
	if (Port == IOPORTD)
	{
		ledHigh = false;
		return;
	}
	if (Pin == 3)
	{
		resetLow++;
	}
	else
	{
		dataMode = false;
	}
}

static bool Start(DevIntrf_t *, uint32_t Address)
{
	assert(Address == 0 && !selected);
	selected = true;
	return true;
}

static void Stop(DevIntrf_t *)
{
	assert(selected);
	selected = false;
}

static int Tx(DevIntrf_t *, const uint8_t *p, int n)
{
	assert(selected && n > 0);
	transfers++;
	if (dataMode)
	{
		assert(pageMode);
		for (int i = 0; i < n; i++)
		{
			assert(page < 8 && column < 128);
			ram[page][column++] = p[i];
		}
	}
	else
	{
		for (int i = 0; i < n; i++)
		{
			uint8_t cmd = p[i];
			if (cmd == 0x20 || cmd == 0xD5 || cmd == 0xA8 || cmd == 0xD3 ||
				cmd == 0x8D || cmd == 0xDA || cmd == 0x81 || cmd == 0xD9 || cmd == 0xDB)
			{
				assert(++i < n);
				if (cmd == 0x20)
				{
					assert(p[i] == 2);
					pageMode = true;
				}
				if (cmd == 0xA8)
				{
					assert(p[i] >= 15 && p[i] <= 63);
				}
				if (cmd == 0xDB)
				{
					assert((p[i] & ~0x30U) == 0);
				}
			}
			else if ((cmd & 0xF8) == 0xB0)
			{
				page = cmd & 7;
			}
			else if ((cmd & 0xF0) == 0x00)
			{
				column = (column & 0xF0) | (cmd & 15);
			}
			else if ((cmd & 0xF0) == 0x10)
			{
				column = (column & 15) | ((cmd & 15) << 4);
			}
			else if (cmd == 0xAF)
			{
				displayOn = true;
			}
			else if (cmd == 0xAE)
			{
				displayOn = false;
			}
		}
	}
	return n;
}

extern "C" bool SPIInit(SPIDev_t *pDev, const SPICfg_t *pCfg)
{
	assert(pCfg->DevNo == 2 && pCfg->Phy == SPIPHY_NORMAL);
	assert(pCfg->DataSize == 8 && pCfg->BitOrder == SPIDATABIT_MSB);
	// Active-high clock polarity means CPOL=0, not CPOL=1.
	assert(pCfg->ClkPol == SPICLKPOL_HIGH);
	assert(pCfg->DataPhase == SPIDATAPHASE_FIRST_CLK);
	assert(!pCfg->bDmaEn && !pCfg->bIntEn);
	assert(pCfg->pIOPinMap[0].PortNo == IOPORTA && pCfg->pIOPinMap[0].PinNo == 0);
	assert(pCfg->pIOPinMap[0].PinOp == IOPINOP_FUNC6);
	assert(pCfg->pIOPinMap[2].PortNo == IOPORTB && pCfg->pIOPinMap[2].PinNo == 8);
	assert(pCfg->pIOPinMap[2].PinOp == IOPINOP_FUNC6);
	assert(pCfg->pIOPinMap[3].PortNo == IOPORTE && pCfg->pIOPinMap[3].PinNo == 1);
	pDev->DevIntrf.Type = DEVINTRF_TYPE_SPI;
	pDev->DevIntrf.StartTx = Start;
	pDev->DevIntrf.StopTx = Stop;
	pDev->DevIntrf.TxData = Tx;
	atomic_flag_clear(&pDev->DevIntrf.bBusy);
	return true;
}

static void Sample(float v, int count = 3)
{
	voltage = v;
	while (count-- > 0)
	{
		PollJoystick();
	}
}

static void Press(float v)
{
	Sample(3.3f);
	Sample(v);
}

#endif
