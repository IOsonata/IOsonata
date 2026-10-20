/**-------------------------------------------------------------------------
@file	ssd1315_console_test.cpp

@brief	Host test of the console display example on the SSD1315.

Run the shared console's actual HardwareInit with its WBA65 board config.
The SPI controller stand-in checks the SSD1315 bus requirements before
accepting any command. Framebuffer rendering is tested separately.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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

#include <cassert>
#include <cstdio>
#define main ConsoleDisplayMain
#include "../../exemples/display/console_display_demo.cpp"
#undef main

static bool selected;
static unsigned transfers, resetLow, resetHigh;

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
	assert(Port == IOPORTE && (Pin == 0 || Pin == 3));
}

extern "C" void IOPinSet(int Port, int Pin)
{
	assert(Port == IOPORTE && (Pin == 0 || Pin == 3));
	if (Pin == 3)
	{
		resetHigh++;
	}
}

extern "C" void IOPinClear(int Port, int Pin)
{
	assert(Port == IOPORTE && (Pin == 0 || Pin == 3));
	if (Pin == 3)
	{
		resetLow++;
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

static int Tx(DevIntrf_t *, const uint8_t *, int Count)
{
	assert(selected);
	transfers++;
	return Count;
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

int main()
{
	assert(HardwareInit());
	assert(g_Lcd.Valid() && g_Lcd.Width() == 128 && g_Lcd.Height() == 64);
	assert(resetLow == 1 && resetHigh >= 1 && transfers > 16 && !selected);
	std::puts("WBA65 shared console: SPI mode 0, pin map, reset and initialization PASS");
}
