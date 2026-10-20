/**-------------------------------------------------------------------------
@file	ssd1315_gfx_test.cpp

@brief	Host test of the graphics demo on the SSD1315 driver.

Run the shared graphics demo through the real iGfx and SSD1315 driver.
Inspect OLED RAM at the SPI boundary using the WBA65 board configuration.

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

#include <cassert>
#include <cstdio>
#include <cstring>
#include "display/display_ssd1315.h"
#include "board.h"

class CheckedOLED : public LcdSSD1315 {
public:
	void SetPixel(uint16_t X, uint16_t Y, uint32_t Color) override
	{
		assert(X < Width() && Y < Height());
		LcdSSD1315::SetPixel(X, Y, Color);
	}

	void Fill(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint32_t Color) override
	{
		assert(W && H && X + W <= Width() && Y + H <= Height());
		LcdSSD1315::Fill(X, Y, W, H, Color);
	}
};

#undef DISPLAY_DRIVER
#define DISPLAY_DRIVER CheckedOLED
#define main GfxDisplayMain
#include "../../exemples/display/gfx_display_demo.cpp"
#undef main

static bool selected, dataMode;
static uint8_t ram[8][128];
static unsigned page, column;
static bool pageMode, displayOn;
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
	else
	{
		dataMode = true;
	}
}

extern "C" void IOPinClear(int Port, int Pin)
{
	assert(Port == IOPORTE && (Pin == 0 || Pin == 3));
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

static bool On(unsigned x, unsigned y)
{
	return ram[y / 8][x] & (1U << (y & 7));
}

int main()
{
	assert(HardwareInit());
	assert(g_Lcd.Valid() && g_Lcd.Width() == 128 && g_Lcd.Height() == 64);
	assert(resetLow == 1 && resetHigh >= 1 && displayOn && !selected);
	ShowLines();
	uint8_t lines[sizeof(ram)];
	memcpy(lines, ram, sizeof(ram));
	ShowRectangles();
	assert(memcmp(lines, ram, sizeof(ram)) != 0);
	for (unsigned y = 0; y < 64; y++)
	{
		for (unsigned x = 0; x < 128; x++)
		{
			unsigned edge = x < 127 - x ? x : 127 - x;
			unsigned ey = y < 63 - y ? y : 63 - y;
			if (ey < edge)
			{
				edge = ey;
			}
			assert(On(x, y) == (edge % 4 == 0));
		}
	}
	ShowPattern();
	for (unsigned y = 0; y < 64; y++)
	{
		for (unsigned x = 0; x < 128; x++)
		{
			assert(On(x, y) == ((x / 8 + y / 8) % 2 == 0));
		}
	}
	ShowAnimation();
	// Erasing the moving box must leave the border intact and no trails.
	for (unsigned y = 0; y < 64; y++)
	{
		for (unsigned x = 0; x < 128; x++)
		{
			assert(On(x, y) == (x == 0 || x == 127 || y == 0 || y == 63));
		}
	}
	assert(!selected);
	std::puts("WBA65 shared graphics: initialization, distinct pages, checkerboard and animation cleanup PASS");
}
