/**-------------------------------------------------------------------------
@example	gfx_display_demo.cpp

@brief	GFX LCD display example


@author	Hoang Nguyen Hoan
@date	Mar. 9, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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

#include "idelay.h"
#include "coredev/spi.h"
#include "display/display_st77xx.h"
#include "display/display_ili9341.h"
#include "display/display_hx8357.h"
#include "display/display_ssd1315.h"
#include "display/igfx.h"
#include "iopinctrl.h"
#include "board.h"

// The original colour-panel configuration remains the default. Boards use
// the same display selections as console_display_demo.cpp.
#ifndef DISPLAY_DRIVER
#define DISPLAY_DRIVER LcdHX8357
#endif
#ifndef DISPLAY_WIDTH
#define DISPLAY_WIDTH 240
#endif
#ifndef DISPLAY_HEIGHT
#define DISPLAY_HEIGHT 320
#endif
#ifndef DISPLAY_STRIDE
#define DISPLAY_STRIDE (320 * 3)
#endif
#ifndef DISPLAY_PIXEL_SIZE
#define DISPLAY_PIXEL_SIZE 16
#endif
#ifndef DISPLAY_ORIENTATION
#define DISPLAY_ORIENTATION DISPL_ORIENT_PORTRAIT
#endif
#ifndef DISPLAY_SPI_CLKPOL
#define DISPLAY_SPI_CLKPOL SPICLKPOL_HIGH
#endif
#ifndef DISPLAY_SPI_DMA_ENABLE
#define DISPLAY_SPI_DMA_ENABLE true
#endif

static const IOPinCfg_t s_CtrlPins[] = TFT_PINS;
static const IOPinCfg_t s_SpiPins[] = SPI_PINS;
static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPI_PHY,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(s_SpiPins[0]),
	.Rate = SPI_RATE,
	.DataSize = 8,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = DISPLAY_SPI_CLKPOL,
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = DISPLAY_SPI_DMA_ENABLE,
	.bIntEn = false,
	.IntPrio = 6,
	.EvtCB = nullptr
};
static DisplayCfg_t s_LcdCfg = {
	.DevAddr = 0,
	.pPins = s_CtrlPins,
	.NbPins = sizeof(s_CtrlPins) / sizeof(s_CtrlPins[0]),
	.Stride = DISPLAY_STRIDE,
	.Width = DISPLAY_WIDTH,
	.Height = DISPLAY_HEIGHT,
	.PixelSize = DISPLAY_PIXEL_SIZE,
	.Orient = DISPLAY_ORIENTATION
};
static SPI g_Spi;
static DISPLAY_DRIVER g_Lcd;
static iGfx g_Gfx;

static bool HardwareInit()
{
	if (!g_Spi.Init(s_SpiCfg) || !g_Lcd.Init(s_LcdCfg, &g_Spi) ||
		g_Lcd.Width() < 16 || g_Lcd.Height() < 16 || !g_Gfx.Init(&g_Lcd))
	{
		return false;
	}
	g_Lcd.Backlight(true);
	g_Gfx.Clear();
	return true;
}

static uint32_t DemoColor(unsigned Index)
{
	static const uint16_t colors[] = { 0xFFFF, 0xF800, 0x07E0, 0x001F };
	return DISPLAY_PIXEL_SIZE == 1 ? 1 : colors[Index % 4];
}

static void Rectangle(int X, int Y, int Width, int Height, uint32_t Color)
{
	g_Gfx.Line(X, Y, X + Width - 1, Y, Color);
	g_Gfx.Line(X, Y, X, Y + Height - 1, Color);
	g_Gfx.Line(X, Y + Height - 1, X + Width - 1, Y + Height - 1, Color);
	g_Gfx.Line(X + Width - 1, Y, X + Width - 1, Y + Height - 1, Color);
}

static void ShowLines()
{
	g_Gfx.Clear();
	int w = g_Lcd.Width(), h = g_Lcd.Height();
	for (int x = 0; x < w; x += 8)
	{
		g_Gfx.Line(0, 0, x, h - 1, DemoColor(x / 8));
		g_Gfx.Line(w - 1, h - 1, x, 0, DemoColor(x / 8 + 1));
	}
	Rectangle(0, 0, w, h, DemoColor(0));
}

static void ShowRectangles()
{
	g_Gfx.Clear();
	int w = g_Lcd.Width(), h = g_Lcd.Height();
	for (int inset = 0; inset * 2 < w && inset * 2 < h; inset += 4)
	{
		Rectangle(inset, inset, w - 2 * inset, h - 2 * inset, DemoColor(inset / 4));
	}
}

static void ShowPattern()
{
	g_Gfx.Clear();
	int w = g_Lcd.Width(), h = g_Lcd.Height();
	for (int y = 0; y < h; y += 8)
	{
		for (int x = 0; x < w; x += 8)
		{
			if ((x / 8 + y / 8) % 2 == 0)
			{
				g_Lcd.Fill(x, y, w - x < 8 ? w - x : 8,
						   h - y < 8 ? h - y : 8, DemoColor(0));
			}
		}
	}
}

static void ShowAnimation()
{
	g_Gfx.Clear();
	int w = g_Lcd.Width(), h = g_Lcd.Height();
	int bw = w / 5, bh = h / 4;
	int x = 2, y = 2, dx = 1, dy = 1;
	Rectangle(0, 0, w, h, DemoColor(0));
	for (unsigned frame = 0; frame < 160; frame++)
	{
		Rectangle(x, y, bw, bh, DemoColor(frame / 40));
		msDelay(25);
		g_Lcd.Fill(x, y, bw, bh, 0);
		if (x + dx < 2 || x + dx + bw > w - 2)
		{
			dx = -dx;
		}
		if (y + dy < 2 || y + dy + bh > h - 2)
		{
			dy = -dy;
		}
		x += dx;
		y += dy;
	}
}

int main()
{
	if (!HardwareInit())
	{
		return 1;
	}
	while (1)
	{
		ShowLines();
		msDelay(1200);
		ShowRectangles();
		msDelay(1200);
		ShowPattern();
		msDelay(1200);
		ShowAnimation();
	}
}
