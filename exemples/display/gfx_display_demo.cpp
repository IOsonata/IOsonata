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

#include <stdio.h>
#include <math.h>

#include "idelay.h"
#include "convutil.h"
#include "coredev/spi.h"
#include "display/display_st77xx.h"
#include "display/display_ili9341.h"
#include "display/display_hx8357.h"
#include "display/igfx.h"
#include "iopinctrl.h"

#include "board.h"

static const IOPinCfg_t s_TFTCtrlPins[] = TFT_PINS;
const int s_NbTFTCtrlPins = sizeof(s_TFTCtrlPins) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_SpiPins[] = SPI_PINS;

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPI_PHY,
    .Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
    .Rate = SPI_RATE,   // Speed in Hz
    .DataSize = 8,      // Data Size
    .MaxRetry = 5,      // Max retries
    .BitOrder = SPIDATABIT_MSB,
    .DataPhase = SPIDATAPHASE_FIRST_CLK, // Data phase
    .ClkPol = SPICLKPOL_HIGH,         // clock polarity
    .ChipSel = SPICSEL_AUTO,
	.bDmaEn = true,	// DMA
	.bIntEn = false,
    .IntPrio = 6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    .EvtCB = NULL
};

SPI g_Spi;

static const DisplayCfg_t s_LcdCfg = {
	.DevAddr = 0,
	.pPins = s_TFTCtrlPins,
	.NbPins = sizeof(s_TFTCtrlPins) / sizeof(IOPinCfg_t),
	.Stride = 320 * 3,
	.Width = 240,
	.Height = 320,
	.PixelSize = 16,
	.Orient = DISPL_ORIENT_PORTRAIT,
};

//LcdST77xx g_Lcd;
//LcdILI9341 g_Lcd;
LcdHX8357 g_Lcd;
iGfx g_Gfx;

uint8_t g_LineBuff[480 * 4];

void HardwareInit()
{
	g_Spi.Init(s_SpiCfg);

	g_Lcd.Init(s_LcdCfg, &g_Spi);
	g_Lcd.Backlight(true);

	g_Lcd.Clear();

	g_Gfx.Init(&g_Lcd);
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// `--specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group`
//
// Adjust it for other toolchains.
//
// If functionality is not required, to only pass the build, use
// `--specs=nosys.specs`.
//

int main()
{
	uint32_t colorr = 0;
	uint32_t colorg = 0;
	uint32_t colorb = 0;

	HardwareInit();

	colorr = EndianCvt16(0xf800);
	colorg = EndianCvt16(0x7e0);
	colorb = EndianCvt16(0x1f);


	g_Gfx.Line(g_Lcd.Width() - 1, g_Lcd.Height() - 1, 0, 0, 0xFFFF);
	g_Gfx.Line(0, 0, g_Lcd.Width() - 1, 0, 0xFFFF);
	g_Gfx.Line(g_Lcd.Width() - 1, 0, g_Lcd.Width() - 1, g_Lcd.Height() - 1, 0xFFFF);
	g_Gfx.Line(g_Lcd.Width() - 1, g_Lcd.Height() - 1, 0, g_Lcd.Height() - 1, 0xFFFF);
	g_Gfx.Line(0, g_Lcd.Height() - 1, 0, 0, 0xFFFF);
	g_Gfx.Line(0, g_Lcd.Height() - 1, g_Lcd.Width() - 1, 0, 0xFFFF);

	uint16_t w = g_Lcd.Width();
	uint16_t h = g_Lcd.Height();

	for (int i = 2; i < w / 2; i += 4)
	{
		uint32_t c = i * 25;
		g_Gfx.Line(i, i, w - i - 1, i, c);
		g_Gfx.Line(i, i, i, h - i - 1, c);
		g_Gfx.Line(i, h - i - 1, w - i - 1, h - i - 1, c);
		g_Gfx.Line(w - i - 1, i, w - i - 1, h - i - 1, c);
	}

	while(1) __WFE();

	return 0;
}
