/**-------------------------------------------------------------------------
@example	lcd_display_demo.cpp

@brief	LCD display example


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

#include "idelay.h"
#include "convutil.h"
#include "coredev/spi.h"
#include "display/display_st77xx.h"
#include "display/display_ili9341.h"
#include "display/display_hx8357.h"
#include "iopinctrl.h"

#include "board.h"

//extern "C" const uint8_t g_ISYST_Logo_16bits[];
extern "C" const uint32_t g_ISYST_Logo_18bits[];
extern "C" const uint16_t g_ISYST_Logo_16bits_bige[];
extern "C" const uint16_t g_ISYST_Logo_32bits[];

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
	.bDmaEn = false,	// DMA
	.bIntEn = false,
    .IntPrio = 6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    .EvtCB = NULL
};

SPI g_Spi;

DisplayCfg_t s_LcdCfg = {
	.DevAddr = 0,
	.pPins = s_TFTCtrlPins,
	.NbPins = sizeof(s_TFTCtrlPins) / sizeof(IOPinCfg_t),
	.Stride = 320 * 3,
	.Width = 240,
	.Height = 320,
	.PixelSize = 16,
	.Orient = DISPL_ORIENT_PORTRAIT,
};

LcdST77xx g_Lcd;
//LcdILI9341 g_Lcd;
//LcdHX8357 g_Lcd;

uint8_t g_LineBuff[480 * 4];

void HardwareInit()
{
	g_Spi.Init(s_SpiCfg);

	g_Lcd.Init(s_LcdCfg, &g_Spi);
	g_Lcd.Backlight(true);

	g_Lcd.Clear();
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
	uint8_t const *img = (uint8_t*)g_ISYST_Logo_16bits_bige;//g_ISYST_Logo_16bits;
	int pixelbyte = 1;

	HardwareInit();

	int orient = s_LcdCfg.Orient;

	while (1)
	{
		switch (s_LcdCfg.PixelSize)
		{
			case 12:	// not supported
			case 16:
				colorr = EndianCvt16(0xf800);
				colorg = EndianCvt16(0x7e0);
				colorb = EndianCvt16(0x1f);
				img = (uint8_t *)g_ISYST_Logo_16bits_bige;
				pixelbyte = 2;

				// Display gradient
				for (uint16_t i = 0, c = 0; c < 0x20; i+=2, c++)
				{
					uint16_t r = EndianCvt16(c);
					uint16_t g = EndianCvt16(c << 5);
					uint16_t b = EndianCvt16(c << 11);
					for (int j = 80; j < 120; j++)
					{
						g_Lcd.SetPixel(i, j, r);
						g_Lcd.SetPixel(i + 1, j, r);
						g_Lcd.SetPixel(i + 0x40, j, g);
						g_Lcd.SetPixel(i + 0x41, j, g);
						g_Lcd.SetPixel(i + 0x80, j, b);
						g_Lcd.SetPixel(i + 0x81, j, b);
					}
				}
				break;
			case 18:
				colorr = 0xfc0000;
				colorg = 0xfc00;
				colorb = 0xfc;
				img = (uint8_t *)g_ISYST_Logo_18bits;
				pixelbyte = 3;

				// Display gradient
				for (uint16_t i = 0, c = 0; c < 0x30; i+=2, c++)
				{
					for (int j = 80; j < 120; j++)
					{
						g_Lcd.SetPixel(i, j, c);
						g_Lcd.SetPixel(i + 1, j, c);
						g_Lcd.SetPixel(i + 0x60, j, c << 10);
						g_Lcd.SetPixel(i + 0x61, j, c << 10);
						g_Lcd.SetPixel(i + 0xC0, j, c << 18);
						g_Lcd.SetPixel(i + 0xC1, j, c << 18);
					}
				}
				break;
			case 24:
				colorr = 0xfc0000;
				colorg = 0xfc00;
				colorb = 0xfc;
				img = (uint8_t *)g_ISYST_Logo_32bits;
				pixelbyte = 3;

				// Display gradient
				for (uint16_t i = 0, c = 0; c < 0x30; i+=2, c++)
				{
					for (int j = 80; j < 120; j++)
					{
						g_Lcd.SetPixel(i, j, c);
						g_Lcd.SetPixel(i + 1, j, c);
						g_Lcd.SetPixel(i + 0x60, j, c << 10);
						g_Lcd.SetPixel(i + 0x61, j, c << 10);
						g_Lcd.SetPixel(i + 0xC0, j, c << 18);
						g_Lcd.SetPixel(i + 0xC1, j, c << 18);
					}
				}
				break;
		}

		msDelay(1000);

		uint16_t h = g_Lcd.Height() / 3;

		// Display RGB bars
		g_Lcd.Fill(0, 0, g_Lcd.Width(), h, colorr);
		g_Lcd.Fill(0, h, g_Lcd.Width(), h, colorg);
		g_Lcd.Fill(0, h << 1, g_Lcd.Width(), h, colorb);

		msDelay(1000);

		// Display logo
		uint8_t const *p = img;

		for (int i = 0; i < 153; i++)
		{
			memcpy(g_LineBuff, p, 320 * pixelbyte);
			g_Lcd.BitBlt(g_Lcd.Width() / 2 - 160, g_Lcd.Height() / 2 + i - 153/ 2, 320, 1, g_LineBuff);
			p += 320 * pixelbyte;
		}
		msDelay(2000);

		orient++;

		if (orient > DISPL_ORIENT_LANDSCAPE_INV)
		{
			orient = DISPL_ORIENT_PORTRAIT;
		}
		g_Lcd.Orientation((DISPL_ORIENT)orient);
		g_Lcd.Clear();
	}

	while(1) __WFE();

	return 0;
}
