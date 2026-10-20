/**-------------------------------------------------------------------------
@example	console_display_demo.cpp

@brief	LCD console display example

This demo show text print to the LCD screen like the old console terminal

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
#include "display/display_ssd1315.h"
#include "iopinctrl.h"

#include "board.h"

// Existing HX8357 boards keep their original settings. Other panels select
// their driver, native geometry and SPI settings beside the pin map.
#ifndef DISPLAY_DRIVER
#define DISPLAY_DRIVER LcdHX8357
#endif
#ifndef DISPLAY_WIDTH
#define DISPLAY_WIDTH 320
#endif
#ifndef DISPLAY_HEIGHT
#define DISPLAY_HEIGHT 480
#endif
#ifndef DISPLAY_PIXEL_SIZE
#define DISPLAY_PIXEL_SIZE 16
#endif
#ifndef DISPLAY_STRIDE
#define DISPLAY_STRIDE (320 * 3)
#endif
#ifndef DISPLAY_SPI_CLKPOL
#define DISPLAY_SPI_CLKPOL SPICLKPOL_HIGH
#endif
#ifndef DISPLAY_SPI_DMA_ENABLE
#define DISPLAY_SPI_DMA_ENABLE true
#endif

static const IOPinCfg_t s_TFTCtrlPins[] = TFT_PINS;
const int s_NbTFTCtrlPins = sizeof(s_TFTCtrlPins) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_SpiPins[] = SPI_PINS;

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPI_PHY,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
	.Rate = SPI_RATE, // Speed in Hz
	.DataSize = 8,	  // Data Size
	.MaxRetry = 5,	  // Max retries
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK, // Data phase
	.ClkPol = DISPLAY_SPI_CLKPOL,		 // clock polarity
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = DISPLAY_SPI_DMA_ENABLE, // DMA
	.bIntEn = false,
	.IntPrio = 6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
	.EvtCB = NULL
};

SPI g_Spi;

static DisplayCfg_t s_LcdCfg = {
	.DevAddr = 0,
	.pPins = s_TFTCtrlPins,
	.NbPins = sizeof(s_TFTCtrlPins) / sizeof(IOPinCfg_t),
	.Stride = DISPLAY_STRIDE,
	.Width = DISPLAY_WIDTH,
	.Height = DISPLAY_HEIGHT,
	.PixelSize = DISPLAY_PIXEL_SIZE,
	.Orient = DISPL_ORIENT_LANDSCAPE,
};

DISPLAY_DRIVER g_Lcd;

uint8_t g_LineBuff[480 * 4];

static const char s_FontText[] = "!\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~\r\n";

// Keep the font demo usable on small panels. The generic formatted-print
// buffer is 80 bytes, so send the character sample in smaller chunks. "%s"
// formatting also keeps the sample's percent sign literal.
static void ShowFont(const FontDesc_t *pFont, const char *pName)
{
	if (!g_Lcd.SetFont(pFont))
	{
		return;
	}
	g_Lcd.printf("%s\r\n", pName);
	for (size_t off = 0; off < sizeof(s_FontText) - 1; off += 32)
	{
		g_Lcd.printf("%.32s", s_FontText + off);
	}
}

bool HardwareInit()
{
	if (!g_Spi.Init(s_SpiCfg) || !g_Lcd.Init(s_LcdCfg, &g_Spi))
	{
		return false;
	}
	g_Lcd.Backlight(true);

	g_Lcd.Clear();
	return true;
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

	if (!HardwareInit())
	{
		return 1;
	}

	colorr = EndianCvt16(0xf800);
	colorg = EndianCvt16(0x7e0);
	colorb = EndianCvt16(0x1f);

	int orient = g_Lcd.Orientation();

	while (1)
	{
		ShowFont(&iFontFreeMono24pt, "FreeMono 24pt");
		ShowFont(&iFontFreeMonoIta24pt, "FreeMono Italic 24pt");
		ShowFont(&iFontFreeMonoBold24pt, "FreeMono Bold 24pt");
		ShowFont(&iFontFreeMonoBoldIta24pt, "FreeMono Bold Italic 24pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeMono16pt, "FreeMono 16pt");
		ShowFont(&iFontFreeMonoIta16pt, "FreeMono Italic 16pt");
		ShowFont(&iFontFreeMonoBold16pt, "FreeMono Bold 16pt");
		ShowFont(&iFontFreeMonoBoldIta16pt, "FreeMono Bold Italic 16pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeMono12pt, "FreeMono 12pt");
		ShowFont(&iFontFreeMonoIta12pt, "FreeMono Italic 12pt");
		ShowFont(&iFontFreeMonoBold12pt, "FreeMono Bold 12pt");
		ShowFont(&iFontFreeMonoBoldIta12pt, "FreeMono Bold Italic 12pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeMono10pt, "FreeMono 10pt");
		ShowFont(&iFontFreeMonoIta10pt, "FreeMono Italic 10pt");
		ShowFont(&iFontFreeMonoBold10pt, "FreeMono Bold 10pt");
		ShowFont(&iFontFreeMonoBoldIta10pt, "FreeMono Bold Italic 10pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeMono8pt, "FreeMono 8pt");
		ShowFont(&iFontFreeMonoIta8pt, "FreeMono Italic 8pt");
		ShowFont(&iFontFreeMonoBold8pt, "FreeMono Bold 8pt");
		ShowFont(&iFontFreeMonoBoldIta8pt, "FreeMono Bold Italic 8pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeSans24pt, "FreeSans 24pt");
		ShowFont(&iFontFreeSansIta24pt, "FreeSans Italic 24pt");
		ShowFont(&iFontFreeSansBold24pt, "FreeSans Bold 24pt");
		ShowFont(&iFontFreeSansBoldIta24pt, "FreeSans Bold Italic 24pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeSans16pt, "FreeSans 16pt");
		ShowFont(&iFontFreeSansIta16pt, "FreeSans Italic 16pt");
		ShowFont(&iFontFreeSansBold16pt, "FreeSans Bold 16pt");
		ShowFont(&iFontFreeSansBoldIta16pt, "FreeSans Bold Italic 16pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeSans12pt, "FreeSans 12pt");
		ShowFont(&iFontFreeSansIta12pt, "FreeSans Italic 12pt");
		ShowFont(&iFontFreeSansBold12pt, "FreeSans Bold 12pt");
		ShowFont(&iFontFreeSansBoldIta12pt, "FreeSans Bold Italic 12pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeSans10pt, "FreeSans 10Pt");
		ShowFont(&iFontFreeSansIta10pt, "FreeSans Italic 10pt");
		ShowFont(&iFontFreeSansBold10pt, "FreeSans Bold 10pt");
		ShowFont(&iFontFreeSansBoldIta10pt, "FreeSans Bold Italic 10pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeSans8pt, "FreeSans 8Pt");
		ShowFont(&iFontFreeSansIta8pt, "FreeSans Italic 8pt");
		ShowFont(&iFontFreeSansBold8pt, "FreeSans Bold 8pt");
		ShowFont(&iFontFreeSansBoldIta8pt, "FreeSans Bold Italic 8pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeSerif24pt, "FreeSerif 24pt");
		ShowFont(&iFontFreeSerifIta24pt, "FreeSerif ITalic 24pt");
		ShowFont(&iFontFreeSerifBold24pt, "FreeSerif Bold 24pt");
		ShowFont(&iFontFreeSerifBoldIta24pt, "FreeSerif Bold Italic 24pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeSerif16pt, "FreeSerif 16pt");
		ShowFont(&iFontFreeSerifIta16pt, "FreeSerif ITalic 16pt");
		ShowFont(&iFontFreeSerifBold16pt, "FreeSerif Bold 16pt");
		ShowFont(&iFontFreeSerifBoldIta16pt, "FreeSerif Bold Italic 16pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeSerif12pt, "FreeSerif 12pt");
		ShowFont(&iFontFreeSerifIta12pt, "FreeSerif ITalic 12pt");
		ShowFont(&iFontFreeSerifBold12pt, "FreeSerif Bold 12pt");
		ShowFont(&iFontFreeSerifBoldIta12pt, "FreeSerif Bold Italic 12pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeSerif10pt, "FreeSerif 10pt");
		ShowFont(&iFontFreeSerifIta10pt, "FreeSerif ITalic 10pt");
		ShowFont(&iFontFreeSerifBold10pt, "FreeSerif Bold 10pt");
		ShowFont(&iFontFreeSerifBoldIta10pt, "FreeSerif Bold Italic 10pt");

		msDelay(3000);
		g_Lcd.Clear();

		ShowFont(&iFontFreeSerif8pt, "FreeSerif 8pt");
		ShowFont(&iFontFreeSerifIta8pt, "FreeSerif ITalic 8pt");
		ShowFont(&iFontFreeSerifBold8pt, "FreeSerif Bold 8pt");
		ShowFont(&iFontFreeSerifBoldIta8pt, "FreeSerif Bold Italic 8pt");

		ShowFont(&iFontSystem5x7, "System fixed 5x7");

		msDelay(3000);

		orient++;

		if (orient > DISPL_ORIENT_LANDSCAPE_INV)
		{
			orient = DISPL_ORIENT_PORTRAIT;
		}
		g_Lcd.Orientation((DISPL_ORIENT)orient);
		g_Lcd.Clear();
	}

	return 0;
}
