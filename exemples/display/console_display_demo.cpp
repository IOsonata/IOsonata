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

DisplayCfg_t s_LcdCfg = {
	.DevAddr = 0,
	.pPins = s_TFTCtrlPins,
	.NbPins = sizeof(s_TFTCtrlPins) / sizeof(IOPinCfg_t),
	.Stride = 320 * 3,
	.Width = 320,
	.Height = 480,
	.PixelSize = 16,
	.Orient = DISPL_ORIENT_LANDSCAPE,
};

//LcdST77xx g_Lcd;
//LcdILI9341 g_Lcd;
LcdHX8357 g_Lcd;

uint8_t g_LineBuff[480 * 4];

static const char s_FontText[] = "!\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~\r\n";

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

	HardwareInit();

	colorr = EndianCvt16(0xf800);
	colorg = EndianCvt16(0x7e0);
	colorb = EndianCvt16(0x1f);

	int orient = g_Lcd.Orientation();

	while (1)
	{
		g_Lcd.SetFont(&iFontFreeMono24pt);
		g_Lcd.printf("FreeMono 24pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoIta24pt);
		g_Lcd.printf("FreeMono Italic 24pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoBold24pt);
		g_Lcd.printf("FreeMono Bold 24pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoBoldIta24pt);
		g_Lcd.printf("FreeMono Bold Italic 24pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeMono16pt);
		g_Lcd.printf("FreeMono 16pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoIta16pt);
		g_Lcd.printf("FreeMono Italic 16pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoBold16pt);
		g_Lcd.printf("FreeMono Bold 16pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoBoldIta16pt);
		g_Lcd.printf("FreeMono Bold Italic 16pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeMono12pt);
		g_Lcd.printf("FreeMono 12pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoIta12pt);
		g_Lcd.printf("FreeMono Italic 12pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoBold12pt);
		g_Lcd.printf("FreeMono Bold 12pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoBoldIta12pt);
		g_Lcd.printf("FreeMono Bold Italic 12pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeMono10pt);
		g_Lcd.printf("FreeMono 10pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoIta10pt);
		g_Lcd.printf("FreeMono Italic 10pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoBold10pt);
		g_Lcd.printf("FreeMono Bold 10pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoBoldIta10pt);
		g_Lcd.printf("FreeMono Bold Italic 10pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeMono8pt);
		g_Lcd.printf("FreeMono 8pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoIta8pt);
		g_Lcd.printf("FreeMono Italic 8pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoBold8pt);
		g_Lcd.printf("FreeMono Bold 8pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeMonoBoldIta8pt);
		g_Lcd.printf("FreeMono Bold Italic 8pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeSans24pt);
		g_Lcd.printf("FreeSans 24pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansIta24pt);
		g_Lcd.printf("FreeSans Italic 24pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansBold24pt);
		g_Lcd.printf("FreeSans Bold 24pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansBoldIta24pt);
		g_Lcd.printf("FreeSans Bold Italic 24pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();


		g_Lcd.SetFont(&iFontFreeSans16pt);
		g_Lcd.printf("FreeSans 16pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansIta16pt);
		g_Lcd.printf("FreeSans Italic 16pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansBold16pt);
		g_Lcd.printf("FreeSans Bold 16pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansBoldIta16pt);
		g_Lcd.printf("FreeSans Bold Italic 16pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeSans12pt);
		g_Lcd.printf("FreeSans 12pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansIta12pt);
		g_Lcd.printf("FreeSans Italic 12pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansBold12pt);
		g_Lcd.printf("FreeSans Bold 12pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansBoldIta12pt);
		g_Lcd.printf("FreeSans Bold Italic 12pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeSans10pt);
		g_Lcd.printf("FreeSans 10Pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansIta10pt);
		g_Lcd.printf("FreeSans Italic 10pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansBold10pt);
		g_Lcd.printf("FreeSans Bold 10pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansBoldIta10pt);
		g_Lcd.printf("FreeSans Bold Italic 10pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeSans8pt);
		g_Lcd.printf("FreeSans 8Pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansIta8pt);
		g_Lcd.printf("FreeSans Italic 8pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansBold8pt);
		g_Lcd.printf("FreeSans Bold 8pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSansBoldIta8pt);
		g_Lcd.printf("FreeSans Bold Italic 8pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeSerif24pt);
		g_Lcd.printf("FreeSerif 24pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifIta24pt);
		g_Lcd.printf("FreeSerif ITalic 24pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifBold24pt);
		g_Lcd.printf("FreeSerif Bold 24pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifBoldIta24pt);
		g_Lcd.printf("FreeSerif Bold Italic 24pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeSerif16pt);
		g_Lcd.printf("FreeSerif 16pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifIta16pt);
		g_Lcd.printf("FreeSerif ITalic 16pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifBold16pt);
		g_Lcd.printf("FreeSerif Bold 16pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifBoldIta16pt);
		g_Lcd.printf("FreeSerif Bold Italic 16pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeSerif12pt);
		g_Lcd.printf("FreeSerif 12pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifIta12pt);
		g_Lcd.printf("FreeSerif ITalic 12pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifBold12pt);
		g_Lcd.printf("FreeSerif Bold 12pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifBoldIta12pt);
		g_Lcd.printf("FreeSerif Bold Italic 12pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeSerif10pt);
		g_Lcd.printf("FreeSerif 10pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifIta10pt);
		g_Lcd.printf("FreeSerif ITalic 10pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifBold10pt);
		g_Lcd.printf("FreeSerif Bold 10pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifBoldIta10pt);
		g_Lcd.printf("FreeSerif Bold Italic 10pt\r\n");
		g_Lcd.printf(s_FontText);

		msDelay(3000);
		g_Lcd.Clear();

		g_Lcd.SetFont(&iFontFreeSerif8pt);
		g_Lcd.printf("FreeSerif 8pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifIta8pt);
		g_Lcd.printf("FreeSerif ITalic 8pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifBold8pt);
		g_Lcd.printf("FreeSerif Bold 8pt\r\n");
		g_Lcd.printf(s_FontText);
		g_Lcd.SetFont(&iFontFreeSerifBoldIta8pt);
		g_Lcd.printf("FreeSerif Bold Italic 8pt\r\n");
		g_Lcd.printf(s_FontText);

		g_Lcd.SetFont(&iFontSystem5x7);
		g_Lcd.printf("System fixed 5x7\r\n");
		g_Lcd.printf(s_FontText);

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
