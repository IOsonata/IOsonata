//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright 2022 I-SYST inc. All rights reserved.
// Description : Hello World in C++
//============================================================================

#include <stdio.h>

#include "idelay.h"
#include "convutil.h"
#include "coredev/spi.h"
#include "display/display_st77xx.h"
#include "iopinctrl.h"

#include "board.h"

extern "C" const uint8_t g_ISYST_Logo_16bits[];
extern "C" const uint32_t g_ISYST_Logo_18bits[];

#define ST77XX_RED 0xF800
#define ST77XX_GREEN 0x07E0
#define ST77XX_BLUE 0x001F

static const IOPinCfg_t s_TFTCtrlPins[] = TFT_PINS;
const int s_NbTFTCtrlPins = sizeof(s_TFTCtrlPins) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_SpiPins[] = SPI_PINS;

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPI_PHY,
    .Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
    .Rate = 8000000,   // Speed in Hz
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
	.HLen = 320,
	.VLen = 240,
	.PixelSize = 16,
	.Orient = DISPL_ORIENT_LANDSCAPE,
};

DisplST77xx g_Lcd;

uint8_t g_LineBuff[480 * 4];

void HardwareInit()
{
	IOPinCfg(s_TFTCtrlPins, s_NbTFTCtrlPins);

	g_Spi.Init(s_SpiCfg);

	g_Lcd.Init(s_LcdCfg, &g_Spi);

	g_Lcd.Clear();
	g_Lcd.Backlight(false);
	msDelay(2000);
	g_Lcd.Backlight(true);
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
	uint8_t *img = (uint8_t *)g_ISYST_Logo_16bits;
	int pixelbyte = 1;

	HardwareInit();

	switch (s_LcdCfg.PixelSize)
	{
		case 12:	// not supported
		case 16:
			colorr = 0xf800;
			colorg = 0x7e0;
			colorb = 0x1f;
			img = (uint8_t *)g_ISYST_Logo_16bits;
			pixelbyte = 2;

			// Display gradient
			for (uint16_t i = 0, c = 0; c < 0x20; i+=2, c++)
			{
				for (int j = 80; j < 120; j++)
				{
					g_Lcd.SetPixel(i, j, c);
					g_Lcd.SetPixel(i + 1, j, c);
					g_Lcd.SetPixel(i + 0x40, j, c << 5);
					g_Lcd.SetPixel(i + 0x41, j, c << 5);
					g_Lcd.SetPixel(i + 0x80, j, c << 11);
					g_Lcd.SetPixel(i + 0x81, j, c << 11);
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
	}


#if 0
	// Red
	for (int j = 10; j < 110; j++)
	{
		g_Lcd.SetPixel(10, j, colorr);
	}

	// Green
	for (int j = 10; j < 110; j++)
	{
		g_Lcd.SetPixel(12, j, colorg);
	}

	// Blue
	for (int j = 10; j < 110; j++)
	{
		g_Lcd.SetPixel(14, j, colorb);
	}
#endif

	msDelay(3000);

	// Display RGB bars
	g_Lcd.Fill(0, 0, s_LcdCfg.HLen, 80, colorr);
	g_Lcd.Fill(0, 80, s_LcdCfg.HLen, 80, colorg);
	g_Lcd.Fill(0, 160, s_LcdCfg.HLen, 80, colorb);

	msDelay(3000);

	// Display logo
	uint8_t *p = img;

	for (int i = 0; i < 153; i++)
	{
		memcpy(g_LineBuff, p, 320 * pixelbyte);
		g_Lcd.BitBlt(0, 43 + i, s_LcdCfg.HLen, 1, g_LineBuff);
		p += 320 * pixelbyte;
	}

	while(1) __WFE();

	return 0;
}
