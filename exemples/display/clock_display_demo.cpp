/**-------------------------------------------------------------------------
@file	clock_display_demo.cpp

@brief	Large digital clock display example

Time is read from a generic Timer. Board configuration selects the timer,
display, SPI pins and initial time. Drawing and button handling run in main.

@author	Hoang Nguyen Hoan
@date	Nov. 15, 2026

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

#include <stdint.h>
#include <string.h>
#include "coredev/spi.h"
#include "coredev/timer.h"
#include "coredev/interrupt.h"
#include "iopinctrl.h"
#include "board.h"

#define CLOCK_FACE_WIDTH 128
#define CLOCK_FACE_HEIGHT 32
#define CLOCK_FACE_BYTES (CLOCK_FACE_WIDTH * CLOCK_FACE_HEIGHT / 8)

// Segments: top, upper right, lower right, bottom, lower left, upper left, middle.
static const uint8_t s_ClockSegments[10] = {
	0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f
};
static const uint8_t s_ClockDigitX[6] = {2, 20, 45, 63, 88, 106};
static const uint8_t s_ClockBars[7][4] = {
	{3, 0, 10, 3}, {13, 3, 3, 11}, {13, 17, 3, 11},
	{3, 28, 10, 3}, {0, 17, 3, 11}, {0, 3, 3, 11}, {3, 14, 10, 3}
};

typedef struct {
	uint32_t Offset;
	uint32_t LastSample;
	uint32_t PressedAt;
	unsigned Debounce;
	bool Pressed;
	bool Held;
} ClockTime_t;

static_assert(DISPLAY_PIXEL_SIZE == 1, "Clock face requires a monochrome display");
static_assert(DISPLAY_WIDTH >= CLOCK_FACE_WIDTH && DISPLAY_HEIGHT >= CLOCK_FACE_HEIGHT,
	"Clock face requires at least 128x32 pixels");
static_assert(CLOCK_START_HOUR >= 0 && CLOCK_START_HOUR < 24 &&
	CLOCK_START_MINUTE >= 0 && CLOCK_START_MINUTE < 60 &&
	CLOCK_START_SECOND >= 0 && CLOCK_START_SECOND < 60,
	"Initial clock time is out of range");

static const IOPinCfg_t s_SpiPins[] = SPI_PINS;
static const IOPinCfg_t s_DisplayPins[] = TFT_PINS;
static const IOPinCfg_t s_Button = {
	CLOCK_BUTTON_PORT, CLOCK_BUTTON_PIN, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL
};
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
	.bIntEn = DISPLAY_SPI_INT_ENABLE,
	.IntPrio = CLOCK_TIMER_PRIORITY,
};
static const DisplayCfg_t s_DisplayCfg = {
	.DevAddr = 0,
	.pPins = s_DisplayPins,
	.NbPins = sizeof(s_DisplayPins) / sizeof(s_DisplayPins[0]),
	.Stride = DISPLAY_STRIDE,
	.Width = DISPLAY_WIDTH,
	.Height = DISPLAY_HEIGHT,
	.PixelSize = DISPLAY_PIXEL_SIZE,
	.Orient = DISPL_ORIENT_LANDSCAPE,
};
static const TimerCfg_t s_TimerCfg = {
	.DevNo = CLOCK_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = CLOCK_TIMER_PRIORITY,
};
static SPI s_Spi;
static DISPLAY_DRIVER s_Display;
static TimerDev_t s_Timer;
static uint8_t s_Frame[CLOCK_FACE_BYTES];

static void ClockRect(uint8_t *pFrame, unsigned X, unsigned Y, unsigned Width, unsigned Height)
{
	for (unsigned y = Y; y < Y + Height; y++)
	{
		for (unsigned x = X; x < X + Width; x++)
		{
			pFrame[y * (CLOCK_FACE_WIDTH / 8) + x / 8] |= 0x80U >> (x & 7);
		}
	}
}

static void ClockRender(uint8_t *pFrame, uint32_t Seconds)
{
	uint32_t time = Seconds % 86400U;
	uint8_t digits[6];

	digits[0] = time / 36000U;
	digits[1] = (time / 3600U) % 10U;
	digits[2] = (time / 600U) % 6U;
	digits[3] = (time / 60U) % 10U;
	digits[4] = (time / 10U) % 6U;
	digits[5] = time % 10U;
	memset(pFrame, 0, CLOCK_FACE_BYTES);
	for (unsigned i = 0; i < 6; i++)
	{
		uint8_t segments = s_ClockSegments[digits[i]];
		for (unsigned b = 0; b < 7; b++)
		{
			if (segments & (1U << b))
			{
				ClockRect(pFrame, s_ClockDigitX[i] + s_ClockBars[b][0], s_ClockBars[b][1],
						  s_ClockBars[b][2], s_ClockBars[b][3]);
			}
		}
	}
	ClockRect(pFrame, 39, 9, 3, 3);
	ClockRect(pFrame, 39, 21, 3, 3);
	ClockRect(pFrame, 82, 9, 3, 3);
	ClockRect(pFrame, 82, 21, 3, 3);
}

static uint32_t ClockTimeUpdate(ClockTime_t *pTime, uint64_t Ticks, uint32_t Frequency, bool Down)
{
	uint32_t elapsed = (uint32_t)((Ticks / Frequency) % 86400U);
	uint32_t sample = (uint32_t)(Ticks * 20U / Frequency);

	if (sample != pTime->LastSample)
	{
		pTime->LastSample = sample;
		if (Down != pTime->Pressed)
		{
			if (++pTime->Debounce == 2)
			{
				pTime->Pressed = Down;
				pTime->Debounce = 0;
				if (pTime->Pressed)
				{
					pTime->PressedAt = sample;
					pTime->Held = false;
				}
				else if (!pTime->Held)
				{
					pTime->Offset = (pTime->Offset + 60U) % 86400U;
				}
			}
		}
		else
		{
			pTime->Debounce = 0;
		}
		if (pTime->Pressed && sample - pTime->PressedAt >= 20U)
		{
			pTime->Offset = (pTime->Offset + 3600U) % 86400U;
			pTime->PressedAt = sample;
			pTime->Held = true;
		}
	}

	return (elapsed + pTime->Offset) % 86400U;
}

int main(void)
{
	ClockTime_t time = {CLOCK_START_HOUR * 3600U + CLOCK_START_MINUTE * 60U + CLOCK_START_SECOND};
	uint32_t shown = UINT32_MAX;

	IOPinCfg(&s_Button, 1);
	if (!s_Spi.Init(s_SpiCfg) || !s_Display.Init(s_DisplayCfg, &s_Spi) ||
		!TimerInit(&s_Timer, &s_TimerCfg))
	{
		return 1;
	}
	// 20 Hz wake for button sampling. Time comes from the counter, not callback counts.
	if (nsTimerEnableTrigger(&s_Timer, 0, 50000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr) == 0)
	{
		return 1;
	}

	while (true)
	{
		uint64_t ticks = TimerGetTickCount(&s_Timer);
		uint32_t seconds = ClockTimeUpdate(&time, ticks, TimerGetFrequency(&s_Timer),
			IOPinRead(s_Button.PortNo, s_Button.PinNo) == 0);
		if (seconds != shown)
		{
			ClockRender(s_Frame, seconds);
			s_Display.BitBlt((DISPLAY_WIDTH - CLOCK_FACE_WIDTH) / 2,
				(DISPLAY_HEIGHT - CLOCK_FACE_HEIGHT) / 2, CLOCK_FACE_WIDTH, CLOCK_FACE_HEIGHT, s_Frame);
			shown = seconds;
		}
		__WFE();
	}
}
