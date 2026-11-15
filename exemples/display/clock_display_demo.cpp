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
#include "coredev/spi.h"
#include "coredev/timer.h"
#include "coredev/interrupt.h"
#include "iopinctrl.h"
#include "board.h"
#include "clock_display_digits.h"
#include "clock_display_time.h"

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
