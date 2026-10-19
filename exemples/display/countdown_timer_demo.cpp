/**-------------------------------------------------------------------------
@file	countdown_timer_demo.cpp

@brief	Joystick-controlled countdown timer and display example


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

#include <math.h>
#include "coredev/spi.h"
#include "coredev/timer.h"
#include "display/display_ssd1315.h"
#include "display/display_hx8357.h"
#include "display/display_ili9341.h"
#include "display/display_st77xx.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "board.h"
#include ADC_DRIVER_HEADER

static const IOPinCfg_t s_CtrlPins[] = TFT_PINS;
static const IOPinCfg_t s_SpiPins[] = SPI_PINS;
static const IOPinCfg_t s_LedPins[] = MENU_LED_PINS;
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
static const AdcRefVolt_t s_Ref[] = { { ADC_REFVOLT_TYPE_SUPPLY, JOYSTICK_REF_VOLTAGE, -1 } };
static const AdcCfg_t s_AdcCfg = {
	.Mode = ADC_CONV_MODE_SINGLE,
	.pRefVolt = s_Ref,
	.NbRefVolt = 1,
	.DevAddr = 0,
	.Resolution = 12,
	.Rate = 0,
	.OvrSample = 0,
	.bInterrupt = false,
	.IntPrio = 6,
	.EvtHandler = nullptr
};
static const AdcChanCfg_t s_JoyCfg = {
	.Chan = 0,
	.RefVoltIdx = 0,
	.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
	.Gain = 0x100,
	.AcqTime = 10,
	.BurstMode = false,
	.PinP = { JOYSTICK_ADC_INPUT, ADC_PIN_CONN_NONE },
	.PinN = { -1, ADC_PIN_CONN_NONE },
	.FifoMemSize = 0,
	.pFifoMem = nullptr
};
static SPI g_Spi;
static DISPLAY_DRIVER g_Lcd;
static ADC_DRIVER g_Adc;

enum MENU_KEY
{
	KEY_SELECT,
	KEY_LEFT,
	KEY_DOWN,
	KEY_UP,
	KEY_RIGHT,
	KEY_NONE,
	KEY_INVALID
};

static const TimerCfg_t s_TimerCfg = {
	.DevNo = TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1000000,
	.IntPrio = 7,
	.EvtHandler = nullptr,
	.bTickInt = false
};
static const unsigned s_MinDurationMs = 10000;
static const unsigned s_MaxDurationMs = 5999000; // 99:59
static unsigned s_DurationMs = 60000;
static unsigned s_RemainingMs = 60000;
static uint32_t s_LastTick;
static uint32_t s_LastPaint;
static Timer s_Clock;
static bool s_Running;
static bool s_Done;
static bool s_InputError;
static MENU_KEY s_Candidate = KEY_INVALID;
static MENU_KEY s_Stable = KEY_INVALID;
static unsigned s_Debounce;

static void SetDone(bool Done)
{
	s_Done = Done;
	if (Done == bool(MENU_LED_ACTIVE))
	{
		IOPinSet(s_LedPins[0].PortNo, s_LedPins[0].PinNo);
	}
	else
	{
		IOPinClear(s_LedPins[0].PortNo, s_LedPins[0].PinNo);
	}
}

static void DrawCountdown()
{
	// Round up to tenths so a running timer never displays zero early.
	unsigned tenths = (s_RemainingMs + 99) / 100;
	unsigned width = s_RemainingMs * 128ULL / s_DurationMs;

	g_Lcd.Clear();
	g_Lcd.SetCursor(0, 0);
	g_Lcd.printf("Countdown %s", s_Done ? "DONE" : s_Running ? "RUN"
															 : "PAUSED");
	g_Lcd.SetCursor(0, 16);
	g_Lcd.printf("%02u:%02u.%u", tenths / 600, tenths / 10 % 60, tenths % 10);
	g_Lcd.Fill(0, 28, width, 4, DISPLAY_PIXEL_SIZE == 1 ? 1 : 0xFFFF);
	g_Lcd.SetCursor(0, 40);
	if (s_InputError)
	{
		g_Lcd.printf("Joystick read failed");
	}
	else
	{
		g_Lcd.printf("SEL run/pause LT reset");
	}
	g_Lcd.SetCursor(0, 48);
	g_Lcd.printf("Paused: RT +10 seconds");
	g_Lcd.SetCursor(0, 56);
	g_Lcd.printf("UP/DN +/-1 minute");
}

static void UpdateCountdown()
{
	uint32_t now = s_Clock.mSecond();
	uint32_t delta = now - s_LastTick;
	bool wasRunning = s_Running;

	s_LastTick = now;
	if (s_Running)
	{
		if (delta >= s_RemainingMs)
		{
			s_RemainingMs = 0;
			s_Running = false;
			SetDone(true);
		}
		else
		{
			s_RemainingMs -= delta;
		}
	}
	if (wasRunning && (now - s_LastPaint >= 100 || s_Done))
	{
		s_LastPaint = now;
		DrawCountdown();
	}
}

static void HandleKey(MENU_KEY KeyCode)
{
	if (KeyCode == KEY_SELECT)
	{
		if (s_Done)
		{
			s_RemainingMs = s_DurationMs;
			SetDone(false);
		}
		s_Running = !s_Running;
	}
	else if (!s_Running)
	{
		if (KeyCode == KEY_LEFT)
		{
			s_RemainingMs = s_DurationMs;
			SetDone(false);
		}
		else
		{
			if (KeyCode == KEY_RIGHT)
			{
				s_DurationMs += 10000;
			}
			else if (KeyCode == KEY_UP)
			{
				s_DurationMs += 60000;
			}
			else if (KeyCode == KEY_DOWN)
			{
				s_DurationMs = s_DurationMs > 60000 ? s_DurationMs - 60000 : s_MinDurationMs;
			}
			if (s_DurationMs > s_MaxDurationMs)
			{
				s_DurationMs = s_MaxDurationMs;
			}
			s_RemainingMs = s_DurationMs;
			SetDone(false);
		}
	}
	DrawCountdown();
}

static MENU_KEY DecodeKey(float Voltage)
{
	static const unsigned levels[] = JOYSTICK_LEVELS_PERMILLE;
	float ratio = Voltage * 1000.0f / JOYSTICK_REF_VOLTAGE;

	if (!isfinite(Voltage) || Voltage < 0 || Voltage > JOYSTICK_REF_VOLTAGE * 1.05f)
	{
		return KEY_INVALID;
	}
	for (unsigned i = 0; i < 5; i++)
	{
		if (ratio < (levels[i] + levels[i + 1]) / 2.0f)
		{
			return MENU_KEY(i);
		}
	}
	return KEY_NONE;
}

// Called every 10 ms. Three equal samples debounce a transition. A press
// produces one action and must be released before another action is accepted.
static void PollJoystick()
{
	AdcData_t data;
	MENU_KEY k;

	UpdateCountdown();
	k = g_Adc.Read(0, &data) ? DecodeKey(data.Data) : KEY_INVALID;
	if (k == KEY_INVALID)
	{
		s_Candidate = s_Stable = KEY_INVALID;
		s_Debounce = 0;
		if (!s_InputError)
		{
			s_InputError = true;
			DrawCountdown();
		}
		return;
	}
	if (s_InputError)
	{
		s_InputError = false;
		DrawCountdown();
	}
	if (k != s_Candidate)
	{
		s_Candidate = k;
		s_Debounce = 1;
		return;
	}
	if (s_Debounce < 3)
	{
		s_Debounce++;
	}
	if (s_Debounce == 3 && k != s_Stable)
	{
		MENU_KEY old = s_Stable;
		s_Stable = k;
		if (old == KEY_NONE && k != KEY_NONE)
		{
			HandleKey(k);
		}
	}
}

static bool HardwareInit()
{
	if (!g_Spi.Init(s_SpiCfg) || !g_Lcd.Init(s_LcdCfg, &g_Spi))
	{
		return false;
	}
	g_Lcd.Backlight(true);
	if (g_Lcd.Width() < 128 || g_Lcd.Height() < 64 || !g_Lcd.SetFont(&iFontSystem5x7))
	{
		return false;
	}
	if (!s_Clock.Init(s_TimerCfg))
	{
		g_Lcd.printf("Timer init failed");
		return false;
	}
	s_LastTick = s_Clock.mSecond();
	IOPinConfig(s_LedPins[0].PortNo, s_LedPins[0].PinNo, s_LedPins[0].PinOp,
				IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	SetDone(false);
	if (!g_Adc.Init(s_AdcCfg) || !g_Adc.OpenChannel(&s_JoyCfg, 1))
	{
		s_InputError = true;
		DrawCountdown();
		return false;
	}
	DrawCountdown();
	return true;
}

int main()
{
	if (!HardwareInit())
	{
		return 1;
	}
	while (1)
	{
		PollJoystick();
		msDelay(10);
	}
}
