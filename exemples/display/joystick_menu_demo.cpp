/**-------------------------------------------------------------------------
@file	joystick_menu_demo.cpp

@brief	Analog joystick and display menu example


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

enum Key {
	KEY_SELECT,
	KEY_LEFT,
	KEY_DOWN,
	KEY_UP,
	KEY_RIGHT,
	KEY_NONE,
	KEY_INVALID
};
#ifdef JOYSTICK_STOPWATCH
static unsigned s_Item = 3, s_Page = 3;
static const unsigned s_ItemCount = 4;
#else
static unsigned s_Item, s_Page;
static const unsigned s_ItemCount = 3;
#endif
static int s_Level = 5;
static bool s_LedOn, s_InputError;
static Key s_Candidate = KEY_INVALID, s_Stable = KEY_INVALID;
static unsigned s_Debounce;

#ifdef JOYSTICK_STOPWATCH
static const TimerCfg_t s_TimerCfg = {
	.DevNo = TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1000000,
	.IntPrio = 7,
	.EvtHandler = nullptr,
	.bTickInt = false
};
static Timer s_Clock;
static uint64_t s_ElapsedMs, s_LapMs;
static uint32_t s_LastTick, s_LastPaint;
static bool s_Running, s_HaveLap;
static const uint64_t s_MaxElapsedMs = 359999990; // 99:59:59.99

static void DrawMenu();

static void PrintElapsed(uint64_t Ms)
{
	g_Lcd.printf("%02u:%02u:%02u.%02u", (unsigned)(Ms / 3600000),
				 (unsigned)(Ms / 60000 % 60), (unsigned)(Ms / 1000 % 60), (unsigned)(Ms / 10 % 100));
}

// Time comes from the free-running hardware timer. Rendering, SPI and ADC
// delays do not become accumulated stopwatch error. Unsigned subtraction
// also covers the millisecond counter wrapping, provided we poll each wrap.
static void UpdateStopwatch()
{
	uint32_t now = s_Clock.mSecond();
	uint32_t delta = now - s_LastTick;
	s_LastTick = now;
	bool wasRunning = s_Running;
	if (s_Running)
	{
		s_ElapsedMs += delta;
		if (s_ElapsedMs >= s_MaxElapsedMs)
		{
			s_ElapsedMs = s_MaxElapsedMs;
			s_Running = false;
		}
	}
	if (s_Page == 3 && !s_InputError && wasRunning &&
		(now - s_LastPaint >= 100 || !s_Running))
	{
		s_LastPaint = now;
		DrawMenu();
	}
}
#endif

static void SetLed(bool On)
{
	s_LedOn = On;
	if (On == bool(MENU_LED_ACTIVE))
	{
		IOPinSet(s_LedPins[0].PortNo, s_LedPins[0].PinNo);
	}
	else
	{
		IOPinClear(s_LedPins[0].PortNo, s_LedPins[0].PinNo);
	}
}

static void DrawMenu()
{
	g_Lcd.Clear();
	g_Lcd.SetCursor(0, 0);
	if (s_InputError)
	{
		g_Lcd.printf("Joystick read failed");
		return;
	}
	if (s_Page == 1)
	{
		g_Lcd.printf("Level: %d", s_Level);
		g_Lcd.Fill(0, 16, s_Level * 10, 12, DISPLAY_PIXEL_SIZE == 1 ? 1 : 0xFFFF);
		g_Lcd.SetCursor(0, 40);
		g_Lcd.printf("Arrows: adjust\nSelect: back");
	}
	else if (s_Page == 2)
	{
		g_Lcd.printf("IOsonata\nJoystick + OLED\nGeneric ADC/display\n\nSelect/left: back");
	}
#ifdef JOYSTICK_STOPWATCH
	else if (s_Page == 3)
	{
		g_Lcd.printf("Stopwatch %s", s_Running ? "RUN" : "PAUSED");
		g_Lcd.SetCursor(0, 16);
		PrintElapsed(s_ElapsedMs);
		g_Lcd.SetCursor(0, 32);
		g_Lcd.printf("Lap ");
		if (s_HaveLap)
		{
			PrintElapsed(s_LapMs);
		}
		else
		{
			g_Lcd.printf("--:--:--.--");
		}
		g_Lcd.SetCursor(0, 48);
		g_Lcd.printf("SEL run/pause RT lap");
		g_Lcd.SetCursor(0, 56);
		g_Lcd.printf("LT reset UP/DN menu");
	}
#endif
	else
	{
		g_Lcd.printf("Joystick menu");
		g_Lcd.SetCursor(0, 16);
		g_Lcd.printf("%c LED: %s\n%c Level: %d\n%c About",
					 s_Item == 0 ? '>' : ' ', s_LedOn ? "on" : "off",
					 s_Item == 1 ? '>' : ' ', s_Level, s_Item == 2 ? '>' : ' ');
#ifdef JOYSTICK_STOPWATCH
		g_Lcd.printf("\n%c Stopwatch", s_Item == 3 ? '>' : ' ');
#endif
		g_Lcd.SetCursor(0, 56);
		g_Lcd.printf("UP/DN move SEL open");
	}
}

static void HandleKey(Key k)
{
	if (s_Page == 1)
	{
		if (k == KEY_SELECT)
		{
			s_Page = 0;
		}
		else if ((k == KEY_UP || k == KEY_RIGHT) && s_Level < 10)
		{
			s_Level++;
		}
		else if ((k == KEY_DOWN || k == KEY_LEFT) && s_Level > 1)
		{
			s_Level--;
		}
	}
	else if (s_Page == 2)
	{
		if (k == KEY_SELECT || k == KEY_LEFT)
		{
			s_Page = 0;
		}
	}
#ifdef JOYSTICK_STOPWATCH
	else if (s_Page == 3)
	{
		if (k == KEY_SELECT)
		{
			s_Running = !s_Running && s_ElapsedMs < s_MaxElapsedMs;
		}
		else if (k == KEY_RIGHT)
		{
			s_LapMs = s_ElapsedMs;
			s_HaveLap = true;
		}
		else if (k == KEY_LEFT && !s_Running)
		{
			s_ElapsedMs = s_LapMs = 0;
			s_HaveLap = false;
		}
		else if (k == KEY_UP || k == KEY_DOWN)
		{
			s_Page = 0;
		}
	}
#endif
	else
	{
		if (k == KEY_UP)
		{
			s_Item = (s_Item + s_ItemCount - 1) % s_ItemCount;
		}
		else if (k == KEY_DOWN)
		{
			s_Item = (s_Item + 1) % s_ItemCount;
		}
		else if (k == KEY_SELECT)
		{
			if (s_Item == 0)
			{
				SetLed(!s_LedOn);
			}
			else
			{
				s_Page = s_Item;
			}
		}
	}
	DrawMenu();
}

static Key DecodeKey(float Voltage)
{
	if (!isfinite(Voltage) || Voltage < 0 || Voltage > JOYSTICK_REF_VOLTAGE * 1.05f)
	{
		return KEY_INVALID;
	}
	// Board supplies the ladder ratios in select/left/down/up/right/idle order.
	static const unsigned levels[] = JOYSTICK_LEVELS_PERMILLE;
	float ratio = Voltage * 1000.0f / JOYSTICK_REF_VOLTAGE;
	for (unsigned i = 0; i < 5; i++)
	{
		if (ratio < (levels[i] + levels[i + 1]) / 2.0f)
		{
			return Key(i);
		}
	}
	return KEY_NONE;
}

// Called every 10 ms. Three equal samples debounce a transition. A press
// produces one action and must be released before another action is accepted.
static void PollJoystick()
{
#ifdef JOYSTICK_STOPWATCH
	UpdateStopwatch();
#endif
	AdcData_t data;
	Key k = g_Adc.Read(0, &data) ? DecodeKey(data.Data) : KEY_INVALID;
	if (k == KEY_INVALID)
	{
		s_Candidate = s_Stable = KEY_INVALID;
		s_Debounce = 0;
		if (!s_InputError)
		{
			s_InputError = true;
			DrawMenu();
		}
		return;
	}
	if (s_InputError)
	{
		s_InputError = false;
		DrawMenu();
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
		Key old = s_Stable;
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
#ifdef JOYSTICK_STOPWATCH
	if (!s_Clock.Init(s_TimerCfg))
	{
		g_Lcd.printf("Timer init failed");
		return false;
	}
	s_LastTick = s_Clock.mSecond();
#endif
	IOPinConfig(s_LedPins[0].PortNo, s_LedPins[0].PinNo, s_LedPins[0].PinOp,
				IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	SetLed(false);
	if (!g_Adc.Init(s_AdcCfg) || !g_Adc.OpenChannel(&s_JoyCfg, 1))
	{
		s_InputError = true;
		DrawMenu();
		return false;
	}
	DrawMenu();
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
