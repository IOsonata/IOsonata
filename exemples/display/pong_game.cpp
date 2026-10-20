/**-------------------------------------------------------------------------
@file	pong_game.cpp

@brief	Joystick-controlled Pong game for a small dot-matrix display

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2026

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
#include "idelay.h"
#include "board.h"
#include ADC_DRIVER_HEADER

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
static const AdcRefVolt_t s_Ref[] = {
	{ ADC_REFVOLT_TYPE_SUPPLY, JOYSTICK_REF_VOLTAGE, -1 }
};
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
static const TimerCfg_t s_TimerCfg = {
	.DevNo = TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1000000,
	.IntPrio = 7,
	.EvtHandler = nullptr,
	.bTickInt = false
};

static SPI g_Spi;
static DISPLAY_DRIVER g_Lcd;
static ADC_DRIVER g_Adc;
static Timer s_Clock;

enum PONG_KEY
{
	KEY_SELECT,
	KEY_LEFT,
	KEY_DOWN,
	KEY_UP,
	KEY_RIGHT,
	KEY_NONE,
	KEY_INVALID
};

enum GAME_STATE
{
	GAME_WAIT,
	GAME_PLAY,
	GAME_PAUSED,
	GAME_OVER
};

static const int s_FieldTop = 10;
static const int s_BallSize = 3;
static const int s_PaddleWidth = 3;
static const int s_PaddleHeight = 14;
static const int s_PlayerX = 3;
static const int s_AiX = DISPLAY_WIDTH - s_PlayerX - s_PaddleWidth;
static const int s_PaddleMinY = s_FieldTop;
static const int s_PaddleMaxY = DISPLAY_HEIGHT - s_PaddleHeight;
static const int s_BallMaxY = DISPLAY_HEIGHT - s_BallSize;
static const uint32_t s_FrameMs = 40;
static const unsigned s_WinScore = 9;

static GAME_STATE s_State = GAME_WAIT;
static PONG_KEY s_Candidate = KEY_INVALID;
static PONG_KEY s_Stable = KEY_INVALID;
static unsigned s_Debounce;
static bool s_SelectArmed;
static bool s_InputError;
static int s_PlayerY;
static int s_AiY;
static int s_BallX;
static int s_BallY;
static int s_BallVx;
static int s_BallVy;
static unsigned s_PlayerScore;
static unsigned s_AiScore;
static unsigned s_Rally;
static unsigned s_FrameNo;
static uint32_t s_LastFrame;

static int Clamp(int Value, int Lo, int Hi)
{
	if (Value < Lo) { return Lo; }
	if (Value > Hi) { return Hi; }
	return Value;
}

static bool Overlap(int Y1, int H1, int Y2, int H2)
{
	return Y1 < Y2 + H2 && Y1 + H1 > Y2;
}

static void ResetRound(int Direction)
{
	s_BallX = (DISPLAY_WIDTH - s_BallSize) / 2;
	s_BallY = (s_FieldTop + s_BallMaxY) / 2;
	s_BallVx = Direction < 0 ? -2 : 2;
	s_BallVy = ((s_PlayerScore + s_AiScore) & 1U) ? -1 : 1;
	s_Rally = 0;
}

static void ResetMatch()
{
	s_PlayerScore = 0;
	s_AiScore = 0;
	s_PlayerY = (s_PaddleMinY + s_PaddleMaxY) / 2;
	s_AiY = s_PlayerY;
	s_FrameNo = 0;
	ResetRound(1);
}

static void DrawGame()
{
	g_Lcd.Clear();
	g_Lcd.SetCursor(48, 0);
	g_Lcd.printf("%u  %u", s_PlayerScore, s_AiScore);
	if (s_InputError)
	{
		g_Lcd.SetCursor(0, 0);
		g_Lcd.printf("!");
	}
	g_Lcd.Fill(s_PlayerX, s_PlayerY, s_PaddleWidth, s_PaddleHeight, 1);
	g_Lcd.Fill(s_AiX, s_AiY, s_PaddleWidth, s_PaddleHeight, 1);
	g_Lcd.Fill(s_BallX, s_BallY, s_BallSize, s_BallSize, 1);
}

static void DrawState()
{
	g_Lcd.Clear();
	g_Lcd.SetCursor(0, 0);
	if (s_State == GAME_WAIT)
	{
		g_Lcd.printf("PONG\n\nUP/DN move paddle\nSELECT start");
	}
	else if (s_State == GAME_PAUSED)
	{
		g_Lcd.printf("PONG PAUSED\n%u - %u\n\nSELECT resume",
					 s_PlayerScore, s_AiScore);
	}
	else if (s_State == GAME_OVER)
	{
		g_Lcd.printf("%s\nScore %u - %u\n\nSELECT new game",
					 s_PlayerScore >= s_WinScore ? "YOU WIN" : "CPU WINS",
					 s_PlayerScore, s_AiScore);
	}
	if (s_InputError)
	{
		g_Lcd.SetCursor(0, 56);
		g_Lcd.printf("JOY ERROR");
	}
}

static void HandleSelect(uint32_t Now)
{
	if (s_State == GAME_WAIT)
	{
		s_State = GAME_PLAY;
		ResetRound(1);
	}
	else if (s_State == GAME_PLAY)
	{
		s_State = GAME_PAUSED;
	}
	else if (s_State == GAME_PAUSED)
	{
		s_State = GAME_PLAY;
	}
	else
	{
		ResetMatch();
		s_State = GAME_PLAY;
	}
	s_LastFrame = Now;
	if (s_State == GAME_PLAY) { DrawGame(); }
	else { DrawState(); }
}

static PONG_KEY DecodeKey(float Voltage)
{
	static const unsigned levels[] = JOYSTICK_LEVELS_PERMILLE;
	if (!isfinite(Voltage) || Voltage < 0 ||
		Voltage > JOYSTICK_REF_VOLTAGE * 1.05f)
	{
		return KEY_INVALID;
	}
	float ratio = Voltage * 1000.0f / JOYSTICK_REF_VOLTAGE;
	for (unsigned i = 0; i < 5; i++)
	{
		if (ratio < (levels[i] + levels[i + 1]) / 2.0f)
		{
			return PONG_KEY(i);
		}
	}
	return KEY_NONE;
}

static void ReadJoystick(uint32_t Now)
{
	AdcData_t data;
	PONG_KEY key = g_Adc.Read(0, &data) ? DecodeKey(data.Data) : KEY_INVALID;
	if (key == KEY_INVALID)
	{
		s_Candidate = s_Stable = KEY_INVALID;
		s_Debounce = 0;
		if (!s_InputError)
		{
			s_InputError = true;
			if (s_State != GAME_PLAY) { DrawState(); }
		}
		return;
	}
	if (s_InputError)
	{
		s_InputError = false;
		if (s_State != GAME_PLAY) { DrawState(); }
	}
	if (key != s_Candidate)
	{
		s_Candidate = key;
		s_Debounce = 1;
		return;
	}
	if (s_Debounce < 3)
	{
		s_Debounce++;
		if (s_Debounce < 3) { return; }
	}
	if (key == s_Stable) { return; }
	s_Stable = key;
	if (key == KEY_NONE)
	{
		s_SelectArmed = true;
	}
	else
	{
		if (key == KEY_SELECT && s_SelectArmed)
		{
			HandleSelect(Now);
		}
		s_SelectArmed = false;
	}
}

static void PaddleBounce(bool Player)
{
	int paddleY = Player ? s_PlayerY : s_AiY;
	int ballCenter = s_BallY + s_BallSize / 2;
	int paddleCenter = paddleY + s_PaddleHeight / 2;
	int offset = ballCenter - paddleCenter;
	int speed = s_Rally >= 6 ? 3 : 2;

	s_Rally++;
	if (Player)
	{
		s_BallX = s_PlayerX + s_PaddleWidth;
		s_BallVx = speed;
	}
	else
	{
		s_BallX = s_AiX - s_BallSize;
		s_BallVx = -speed;
	}
	if (offset <= -4) { s_BallVy = -2; }
	else if (offset >= 4) { s_BallVy = 2; }
	else { s_BallVy = offset < 0 ? -1 : 1; }
}

static void Score(bool PlayerPoint)
{
	if (PlayerPoint) { s_PlayerScore++; }
	else { s_AiScore++; }

	if (s_PlayerScore >= s_WinScore || s_AiScore >= s_WinScore)
	{
		s_State = GAME_OVER;
		DrawState();
		return;
	}
	// Serve toward the player that lost the point.
	ResetRound(PlayerPoint ? -1 : 1);
}

static void StepGame()
{
	if (s_Stable == KEY_UP)
	{
		s_PlayerY -= 2;
	}
	else if (s_Stable == KEY_DOWN)
	{
		s_PlayerY += 2;
	}
	s_PlayerY = Clamp(s_PlayerY, s_PaddleMinY, s_PaddleMaxY);

	// The CPU paddle is deliberately slower than the fastest ball trajectory.
	// This makes rallies sustainable without making the opponent unbeatable.
	if ((s_FrameNo++ & 1U) == 0)
	{
		int target = s_BallY + s_BallSize / 2 - s_PaddleHeight / 2;
		if (s_AiY < target) { s_AiY++; }
		else if (s_AiY > target) { s_AiY--; }
		s_AiY = Clamp(s_AiY, s_PaddleMinY, s_PaddleMaxY);
	}

	s_BallX += s_BallVx;
	s_BallY += s_BallVy;
	if (s_BallY < s_FieldTop)
	{
		s_BallY = s_FieldTop;
		s_BallVy = -s_BallVy;
	}
	else if (s_BallY > s_BallMaxY)
	{
		s_BallY = s_BallMaxY;
		s_BallVy = -s_BallVy;
	}

	if (s_BallVx < 0 &&
		s_BallX <= s_PlayerX + s_PaddleWidth &&
		s_BallX + s_BallSize >= s_PlayerX &&
		Overlap(s_BallY, s_BallSize, s_PlayerY, s_PaddleHeight))
	{
		PaddleBounce(true);
	}
	else if (s_BallVx > 0 &&
		s_BallX <= s_AiX + s_PaddleWidth &&
		s_BallX + s_BallSize >= s_AiX &&
		Overlap(s_BallY, s_BallSize, s_AiY, s_PaddleHeight))
	{
		PaddleBounce(false);
	}

	if (s_BallX < 0)
	{
		Score(false);
	}
	else if (s_BallX > DISPLAY_WIDTH - s_BallSize)
	{
		Score(true);
	}
}

static void PollJoystick()
{
	uint32_t now = s_Clock.mSecond();
	ReadJoystick(now);
	if (s_State == GAME_PLAY && now - s_LastFrame >= s_FrameMs)
	{
		s_LastFrame = now;
		StepGame();
		if (s_State == GAME_PLAY) { DrawGame(); }
	}
}

static bool HardwareInit()
{
	if (!g_Spi.Init(s_SpiCfg) || !g_Lcd.Init(s_LcdCfg, &g_Spi))
	{
		return false;
	}
	g_Lcd.Backlight(true);
	if (g_Lcd.Width() != DISPLAY_WIDTH || g_Lcd.Height() != DISPLAY_HEIGHT ||
		!g_Lcd.SetFont(&iFontSystem5x7))
	{
		return false;
	}
	if (!s_Clock.Init(s_TimerCfg))
	{
		g_Lcd.printf("Timer init failed");
		return false;
	}
	if (!g_Adc.Init(s_AdcCfg) || !g_Adc.OpenChannel(&s_JoyCfg, 1))
	{
		g_Lcd.printf("Joystick init failed");
		return false;
	}
	ResetMatch();
	s_State = GAME_WAIT;
	s_LastFrame = s_Clock.mSecond();
	DrawState();
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
		msDelay(5);
	}
}
