/**-------------------------------------------------------------------------
@file	i2s_mic_level_demo.cpp

@brief	I2S microphone capture and display level meter


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
#include "coredev/i2c.h"
#include "coredev/i2s.h"
#include "coredev/spi.h"
#include "coredev/timer.h"
#include "audio/wm8904.h"
#include "display/display_ssd1315.h"
#include "display/display_hx8357.h"
#include "display/display_ili9341.h"
#include "display/display_st77xx.h"
#include "idelay.h"
#include "board.h"

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
static const IOPinCfg_t s_AudioPins[] = AUDIO_I2S_PINS;
static const IOPinCfg_t s_ControlPins[] = AUDIO_I2C_PINS;
alignas(4) static uint8_t s_AudioMem[CFIFO_TOTAL_MEMSIZE(1024, 2)];
static volatile bool s_AudioFailed;
static I2S s_Audio;
static I2C s_Control;
static Wm8904 s_Codec;
static SPI s_Spi;
static DISPLAY_DRIVER s_Lcd;
static Timer s_Clock;
static uint32_t s_LastPaint;
static uint32_t s_LastSample;
static uint32_t s_Samples;
static unsigned s_Peak;
static unsigned s_ClipCount;
static int AudioEvent(DevIntrf_t *pDev, DEVINTRF_EVT Evt, uint8_t *pData, int Count);

static const I2CCfg_t s_ControlCfg = {
	.DevNo = AUDIO_I2C_DEVNO,
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_ControlPins,
	.NbIOPins = sizeof(s_ControlPins) / sizeof(s_ControlPins[0]),
	.Rate = 100000,
	.MaxRetry = 2,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,
	.SlaveAddr = { 0 },
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = 6,
	.EvtCB = nullptr
};
static const I2SCfg_t s_AudioCfg = {
	.DevNo = AUDIO_I2S_DEVNO,
	.pIOPinMap = s_AudioPins,
	.NbIOPins = sizeof(s_AudioPins) / sizeof(s_AudioPins[0]),
	.Mode = I2SMODE_MASTER,
	.Format = I2SFMT_NORMAL,
	.SampleWidth = I2SSWIDTH_16BITS,
	.Chan = I2SCHAN_LEFT,
	.Freq = 16000,
	.MaxRetry = 0,
	.bDmaEn = true,
	.bIntEn = true,
	.IntPrio = 6,
	.EvtCB = AudioEvent,
	.pRxFifoMem = s_AudioMem,
	.RxFifoMemSize = sizeof(s_AudioMem),
	.pTxFifoMem = nullptr,
	.TxFifoMemSize = 0
};
static const TimerCfg_t s_TimerCfg = {
	.DevNo = TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1000000,
	.IntPrio = 7,
	.EvtHandler = nullptr,
	.bTickInt = false
};

static int AudioEvent(DevIntrf_t *pDev, DEVINTRF_EVT Evt, uint8_t *pData, int Count)
{
	(void)pDev;
	(void)pData;
	(void)Count;
	if (Evt == DEVINTRF_EVT_STATECHG || Evt == DEVINTRF_EVT_RX_FIFO_FULL)
	{
		s_AudioFailed = true;
	}
	return 0;
}

static void DrawLevel()
{
	int db = s_Peak ? int(200.0f * log10f(float(s_Peak) / 32768.0f)) : -960;
	unsigned magnitude = db < 0 ? unsigned(-db) : unsigned(db);
	unsigned width = db > -720 ? unsigned(db + 720) * 128U / 720U : 0;

	s_Lcd.Clear();
	s_Lcd.SetCursor(0, 0);
	s_Lcd.printf("I2S mic %lu Hz", (unsigned long)s_Audio.Rate());
	s_Lcd.SetCursor(0, 16);
	s_Lcd.printf("Peak %s%u.%u dBFS", db < 0 ? "-" : "", magnitude / 10, magnitude % 10);
	s_Lcd.Fill(0, 28, width, 8, DISPLAY_PIXEL_SIZE == 1 ? 1 : 0xFFFF);
	s_Lcd.SetCursor(0, 40);
	s_Lcd.printf("Samples %lu", (unsigned long)s_Samples);
	s_Lcd.SetCursor(0, 56);
	s_Lcd.printf("Clipped %u", s_ClipCount);
	s_Peak = 0;
}

static bool HardwareInit()
{
	if (!s_Spi.Init(s_SpiCfg) || !s_Lcd.Init(s_LcdCfg, &s_Spi))
	{
		return false;
	}
	s_Lcd.Backlight(true);
	if (!s_Lcd.SetFont(&iFontSystem5x7) || !s_Clock.Init(s_TimerCfg))
	{
		return false;
	}
	if (!s_Control.Init(s_ControlCfg) || !s_Audio.Init(s_AudioCfg))
	{
		s_Lcd.printf("Audio bus init failed");
		return false;
	}
	// Do not fill the capture FIFO during the codec's bias settling delay.
	s_Audio.Disable();
	Wm8904Cfg_t codecCfg = {
		.DevAddr = AUDIO_CODEC_ADDR,
		.Freq = s_Audio.Rate(),
		.MClkFreq = ((I2SDev_t *)s_Audio)->MClkFreq,
		.SampleWidth = I2SSWIDTH_16BITS,
		.Format = I2SFMT_NORMAL,
		.MicInput = AUDIO_MIC_INPUT,
		.Gain = 18000,
		.bHeadphoneEn = false,
		.HeadphoneGain = -24000
	};
	if (!s_Codec.Init(codecCfg, &s_Control))
	{
		s_Audio.PowerOff();
		s_Lcd.printf("WM8904 init failed");
		return false;
	}
	s_Audio.Enable();
	s_LastSample = s_Clock.mSecond();
	s_LastPaint = s_LastSample;
	DrawLevel();
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
		int16_t samples[64];
		int count = s_Audio.Rx(0, (uint8_t *)samples, sizeof(samples));
		uint32_t now = s_Clock.mSecond();

		if (count > 0)
		{
			s_LastSample = now;
			s_Samples += count / sizeof(samples[0]);
			for (unsigned i = 0; i < unsigned(count) / sizeof(samples[0]); i++)
			{
				unsigned magnitude = samples[i] < 0 ? -int(samples[i]) : samples[i];

				if (magnitude > s_Peak)
				{
					s_Peak = magnitude;
				}
				if (magnitude >= 32760)
				{
					s_ClipCount++;
				}
			}
		}
		if (s_AudioFailed || now - s_LastSample > 500)
		{
			s_Codec.Disable();
			s_Audio.PowerOff();
			s_Lcd.Clear();
			s_Lcd.printf("Audio capture failed");
			return 1;
		}
		if (now - s_LastPaint >= 100)
		{
			s_LastPaint = now;
			DrawLevel();
		}
		if (!count)
		{
			usDelay(100);
		}
	}
}
