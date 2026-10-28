/**-------------------------------------------------------------------------
@file	i2s_tone_demo.cpp

@brief	I2S stereo headphone tone example


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

#include "coredev/i2c.h"
#include "coredev/i2s.h"
#include "coredev/spi.h"
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
alignas(4) static uint8_t s_AudioMem[CFIFO_TOTAL_MEMSIZE(256, 4)];
static volatile bool s_AudioFailed;
static I2S s_Audio;
static I2C s_Control;
static Wm8904 s_Codec;
static SPI s_Spi;
static DISPLAY_DRIVER s_Lcd;
static uint32_t s_Rate;
static uint32_t s_Phase;
static uint32_t s_PhaseStep;
static uint32_t s_Frame;
static unsigned s_Step;
static unsigned s_LastPaint = 4;
static int AudioEvent(DevIntrf_t *pDev, DEVINTRF_EVT Evt, uint8_t *pData, int Count);

static const int16_t s_Sine[32] = {
	0, 799, 1567, 2276, 2896, 3406, 3784, 4017,
	4096, 4017, 3784, 3406, 2896, 2276, 1567, 799,
	0, -799, -1567, -2276, -2896, -3406, -3784, -4017,
	-4096, -4017, -3784, -3406, -2896, -2276, -1567, -799
};
static const unsigned s_Frequencies[] = { 440, 880, 660, 0 };
static const char *const s_Labels[] = { "LEFT 440 Hz", "RIGHT 880 Hz", "BOTH 660 Hz", "SILENCE" };

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
	.Chan = I2SCHAN_STEREO,
	.Freq = 16000,
	.MaxRetry = 0,
	.bDmaEn = true,
	.bIntEn = true,
	.IntPrio = 6,
	.EvtCB = AudioEvent,
	.pRxFifoMem = nullptr,
	.RxFifoMemSize = 0,
	.pTxFifoMem = s_AudioMem,
	.TxFifoMemSize = sizeof(s_AudioMem)
};

static int AudioEvent(DevIntrf_t *pDev, DEVINTRF_EVT Evt, uint8_t *pData, int Count)
{
	(void)pDev;
	(void)pData;
	(void)Count;
	if (Evt == DEVINTRF_EVT_STATECHG)
	{
		s_AudioFailed = true;
	}
	return 0;
}

static void FillTone(int16_t *pSamples, unsigned Frames)
{
	for (unsigned frame = 0; frame < Frames; frame++)
	{
		uint32_t fade = s_Rate / 100;
		uint32_t envelope = fade;
		int value;

		if (s_Frame < fade)
		{
			envelope = s_Frame;
		}
		else if (s_Rate - s_Frame < fade)
		{
			envelope = s_Rate - s_Frame;
		}
		if (!s_Frame)
		{
			s_PhaseStep = uint32_t((uint64_t(s_Frequencies[s_Step]) << 32) / s_Rate);
		}
		value = s_Sine[s_Phase >> 27] * int(envelope) / int(fade);
		pSamples[frame * 2] = (s_Step == 0 || s_Step == 2) ? value : 0;
		pSamples[frame * 2 + 1] = (s_Step == 1 || s_Step == 2) ? value : 0;
		s_Phase += s_PhaseStep;
		if (++s_Frame == s_Rate)
		{
			s_Frame = 0;
			s_Phase = 0;
			s_Step = (s_Step + 1) % 4;
		}
	}
}

static void DrawStep()
{
	s_Lcd.Clear();
	s_Lcd.SetCursor(0, 0);
	s_Lcd.printf("I2S tone %lu Hz", (unsigned long)s_Rate);
	s_Lcd.SetCursor(0, 16);
	s_Lcd.printf("%s", s_Labels[s_Step]);
	s_Lcd.SetCursor(0, 32);
	s_Lcd.printf("Headphones -24 dB");
	s_Lcd.SetCursor(0, 48);
	s_Lcd.printf("L / R / BOTH / OFF");
	s_LastPaint = s_Step;
}

static bool HardwareInit()
{
	if (!s_Spi.Init(s_SpiCfg) || !s_Lcd.Init(s_LcdCfg, &s_Spi))
	{
		return false;
	}
	s_Lcd.Backlight(true);
	if (!s_Lcd.SetFont(&iFontSystem5x7))
	{
		return false;
	}
	// TX starts with silence and keeps MCLK running through codec startup.
	if (!s_Control.Init(s_ControlCfg) || !s_Audio.Init(s_AudioCfg))
	{
		s_Lcd.printf("Audio bus init failed");
		return false;
	}
	s_Rate = s_Audio.Rate();
	Wm8904Cfg_t codecCfg = {
		.DevAddr = AUDIO_CODEC_ADDR,
		.Freq = s_Rate,
		.MClkFreq = ((I2SDev_t *)s_Audio)->MClkFreq,
		.SampleWidth = I2SSWIDTH_16BITS,
		.Format = I2SFMT_NORMAL,
		.MicInput = 0,
		.Gain = 0,
		.bHeadphoneEn = true,
		.HeadphoneGain = -24000,
		.LineInput = 0
	};
	if (!s_Codec.Init(codecCfg, &s_Control))
	{
		s_Audio.PowerOff();
		s_Lcd.printf("WM8904 init failed");
		return false;
	}
	DrawStep();
	return true;
}

int main()
{
	int16_t samples[64 * 2];
	unsigned offset = sizeof(samples);

	if (!HardwareInit())
	{
		return 1;
	}
	while (1)
	{
		if (s_AudioFailed)
		{
			s_Codec.PowerOff();
			s_Audio.PowerOff();
			s_Lcd.Clear();
			s_Lcd.printf("Audio output failed");
			return 1;
		}
		if (offset == sizeof(samples))
		{
			FillTone(samples, 64);
			offset = 0;
		}
		int count = s_Audio.Tx(0, (const uint8_t *)samples + offset, sizeof(samples) - offset);

		offset += count;
		if (s_Step != s_LastPaint)
		{
			DrawStep();
		}
		if (!count)
		{
			usDelay(100);
		}
	}
}
