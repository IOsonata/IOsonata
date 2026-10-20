/**-------------------------------------------------------------------------
@file	i2s_loopback_demo.cpp

@brief	I2S microphone to headphones loopback example


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
#include <stdio.h>
#include <string.h>
#include "coredev/timer.h"
#include "coredev/i2c.h"
#include "coredev/i2s.h"
#include "coredev/spi.h"
#include "coredev/system_core_clock.h"
#include "audio/wm8904.h"
#include "display/display_ssd1315.h"
#include "display/display_hx8357.h"
#include "display/display_ili9341.h"
#include "display/display_st77xx.h"
#include "idelay.h"
#include "board.h"

// Disable capture while driving the microphone with an independent clock.
#ifndef AUDIO_LOOPBACK_MIC_ENABLE
#define AUDIO_LOOPBACK_MIC_ENABLE 1
#endif
#ifndef AUDIO_LOOPBACK_MIC_FORWARD
#define AUDIO_LOOPBACK_MIC_FORWARD 0
#endif
#ifndef AUDIO_LOOPBACK_MIC_INVERT
#define AUDIO_LOOPBACK_MIC_INVERT 0
#endif
#ifndef AUDIO_LOOPBACK_CLOCK_ONLY
#define AUDIO_LOOPBACK_CLOCK_ONLY 1
#endif

enum AudioError_t
{
	AUDIO_ERROR_NONE,
	AUDIO_ERROR_TRANSPORT,
	AUDIO_ERROR_RX_OVERFLOW,
	AUDIO_ERROR_RX_COUNT,
	AUDIO_ERROR_TX_COUNT,
	AUDIO_ERROR_RX_TIMEOUT,
	AUDIO_ERROR_TX_STALLED,
	AUDIO_ERROR_CODEC,
	AUDIO_ERROR_MIC_CLOCK,
	AUDIO_ERROR_DISPLAY
};

enum AudioStage_t
{
	AUDIO_STAGE_MIC,
	AUDIO_STAGE_MUTED,
	AUDIO_STAGE_ZERO,
	AUDIO_STAGE_TONE
};

#ifdef AUDIO_OSC_CFG
McuOsc_t g_McuOsc = AUDIO_OSC_CFG;
#endif

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
// Hold input through the codec's 100 ms bias and <=300 ms servo startup.
alignas(4) static uint8_t s_RxMem[CFIFO_TOTAL_MEMSIZE(8192, 4)];
alignas(4) static uint8_t s_TxMem[CFIFO_TOTAL_MEMSIZE(256, 4)];
static int16_t s_Samples[64 * 2];
static unsigned s_Pending;
static unsigned s_Offset;
static unsigned s_Peak;
static unsigned s_PairMismatches;
static uint32_t s_Frames;
static uint32_t s_LastPaint;
static uint32_t s_LastSample;
static uint32_t s_TestRate;
static uint32_t s_TestRemaining;
static uint32_t s_TestPhase;
static uint32_t s_TestStep;
static AudioStage_t s_LastStage;
static bool s_TestMuted;
static bool s_MicClockRestored;
static bool s_LastMicClockRestored;
static bool s_DisplayOff;
static bool s_DisplayTestDone;
static uint32_t s_DisplayOffSince;
static const char *const s_StageNames[] = {
	!AUDIO_LOOPBACK_MIC_ENABLE ? "MIC OFF - ZERO PCM" :
	(AUDIO_LOOPBACK_MIC_FORWARD ?
	(AUDIO_LOOPBACK_MIC_INVERT ? "MIC 180 DEG -> HP" : "MIC -> HEADPHONES") :
	(AUDIO_LOOPBACK_CLOCK_ONLY ? "CAP OFF CLK 2MHz" : "MIC CLK OFF ZERO")),
	"DAC muted",
	"Zero PCM",
	"Duplex tone 1 kHz"
};
static char s_DisplayText[4][32];
static unsigned s_DisplayRow = 4;
static const uint16_t s_DisplayY[] = { 0, 32, 40, 56 };
static const int16_t s_TestSine[] = {
	0, 1567, 2896, 3784, 4096, 3784, 2896, 1567,
	0, -1567, -2896, -3784, -4096, -3784, -2896, -1567
};
static volatile AudioError_t s_AudioError;
static I2S s_Audio;
static I2C s_Control;
static Wm8904 s_Codec;
static SPI s_Spi;
static DISPLAY_DRIVER s_Lcd;
static Timer s_Clock;
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
	.Chan = I2SCHAN_STEREO,
	.Freq = 15625,
	.MaxRetry = 0,
	.bDmaEn = true,
	.bIntEn = true,
	.IntPrio = 6,
	.EvtCB = AudioEvent,
	.pRxFifoMem = s_RxMem,
	.RxFifoMemSize = sizeof(s_RxMem),
	.pTxFifoMem = s_TxMem,
	.TxFifoMemSize = sizeof(s_TxMem)
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
	if (s_AudioError == AUDIO_ERROR_NONE)
	{
		if (Evt == DEVINTRF_EVT_STATECHG)
		{
			s_AudioError = AUDIO_ERROR_TRANSPORT;
		}
		else if (Evt == DEVINTRF_EVT_RX_FIFO_FULL)
		{
			s_AudioError = AUDIO_ERROR_RX_OVERFLOW;
		}
	}
	return 0;
}

static AudioStage_t OutputStage()
{
	if (s_TestRemaining > s_TestRate * 4)
	{
		return AUDIO_STAGE_TONE;
	}
	if (s_TestRemaining > s_TestRate * 2)
	{
		return AUDIO_STAGE_ZERO;
	}
	return s_TestRemaining ? AUDIO_STAGE_MUTED : AUDIO_STAGE_MIC;
}

static bool UpdateOutputMute()
{
	bool mute = OutputStage() == AUDIO_STAGE_MUTED;

	if (mute == s_TestMuted)
	{
		return true;
	}
	if (!s_Codec.Mute(mute))
	{
		return false;
	}
	s_TestMuted = mute;
	return true;
}

static bool MicrophoneClockRestoreDue()
{
	// Five seconds of startup, then five seconds of quiet zero PCM.
	return AUDIO_LOOPBACK_MIC_ENABLE && !AUDIO_LOOPBACK_MIC_FORWARD && !AUDIO_LOOPBACK_CLOCK_ONLY &&
		!s_MicClockRestored && s_TestRate && s_Frames >= s_TestRate * 10;
}

static void PrepareOutput(unsigned Frames)
{
	if (!AUDIO_LOOPBACK_MIC_ENABLE || !AUDIO_LOOPBACK_MIC_FORWARD)
	{
		memset(s_Samples, 0, Frames * 2 * sizeof(s_Samples[0]));
	}
	else if (AUDIO_LOOPBACK_MIC_INVERT)
	{
		for (unsigned sample = 0; sample < Frames * 2; sample++)
		{
			int32_t value = -int32_t(s_Samples[sample]);

			// -32768 has no positive counterpart in signed 16-bit PCM.
			s_Samples[sample] = value > INT16_MAX ? INT16_MAX : value;
		}
	}
	for (unsigned frame = 0; frame < Frames && s_TestRemaining; frame++)
	{
		int16_t value = 0;

		if (s_TestRemaining > s_TestRate * 4)
		{
			value = s_TestSine[s_TestPhase >> 28];
			s_TestPhase += s_TestStep;
		}
		s_Samples[frame * 2] = value;
		s_Samples[frame * 2 + 1] = value;
		s_TestRemaining--;
	}
}

static bool PumpAudio()
{
	bool progressed = false;

	if (s_Offset == s_Pending)
	{
		int count = s_Audio.Rx(0, (uint8_t *)s_Samples, sizeof(s_Samples));

		if (count < 0 || unsigned(count) > sizeof(s_Samples) || count % 4)
		{
			s_AudioError = AUDIO_ERROR_RX_COUNT;
			return false;
		}
		s_Pending = count;
		s_Offset = 0;
		if (count)
		{
			s_LastSample = s_Clock.mSecond();
			s_Frames += count / 4;
			progressed = true;
			for (unsigned i = 0; i < unsigned(count) / sizeof(s_Samples[0]); i++)
			{
				unsigned magnitude = s_Samples[i] < 0 ? -int(s_Samples[i]) : s_Samples[i];

				if (!(i & 1) && s_Samples[i] != s_Samples[i + 1])
				{
					s_PairMismatches++;
				}
				if (magnitude > s_Peak)
				{
					s_Peak = magnitude;
				}
			}
			PrepareOutput(count / 4);
		}
	}
	if (s_Offset < s_Pending)
	{
		int count = s_Audio.Tx(0, (const uint8_t *)s_Samples + s_Offset, s_Pending - s_Offset);

		if (count < 0 || unsigned(count) > s_Pending - s_Offset || count % 4)
		{
			s_AudioError = AUDIO_ERROR_TX_COUNT;
			return false;
		}
		s_Offset += count;
		progressed = progressed || count > 0;
	}
	return progressed;
}

static void DrawLevel()
{
	int db = s_Peak ? int(200.0f * log10f(float(s_Peak) / 32768.0f)) : -960;
	unsigned magnitude = db < 0 ? unsigned(-db) : unsigned(db);
	AudioStage_t stage = OutputStage();
	const char *label = s_StageNames[stage];

	if (stage == AUDIO_STAGE_MIC && AUDIO_LOOPBACK_MIC_ENABLE && !AUDIO_LOOPBACK_MIC_FORWARD && !AUDIO_LOOPBACK_CLOCK_ONLY && s_MicClockRestored)
	{
		label = "MIC CLK ON ZERO";
	}

	// Snapshot text only. ServiceDisplay sends one row after servicing audio.
	snprintf(s_DisplayText[0], sizeof(s_DisplayText[0]), "%-20s", label);
	snprintf(s_DisplayText[1], sizeof(s_DisplayText[1]), "L/R mismatches %-6u", s_PairMismatches);
	snprintf(s_DisplayText[2], sizeof(s_DisplayText[2]), "Peak %s%u.%u dBFS   ", db < 0 ? "-" : "", magnitude / 10, magnitude % 10);
	snprintf(s_DisplayText[3], sizeof(s_DisplayText[3]), "Frames %-13lu", (unsigned long)s_Frames);
	s_DisplayRow = stage != s_LastStage || s_MicClockRestored != s_LastMicClockRestored ? 0 : 1;
	s_LastStage = stage;
	s_LastMicClockRestored = s_MicClockRestored;
	s_Peak = 0;
	s_PairMismatches = 0;
}

static bool ServiceDisplay(bool AudioProgressed, uint32_t Now)
{
	if (AudioProgressed || s_Offset < s_Pending)
	{
		return false;
	}
	if (s_DisplayOff)
	{
		if (Now - s_DisplayOffSince < 5000)
		{
			return false;
		}
		if (!s_Lcd.Enable())
		{
			s_AudioError = AUDIO_ERROR_DISPLAY;
			return false;
		}
		s_DisplayOff = false;
		s_LastPaint = Now;
		DrawLevel();
		return true;
	}
	if (AUDIO_LOOPBACK_MIC_ENABLE && AUDIO_LOOPBACK_MIC_FORWARD && !AUDIO_LOOPBACK_MIC_INVERT && !s_DisplayTestDone && OutputStage() == AUDIO_STAGE_MIC)
	{
		// Compare live audio with the OLED panel and charge pump disabled.
		s_Lcd.Disable();
		s_DisplayOff = true;
		s_DisplayTestDone = true;
		s_DisplayOffSince = Now;
		return true;
	}
	if (Now - s_LastPaint >= 100 && s_DisplayRow == 4)
	{
		s_LastPaint = Now;
		DrawLevel();
	}
	if (s_DisplayRow == 4)
	{
		return false;
	}
	s_Lcd.SetCursor(0, s_DisplayY[s_DisplayRow]);
	s_Lcd.printf("%-20.20s", s_DisplayText[s_DisplayRow]);
	s_DisplayRow++;
	return true;
}

static bool PrepareStream()
{
	// Discard startup audio without stopping the clock needed by the codec.
	for (unsigned block = 0; block < 256; block++)
	{
		int count = s_Audio.Rx(0, (uint8_t *)s_Samples, sizeof(s_Samples));

		if (count < 0 || s_AudioError != AUDIO_ERROR_NONE)
		{
			return false;
		}
		if (!count)
		{
			// Keep about 8 ms of silence ahead of the live stream so OLED
			// updates do not starve output. Retain both DMA clocks throughout.
			memset(s_Samples, 0, sizeof(s_Samples));
			for (unsigned i = 0; i < 2; i++)
			{
				if (s_Audio.Tx(0, (const uint8_t *)s_Samples, sizeof(s_Samples)) != int(sizeof(s_Samples)))
				{
					return false;
				}
			}
			s_Pending = 0;
			s_Offset = 0;
			return true;
		}
	}
	return false;
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
	s_Lcd.printf("Starting audio...");
#ifdef AUDIO_OSC_CFG
	if (GetHighFreqOscType() == OSC_TYPE_RC || SystemCoreClockGet() != AUDIO_CORE_FREQ)
	{
		s_Lcd.printf("Crystal clock failed");
		return false;
	}
#endif
	if (!s_Control.Init(s_ControlCfg) || !s_Audio.Init(s_AudioCfg))
	{
		s_Lcd.printf("Audio bus init failed");
		return false;
	}
	Wm8904Cfg_t codecCfg = {
		.DevAddr = AUDIO_CODEC_ADDR,
		.Freq = s_Audio.Rate(),
		.MClkFreq = ((I2SDev_t *)s_Audio)->MClkFreq,
		.SampleWidth = I2SSWIDTH_16BITS,
		.Format = I2SFMT_NORMAL,
		.MicInput = AUDIO_LOOPBACK_MIC_ENABLE ? AUDIO_MIC_INPUT : 0,
		.Gain = 18000,
		.bHeadphoneEn = true,
		.HeadphoneGain = -12000
	};
	if (!s_Codec.Init(codecCfg, &s_Control))
	{
		s_Audio.PowerOff();
		s_Lcd.printf("WM8904 init failed");
		return false;
	}
	if (AUDIO_LOOPBACK_MIC_ENABLE && !AUDIO_LOOPBACK_MIC_FORWARD)
	{
		bool ready = AUDIO_LOOPBACK_CLOCK_ONLY ? s_Codec.MicrophoneClockOnly() : s_Codec.MicrophoneClock(false);

		if (!ready)
		{
			s_Audio.PowerOff();
			s_Lcd.printf("Mic clock setup failed");
			return false;
		}
	}
	s_Lcd.Clear();
	s_Lcd.SetCursor(0, 0);
	s_Lcd.printf("Duplex tone 1 kHz");
	s_Lcd.SetCursor(0, 16);
#ifdef AUDIO_OSC_CFG
	s_Lcd.printf("XTAL %lu Hz", (unsigned long)s_Audio.Rate());
#else
	s_Lcd.printf("%lu Hz stereo", (unsigned long)s_Audio.Rate());
#endif
	s_Lcd.SetCursor(0, 24);
	s_Lcd.printf("MICBIAS OFF -12dB");
	if (!PrepareStream())
	{
		s_Codec.PowerOff();
		s_Audio.PowerOff();
		s_Lcd.printf("Audio startup failed");
		return false;
	}
	s_TestRate = s_Audio.Rate();
	s_TestRemaining = s_TestRate * 5;
	s_TestPhase = 0;
	s_TestStep = (uint64_t(1000) << 32) / s_TestRate;
	s_LastStage = AUDIO_STAGE_TONE;
	s_TestMuted = false;
	s_MicClockRestored = false;
	s_LastMicClockRestored = false;
	s_DisplayOff = false;
	s_DisplayTestDone = false;
	s_LastSample = s_Clock.mSecond();
	s_LastPaint = s_LastSample;
	return true;
}

static const char *AudioErrorText(uint32_t Now)
{
	if (s_AudioError == AUDIO_ERROR_NONE && Now - s_LastSample > 500)
	{
		s_AudioError = s_Offset < s_Pending ? AUDIO_ERROR_TX_STALLED : AUDIO_ERROR_RX_TIMEOUT;
	}
	switch (s_AudioError)
	{
		case AUDIO_ERROR_TRANSPORT:
			return "SAI/DMA error";
		case AUDIO_ERROR_RX_OVERFLOW:
			return "RX FIFO overflow";
		case AUDIO_ERROR_RX_COUNT:
			return "Invalid RX count";
		case AUDIO_ERROR_TX_COUNT:
			return "Invalid TX count";
		case AUDIO_ERROR_RX_TIMEOUT:
			return "RX timeout";
		case AUDIO_ERROR_TX_STALLED:
			return "TX stalled";
		case AUDIO_ERROR_CODEC:
			return "Codec mute failed";
		case AUDIO_ERROR_MIC_CLOCK:
			return "Mic clock restore failed";
		case AUDIO_ERROR_DISPLAY:
			return "OLED enable failed";
		default:
			return nullptr;
	}
}

int main()
{
	if (!HardwareInit())
	{
		return 1;
	}
	while (1)
	{
		bool progressed = PumpAudio();

		if (s_AudioError == AUDIO_ERROR_NONE && !UpdateOutputMute())
		{
			s_AudioError = AUDIO_ERROR_CODEC;
		}
		uint32_t now = s_Clock.mSecond();

		if (s_AudioError == AUDIO_ERROR_NONE && MicrophoneClockRestoreDue())
		{
			if (s_Codec.MicrophoneClock(true))
			{
				s_MicClockRestored = true;
			}
			else
			{
				s_AudioError = AUDIO_ERROR_MIC_CLOCK;
			}
		}
		const char *error = AudioErrorText(now);

		if (error)
		{
			s_Codec.PowerOff();
			s_Audio.PowerOff();
			s_Lcd.Enable();
			s_Lcd.Clear();
			s_Lcd.printf("Loopback stopped");
			s_Lcd.SetCursor(0, 16);
			s_Lcd.printf("%s", error);
			s_Lcd.SetCursor(0, 32);
			s_Lcd.printf("Frames %lu", (unsigned long)s_Frames);
			s_Lcd.SetCursor(0, 48);
			s_Lcd.printf("Pending %u bytes", s_Pending - s_Offset);
			return 1;
		}
		bool painted = ServiceDisplay(progressed, now);

		if (!progressed && !painted)
		{
			usDelay(100);
		}
	}
}
