/**-------------------------------------------------------------------------
@example	codec_tone.cpp

@brief	Tone on the headphones of a WM8904 codec

The MCU is the I2S master and sends MCLK, the WM8904 is set up over I2C for
headphone playback. The tone changes every second: 440 Hz on the left, 880 Hz
on the right, 660 Hz on both, then silence, with a 10 ms fade at each change.
The I2S rate, the MCLK and each step are printed. board.h gives:

	AUDIO_I2S_DEVNO		I2S device
	AUDIO_I2S_PINS		I2S pin map with MCLK (coredev/i2s.h indexes)
	AUDIO_I2C_DEVNO		I2C bus of the codec
	AUDIO_I2C_PINS		its SDA and SCL pins
	AUDIO_CODEC_ADDR	7 bit codec address, normally 0x1A
	AUDIO_RATE			frame rate asked, 48000 when not given

With UART_RETARGET and the UART pins, the Release build prints on the UART
at 1000000 baud 8N1, the Debug build uses semihosting. Semihosting stops the CPU
while it prints: the Debug build has short silences at each step.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#include <stdint.h>

#include "coredev/uart.h"
#include "coredev/i2c.h"
#include "coredev/i2s.h"
#include "audio/wm8904.h"
#include "idelay.h"
#include "stddev.h"

#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE		1000000
#endif

#ifndef AUDIO_RATE
#define AUDIO_RATE			48000
#endif

#define TONE_TABLE_SIZE		256			// Sine table, a power of 2
#define TONE_AMPLITUDE		4096		// -18 dBFS
#define TONE_HP_GAIN		-18000		// Headphone gain, milli-dB
#define TONE_BLOCK			64			// Frames given to the I2S at a time

#ifdef UART_RETARGET
static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(256)];

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.EvtCallback = NULL,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
};

static UART s_Uart;
#endif

static int AudioEvent(DevIntrf_t *pDev, DEVINTRF_EVT Evt, uint8_t *pData, int Count);

static const IOPinCfg_t s_AudioPins[] = AUDIO_I2S_PINS;
static const IOPinCfg_t s_ControlPins[] = AUDIO_I2C_PINS;

// 10 ms at 48 kHz
alignas(4) static uint8_t s_TxMem[CFIFO_TOTAL_MEMSIZE(512, 4)];

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
	.Freq = AUDIO_RATE,
	.MaxRetry = 0,
	.bDmaEn = true,
	.bIntEn = true,
	.IntPrio = 6,
	.EvtCB = AudioEvent,
	.pRxFifoMem = nullptr,
	.RxFifoMemSize = 0,
	.pTxFifoMem = s_TxMem,
	.TxFifoMemSize = sizeof(s_TxMem)
};

// One second each
static const struct {
	unsigned Freq;
	bool bLeft;
	bool bRight;
	const char *pName;
} s_Steps[] = {
	{ 440, true, false, "440 Hz left" },
	{ 880, false, true, "880 Hz right" },
	{ 660, true, true, "660 Hz left and right" },
	{ 0, false, false, "silence" },
};

static I2S s_Audio;
static I2C s_Control;
static Wm8904 s_Codec;
static volatile bool s_bStopped = false;
static int16_t s_Sine[TONE_TABLE_SIZE];
static uint32_t s_Rate;
static uint32_t s_Phase;
static uint32_t s_PhaseStep;
static uint32_t s_Frame;
static unsigned s_Step;

static int AudioEvent(DevIntrf_t *pDev, DEVINTRF_EVT Evt, uint8_t *pData, int Count)
{
	(void)pDev;
	(void)pData;
	(void)Count;

	if (Evt == DEVINTRF_EVT_STATECHG)
	{
		s_bStopped = true;
	}

	return 0;
}

// Stereo frames of the current step. Returns true when a new step starts.
static bool FillTone(int16_t *pSamples, unsigned Frames)
{
	bool next = false;
	uint32_t fade = s_Rate / 100;

	for (unsigned i = 0; i < Frames; i++)
	{
		uint32_t env = fade;

		if (s_Frame < fade)
		{
			env = s_Frame;
		}
		else if (s_Rate - s_Frame < fade)
		{
			env = s_Rate - s_Frame;
		}

		int v = s_Sine[s_Phase >> (32 - 8)] * (int)env / (int)fade;

		pSamples[i * 2] = s_Steps[s_Step].bLeft ? (int16_t)v : 0;
		pSamples[i * 2 + 1] = s_Steps[s_Step].bRight ? (int16_t)v : 0;
		s_Phase += s_PhaseStep;
		if (++s_Frame >= s_Rate)
		{
			s_Frame = 0;
			s_Phase = 0;
			s_Step = (s_Step + 1) % (sizeof(s_Steps) / sizeof(s_Steps[0]));
			s_PhaseStep = (uint32_t)(((uint64_t)s_Steps[s_Step].Freq << 32) / s_Rate);
			next = true;
		}
	}

	return next;
}

int main()
{
#ifdef UART_RETARGET
	if (s_Uart.Init(s_UartCfg) == false)
	{
		while (1);
	}

#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
#endif
#endif

	printf("\r\nWM8904 tone\r\n");

	for (int i = 0; i < TONE_TABLE_SIZE; i++)
	{
		s_Sine[i] = (int16_t)lrintf(TONE_AMPLITUDE * sinf(6.2831853f * (float)i / TONE_TABLE_SIZE));
	}

	// The I2S sends silence from the start, MCLK runs while the codec starts
	if (s_Control.Init(s_ControlCfg) == false || s_Audio.Init(s_AudioCfg) == false)
	{
		printf("I2C or I2S init failed\r\n");

		return 1;
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
		.HeadphoneGain = TONE_HP_GAIN,
		.LineInput = 0
	};

	if (s_Codec.Init(codecCfg, &s_Control) == false)
	{
		s_Audio.PowerOff();
		printf("WM8904 init failed\r\n");

		return 1;
	}
	printf("I2S %lu Hz, MCLK %lu Hz, headphones %d dB\r\n", (unsigned long)s_Rate,
		   (unsigned long)((I2SDev_t *)s_Audio)->MClkFreq, TONE_HP_GAIN / 1000);

	s_PhaseStep = (uint32_t)(((uint64_t)s_Steps[0].Freq << 32) / s_Rate);
	printf("%s\r\n", s_Steps[0].pName);

	int16_t samples[TONE_BLOCK * 2];
	unsigned offset = sizeof(samples);
	bool next = false;

	while (1)
	{
		if (s_bStopped)
		{
			s_Codec.PowerOff();
			s_Audio.PowerOff();
			printf("I2S stream stopped\r\n");

			return 1;
		}
		if (offset >= sizeof(samples))
		{
			next = FillTone(samples, TONE_BLOCK);
			offset = 0;
		}

		int n = s_Audio.Tx(0, (const uint8_t *)samples + offset, sizeof(samples) - offset);

		offset += n;
		if (next && offset >= sizeof(samples))
		{
			// The block with the change is given, the CFIFO holds 10 ms
			printf("%s\r\n", s_Steps[s_Step].pName);
			next = false;
		}
		if (n == 0)
		{
			usDelay(100);
		}
	}
}
