/**-------------------------------------------------------------------------
@example	codec_line_loopback.cpp

@brief	Line input to headphones through a WM8904 codec and the MCU

The WM8904 takes its stereo line input and plays the headphones. The MCU is
the I2S master and sends MCLK; it receives the frames of the codec and sends
them back. Each second the peak level of each channel in dBFS and the frames
moved are printed. board.h gives:

	AUDIO_I2S_DEVNO		I2S device
	AUDIO_I2S_PINS		I2S pin map with MCLK, SDI and SDO (coredev/i2s.h
						indexes) and what the I2S port needs for full duplex
	AUDIO_I2C_DEVNO		I2C bus of the codec
	AUDIO_I2C_PINS		its SDA and SCL pins
	AUDIO_CODEC_ADDR	7 bit codec address, normally 0x1A
	AUDIO_LINE_INPUT	codec line input, 1 to 3, 2 when not given
	AUDIO_RATE			frame rate asked, 48000 when not given

The I2S is stopped while the codec starts (about 0.4 s), MCLK runs: the
receive side does not fill up meanwhile. A failure of the stream, or no frame
for 0.5 s, stops the example with a message.

With UART_RETARGET and the UART pins, the Release build prints on the UART
at 1000000 baud 8N1, the Debug build uses semihosting. Semihosting stops the CPU
while it prints: use the Release build to listen.

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
#include <string.h>

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

#ifndef AUDIO_LINE_INPUT
#define AUDIO_LINE_INPUT	2
#endif

#define LOOP_HP_GAIN		-12000		// Headphone gain, milli-dB
#define LOOP_BLOCK			64			// Frames taken at a time
#define LOOP_IDLE_US		100			// Wait when nothing moves
#define LOOP_TIMEOUT_MS		500			// No frame received for this long

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

// About 20 ms at 48 kHz each way
alignas(4) static uint8_t s_RxMem[CFIFO_TOTAL_MEMSIZE(1024, 4)];
alignas(4) static uint8_t s_TxMem[CFIFO_TOTAL_MEMSIZE(1024, 4)];

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
	.pRxFifoMem = s_RxMem,
	.RxFifoMemSize = sizeof(s_RxMem),
	.pTxFifoMem = s_TxMem,
	.TxFifoMemSize = sizeof(s_TxMem)
};

static I2S s_Audio;
static I2C s_Control;
static Wm8904 s_Codec;
static volatile DEVINTRF_EVT s_StopEvt;
static volatile bool s_bStopped = false;

static int AudioEvent(DevIntrf_t *pDev, DEVINTRF_EVT Evt, uint8_t *pData, int Count)
{
	(void)pDev;
	(void)pData;
	(void)Count;

	if (Evt == DEVINTRF_EVT_STATECHG || Evt == DEVINTRF_EVT_RX_FIFO_FULL)
	{
		s_StopEvt = Evt;
		s_bStopped = true;
	}

	return 0;
}

// Peak of a channel in tenths of dBFS, -960 for silence
static int PeakDb10(unsigned Peak)
{
	return Peak > 0 ? (int)lrintf(200.0f * log10f((float)Peak / 32768.0f)) : -960;
}

static void PrintDb(const char *pName, int Db10)
{
	unsigned m = Db10 < 0 ? (unsigned)-Db10 : (unsigned)Db10;

	printf("%s %s%u.%u dBFS", pName, Db10 < 0 ? "-" : "", m / 10, m % 10);
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

	printf("\r\nWM8904 line input to headphones\r\n");

	if (s_Control.Init(s_ControlCfg) == false || s_Audio.Init(s_AudioCfg) == false)
	{
		printf("I2C or I2S init failed\r\n");

		return 1;
	}

	uint32_t rate = s_Audio.Rate();

	// MCLK keeps running while the I2S is stopped for the codec start
	s_Audio.Disable();

	Wm8904Cfg_t codecCfg = {
		.DevAddr = AUDIO_CODEC_ADDR,
		.Freq = rate,
		.MClkFreq = ((I2SDev_t *)s_Audio)->MClkFreq,
		.SampleWidth = I2SSWIDTH_16BITS,
		.Format = I2SFMT_NORMAL,
		.MicInput = 0,
		.Gain = 0,
		.bHeadphoneEn = true,
		.HeadphoneGain = LOOP_HP_GAIN,
		.LineInput = AUDIO_LINE_INPUT
	};

	if (s_Codec.Init(codecCfg, &s_Control) == false)
	{
		s_Audio.PowerOff();
		printf("WM8904 init failed\r\n");

		return 1;
	}
	s_Audio.Enable();
	printf("I2S %lu Hz, MCLK %lu Hz, line input %d, headphones %d dB\r\n", (unsigned long)rate,
		   (unsigned long)((I2SDev_t *)s_Audio)->MClkFreq, AUDIO_LINE_INPUT, LOOP_HP_GAIN / 1000);

	int16_t samples[LOOP_BLOCK * 2];
	unsigned pending = 0;		// Bytes received, not sent yet
	unsigned offset = 0;
	unsigned peak[2] = { 0, 0 };
	uint32_t frames = 0;		// Frames of the current second
	uint32_t total = 0;
	uint32_t idle = 0;			// Waits without a frame received

	while (1)
	{
		if (s_bStopped)
		{
			s_Codec.PowerOff();
			s_Audio.PowerOff();
			printf("I2S stream stopped: %s, %lu frames\r\n",
				   s_StopEvt == DEVINTRF_EVT_RX_FIFO_FULL ? "receive FIFO full" : "FIFO or DMA error",
				   (unsigned long)total);

			return 1;
		}

		bool moved = false;

		if (offset >= pending)
		{
			int n = s_Audio.Rx(0, (uint8_t *)samples, sizeof(samples));

			pending = n > 0 ? (unsigned)n : 0;
			offset = 0;
			if (n > 0)
			{
				moved = true;
				idle = 0;
				for (int i = 0; i < n / 2; i++)
				{
					unsigned m = samples[i] < 0 ? (unsigned)-samples[i] : (unsigned)samples[i];

					if (m > peak[i & 1])
					{
						peak[i & 1] = m;
					}
				}
				frames += n / 4;
				total += n / 4;
			}
		}
		if (offset < pending)
		{
			int n = s_Audio.Tx(0, (const uint8_t *)samples + offset, pending - offset);

			offset += n;
			moved = moved || n > 0;
		}
		if (frames >= rate)
		{
			PrintDb("L", PeakDb10(peak[0]));
			PrintDb(", R", PeakDb10(peak[1]));
			printf(", %lu frames\r\n", (unsigned long)total);
			frames = 0;
			peak[0] = peak[1] = 0;
		}
		if (moved == false)
		{
			usDelay(LOOP_IDLE_US);
			if (++idle > LOOP_TIMEOUT_MS * 1000 / LOOP_IDLE_US)
			{
				s_Codec.PowerOff();
				s_Audio.PowerOff();
				printf("No frame received for %d ms, %lu frames\r\n", LOOP_TIMEOUT_MS, (unsigned long)total);

				return 1;
			}
		}
	}
}
