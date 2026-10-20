/**-------------------------------------------------------------------------
@file	i2s_loopback_test.cpp

@brief	Host test of the I2S loopback example: forwarding, startup and configuration.

Run by tests/audio/run_i2s_loopback.py.

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

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define main LoopbackDemoMain
#include "../../exemples/audio/i2s_loopback_demo.cpp"
#undef main

static const unsigned s_RxChunks[] = { 4, 28, 256 };
static const unsigned s_TxChunks[] = { 4, 12, 256 };
static uint8_t s_Input[2048];
static uint8_t s_Output[2048];
static unsigned s_InputSize;
static unsigned s_InputPos;
static unsigned s_OutputPos;
static unsigned s_RxChunk;
static unsigned s_TxChunk;
static unsigned s_RxCalls;
static unsigned s_TxCalls;
static bool s_Stall;
static bool s_Forever;
static uint64_t s_Ticks;
static bool s_DisplayTesting;
static bool s_Continuous;
static uint64_t s_StreamUs;
static uint64_t s_StreamFraction;
static unsigned s_StreamRx;
static unsigned s_StreamTx;
static unsigned s_StreamUnderruns;
static unsigned s_StreamMaxRx;
static unsigned s_DisplayCalls;
static bool s_DisplayData;
static void AdvanceStream(unsigned Microseconds)
{
	s_StreamUs += Microseconds;
	s_StreamFraction += uint64_t(Microseconds) * 15625;
	unsigned frames = s_StreamFraction / 1000000;

	s_StreamFraction %= 1000000;
	s_StreamRx += frames;
	if (s_StreamRx > s_StreamMaxRx)
	{
		s_StreamMaxRx = s_StreamRx;
	}
	if (s_StreamRx > 8192)
	{
		AudioEvent(s_Audio, DEVINTRF_EVT_RX_FIFO_FULL, nullptr, 0);
	}
	if (frames > s_StreamTx)
	{
		s_StreamUnderruns += frames - s_StreamTx;
		s_StreamTx = 0;
	}
	else
	{
		s_StreamTx -= frames;
	}
}

static int DisplayTransmit(DevIntrf_t *, const uint8_t *pData, int Count)
{
	assert(s_DisplayTesting && Count > 0);
	// The normal loopback must never send the OLED power-off sequence.
	assert(s_DisplayData || Count != 3 || pData[0] != 0xAE || pData[1] != 0x8D || pData[2] != 0x10);
	s_DisplayCalls++;
	if (s_Continuous)
	{
		// Stress polling and rendering overhead at 20 us per SPI byte.
		// This is an injected delay, not measured board timing.
		AdvanceStream(Count * 20);
	}
	return Count;
}

static void ControlDisable(DevIntrf_t *)
{
}

extern "C" void I2CBusReset(I2CDev_t *)
{
	assert(false);
}

extern "C" void I2CSetReadRqstData(I2CDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" void I2CSetWriteRqstBuffer(I2CDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" void SPISetSlaveRxBuffer(SPIDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" void SPISetSlaveTxData(SPIDev_t *, int, uint8_t *, int)
{
	assert(false);
}

extern "C" SPIPHY SPISetPhy(SPIDev_t *, SPIPHY Phy)
{
	assert(false);
	return Phy;
}

extern "C" void IOPinConfig(int Port, int Pin, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
	assert(s_DisplayTesting && Port == IOPORTE && (Pin == 0 || Pin == 3));
}

extern "C" void IOPinSet(int Port, int Pin)
{
	assert(s_DisplayTesting && Port == IOPORTE && (Pin == 0 || Pin == 3));
	if (Pin == 0)
	{
		s_DisplayData = true;
	}
}

extern "C" void IOPinClear(int Port, int Pin)
{
	assert(s_DisplayTesting && Port == IOPORTE && (Pin == 0 || Pin == 3));
	if (Pin == 0)
	{
		s_DisplayData = false;
	}
}

static uint64_t TickCount(TimerDev_t *)
{
	return s_Continuous ? s_StreamUs : ++s_Ticks;
}

extern "C" bool TimerInit(TimerDev_t *, const TimerCfg_t *)
{
	assert(false);
	return false;
}

static bool Start(DevIntrf_t *, uint32_t Addr)
{
	assert(Addr == 0);
	return true;
}

static void Stop(DevIntrf_t *)
{
}

static int Receive(DevIntrf_t *, uint8_t *pBuff, int Len)
{
	s_RxCalls++;
	if (s_Continuous)
	{
		unsigned frames = s_StreamRx & ~31U;

		if (frames > unsigned(Len) / 4)
		{
			frames = Len / 4;
		}
		memset(pBuff, 0, frames * 4);
		s_StreamRx -= frames;
		return frames * 4;
	}
	if (s_Forever)
	{
		memset(pBuff, 0, Len);
		return Len;
	}
	unsigned count = s_InputSize - s_InputPos;

	if (count > unsigned(Len))
	{
		count = Len;
	}
	if (count > s_RxChunk)
	{
		count = s_RxChunk;
	}
	memcpy(pBuff, s_Input + s_InputPos, count);
	s_InputPos += count;
	return count;
}

static int Transmit(DevIntrf_t *, const uint8_t *pData, int Len)
{
	s_TxCalls++;
	if (s_Continuous)
	{
		unsigned frames = 256 - s_StreamTx;

		if (frames > unsigned(Len) / 4)
		{
			frames = Len / 4;
		}
		s_StreamTx += frames;
		return frames * 4;
	}
	if (s_Stall && s_TxCalls % 3 == 0)
	{
		return 0;
	}
	unsigned count = s_TxChunk;

	if (count > unsigned(Len))
	{
		count = Len;
	}
	assert(s_OutputPos + count <= sizeof(s_Output));
	memcpy(s_Output + s_OutputPos, pData, count);
	s_OutputPos += count;
	return count;
}

static void ResetModel()
{
	s_InputSize = sizeof(s_Input);
	s_InputPos = 0;
	s_OutputPos = 0;
	s_RxCalls = 0;
	s_TxCalls = 0;
	s_Stall = false;
	s_Forever = false;
	s_Pending = 0;
	s_Offset = 0;
	s_Frames = 0;
	s_Peak = 0;
	s_PairMismatches = 0;
	s_AudioError = AUDIO_ERROR_NONE;
	DevIntrf_t *audio = s_Audio;

	atomic_flag_clear(&audio->bBusy);
	memset(s_Output, 0xA5, sizeof(s_Output));
}

int main()
{
	DevIntrf_t *control = s_Control;
	DevIntrf_t *audio = s_Audio;
	TimerDev_t *clock = s_Clock;

	control->Disable = ControlDisable;
	atomic_store(&control->EnCnt, 1);
	audio->StartRx = Start;
	audio->StartTx = Start;
	audio->StopRx = Stop;
	audio->StopTx = Stop;
	audio->RxData = Receive;
	audio->TxData = Transmit;
	audio->MaxRetry = 0;
	clock->Freq = 1000;
	clock->GetTickCount = TickCount;
	assert(s_AudioCfg.Chan == I2SCHAN_STEREO && s_AudioCfg.bDmaEn && s_AudioCfg.bIntEn);
	assert(s_AudioCfg.pRxFifoMem == s_RxMem && s_AudioCfg.pTxFifoMem == s_TxMem);
	assert(s_AudioCfg.RxFifoMemSize >= int(CFIFO_TOTAL_MEMSIZE(8192, 4)));
	assert(s_AudioPins[I2S_SDI_IOPIN_IDX].PortNo == IOPORTC);
	assert(s_AudioPins[I2S_SDO_IOPIN_IDX].PortNo == IOPORTD);
	for (unsigned i = 0; i < sizeof(s_Input); i++)
	{
		s_Input[i] = i * 37 + 19;
	}
	s_Input[0] = 0;
	s_Input[1] = 0x80;
	for (unsigned rx : s_RxChunks)
	{
		for (unsigned tx : s_TxChunks)
		{
			ResetModel();
			s_RxChunk = rx;
			s_TxChunk = tx;
			s_Stall = true;
			for (unsigned i = 0; i < 4096 && s_OutputPos < sizeof(s_Input); i++)
			{
				unsigned calls = s_RxCalls;
				bool pending = s_Offset < s_Pending;

				PumpAudio();
				assert(!pending || s_RxCalls == calls);
				assert(s_AudioError == AUDIO_ERROR_NONE);
			}
			assert(s_OutputPos == sizeof(s_Input));
			assert(memcmp(s_Input, s_Output, sizeof(s_Input)) == 0);
			assert(s_Frames == sizeof(s_Input) / 4 && s_Peak == 32768);
			assert(s_PairMismatches == sizeof(s_Input) / 4);
			assert(s_LastSample > 0 && s_Offset == s_Pending);
			assert(!PumpAudio());
		}
	}
	ResetModel();
	s_RxChunk = 256;
	s_TxChunk = 256;
	assert(PrepareStream());
	assert(s_InputPos == sizeof(s_Input) && s_OutputPos == 512 && s_Frames == 0);
	for (unsigned i = 0; i < s_OutputPos; i++)
	{
		assert(s_Output[i] == 0);
	}
	ResetModel();
	s_Forever = true;
	assert(!PrepareStream() && s_RxCalls == 256);
	ResetModel();
	s_RxChunk = 2;
	assert(!PumpAudio() && s_AudioError != AUDIO_ERROR_NONE && s_OutputPos == 0);
	ResetModel();
	s_RxChunk = 256;
	s_TxChunk = 2;
	assert(!PumpAudio() && s_AudioError != AUDIO_ERROR_NONE);
	ResetModel();
	s_TxChunk = 128;
	assert(!PrepareStream());
	ResetModel();
	AudioEvent(audio, DEVINTRF_EVT_TX_READY, nullptr, 0);
	assert(s_AudioError == AUDIO_ERROR_NONE);
	AudioEvent(audio, DEVINTRF_EVT_RX_FIFO_FULL, nullptr, 0);
	assert(s_AudioError != AUDIO_ERROR_NONE);
	ResetModel();
	AudioEvent(audio, DEVINTRF_EVT_STATECHG, nullptr, 0);
	assert(s_AudioError != AUDIO_ERROR_NONE);
	assert(strcmp(AudioErrorText(0), "SAI/DMA error") == 0);
	AudioEvent(audio, DEVINTRF_EVT_RX_FIFO_FULL, nullptr, 0);
	assert(strcmp(AudioErrorText(0), "SAI/DMA error") == 0);
	ResetModel();
	s_LastSample = 100;
	assert(AudioErrorText(600) == nullptr);
	assert(strcmp(AudioErrorText(601), "RX timeout") == 0);
	ResetModel();
	s_LastSample = UINT32_MAX - 100;
	assert(AudioErrorText(399) == nullptr);
	assert(strcmp(AudioErrorText(400), "RX timeout") == 0);
	ResetModel();
	s_LastSample = 100;
	s_Pending = 256;
	s_Offset = 128;
	assert(strcmp(AudioErrorText(601), "TX stalled") == 0);
	ResetModel();
	AudioEvent(audio, DEVINTRF_EVT_RX_FIFO_FULL, nullptr, 0);
	assert(strcmp(AudioErrorText(0), "RX FIFO overflow") == 0);
	ResetModel();
	DrawLevel();
	assert(strncmp(s_DisplayText[0], "MIC -> HEADPHONES", 17) == 0);
	assert(s_DisplayRow == 1);
	// Run the real OLED driver and application scheduler for one minute
	// with continuous 15,625 Hz RX/TX and injected SPI transfer delays.
	ResetModel();
	s_DisplayTesting = true;
	DevIntrf_t *spi = s_Spi;

	spi->Type = DEVINTRF_TYPE_SPI;
	spi->StartTx = Start;
	spi->StopTx = Stop;
	spi->TxData = DisplayTransmit;
	assert(s_Lcd.Init(s_LcdCfg, &s_Spi));
	assert(s_Lcd.SetFont(&iFontSystem5x7));
	clock->Freq = 1000000;
	s_Continuous = true;
	s_StreamTx = 128;
	s_LastPaint = 0;
	s_LastSample = 0;
	s_DisplayRow = 4;
	s_DisplayCalls = 0;
	while (s_StreamUs < 60000000 && s_AudioError == AUDIO_ERROR_NONE)
	{
		AdvanceStream(100);
		bool progressed = PumpAudio();
		uint32_t now = s_Clock.mSecond();
		unsigned calls = s_DisplayCalls;

		assert(AudioErrorText(now) == nullptr);
		ServiceDisplay(progressed, now);
		if (progressed || s_Offset < s_Pending)
		{
			assert(s_DisplayCalls == calls);
		}
	}
	assert(s_AudioError == AUDIO_ERROR_NONE);
	printf("Duplex OLED stress: 60 s, %u TX underrun frames, max RX %u frames\n", s_StreamUnderruns, s_StreamMaxRx);
	assert(s_StreamUnderruns == 0 && s_StreamMaxRx < 256);
	assert(s_Frames > 900000 && s_DisplayCalls > 1000);
	s_Continuous = false;
	puts("I2S loopback config, exact forwarding, short TX, stalls, startup and errors PASS");
}
