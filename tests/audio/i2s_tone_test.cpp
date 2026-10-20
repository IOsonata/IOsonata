/**-------------------------------------------------------------------------
@file	i2s_tone_test.cpp

@brief	Host test of the I2S tone example: generated samples and configuration.

Run by tests/audio/run_i2s_tone.py.

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

#define main ToneDemoMain
#include "../../exemples/audio/i2s_tone_demo.cpp"
#undef main

static const unsigned s_Chunks[] = { 1, 17, 64 };

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

extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
	assert(false);
}

extern "C" void IOPinSet(int, int)
{
	assert(false);
}

extern "C" void IOPinClear(int, int)
{
	assert(false);
}

static void CheckWaveform(uint32_t Rate, unsigned Chunk)
{
	int16_t samples[128 + 2];
	unsigned crossings[4] = {};
	int previous[4] = {};
	unsigned peak[4] = {};
	uint32_t total = 0;

	s_Rate = Rate;
	s_Frame = 0;
	s_Phase = 0;
	s_Step = 0;
	while (total < Rate * 4)
	{
		unsigned count = Chunk;

		if (count > Rate * 4 - total)
		{
			count = Rate * 4 - total;
		}
		samples[0] = 123;
		samples[count * 2 + 1] = 456;
		FillTone(samples + 1, count);
		assert(samples[0] == 123 && samples[count * 2 + 1] == 456);
		for (unsigned i = 0; i < count; i++)
		{
			unsigned step = (total + i) / Rate;
			int left = samples[1 + i * 2];
			int right = samples[1 + i * 2 + 1];
			int value = step == 1 ? right : left;
			unsigned amplitude = value < 0 ? -value : value;

			assert(left >= -4096 && left <= 4096 && right >= -4096 && right <= 4096);
			assert(step != 0 || right == 0);
			assert(step != 1 || left == 0);
			assert(step != 2 || left == right);
			assert(step != 3 || (left == 0 && right == 0));
			if (value > 0 && previous[step] <= 0)
			{
				crossings[step]++;
			}
			previous[step] = value;
			if (amplitude > peak[step])
			{
				peak[step] = amplitude;
			}
			if ((total + i) % Rate == 0)
			{
				assert(value == 0);
			}
		}
		total += count;
	}
	for (unsigned step = 0; step < 3; step++)
	{
		assert(crossings[step] >= s_Frequencies[step] - 1);
		assert(crossings[step] <= s_Frequencies[step] + 1);
		assert(peak[step] == 4096);
	}
	assert(crossings[3] == 0 && peak[3] == 0);
	assert(s_Step == 0 && s_Frame == 0 && s_Phase == 0);
}

int main()
{
	// No hardware Init in this waveform test. Supply the global I2C object's
	// destructor hook, which its normal Init would install.
	DevIntrf_t *control = s_Control;

	control->Disable = ControlDisable;
	atomic_store(&control->EnCnt, 1);
	assert(s_AudioCfg.Chan == I2SCHAN_STEREO && s_AudioCfg.bDmaEn && s_AudioCfg.bIntEn);
	assert(s_AudioCfg.pRxFifoMem == nullptr && s_AudioCfg.pTxFifoMem == s_AudioMem);
	assert(s_AudioPins[I2S_SDI_IOPIN_IDX].PortNo == -1);
	assert(s_AudioPins[I2S_SDO_IOPIN_IDX].PortNo == IOPORTD && s_AudioPins[I2S_SDO_IOPIN_IDX].PinNo == 5);
	for (unsigned chunk : s_Chunks)
	{
		CheckWaveform(15625, chunk);
		CheckWaveform(16000, chunk);
	}
	puts("I2S tone config, stereo routing, pitch, fade, silence and partial blocks PASS");
}
