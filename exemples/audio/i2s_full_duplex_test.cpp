/**-------------------------------------------------------------------------
@file	i2s_full_duplex_test.cpp

@brief	Generic IOsonata I2S full-duplex audio test

@author	Nguyen Hoan Hoang
@date	Oct. 24, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

#include <stdint.h>
#include <stdio.h>
#include "idelay.h"
#include "coredev/i2s.h"

static const int16_t s_Sine[32] = {
	0, 799, 1567, 2276, 2896, 3406, 3784, 4017,
	4096, 4017, 3784, 3406, 2896, 2276, 1567, 799,
	0, -799, -1567, -2276, -2896, -3406, -3784, -4017,
	-4096, -4017, -3784, -3406, -2896, -2276, -1567, -799
};

// Integer RMS without libm: input is the mean-square magnitude of 128 samples.
static uint32_t AudioSqrt(uint64_t value)
{
	uint32_t root = 0;
	for (uint32_t bit = 1U << 15; bit != 0; bit >>= 1)
	{
		uint32_t candidate = root | bit;
		if ((uint64_t)candidate * candidate <= value)
		{
			root = candidate;
		}
	}
	return root;
}

int I2SFullDuplexTest(I2S &audio, bool (*mute)(bool), uint32_t (*voltage)(), bool restartTest)
{
	alignas(4) uint32_t tx[128];
	alignas(4) uint32_t rx[128];
	unsigned phase = 0;
	unsigned blocks = 0;
	unsigned reports = 0;
	unsigned restarts = 0;
	int count;
	while (1)
	{
		for (unsigned i = 0; i < 128; i++)
		{
			// nRF I2S 16-bit stereo DMA packs left/right into one
			// 32-bit word. Advance the 32-sample tone once per frame.
			uint32_t sample = (uint16_t)(s_Sine[phase++ & 31] * 4);
			tx[i] = sample | (sample << 16);
		}
		count = audio.Tx(0, (const uint8_t *)tx, sizeof(tx));
		if (count != (int)sizeof(tx))
		{
			usDelay(100);
			continue;
		}
		if (++blocks == 750)
		{
			int received;
			blocks = 0;
			received = audio.Rx(0, (uint8_t *)rx, sizeof(rx));
			if (received == (int)sizeof(rx))
			{
				// Audio DK has one onboard PDM microphone. The codec can
				// expose its signal in both I2S slots; analyze one mono slot.
				int64_t sum = 0;
				uint64_t squares = 0;
				uint32_t peak = 0;
				for (unsigned i = 0; i < 128; i++)
				{
					int32_t sample = (int16_t)(uint16_t)rx[i];
					uint32_t magnitude = sample < 0 ? (uint32_t)-sample : (uint32_t)sample;
					if (magnitude > peak)
					{
						peak = magnitude;
					}
					sum += sample;
					squares += (uint64_t)(int64_t)sample * sample;
				}
				// Remove DC offset from the measured signal power.
				uint64_t power = (squares - (uint64_t)(sum * sum / 128)) / 128;
				printf("I2S mono mic peak=%lu RMS(AC)=%lu VBAT=%lu uV\n",
					(unsigned long)peak, (unsigned long)AudioSqrt(power),
					(unsigned long)voltage());
			}
			else
			{
				puts("I2S microphone capture timeout");
			}
			// Restart the peripheral in place every ten reports. Do not
			// reinitialize the codec or MCU: this exercises I2S STOPPED,
			// IRQ shutdown, FIFO reset and restart handling.
			if (restartTest && ++reports == 10)
			{
				reports = 0;
				printf("I2S restart %u: muting and stopping for 100 ms\n", ++restarts);
				if (!mute(true))
				{
					puts("Codec headphone mute failed");
				}
				msDelay(10);
				audio.Disable();
				msDelay(100);
				audio.Enable();
				// Allow the DMA to queue and transmit fresh audio blocks
				// before enabling the headphone output again.
				for (unsigned i = 0; i < 8; i++)
				{
					for (unsigned j = 0; j < 128; j++)
					{
						uint32_t sample = (uint16_t)(s_Sine[phase++ & 31] * 4);
						tx[j] = sample | (sample << 16);
					}
					unsigned retry = 10000;
					int written = 0;
					while (retry > 0)
					{
						written = audio.Tx(0, (const uint8_t *)tx, sizeof(tx));
						if (written == (int)sizeof(tx))
						{
							break;
						}
						retry--;
						usDelay(100);
					}
					if (written != (int)sizeof(tx))
					{
						puts("I2S restart TX timeout; headphone remains muted");
						audio.Disable();
						return 1;
					}
				}
				msDelay(10);
				if (!mute(false))
				{
					puts("Codec headphone unmute failed");
				}
				printf("I2S restart %u: streaming resumed\n", restarts);
			}
		}
	}
}
