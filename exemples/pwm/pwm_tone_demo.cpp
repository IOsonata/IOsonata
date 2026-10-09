/**-------------------------------------------------------------------------
@example	pwm_tone_demo.cpp

@brief	Play the opening rhythm of Holst's Mars with PWM

@author	Hoang Nguyen Hoan
@date	May 15, 2018

@license

Copyright (c) 2018, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#include "coredev/pwm.h"
#include "idelay.h"
#include "board.h"

static const PwmCfg_t s_PwmCfg = {
	.DevNo = 0,
	.Freq = 4978,
	.Mode = PWM_MODE_EDGE,
	.bIntEn = false,
	.IntPrio = 6,
	.pEvtHandler = nullptr
};

static const PwmChanCfg_t s_ToneChannel = {
	.Chan = 0,
	.Pol = PWM_POL_HIGH,
	.Port = TONE_PORT,
	.Pin = TONE_PIN
};

// Holst, The Planets, Mars: opening 5/4 ostinato.
// Three triplet eighths, two quarters, two eighths, one quarter.
// Six ticks per quarter at 120 BPM: 30 ticks per bar.
// The original repeated G is transposed to Eb8 (4978 Hz) for this buzzer.
static const uint32_t s_ToneFreq = 4978;
static const uint8_t s_MarsTicks[] = {2, 2, 2, 6, 6, 3, 3, 6};
static const uint32_t s_QuarterUs = 500000;
static const uint32_t s_GapUs = 20000;

Pwm g_Pwm;

int main()
{
	if (!g_Pwm.Init(s_PwmCfg))
	{
		return 1;
	}

	if (!g_Pwm.OpenChannel(&s_ToneChannel, 1))
	{
		g_Pwm.Disable();
		return 1;
	}

	while (1)
	{
		for (unsigned int bar = 0; bar < 8; bar++)
		{
			uint32_t ticks = 0;
			uint32_t elapsed = 0;
			for (unsigned int i = 0; i < sizeof(s_MarsTicks); i++)
			{
				ticks += s_MarsTicks[i];
				uint32_t end = ticks * s_QuarterUs / 6;
				uint32_t duration = end - elapsed;
				elapsed = end;

				if (!g_Pwm.Frequency(s_ToneFreq) ||
					!g_Pwm.DutyCycle(s_ToneChannel.Chan, 50) ||
					!g_Pwm.Start())
				{
					g_Pwm.Stop();
					g_Pwm.CloseChannel(s_ToneChannel.Chan);
					g_Pwm.Disable();
					return 1;
				}

				usDelay(duration - s_GapUs);
				// Load an inactive sample before stopping the output.
				g_Pwm.DutyCycle(s_ToneChannel.Chan, 0);
				usDelay(1000);
				g_Pwm.Stop();
				usDelay(s_GapUs - 1000);
			}
		}
		usDelay(1000000);
	}
}
