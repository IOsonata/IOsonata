/**-------------------------------------------------------------------------
@example	pwm_tone_demo.cpp

@brief	Play the Jingle Bells chorus with PWM

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
	.Freq = 5274,
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

// Jingle Bells, James Lord Pierpont (1857), chorus in C major.
// C8-G8 keeps the melody near the buzzer's useful frequency range.
static const uint32_t C8 = 4186;
static const uint32_t D8 = 4699;
static const uint32_t E8 = 5274;
static const uint32_t F8 = 5588;
static const uint32_t G8 = 6272;

typedef struct {
	uint32_t Freq;
	uint8_t Eighths;
} ToneNote_t;

static const ToneNote_t s_Melody[] = {
	{E8, 2}, {E8, 2}, {E8, 4},
	{E8, 2}, {E8, 2}, {E8, 4},
	{E8, 2}, {G8, 2}, {C8, 3}, {D8, 1},
	{E8, 8},
	{F8, 2}, {F8, 2}, {F8, 3}, {F8, 1},
	{F8, 2}, {E8, 2}, {E8, 2}, {E8, 1}, {E8, 1},
	{E8, 2}, {D8, 2}, {D8, 2}, {E8, 2},
	{D8, 4}, {G8, 4},

	{E8, 2}, {E8, 2}, {E8, 4},
	{E8, 2}, {E8, 2}, {E8, 4},
	{E8, 2}, {G8, 2}, {C8, 3}, {D8, 1},
	{E8, 8},
	{F8, 2}, {F8, 2}, {F8, 3}, {F8, 1},
	{F8, 2}, {E8, 2}, {E8, 2}, {E8, 1}, {E8, 1},
	{G8, 2}, {G8, 2}, {F8, 2}, {D8, 2},
	{C8, 8}
};

static const uint32_t s_EighthUs = 250000; // 120 quarter notes per minute.
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
		for (unsigned int i = 0; i < sizeof(s_Melody) / sizeof(s_Melody[0]); i++)
		{
			uint32_t duration = s_Melody[i].Eighths * s_EighthUs;
			if (!g_Pwm.Frequency(s_Melody[i].Freq) ||
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
		usDelay(1000000);
	}
}
