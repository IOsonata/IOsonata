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
#include "coredev/timer.h"
#include "miscdev/buzzer.h"
#include "iopinctrl.h"
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
using Note = BuzzerPitch;
using Length = BuzzerDuration;

static const BuzzerNote_t s_Melody[] = {
	{Note::Rest, Length::Half}, // One second of silence before each chorus.
	{Note::E8, Length::Quarter}, {Note::E8, Length::Quarter}, {Note::E8, Length::Half},
	{Note::E8, Length::Quarter}, {Note::E8, Length::Quarter}, {Note::E8, Length::Half},
	{Note::E8, Length::Quarter}, {Note::G8, Length::Quarter}, {Note::C8, Length::DottedQuarter}, {Note::D8, Length::Eighth},
	{Note::E8, Length::Whole},
	{Note::F8, Length::Quarter}, {Note::F8, Length::Quarter}, {Note::F8, Length::DottedQuarter}, {Note::F8, Length::Eighth},
	{Note::F8, Length::Quarter}, {Note::E8, Length::Quarter}, {Note::E8, Length::Quarter}, {Note::E8, Length::Eighth}, {Note::E8, Length::Eighth},
	{Note::E8, Length::Quarter}, {Note::D8, Length::Quarter}, {Note::D8, Length::Quarter}, {Note::E8, Length::Quarter},
	{Note::D8, Length::Half}, {Note::G8, Length::Half},

	{Note::E8, Length::Quarter}, {Note::E8, Length::Quarter}, {Note::E8, Length::Half},
	{Note::E8, Length::Quarter}, {Note::E8, Length::Quarter}, {Note::E8, Length::Half},
	{Note::E8, Length::Quarter}, {Note::G8, Length::Quarter}, {Note::C8, Length::DottedQuarter}, {Note::D8, Length::Eighth},
	{Note::E8, Length::Whole},
	{Note::F8, Length::Quarter}, {Note::F8, Length::Quarter}, {Note::F8, Length::DottedQuarter}, {Note::F8, Length::Eighth},
	{Note::F8, Length::Quarter}, {Note::E8, Length::Quarter}, {Note::E8, Length::Quarter}, {Note::E8, Length::Eighth}, {Note::E8, Length::Eighth},
	{Note::G8, Length::Quarter}, {Note::G8, Length::Quarter}, {Note::F8, Length::Quarter}, {Note::D8, Length::Quarter},
	{Note::C8, Length::Whole}
};

// The application chooses and owns the timer. On nRF52840, device 2 is RTC2.
#ifndef TONE_TIMER_DEVNO
#define TONE_TIMER_DEVNO 2
#endif

static void MelodyWake(TimerDev_t *, int, void *)
{
	// Only wake the main loop; PWM and melody work runs outside the ISR.
	__SEV();
}

static const TimerCfg_t s_TimerCfg = {
	.DevNo = TONE_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = 6,
	.EvtHandler = nullptr,
	.bTickInt = false
};

Pwm g_Pwm;
Timer g_Timer;
Buzzer g_Buzzer;
BuzzerMelody g_Melody;

int main()
{
	if (!g_Pwm.Init(s_PwmCfg) || !g_Pwm.OpenChannel(&s_ToneChannel, 1) ||
		!g_Buzzer.Init(&g_Pwm, s_ToneChannel.Chan)) return 1;
	g_Buzzer.Stop();
	if (!g_Timer.Init(s_TimerCfg)) return 1;
	if (!g_Timer.EnableTimerTrigger(0, (uint32_t)5,
		TIMER_TRIG_TYPE_CONTINUOUS, MelodyWake, nullptr)) return 1;
	if (!g_Melody.Init(&g_Buzzer, &g_Timer) ||
		!g_Melody.Play(s_Melody, sizeof(s_Melody) / sizeof(s_Melody[0]),
					  120, 0, 20)) return 1;

	while (g_Melody.IsPlaying())
	{
		g_Melody.Process();
		__WFE();
	}
	g_Timer.DisableTimerTrigger(0);
	g_Timer.Disable();
	return g_Melody.Failed() ? 1 : 0;
}
