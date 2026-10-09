/**-------------------------------------------------------------------------
@file	buzzer.cpp

@brief	Generic implementation of buzzer driver

@author	Hoang Nguyen Hoan
@date	May 22, 2018

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
#include "idelay.h"
#include "miscdev/buzzer.h"
#include "coredev/timer.h"

// Equal-tempered MIDI frequencies, rounded to millihertz (A4 = 440 Hz).
static const uint32_t s_MidiNoteFreq[128] = {
	8176, 8662, 9177, 9723, 10301, 10913, 11562, 12250,
	12978, 13750, 14568, 15434, 16352, 17324, 18354, 19445,
	20602, 21827, 23125, 24500, 25957, 27500, 29135, 30868,
	32703, 34648, 36708, 38891, 41203, 43654, 46249, 48999,
	51913, 55000, 58270, 61735, 65406, 69296, 73416, 77782,
	82407, 87307, 92499, 97999, 103826, 110000, 116541, 123471,
	130813, 138591, 146832, 155563, 164814, 174614, 184997, 195998,
	207652, 220000, 233082, 246942, 261626, 277183, 293665, 311127,
	329628, 349228, 369994, 391995, 415305, 440000, 466164, 493883,
	523251, 554365, 587330, 622254, 659255, 698456, 739989, 783991,
	830609, 880000, 932328, 987767, 1046502, 1108731, 1174659, 1244508,
	1318510, 1396913, 1479978, 1567982, 1661219, 1760000, 1864655, 1975533,
	2093005, 2217461, 2349318, 2489016, 2637020, 2793826, 2959955, 3135963,
	3322438, 3520000, 3729310, 3951066, 4186009, 4434922, 4698636, 4978032,
	5274041, 5587652, 5919911, 6271927, 6644875, 7040000, 7458620, 7902133,
	8372018, 8869844, 9397273, 9956063, 10548082, 11175303, 11839822, 12543854,
};

bool Buzzer::Init(Pwm * const pPwm, int Chan)
{
	if (pPwm == nullptr || Chan < 0) return false;
	Stop();
	vpPwm = pPwm;
	vChan = Chan;
	vDutyCycle = 50;
	return true;
}

void Buzzer::Volume(int Volume)
{
	if (Volume < 0) Volume = 0;
	if (Volume > 100) Volume = 100;
	vDutyCycle = Volume / 2;
	if (vpPwm)
	{
		if (vDutyCycle == 0) Stop();
		else vpPwm->DutyCycle(vChan, vDutyCycle);
	}
}

bool Buzzer::Start(uint32_t Freq)
{
	if (!vpPwm) return false;
	Stop();
	if (Freq == 0 || vDutyCycle == 0) return true;
	if (!vpPwm->Enable() || !vpPwm->Frequency(Freq) ||
		!vpPwm->DutyCycle(vChan, vDutyCycle) || !vpPwm->Start())
	{
		Stop();
		return false;
	}
	return true;
}

void Buzzer::Play(uint32_t Freq, uint32_t msDuration)
{
	if (!Start(Freq)) return;
	if (msDuration)
	{
		// Preserve the legacy blocking API without overflowing a microsecond count.
		while (msDuration--) usDelay(1000);
		Stop();
	}
}

void Buzzer::Play(uint8_t MidiNote, uint32_t msDuration)
{
	if (MidiNote >= 128)
	{
		Stop();
		return;
	}
	Play((s_MidiNoteFreq[MidiNote] + 500) / 1000, msDuration);
}

void Buzzer::Stop()
{
	if (vpPwm) vpPwm->Stop();
}

bool BuzzerMelody::Init(Buzzer *pBuzzer, Timer *pTimer)
{
	if (!pBuzzer || !pTimer) return false;
	Stop();
	vpBuzzer = pBuzzer;
	vpTimer = pTimer;
	return true;
}

bool BuzzerMelody::Play(const BuzzerNote_t *pNotes, unsigned int Count,
					   uint16_t Bpm, uint32_t Repeats, uint32_t GapMs)
{
	Stop();
	if (!vpBuzzer || !vpTimer || !pNotes || Count == 0 || Bpm == 0)
	{
		vbFailed = true;
		return false;
	}
	for (unsigned int i = 0; i < Count; i++)
	{
		uint64_t duration = (uint64_t)pNotes[i].Duration * 60000 / (24UL * Bpm);
		if ((unsigned)pNotes[i].Note > (unsigned)BuzzerPitch::Rest ||
			duration == 0 || duration > 0x7fffffffUL)
		{
			vbFailed = true;
			return false;
		}
	}
	vpNotes = pNotes;
	vCount = Count;
	vIndex = 0;
	vBpm = Bpm;
	vRepeats = Repeats;
	vGapMs = GapMs;
	vbPlaying = true;
	return BeginNote();
}

namespace {
using Pitch = BuzzerPitch;
static const BuzzerEffectStep_t s_Chirp[] = {
	{Pitch::C8, Pitch::G8, 160}
};
static const BuzzerEffectStep_t s_Laser[] = {
	{Pitch::G8, Pitch::C7, 300, 100, 20}
};
static const BuzzerEffectStep_t s_Siren[] = {
	{Pitch::C8, Pitch::G8, 600}, {Pitch::G8, Pitch::C8, 600}
};
static const BuzzerEffectStep_t s_Warble[] = {
	{Pitch::C8, Pitch::C8, 100}, {Pitch::G8, Pitch::G8, 100}
};
static const BuzzerEffectStep_t s_Pulse[] = {
	{Pitch::E8, Pitch::E8, 100}, {Pitch::Rest, Pitch::Rest, 100}
};
static const BuzzerEffectStep_t s_Fade[] = {
	{Pitch::E8, Pitch::E8, 800, 100, 0}
};
}

bool BuzzerMelody::Play(BuzzerEffect Effect, uint32_t DurationMs)
{
	const BuzzerEffectStep_t *steps = nullptr;
	unsigned int count = 0;
	switch (Effect)
	{
		case BuzzerEffect::Chirp: steps = s_Chirp; count = 1; break;
		case BuzzerEffect::Laser: steps = s_Laser; count = 1; break;
		case BuzzerEffect::Siren: steps = s_Siren; count = 2; break;
		case BuzzerEffect::Warble: steps = s_Warble; count = 2; break;
		case BuzzerEffect::Pulse: steps = s_Pulse; count = 2; break;
		case BuzzerEffect::Fade: steps = s_Fade; count = 1; break;
	}
	if (DurationMs > 0x7fffffffUL)
	{
		Stop();
		vbFailed = true;
		return false;
	}
	if (!PlayEffect(steps, count, DurationMs ? 0 : 1)) return false;
	vEffectLimit = DurationMs;
	vEffectStarted = vpTimer->mSecond();
	return true;
}

bool BuzzerMelody::PlayEffect(const BuzzerEffectStep_t *pSteps,
							unsigned int Count, uint32_t Repeats)
{
	Stop();
	if (!vpBuzzer || !vpTimer || !pSteps || !Count)
	{
		vbFailed = true;
		return false;
	}
	for (unsigned int i = 0; i < Count; ++i)
	{
		const BuzzerEffectStep_t &step = pSteps[i];
		if ((unsigned)step.From > (unsigned)Pitch::Rest ||
			(unsigned)step.To > (unsigned)Pitch::Rest ||
			((step.From == Pitch::Rest) != (step.To == Pitch::Rest)) ||
			!step.DurationMs || step.DurationMs > 0x7fffffffUL ||
			step.VolumeFrom > 100 || step.VolumeTo > 100)
		{
			vbFailed = true;
			return false;
		}
	}
	vpEffects = pSteps;
	vSavedVolume = vpBuzzer->GetVolume();
	vCount = Count;
	vIndex = 0;
	vRepeats = Repeats;
	vbPlaying = true;
	return BeginEffect();
}

bool BuzzerMelody::BeginEffect()
{
	vStarted = vpTimer->mSecond();
	vDuration = vpEffects[vIndex].DurationMs;
	vEffectVolume = -1;
	return UpdateEffect(0);
}

bool BuzzerMelody::UpdateEffect(uint32_t Elapsed)
{
	const BuzzerEffectStep_t &step = vpEffects[vIndex];
	uint32_t frequency = 0;
	if (step.From != Pitch::Rest)
	{
		int64_t from = s_MidiNoteFreq[(unsigned)step.From];
		int64_t to = s_MidiNoteFreq[(unsigned)step.To];
		frequency = (from + (to - from) * Elapsed / vDuration + 500) / 1000;
	}
	int volume = (int)step.VolumeFrom +
		((int64_t)step.VolumeTo - step.VolumeFrom) * Elapsed / vDuration;
	volume = volume * vSavedVolume / 100;
	// Avoid touching PWM when neither pitch nor volume has changed.
	if (frequency == vEffectFrequency && volume == vEffectVolume) return true;
	vEffectFrequency = frequency;
	vEffectVolume = volume;
	vpBuzzer->Stop();
	vpBuzzer->Volume(volume);
	if (!vpBuzzer->Start(frequency))
	{
		Stop();
		vbFailed = true;
		return false;
	}
	return true;
}

bool BuzzerMelody::BeginNote()
{
	const BuzzerNote_t &note = vpNotes[vIndex];
	vDuration = (uint64_t)note.Duration * 60000 / (24UL * vBpm);
	uint32_t gap = vGapMs < vDuration ? vGapMs : vDuration - 1;
	vSoundDuration = vDuration - gap;
	vStarted = vpTimer->mSecond();
	vbSounding = note.Note != BuzzerPitch::Rest;
	uint32_t frequency = vbSounding ?
		(s_MidiNoteFreq[(unsigned)note.Note] + 500) / 1000 : 0;
	if (!vpBuzzer->Start(frequency))
	{
		Stop();
		vbFailed = true;
		return false;
	}
	return true;
}

void BuzzerMelody::Process()
{
	if (!vbPlaying) return;
	uint32_t now = vpTimer->mSecond();
	uint32_t elapsed = now - vStarted;
	if (vpEffects)
	{
		if (vEffectLimit && (uint32_t)(now - vEffectStarted) >= vEffectLimit)
		{
			Stop();
			return;
		}
		if (elapsed < vDuration)
		{
			UpdateEffect(elapsed);
			return;
		}
	}
	if (!vpEffects && vbSounding && elapsed >= vSoundDuration)
	{
		vpBuzzer->Stop();
		vbSounding = false;
	}
	if (elapsed < vDuration) return;
	if (++vIndex == vCount)
	{
		if (vRepeats == 1)
		{
			Stop();
			return;
		}
		if (vRepeats > 1) --vRepeats;
		vIndex = 0;
	}
	// At most one note transition per call. A late caller extends playback;
	// it does not emit a burst of missed notes.
	if (vpEffects) BeginEffect();
	else BeginNote();
}

void BuzzerMelody::Stop()
{
	if (vpBuzzer)
	{
		vpBuzzer->Stop();
		if (vpEffects) vpBuzzer->Volume(vSavedVolume);
	}
	vpEffects = nullptr;
	vEffectLimit = 0;
	vbPlaying = false;
	vbSounding = false;
	vbFailed = false;
	vpNotes = nullptr;
}
