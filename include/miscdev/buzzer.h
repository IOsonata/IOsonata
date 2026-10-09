/**-------------------------------------------------------------------------
@file	buzzer.h

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

#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "coredev/pwm.h"

/** @addtogroup MiscDev
  * @{
  */

typedef struct __Buzzer_Device {
	PwmDev_t *pPwm;			//!< Pointer to external PWM interface
	int DutyCycle;			//!< PWM duty cycle value for volume
	int Chan;				//!< PWM Channel used for the buzzer
} BuzzerDev_t;

#ifdef __cplusplus

class Buzzer {
public:

	virtual ~Buzzer() {}

	/**
	 * @brief	Buzzer initialization
	 *
	 * @param 	pPwm	: Pointer PWM interface connected to buzzer
	 * @param 	Chan	: PWM channel used for the buzzer
	 *
	 * @return	true - success
	 */
	virtual bool Init(Pwm * const pPwm, int Chan);

	/**
	 * @brief	Set buzzer volume
	 *
	 * @param 	Volume	: Volume in % (0-100)
	 */
	virtual void Volume(int Volume);

	// Start a continuous tone without waiting. Zero frequency means silence.
	virtual bool Start(uint32_t Freq);

	/**
	 * @brief	Play frequency
	 *
	 * @param	Freq :
	 * @param	msDuration	: Play duration in msec.
	 *							if != 0, wait for it then stop
	 *							else let running and return (no stop)
	 */
	virtual void Play(uint32_t Freq, uint32_t msDuration);

	/**
	 * @brief	Play Midi note
	 *
	 * @param	MidiNote : MIDI note value 0-127
	 * @param	msDuration	: Play duration in msec.
	 *							if != 0, wait for it then stop
	 *							else let running and return (no stop)
	 */
	virtual void Play(uint8_t MidiNote, uint32_t msDuration);

	/**
	 * @brief	Stop buzzer
	 *
	 */
	virtual void Stop();

private:

	Pwm *vpPwm = nullptr;			//!< Pointer to external PWM interface
	int vDutyCycle = 50;		//!< PWM duty cycle value for volume
	int vChan = -1;			//!< PWM Channel used for the buzzer
};

class Timer;

// Scientific pitch notation: C4 is middle C; A4 is 440 Hz.
// Cs/Db, Ds/Eb, Fs/Gb, Gs/Ab and As/Bb are equivalent pitches.
enum class BuzzerPitch : uint8_t {
	C0 = 12, Cs0 = 13, Db0 = Cs0, D0 = 14, Ds0 = 15, Eb0 = Ds0, E0 = 16, F0 = 17, Fs0 = 18, Gb0 = Fs0, G0 = 19, Gs0 = 20, Ab0 = Gs0, A0 = 21, As0 = 22, Bb0 = As0, B0 = 23,
	C1 = 24, Cs1 = 25, Db1 = Cs1, D1 = 26, Ds1 = 27, Eb1 = Ds1, E1 = 28, F1 = 29, Fs1 = 30, Gb1 = Fs1, G1 = 31, Gs1 = 32, Ab1 = Gs1, A1 = 33, As1 = 34, Bb1 = As1, B1 = 35,
	C2 = 36, Cs2 = 37, Db2 = Cs2, D2 = 38, Ds2 = 39, Eb2 = Ds2, E2 = 40, F2 = 41, Fs2 = 42, Gb2 = Fs2, G2 = 43, Gs2 = 44, Ab2 = Gs2, A2 = 45, As2 = 46, Bb2 = As2, B2 = 47,
	C3 = 48, Cs3 = 49, Db3 = Cs3, D3 = 50, Ds3 = 51, Eb3 = Ds3, E3 = 52, F3 = 53, Fs3 = 54, Gb3 = Fs3, G3 = 55, Gs3 = 56, Ab3 = Gs3, A3 = 57, As3 = 58, Bb3 = As3, B3 = 59,
	C4 = 60, Cs4 = 61, Db4 = Cs4, D4 = 62, Ds4 = 63, Eb4 = Ds4, E4 = 64, F4 = 65, Fs4 = 66, Gb4 = Fs4, G4 = 67, Gs4 = 68, Ab4 = Gs4, A4 = 69, As4 = 70, Bb4 = As4, B4 = 71,
	C5 = 72, Cs5 = 73, Db5 = Cs5, D5 = 74, Ds5 = 75, Eb5 = Ds5, E5 = 76, F5 = 77, Fs5 = 78, Gb5 = Fs5, G5 = 79, Gs5 = 80, Ab5 = Gs5, A5 = 81, As5 = 82, Bb5 = As5, B5 = 83,
	C6 = 84, Cs6 = 85, Db6 = Cs6, D6 = 86, Ds6 = 87, Eb6 = Ds6, E6 = 88, F6 = 89, Fs6 = 90, Gb6 = Fs6, G6 = 91, Gs6 = 92, Ab6 = Gs6, A6 = 93, As6 = 94, Bb6 = As6, B6 = 95,
	C7 = 96, Cs7 = 97, Db7 = Cs7, D7 = 98, Ds7 = 99, Eb7 = Ds7, E7 = 100, F7 = 101, Fs7 = 102, Gb7 = Fs7, G7 = 103, Gs7 = 104, Ab7 = Gs7, A7 = 105, As7 = 106, Bb7 = As7, B7 = 107,
	C8 = 108, Cs8 = 109, Db8 = Cs8, D8 = 110, Ds8 = 111, Eb8 = Ds8, E8 = 112, F8 = 113, Fs8 = 114, Gb8 = Fs8, G8 = 115, Gs8 = 116, Ab8 = Gs8, A8 = 117, As8 = 118, Bb8 = As8, B8 = 119,
	C9 = 120, Cs9 = 121, Db9 = Cs9, D9 = 122, Ds9 = 123, Eb9 = Ds9, E9 = 124, F9 = 125, Fs9 = 126, Gb9 = Fs9, G9 = 127,
	Rest = 128
};

// Musical lengths; numeric values are internal timing subdivisions.
enum class BuzzerDuration : uint16_t {
	Whole = 96, Half = 48, Quarter = 24, Eighth = 12, Sixteenth = 6, ThirtySecond = 3,
	DottedWhole = 144, DottedHalf = 72, DottedQuarter = 36,
	DottedEighth = 18, DottedSixteenth = 9,
	HalfTriplet = 32, QuarterTriplet = 16, EighthTriplet = 8, SixteenthTriplet = 4
};

struct BuzzerNote_t {
	BuzzerPitch Note;
	BuzzerDuration Duration;
};

// All calls belong to one application context or RTOS thread, never an ISR.
// The caller owns the initialized timer, buzzer and immutable note table.
class BuzzerMelody {
public:
	bool Init(Buzzer *pBuzzer, Timer *pTimer);
	// Repeats: 1 plays once, 0 repeats until Stop(). BPM is quarter notes/minute.
	bool Play(const BuzzerNote_t *pNotes, unsigned int Count,
			uint16_t Bpm = 120, uint32_t Repeats = 1, uint32_t GapMs = 20);
	// Call from the main loop or worker thread. No note-duration waits or allocation.
	void Process();
	void Stop();
	bool IsPlaying() const { return vbPlaying; }
	bool Failed() const { return vbFailed; }
private:
	bool BeginNote();
	Buzzer *vpBuzzer = nullptr;
	Timer *vpTimer = nullptr;
	const BuzzerNote_t *vpNotes = nullptr;
	unsigned int vCount = 0;
	unsigned int vIndex = 0;
	uint16_t vBpm = 120;
	uint32_t vRepeats = 1;
	uint32_t vGapMs = 20;
	uint32_t vStarted = 0;
	uint32_t vDuration = 0;
	uint32_t vSoundDuration = 0;
	bool vbPlaying = false;
	bool vbSounding = false;
	bool vbFailed = false;
};

extern "C" {
#endif // __cplusplus

#ifdef __cplusplus
}
#endif	// __cplusplus

/** @} end group MiscDev */

#endif // __BUZZER_H__

