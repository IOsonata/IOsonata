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

// Duration in 1/24-quarter-note ticks; frequency zero is a rest.
struct BuzzerNote_t {
	uint32_t Freq;
	uint16_t Ticks;
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

