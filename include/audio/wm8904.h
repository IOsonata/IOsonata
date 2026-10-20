/**-------------------------------------------------------------------------
@file	wm8904.h

@brief	WM8904 digital microphone control


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

#ifndef __AUDIO_WM8904_H__
#define __AUDIO_WM8904_H__

#include "device.h"
#include "coredev/i2s.h"

// Digital microphone capture and optional stereo headphone playback. The codec is the I2S slave; its external MCLK
// must be 256 times the actual frame rate returned by the audio interface.
// SYSCLK uses MCLK directly and must be at least 3 MHz. Double DAC OSR
// is selected only with MCLK >= 6 MHz; lower clocks use normal DAC OSR.
// Init/Enable stage configuration with SYSCLK disabled and enable it last, so
// MCLK may start afterward. MCLK must run during live gain changes. Disable
// and PowerOff first gate SYSCLK and also work after MCLK has stopped.
// Headphone Init/Enable additionally require MCLK throughout DC servo startup.
typedef struct __Wm8904_Config
{
	uint8_t DevAddr;	   //!< Seven-bit control address, normally 0x1A.
	uint32_t Freq;		   //!< Actual audio frame rate in Hz.
	uint32_t MClkFreq;	   //!< External MCLK in Hz.
	I2SSWIDTH SampleWidth; //!< 16 or 24 bits.
	I2SFMT Format;
	unsigned MicInput; //!< 0 = off, DMICDAT1 = 1, DMICDAT2 = 2.
	int Gain;		   //!< ADC digital gain in milli-dB, rounded to 375 milli-dB.
	bool bHeadphoneEn; //!< Enable stereo headphone playback.
	int HeadphoneGain; //!< Headphone gain in milli-dB, -57000 to 6000, 1000 steps.
} Wm8904Cfg_t;

// Control uses the injected DeviceIntrf. PCM travels through the application's
// separately configured I2S interface, keeping MCU registers out of this driver.
class Wm8904 : public Device {
public:
	bool Init(const Wm8904Cfg_t &Cfg, DeviceIntrf *pIntrf);
	bool Enable();
	void Disable();
	void Reset();
	void PowerOff();
	int Gain(int MilliDb);
	int Gain() const;
	int HeadphoneGain(int MilliDb);
	int HeadphoneGain() const;
	// Soft-mute the DAC while capture, clocks and headphone stages stay on.
	// Requires enabled playback and running MCLK. Re-enable after Disable unmutes.
	bool Mute(bool bMute);
	// Select DMIC clock output or hold GPIO1 low, leaving capture enabled.
	// Requires enabled microphone capture and running MCLK. Init restores clock.
	bool MicrophoneClock(bool bEnable);
	// Disable DMIC capture and both ADCs, but drive GPIO1 with SYSCLK / 2.
	// Requires enabled microphone configuration and running MCLK. Headphones
	// retain their state. Disable/Enable or Init restores normal capture.
	bool MicrophoneClockOnly();

private:
	uint16_t DacControl(bool bMute) const;
	bool StartHeadphones();
	bool StopHeadphones();
	bool ReadReg(uint8_t Reg, uint16_t &Value);
	bool WriteReg(uint8_t Reg, uint16_t Value);
	bool VerifyReg(uint8_t Reg, uint16_t Value, uint16_t Mask);
	Wm8904Cfg_t vCfg{};
	bool vbEnabled = false;
};

#endif
