/**-------------------------------------------------------------------------
@file	audiodev.h

@brief	Generic audio device definitions


@author	Nguyen Hoan Hoang
@date	May. 18, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#ifndef __AUDIODEV_H__
#define __AUDIODEV_H__

#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "device.h"
#include "coredev/iopincfg.h"

typedef enum __Audio_Sample_Width {
	AUDIO_SWIDTH_8BITS = 8,
	AUDIO_SSWIDTH_16BITS = 16,
	AUDIO_SWIDTH_24BITS = 24,
	AUDIO_SWIDTH_32BITS = 32,
} AUDIO_SWIDTH;

typedef enum __Audio_Chan {
	AUDIO_CHAN_STEREO,	//!< Left & right channels
	AUDIO_CHAN_LEFT,	//!< Left channel
	AUDIO_CHAN_RIGHT,	//!< Right channel
	AUDIO_CHAN_CENTER,
	AUDIO_CHAN_REAR_LEFT,
	AUDIO_CHAN_REAR_RIGHT,
	AUDIO_CHAN_ALL
} AUDIO_CHAN;

#pragma pack(push, 4)

typedef struct __Audio_Data {
	AUDIO_CHAN Chan;	//!< Channel id
	uint8_t Data[4];
} AudioData_t;

/// Configuration data used to initialize device
typedef struct __Audio_Device_Config {
	AUDIO_CHAN Chan;			//! Channel stereo/left/right
	AUDIO_SWIDTH SampleWidth;	//!< Sample width 8/16/24/32 bits
	uint32_t Freq;				//! Sampling frequency in Hz
} AudioDevCfg_t;

/// Device driver data require by low level functions
typedef struct __Audio_Device {
	AUDIO_CHAN Chan;			//! Channel stereo/left/right
	AUDIO_SWIDTH SampleWidth;	//!<
	uint32_t Freq;				//! sampling frequency in mHz
	DEVINTRF DevIntrf;			//!< Device interface instance
} AudioDev_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}

class AudioDevice : virtual public Device {
public:
	bool Init(AudioDevCfg_t &CfgData, DeviceIntrf * const pIntrf);

	/**
	 * @brief	Start sampling data
	 *
	 * This is a require implementation by sensor implementer.\n
	 * This function initiates sensor to do actual measurement.
	 *
	 * @return	true - success
	 * 			false - in case of error or sensor busy measuring
	 */
	virtual bool StartSampling() = 0;

	/**
	 * @brief	Read sensor and update internal data with new readings
	 *
	 * This function should be called by a periodic timer to update
	 * sensor data in SENSOR_OPMODE_CONTINUOUS or interrupt or when Read is called
	 * in SENSOR_OPMODE_SINGLE
	 *
	 * @return	true - New data is updated
	 */
	virtual bool UpdateData() = 0;

	/**
	 * @brief	Interrupt handler (optional)
	 *
	 * Sensor that supports interrupt can implement this to handle interrupt.
	 * Use generic DEVEVTCB callback and DEV_EVT to send event to user application
	 */
	virtual void IntHandler() {}

	/**
	 * @brief	Get sampling frequency.
	 * 		The sampling frequency is relevant only in continuous mode
	 *
	 * @return	Frequency in Hz
	 */
	virtual uint32_t SamplingFrequency() { return vSampFreq; }

	/**
	 * @brief	Set sampling frequency.
	 *
	 * The sampling frequency is relevant only in continuous mode.
	 *
	 * @return	Frequency in Hz
	 */
	virtual uint32_t SamplingFrequency(uint32_t Freq) {
		vSampFreq = Freq;
		//vSampPeriod = vSampFreq > 0 ? 1000000000LL / vSampFreq : 0;

		return vSampFreq;
	}


protected:
	uint32_t vSampFreq;			//!< Sampling frequency in Hz
	uint8_t vSampWidth;			//!< Sampling width int bits
	AUDIO_CHAN vAudioMode;
	AUDIO_SWIDTH vSWidth;		//!< Sample width in bits

private:
};

#endif // __cplusplus

#endif // __AUDIODEV_H__
