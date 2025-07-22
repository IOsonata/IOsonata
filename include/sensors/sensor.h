/**--------------------------------------------------------------------------
@file	sensor.h

@brief	Generic sensor abstraction.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2017


@license

MIT

Copyright (c) 2017, I-SYST inc., all rights reserved

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
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
	#include <atomic>
	using namespace std;
#else
#include <stdbool.h>
#include <stdatomic.h>
#endif

#include "coredev/iopincfg.h"
#include "device.h"
#include "coredev/timer.h"

/** @addtogroup Sensors
  * @{
  */

/// @brief	Sensor type
///
/// This enum defines different sensor types.  It is sometime convenient to know
/// which type of sensor the object is
#if 0
typedef enum __Sensor_Type {
	SENSOR_TYPE_TEMP,				//!< Temperature
	SENSOR_TYPE_HUMI,				//!< Humidity
	SENSOR_TYPE_MOIST,				//!< Moisture
	SENSOR_TYPE_PRESSURE,			//!< Pressure
	SENSOR_TYPE_ACCEL,				//!< Accelerometer
	SENSOR_TYPE_GYRO,				//!< Gyroscope
	SENSOR_TYPE_MAG,				//!< Magnetometer
	SENSOR_TYPE_IR,					//!< Infrared
	SENSOR_TYPE_LIGHT,				//!< Luminosity
	SENSOR_TYPE_SOUND,				//!< Sound sensor suck as ultrasound. Microphone can also be consider as sensor
	SENSOR_TYPE_FORCE,				//!< Force tension/pressure kind of
	SENSOR_TYPE_VIBRATION,			//!< Vibration
	SENSOR_TYPE_LIQUID,				//!< Liquid level or other type of liquid measurement
	SENSOR_TYPE_SPECTRAL,			//!< Spectroscopy
	SENSOR_TYPE_RADAR,				//!< Radar type
} SENSOR_TYPE;
#else
///
/// Bit field definition for sensor type.  Orable for combo sensor
///
#define SENSOR_TYPE_TEMP				(1<<0)	//!< Temperature
#define SENSOR_TYPE_HUMI				(1<<1)	//!< Humidity
#define SENSOR_TYPE_MOIST				(1<<2)	//!< Moisture
#define SENSOR_TYPE_PRESSURE			(1<<3)	//!< Pressure
#define SENSOR_TYPE_ACCEL				(1<<4)	//!< Accelerometer
#define SENSOR_TYPE_GYRO				(1<<5)	//!< Gyroscope
#define SENSOR_TYPE_MAG					(1<<6)	//!< Magnetometer
#define SENSOR_TYPE_IR					(1<<7)	//!< Infrared
#define SENSOR_TYPE_LIGHT				(1<<8)	//!< Luminosity
#define SENSOR_TYPE_SOUND				(1<<9)	//!< Sound sensor suck as ultrasound. Microphone can also be consider as sensor
#define SENSOR_TYPE_FORCE				(1<<10)	//!< Force tension/pressure kind of
#define SENSOR_TYPE_VIBRATION			(1<<11)	//!< Vibration
#define SENSOR_TYPE_LIQUID				(1<<12)	//!< Liquid level or other type of liquid measurement
#define SENSOR_TYPE_SPECTRAL			(1<<13)	//!< Spectroscopy
#define SENSOR_TYPE_RADAR				(1<<14)	//!< Radar type

typedef uint32_t		SENSOR_TYPE;
#endif

/// @brief	Sensor operating mode.
///
/// Single mode sensor capture once.\n
/// Continuous mode sensor capture at a constant rate set by sampling frequency.
/// Not all sensor devices have CONTINUOUS mode. If not avail, it can be implemented
/// using timer with SINGLE mode.
///
/// @note	Timer configuration and operation is handled by user firmware application
typedef enum __Sensor_OpMode {
	SENSOR_OPMODE_SINGLE,			//!< Single capture
	SENSOR_OPMODE_CONTINUOUS,		//!< Hardware continuous capture high power
	SENSOR_OPMODE_TIMER,			//!< Using periodic timer
	SENSOR_OPMODE_LOW_POWER,		//!< Hardware low power
} SENSOR_OPMODE;

/// @brief	Sensor state.
///
/// To indicate current state of the sensor.
///
typedef enum __Sensor_State {
	SENSOR_STATE_SLEEP,				//!< Sleep state low power
	SENSOR_STATE_IDLE,				//!< Idle state powered on
	SENSOR_STATE_SAMPLING			//!< Sampling in progress. In continuous operating mode
									//!< the sensor would always be in sampling state.
} SENSOR_STATE;

#ifdef __cplusplus

/// @brief	Sensor generic base class.
///
/// Require implementations :
///	- bool StartSampling();
///
/// Require implementations from Device base class
///	- bool Enable();
///	- void Disable();
///	- void Reset();
///
class Sensor : virtual public Device {
public:
	/**
	 * @brief	Start sampling data
	 *
	 * This is a require implementation by sensor implementer.\n
	 * This function initiates sensor to do actual measurement.
	 *
	 * @return	true - success
	 * 			false - in case of error or sensor busy measuring
	 */
	virtual bool StartSampling(void) = 0;

	/**
	 * @brief	Read sensor and update internal data with new readings
	 *
	 * This function should be called by a periodic timer to update
	 * sensor data in SENSOR_OPMODE_CONTINUOUS or interrupt or when Read is called
	 * in SENSOR_OPMODE_SINGLE
	 *
	 * @return	true - New data is updated
	 */
	virtual bool UpdateData(void) = 0;

	/**
	 * @brief	Interrupt handler (optional)
	 *
	 * Sensor that supports interrupt can implement this to handle interrupt.
	 * Use generic DEVEVTCB callback and DEV_EVT to send event to user application
	 */
	virtual void IntHandler(void) {}

	/**
	 * @brief	Set operating mode.
	 *
	 * Sensor implementation must overload this function to do necessary
	 * hardware settings for the operating mode.
	 * This base implementation only stores the values into member variables
	 *
	 * @param	OpMode : Operating mode
	 * 					- SENSOR_OPMODE_SINGLE
	 * 					- SENSOR_OPMODE_CONTINUOUS
	 * 					- SENSOR_OPMODE_TIMER
	 * @param	Freq : Sampling frequency in mHz (miliHertz) for continuous mode
	 *
	 * @return	true- if success
	 */
	virtual bool Mode(SENSOR_OPMODE OpMode, uint32_t Freq) {
		vOpMode = OpMode;
		vSampFreq = Freq;
        vSampPeriod = vSampFreq > 0 ? 1000000000000LL / vSampFreq : 0;

		if (vpTimer && OpMode == SENSOR_OPMODE_TIMER)
		{
		    vTimerTrigId = vpTimer->EnableTimerTrigger(vSampPeriod, TIMER_TRIG_TYPE_CONTINUOUS,
		                                               TimerTrigHandler, (void*)this);
		}
		return true;
	}

	/**
	 * @brief	Get current operating mode.
	 *
	 * @return	Operating mode.
	 * 				- SENSOR_OPMODE_SINGLE
	 * 				- SENSOR_OPMODE_CONTINUOUS
	 */
	virtual SENSOR_OPMODE Mode(void) { return vOpMode; }
	operator SENSOR_OPMODE () { return vOpMode; }

	/**
	 * @brief	Get sampling period.
	 *
	 * @return	Sampling period in nsec
	 */
	virtual uint64_t SamplingPeriod(void) { return vSampPeriod; }

	/**
	 * @brief	Get sampling frequency.
	 * 		The sampling frequency is relevant only in continuous mode
	 *
	 * @return	Frequency in mHz (milliHertz)
	 */
	virtual uint32_t SamplingFrequency(void) { return vSampFreq; }

	/**
	 * @brief	Set sampling frequency.
	 *
	 * The sampling frequency is relevant only in continuous mode.
	 *
	 * @return	Frequency in mHz (milliHertz)
	 */
	virtual uint32_t SamplingFrequency(uint32_t Freq) {
		vSampFreq = Freq;
		vSampPeriod = vSampFreq > 0 ? 1000000000000LL / vSampFreq : 0;

		return vSampFreq;
	}

	/**
	 * @brief	Set current sensor state
	 *
	 * @param 	State : New state to be set.
	 *				- SENSOR_STATE_SLEEP
	 *				- SENSOR_STATE_IDLE
	 *				- SENSOR_STATE_SAMPLING
	 *
	 * @return	Actual state. In the case where the new state could
	 * 			not be set, it returns the actual state of the sensor.
	 */
	virtual SENSOR_STATE State(SENSOR_STATE State) {
		vState = State;
		return vState;
	}

	/**
	 * @brief	Get current sensor state.
	 *
	 * @return	Current state.
	 *				- SENSOR_STATE_SLEEP
	 *				- SENSOR_STATE_IDLE
	 *				- SENSOR_STATE_SAMPLING
	 */
	virtual SENSOR_STATE State(void) { return vState; }
	operator SENSOR_STATE () { return vState; }

	static void TimerTrigHandler(TimerDev_t * const pTimer, int TrigNo, void * const pContext) {
	    Sensor *sensor = (Sensor*)pContext;

	    sensor->UpdateData();
	    sensor->StartSampling();
	}

	/**
	 * @brief	Wake on sensor detection event
	 *
	 * This function allows implementing event to wake up MCU up on sensor detection pattern.
	 * It can be implemented by sensor that can support this feature. For example motion sensor
	 * to wake on motion detection, water leak sensor to wake on leak detection.
	 *
	 * @param	bEnable	: true - Enable, false - Disable
	 * @param	Threshold : Threshold value for the detection event
	 *
	 * @return	true - Success
	 * 			false - Feature not supported
	 */
	virtual bool WakeOnEvent(bool bEnable, int Threshold) { return false; }

	/**
	 * @brief	Get type of this object.
	 */
	SENSOR_TYPE Type(void) { return vType; }
	operator SENSOR_TYPE () { return vType; }

	/**
	 * @brief	Set this object type.
	 */
	SENSOR_TYPE Type(SENSOR_TYPE SensorType) { vType = SensorType; return vType; }

	/**
	 * @brief	Get the current filter cutoff frequency
	 *
	 * @return	Frequency in mHz
	 */
	virtual uint32_t FilterFreq(void) { return vFilterrFreq; }

	/**
	 * @brief	Set and enable filter cutoff frequency
	 *
	 * Optional implementation can override this to implement filtering supported by the device
	 *
	 * @param	Freq : Filter frequency in mHz
	 *
	 * @return	Actual frequency in mHz
	 */
	virtual uint32_t FilterFreq(uint32_t Freq) { vFilterrFreq = Freq; return vFilterrFreq; }

	/**
	 * @brief	Get max measurement range of the device
	 *
	 * This function gets the maximum positive value of the raw data that can be read from
	 * the sensor.
	 *
	 * @return	Maximum positive range value of the raw data
	 */
	virtual uint32_t Range(void) { return vRange; }

	/**
	 * @brief	Set max measurement range of the device
	 *
	 * This function sets the maximum positive value of the raw data that can be read from
	 * the sensor. Sensor device can implement this function to allow configuration variable
	 * range value.
	 *
	 * @param	Max positive range value
	 *
	 * @return	Actual maximum positive range value of the raw data
	 */
	virtual uint32_t Range(uint32_t Value) { vRange = Value; return vRange; }

	bool isDataReady(void) { return atomic_flag_test(&vbDataRdy); }//.test(memory_order_consume); }
	bool DataReadySet(void) { return atomic_flag_test_and_set(&vbDataRdy); }//.test_and_set(memory_order_acquire); }
	void DataReadyClear(void) { atomic_flag_clear(&vbDataRdy); }

protected:

	SENSOR_TYPE vType;			//!< Sensor type
	SENSOR_STATE vState;		//!< Current sensor state
	SENSOR_OPMODE vOpMode;		//!< Current operating mode
	uint32_t vSampFreq;			//!< Sampling frequency in milliHerz, relevant to CONTINUOUS mode
	uint64_t vSampPeriod;		//!< Sampling period in nanosecond.
	bool vbSampling;			//!< true - measurement in progress
	uint64_t vSampleCnt;		//!< Keeping sample count
	uint64_t vSampleTime;		//!< Time stamp when sampling is started
	uint32_t vDropCnt;			//!< Count the number of sample that was dropped
	uint32_t vFilterrFreq;		//!< Filter frequency in mHz, many sensors can set a filter cutoff frequency
	int vTimerTrigId;			//!< Timer interrupt trigger id (implementation dependent
	uint32_t vRange;            //!< ADC range of the sensor, contains max value for conversion factor
	atomic_flag vbDataRdy;		//!< Flag to indicate raw sensor data is ready for retrieval.
								//!< This flag is normally set by interrupt and cleared by UpdateData
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}
#endif	// __cplusplus

/** @} End of group Sensors */

#endif	// __SENSOR_H__
