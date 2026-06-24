/**-------------------------------------------------------------------------
@file	act.h

@brief	Activity classification capability interface

Activity capability for motion devices: pedometer, tap and similar activity
events. This is a separate capability from attitude (att.h). A device that
provides both, for example a DMP based imu, inherits both Att and Act. A pure
software attitude backend inherits Att only and carries no activity methods.

@author	Hoang Nguyen Hoan
@date	Jun. 25, 2026

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
#ifndef __ACT_H__
#define __ACT_H__

#include "device.h"

/** @addtogroup Motion
  * @{
  */

/// Pedometer
typedef struct __Act_Pedometer {
	uint32_t Timestamp;		//!< Time stamp count in msec
	uint16_t StepCount;		//!< Number of step taken
	uint8_t Cadence;		//!< in steps per minute
	float Direction;		//!< Direction of the movement (yaw angle in degrees)
	uint16_t UpCount;		//!< Number of upstairs taken
	uint16_t DownCount;		//!< Number of downstairs taken
	uint8_t StrideLength;	//!< in cm
	uint16_t TotalDistance;	//!< in dm
} ActPedometer_t;

#ifdef __cplusplus

class Act : virtual public Device {
public:

	/**
	 * @brief	Enable or disable pedometer
	 *
	 * @param	bEn : true to enable, false to disable
	 *
	 * @return	true if the device applied the request, false otherwise
	 */
	virtual bool Pedometer(bool bEn) = 0;

	/**
	 * @brief	Enable or disable tap detection
	 *
	 * @param	bEn : true to enable, false to disable
	 *
	 * @return	true if the device applied the request, false otherwise
	 */
	virtual bool Tap(bool bEn) = 0;
};

#endif // __cplusplus

/** @} */

#endif // __ACT_H__
