/**-------------------------------------------------------------------------
@file	inertial_nav.h

@brief	Generic inertial navigation abstraction layer

Common interface for inertial navigation filters that estimate full pose
(position, velocity, attitude) plus inertial sensor biases. This is the
navigation tier, separate from the attitude only Ahrs tier: an Ahrs backend
outputs orientation from accel/gyro/mag, while an InertialNav backend adds
position and velocity and fuses aiding sources such as GNSS, barometer and
optical flow.

State is held in a local North-East-Down (NED) tangent frame whose origin is
set at alignment, either from the first GNSS fix or from a supplied origin.
Position is metres from the origin, velocity is m/s in NED, attitude is a unit
quaternion mapping body to NED, [w x y z], matching the Ahrs convention.

Aiding is dependency inverted. The filter runs its own strapdown from the
high-rate accel and gyro through UpdateData(). Aiding measurements arrive
through Inject* methods, so the filter does not own or poll the GNSS, baro or
flow drivers; the application feeds measurements as they arrive. Each Inject*
returns true when the measurement was fused, false when rejected or not ready.

@author	Hoang Nguyen Hoan
@date	Jun. 23, 2026

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
#ifndef __NAV_H__
#define __NAV_H__

#include <stdint.h>

#include "device.h"
#include "sensors/accel_sensor.h"
#include "sensors/gyro_sensor.h"
#include "sensors/mag_sensor.h"

/** @addtogroup AHRS
  * @{
  */

/// Navigation status flags, ORable into NavState_t.Flags.
#define NAV_FLAG_ATTITUDE_ALIGNED	(1<<0)	//!< Initial attitude set
#define NAV_FLAG_ORIGIN_SET			(1<<1)	//!< NED origin established
#define NAV_FLAG_GNSS_FUSED			(1<<2)	//!< At least one GNSS update fused
#define NAV_FLAG_DEAD_RECKONING		(1<<3)	//!< No aiding, inertial only
#define NAV_FLAG_DIVERGED			(1<<4)	//!< Covariance health check failed

/// Geodetic origin of the local NED frame.
typedef struct __Nav_Origin {
	double Lat;		//!< Latitude, deg
	double Lon;		//!< Longitude, deg
	float Alt;		//!< Altitude, m
	bool bValid;		//!< Origin has been set
} NavOrigin_t;

/// Full navigation state output.
typedef struct __Nav_State {
	uint64_t Timestamp;	//!< Time stamp count in usec
	float Pos[3];		//!< NED position from origin, m
	float Vel[3];		//!< NED velocity, m/s
	float Q[4];		//!< Attitude quaternion, body to NED, [w x y z]
	float AccelBias[3];	//!< Accel bias estimate, m/s^2
	float GyroBias[3];	//!< Gyro bias estimate, rad/s
	uint32_t Flags;		//!< NAV_FLAG_* status bits
} NavState_t;

/// GNSS aiding measurement. Provide geodetic position; velocity is optional.
typedef struct __Nav_GnssMeas {
	uint64_t Timestamp;	//!< Time stamp count in usec
	double Lat;		//!< Latitude, deg
	double Lon;		//!< Longitude, deg
	float Alt;		//!< Altitude, m
	float Vel[3];		//!< NED velocity, m/s (valid when bVelValid)
	float PosAccH;		//!< Horizontal position std-dev, m
	float PosAccV;		//!< Vertical position std-dev, m
	float VelAcc;		//!< Velocity std-dev, m/s
	bool bVelValid;		//!< Velocity field is usable
} NavGnssMeas_t;

/// Barometric height aiding measurement.
typedef struct __Nav_BaroMeas {
	uint64_t Timestamp;	//!< Time stamp count in usec
	float Alt;		//!< Height, m. Bias against GNSS altitude is estimated
	float Acc;		//!< Height std-dev, m
} NavBaroMeas_t;

/// Optical flow aiding measurement, for GNSS denied low altitude flight.
typedef struct __Nav_FlowMeas {
	uint64_t Timestamp;	//!< Time stamp count in usec
	float Flow[2];		//!< Optical flow rate about body x, y, rad/s
	float Gyro[2];		//!< Gyro rate about body x, y at the same time, rad/s
	float Range;		//!< Distance to ground along body z, m
	float Quality;		//!< Sensor quality metric, 0..1
	float Acc;		//!< Flow std-dev, rad/s
} NavFlowMeas_t;

typedef struct __Nav_Config {
	DevEvtHandler_t EvtHandler;
	NavOrigin_t Origin;	//!< If bValid is false, origin is set from first GNSS
} NavCfg_t;

#ifdef __cplusplus

class InertialNav : virtual public Device {
public:

	/**
	 * @brief	Bind the inertial sensors that drive the strapdown.
	 *
	 * The accel and gyro are mandatory. The mag is optional and, when present,
	 * a backend may use it as a heading aiding source.
	 */
	virtual bool Init(const NavCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);

	/**
	 * @brief	One strapdown propagation step from the bound accel and gyro.
	 *
	 * Called at the inertial sample rate. Reads the latest accel and gyro,
	 * integrates the nominal state and propagates the error covariance.
	 *
	 * @return	True when a step was taken.
	 */
	virtual bool UpdateData() = 0;
	virtual void IntHandler() = 0;

	/**
	 * @brief	Fuse a GNSS position (and optional velocity) measurement.
	 *
	 * Sets the NED origin from this fix when no origin is yet valid.
	 *
	 * @return	True when fused, false when rejected or not ready.
	 */
	virtual bool InjectGnss(const NavGnssMeas_t &Meas) = 0;

	/**
	 * @brief	Fuse a barometric height measurement.
	 * @return	True when fused.
	 */
	virtual bool InjectBaro(const NavBaroMeas_t &Meas) = 0;

	/**
	 * @brief	Fuse an optical flow measurement.
	 * @return	True when fused.
	 */
	virtual bool InjectFlow(const NavFlowMeas_t &Meas) = 0;

	/**
	 * @brief	Apply a zero velocity update, for known stationary periods.
	 * @return	True when applied.
	 */
	virtual bool ZeroVelocityUpdate() = 0;

	/// Read the latest navigation state.
	virtual bool Read(NavState_t &Data) { Data = vState; return true; }

	/// Set or query the local NED origin.
	virtual bool SetOrigin(const NavOrigin_t &Origin) { vOrigin = Origin; return true; }
	virtual bool GetOrigin(NavOrigin_t &Origin) { Origin = vOrigin; return vOrigin.bValid; }

protected:
	AccelSensor *vpAccel;	//!< Pointer to accelerometer sensor
	GyroSensor *vpGyro;	//!< Pointer to gyro sensor
	MagSensor *vpMag;	//!< Pointer to magnetometer sensor, may be null
	NavState_t vState;	//!< Last updated navigation state
	NavOrigin_t vOrigin;	//!< Local NED frame origin
};

#endif // __cplusplus

/** @} */

#endif // __NAV_H__
