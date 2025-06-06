/**-------------------------------------------------------------------------
@file	imu_vqf.h

@brief	Implementation of software imu class using vqf fusion

Implementation of Daniel Laidig Vqf fusion algo
https://github.com/dlaidig/vqf

@author	Hoang Nguyen Hoan
@date	May. 28, 2025

@license

MIT License

Copyright (c) 2025 I-SYST inc. All rights reserved.

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
#ifndef __IMU_VQF_H__
#define __IMU_VQF_H__

#include "imu/imu.h"

/** @addtogroup IMU
  * @{
  */

#pragma pack(push, 4)

typedef struct __Vqf_params {

    /**
     * @brief Time constant \f$\tau_\mathrm{acc}\f$ for accelerometer low-pass filtering in seconds.
     *
     * Small values for \f$\tau_\mathrm{acc}\f$ imply trust on the accelerometer measurements and while large values of
     * \f$\tau_\mathrm{acc}\f$ imply trust on the gyroscope measurements.
     *
     * The time constant \f$\tau_\mathrm{acc}\f$ corresponds to the cutoff frequency \f$f_\mathrm{c}\f$ of the
     * second-order Butterworth low-pass filter as follows: \f$f_\mathrm{c} = \frac{\sqrt{2}}{2\pi\tau_\mathrm{acc}}\f$.
     *
     * Default value: 3.0 s
     */
    float tauAcc;
    /**
     * @brief Time constant \f$\tau_\mathrm{mag}\f$ for magnetometer update in seconds.
     *
     * Small values for \f$\tau_\mathrm{mag}\f$ imply trust on the magnetometer measurements and while large values of
     * \f$\tau_\mathrm{mag}\f$ imply trust on the gyroscope measurements.
     *
     * The time constant \f$\tau_\mathrm{mag}\f$ corresponds to the cutoff frequency \f$f_\mathrm{c}\f$ of the
     * first-order low-pass filter for the heading correction as follows:
     * \f$f_\mathrm{c} = \frac{1}{2\pi\tau_\mathrm{mag}}\f$.
     *
     * Default value: 9.0 s
     */
    float tauMag;

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    /**
     * @brief Enables gyroscope bias estimation during motion phases.
     *
     * If set to true (default), gyroscope bias is estimated based on the inclination correction only, i.e. without
     * using magnetometer measurements.
     */
    bool motionBiasEstEnabled;
#endif
    /**
     * @brief Enables rest detection and gyroscope bias estimation during rest phases.
     *
     * If set to true (default), phases in which the IMU is at rest are detected. During rest, the gyroscope bias
     * is estimated from the low-pass filtered gyroscope readings.
     */
    bool restBiasEstEnabled;
    /**
     * @brief Enables magnetic disturbance detection and magnetic disturbance rejection.
     *
     * If set to true (default), the magnetic field is analyzed. For short disturbed phases, the magnetometer-based
     * correction is disabled totally. If the magnetic field is always regarded as disturbed or if the duration of
     * the disturbances exceeds #magMaxRejectionTime, magnetometer-based updates are performed, but with an increased
     * time constant.
     */
    bool magDistRejectionEnabled;

    /**
     * @brief Standard deviation of the initial bias estimation uncertainty (in degrees per second).
     *
     * Default value: 0.5 °/s
     */
    float biasSigmaInit;
    /**
     * @brief Time in which the bias estimation uncertainty increases from 0 °/s to 0.1 °/s (in seconds).
     *
     * This value determines the system noise assumed by the Kalman filter.
     *
     * Default value: 100.0 s
     */
    float biasForgettingTime;
    /**
     * @brief Maximum expected gyroscope bias (in degrees per second).
     *
     * This value is used to clip the bias estimate and the measurement error in the bias estimation update step. It is
     * further used by the rest detection algorithm in order to not regard measurements with a large but constant
     * angular rate as rest.
     *
     * Default value: 2.0 °/s
     */
    float biasClip;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    /**
     * @brief Standard deviation of the converged bias estimation uncertainty during motion (in degrees per second).
     *
     * This value determines the trust on motion bias estimation updates. A small value leads to fast convergence.
     *
     * Default value: 0.1 °/s
     */
    float biasSigmaMotion;
    /**
     * @brief Forgetting factor for unobservable bias in vertical direction during motion.
     *
     * As magnetometer measurements are deliberately not used during motion bias estimation, gyroscope bias is not
     * observable in vertical direction. This value is the relative weight of an artificial zero measurement that
     * ensures that the bias estimate in the unobservable direction will eventually decay to zero.
     *
     * Default value: 0.0001
     */
    float biasVerticalForgettingFactor;
#endif
    /**
     * @brief Standard deviation of the converged bias estimation uncertainty during rest (in degrees per second).
     *
     * This value determines the trust on rest bias estimation updates. A small value leads to fast convergence.
     *
     * Default value: 0.03 °
     */
    float biasSigmaRest;

    /**
     * @brief Time threshold for rest detection (in seconds).
     *
     * Rest is detected when the measurements have been close to the low-pass filtered reference for the given time.
     *
     * Default value: 1.5 s
     */
    float restMinT;
    /**
     * @brief Time constant for the low-pass filter used in rest detection (in seconds).
     *
     * This time constant characterizes a second-order Butterworth low-pass filter used to obtain the reference for
     * rest detection.
     *
     * Default value: 0.5 s
     */
    float restFilterTau;
    /**
     * @brief Angular velocity threshold for rest detection (in °/s).
     *
     * For rest to be detected, the norm of the deviation between measurement and reference must be below the given
     * threshold. (Furthermore, the absolute value of each component must be below #biasClip).
     *
     * Default value: 2.0 °/s
     */
    float restThGyr;
    /**
     * @brief Acceleration threshold for rest detection (in m/s²).
     *
     * For rest to be detected, the norm of the deviation between measurement and reference must be below the given
     * threshold.
     *
     * Default value: 0.5 m/s²
     */
    float restThAcc;

    /**
     * @brief Time constant for current norm/dip value in magnetic disturbance detection (in seconds).
     *
     * This (very fast) low-pass filter is intended to provide additional robustness when the magnetometer measurements
     * are noisy or not sampled perfectly in sync with the gyroscope measurements. Set to -1 to disable the low-pass
     * filter and directly use the magnetometer measurements.
     *
     * Default value: 0.05 s
     */
    float magCurrentTau;
    /**
     * @brief Time constant for the adjustment of the magnetic field reference (in seconds).
     *
     * This adjustment allows the reference estimate to converge to the observed undisturbed field.
     *
     * Default value: 20.0 s
     */
    float magRefTau;
    /**
     * @brief Relative threshold for the magnetic field strength for magnetic disturbance detection.
     *
     * This value is relative to the reference norm.
     *
     * Default value: 0.1 (10%)
     */
    float magNormTh;
    /**
     * @brief Threshold for the magnetic field dip angle for magnetic disturbance detection (in degrees).
     *
     * Default vaule: 10 °
     */
    float magDipTh;
    /**
     * @brief Duration after which to accept a different homogeneous magnetic field (in seconds).
     *
     * A different magnetic field reference is accepted as the new field when the measurements are within the thresholds
     * #magNormTh and #magDipTh for the given time. Additionally, only phases with sufficient movement, specified by
     * #magNewMinGyr, count.
     *
     * Default value: 20.0
     */
    float magNewTime;
    /**
     * @brief Duration after which to accept a homogeneous magnetic field for the first time (in seconds).
     *
     * This value is used instead of #magNewTime when there is no current estimate in order to allow for the initial
     * magnetic field reference to be obtained faster.
     *
     * Default value: 5.0
     */
    float magNewFirstTime;
    /**
     * @brief Minimum angular velocity needed in order to count time for new magnetic field acceptance (in °/s).
     *
     * Durations for which the angular velocity norm is below this threshold do not count towards reaching #magNewTime.
     *
     * Default value: 20.0 °/s
     */
    float magNewMinGyr;
    /**
     * @brief Minimum duration within thresholds after which to regard the field as undisturbed again (in seconds).
     *
     * Default value: 0.5 s
     */
    float magMinUndisturbedTime;
    /**
     * @brief Maximum duration of full magnetic disturbance rejection (in seconds).
     *
     * For magnetic disturbances up to this duration, heading correction is fully disabled and heading changes are
     * tracked by gyroscope only. After this duration (or for many small disturbed phases without sufficient time in the
     * undisturbed field in between), the heading correction is performed with an increased time constant (see
     * #magRejectionFactor).
     *
     * Default value: 60.0 s
     */
    float magMaxRejectionTime;
    /**
     * @brief Factor by which to slow the heading correction during long disturbed phases.
     *
     * After #magMaxRejectionTime of full magnetic disturbance rejection, heading correction is performed with an
     * increased time constant. This parameter (approximately) specifies the factor of the increase.
     *
     * Furthermore, after spending #magMaxRejectionTime/#magRejectionFactor seconds in an undisturbed magnetic field,
     * the time is reset and full magnetic disturbance rejection will be performed for up to #magMaxRejectionTime again.
     *
     * Default value: 2.0
     */
    float magRejectionFactor;
} VqfParam_t;

/**
 * @brief Struct containing the filter state of the VQF class.
 *
 * The relevant parts of the state can be accessed via functions of the VQF class, e.g. VQF::getQuat6D(),
 * VQF::getQuat9D(), VQF::getGyrBiasEstimate(), VQF::setGyrBiasEstimate(), VQF::getRestDetected() and
 * VQF::getMagDistDetected(). To reset the state to the initial values, use VQF::resetState().
 *
 * Direct access to the full state is typically not needed but can be useful in some cases, e.g. for debugging. For this
 * purpose, the state can be accessed by VQF::getState() and set by VQF::setState().
 */
typedef struct __VQF_State {
    /**
     * @brief Angular velocity strapdown integration quaternion \f$^{\mathcal{S}_i}_{\mathcal{I}_i}\mathbf{q}\f$.
     */
    float gyrQuat[4];
    /**
     * @brief Inclination correction quaternion \f$^{\mathcal{I}_i}_{\mathcal{E}_i}\mathbf{q}\f$.
     */
    float accQuat[4];
    /**
     * @brief Heading difference \f$\delta\f$ between \f$\mathcal{E}_i\f$ and \f$\mathcal{E}\f$.
     *
     * \f$^{\mathcal{E}_i}_{\mathcal{E}}\mathbf{q} = \begin{bmatrix}\cos\frac{\delta}{2} & 0 & 0 &
     * \sin\frac{\delta}{2}\end{bmatrix}^T\f$.
     */
    float delta;
    /**
     * @brief True if it has been detected that the IMU is currently at rest.
     *
     * Used to switch between rest and motion gyroscope bias estimation.
     */
    bool restDetected;
    /**
     * @brief True if magnetic disturbances have been detected.
     */
    bool magDistDetected;

    /**
     * @brief Last low-pass filtered acceleration in the \f$\mathcal{I}_i\f$ frame.
     */
    float lastAccLp[3];
    /**
     * @brief Internal low-pass filter state for #lastAccLp.
     */
    double accLpState[3*2];
    /**
     * @brief Last inclination correction angular rate.
     *
     * Change to inclination correction quaternion \f$^{\mathcal{I}_i}_{\mathcal{E}_i}\mathbf{q}\f$ performed in the
     * last accelerometer update, expressed as an angular rate (in rad/s).
     */
    float lastAccCorrAngularRate;

    /**
     * @brief Gain used for heading correction to ensure fast initial convergence.
     *
     * This value is used as the gain for heading correction in the beginning if it is larger than the normal filter
     * gain. It is initialized to 1 and then updated to 0.5, 0.33, 0.25, ... After VQFParams::tauMag seconds, it is
     * set to zero.
     */
    float kMagInit;
    /**
     * @brief Last heading disagreement angle.
     *
     * Disagreement between the heading \f$\hat\delta\f$ estimated from the last magnetometer sample and the state
     * \f$\delta\f$ (in rad).
     */
    float lastMagDisAngle;
    /**
     * @brief Last heading correction angular rate.
     *
     * Change to heading \f$\delta\f$ performed in the last magnetometer update,
     * expressed as an angular rate (in rad/s).
     */
    float lastMagCorrAngularRate;

    /**
     * @brief Current gyroscope bias estimate (in rad/s).
     */
    float bias[3];
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    /**
     * @brief Covariance matrix of the gyroscope bias estimate.
     *
     * The 3x3 matrix is stored in row-major order. Note that for numeric reasons the internal unit used is 0.01 °/s,
     * i.e. to get the standard deviation in degrees per second use \f$\sigma = \frac{\sqrt{p_{ii}}}{100}\f$.
     */
    float biasP[9];
#else
    // If only rest gyr bias estimation is enabled, P and K of the KF are always diagonal
    // and matrix inversion is not needed. If motion bias estimation is disabled at compile
    // time, storing the full P matrix is not necessary.
    float biasP;
#endif

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    /**
     * @brief Internal state of the Butterworth low-pass filter for the rotation matrix coefficients used in motion
     * bias estimation.
     */
    double motionBiasEstRLpState[9*2];
    /**
     * @brief Internal low-pass filter state for the rotated bias estimate used in motion bias estimation.
     */
    double motionBiasEstBiasLpState[2*2];
#endif
    /**
     * @brief Last (squared) deviations from the reference of the last sample used in rest detection.
     *
     * Looking at those values can be useful to understand how rest detection is working and which thresholds are
     * suitable. The array contains the last values for gyroscope and accelerometer in the respective
     * units. Note that the values are squared.
     *
     * The method VQF::getRelativeRestDeviations() provides an easier way to obtain and interpret those values.
     */
    float restLastSquaredDeviations[2];
    /**
     * @brief The current duration for which all sensor readings are within the rest detection thresholds.
     *
     * Rest is detected if this value is larger or equal to VQFParams::restMinT.
     */
    float restT;
    /**
     * @brief Last low-pass filtered gyroscope measurement used as the reference for rest detection.
     *
     * Note that this value is also used for gyroscope bias estimation when rest is detected.
     */
    float restLastGyrLp[3];
    /**
     * @brief Internal low-pass filter state for #restLastGyrLp.
     */
    double restGyrLpState[3*2];
    /**
     * @brief Last low-pass filtered accelerometer measurement used as the reference for rest detection.
     */
    float restLastAccLp[3];
    /**
     * @brief Internal low-pass filter state for #restLastAccLp.
     */
    double restAccLpState[3*2];

    /**
     * @brief Norm of the currently accepted magnetic field reference.
     *
     * A value of -1 indicates that no homogeneous field is found yet.
     */
    float magRefNorm;
    /**
     * @brief Dip angle of the currently accepted magnetic field reference.
     */
    float magRefDip;
    /**
     * @brief The current duration for which the current norm and dip are close to the reference.
     *
     * The magnetic field is regarded as undisturbed when this value reaches VQFParams::magMinUndisturbedTime.
     */
    float magUndisturbedT;
    /**
     * @brief The current duration for which the magnetic field was rejected.
     *
     * If the magnetic field is disturbed and this value is smaller than VQFParams::magMaxRejectionTime, heading
     * correction updates are fully disabled.
     */
    float magRejectT;
    /**
     * @brief Norm of the alternative magnetic field reference currently being evaluated.
     */
    float magCandidateNorm;
    /**
     * @brief Dip angle of the alternative magnetic field reference currently being evaluated.
     */
    float magCandidateDip;
    /**
     * @brief The current duration for which the norm and dip are close to the candidate.
     *
     * If this value exceeds VQFParams::magNewTime (or VQFParams::magNewFirstTime if #magRefNorm < 0), the current
     * candidate is accepted as the new reference.
     */
    float magCandidateT;
    /**
     * @brief Norm and dip angle of the current magnetometer measurements.
     *
     * Slightly low-pass filtered, see VQFParams::magCurrentTau.
     */
    float magNormDip[2];
    /**
     * @brief Internal low-pass filter state for the current norm and dip angle.
     */
    double magNormDipLpState[2*2];
} VqfState_t;

/**
 * @brief Struct containing coefficients used by the VQF class.
 *
 * Coefficients are values that depend on the parameters and the sampling times, but do not change during update steps.
 * They are calculated in VQF::setup().
 */
typedef struct __VQF_Coefficients
{
    /**
     * @brief Sampling time of the gyroscope measurements (in seconds).
     */
    float gyrTs;
    /**
     * @brief Sampling time of the accelerometer measurements (in seconds).
     */
    float accTs;
    /**
     * @brief Sampling time of the magnetometer measurements (in seconds).
     */
    float magTs;

    /**
     * @brief Numerator coefficients of the acceleration low-pass filter.
     *
     * The array contains \f$\begin{bmatrix}b_0 & b_1 & b_2\end{bmatrix}\f$.
     */
    double accLpB[3];
    /**
     * @brief Denominator coefficients of the acceleration low-pass filter.
     *
     * The array contains \f$\begin{bmatrix}a_1 & a_2\end{bmatrix}\f$ and \f$a_0=1\f$.
     */
    double accLpA[2];

    /**
     * @brief Gain of the first-order filter used for heading correction.
     */
    float kMag;

    /**
     * @brief Variance of the initial gyroscope bias estimate.
     */
    float biasP0;
    /**
     * @brief System noise variance used in gyroscope bias estimation.
     */
    float biasV;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    /**
     * @brief Measurement noise variance for the motion gyroscope bias estimation update.
     */
    float biasMotionW;
    /**
     * @brief Measurement noise variance for the motion gyroscope bias estimation update in vertical direction.
     */
    float biasVerticalW;
#endif
    /**
     * @brief Measurement noise variance for the rest gyroscope bias estimation update.
     */
    float biasRestW;

    /**
     * @brief Numerator coefficients of the gyroscope measurement low-pass filter for rest detection.
     */
    double restGyrLpB[3];
    /**
     * @brief Denominator coefficients of the gyroscope measurement low-pass filter for rest detection.
     */
    double restGyrLpA[2];
    /**
     * @brief Numerator coefficients of the accelerometer measurement low-pass filter for rest detection.
     */
    double restAccLpB[3];
    /**
     * @brief Denominator coefficients of the accelerometer measurement low-pass filter for rest detection.
     */
    double restAccLpA[2];

    /**
     * @brief Gain of the first-order filter used for to update the magnetic field reference and candidate.
     */
    float kMagRef;
    /**
     * @brief Numerator coefficients of the low-pass filter for the current magnetic norm and dip.
     */
    double magNormDipLpB[3];
    /**
     * @brief Denominator coefficients of the low-pass filter for the current magnetic norm and dip.
     */
    double magNormDipLpA[2];
} VqfCoeff_t;

#pragma pack(pop)

#ifdef __cplusplus

class ImuVqf : public Imu {
public:
	ImuVqf();
	virtual ~ImuVqf() {}
	bool Init(const ImuCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag);
	void SetParam(VqfParam_t &Param) { memcpy(&vParams, &Param, sizeof(VqfParam_t)); }
	virtual bool Enable();
	virtual void Disable();
	virtual void Reset();
	virtual bool UpdateData();
	virtual void IntHandler();
	uint32_t Rate(uint32_t DataRate);
	bool Calibrate();
	void SetAxisAlignmentMatrix(int8_t * const pMatrix);
	virtual bool Compass(bool bEn);
	virtual bool Pedometer(bool bEn);
	virtual bool Euler(bool bEn) { return false; }
	virtual bool Quaternion(bool bEn, int NbAxis);
	virtual bool Tap(bool bEn);
	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(AccelSensorRawData_t &Data) { return vpAccel->Read(Data); }
	virtual bool Read(AccelSensorData_t &Data) { return vpAccel->Read(Data); }

	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(GyroSensorRawData_t &Data) { return vpGyro->Read(Data); }
	virtual bool Read(GyroSensorData_t &Data) { return vpGyro->Read(Data); }

	/**
	 * @brief	Read last updated sensor data
	 *
	 * This function read the currently stored data last updated by UdateData().
	 * Device implementation can add validation if needed and return true or false
	 * in the case of data valid or not.  This default implementation only returns
	 * the stored data with success.
	 *
	 * @param 	Data : Reference to data storage for the returned data
	 *
	 * @return	True - Success.
	 */
	virtual bool Read(MagSensorRawData_t &Data) { return vpMag->Read(Data); }
	virtual bool Read(MagSensorData_t &Data) { return vpMag->Read(Data); }
    virtual bool Read(ImuQuat_t &Data);// { Data = vQuat; return true; }
    virtual bool Read(ImuEuler_t &Data) { Data = vEuler; return true; }

protected:

private:
    void ProcessAccel(void);
    void ProcessGyro(void);
    void ProcessMag(void);

    /**
     * @brief Performs filter step for vector-valued signal with averaging-based initialization.
     *
     * During the first \f$\tau\f$ seconds, the filter output is the mean of the previous samples. At \f$t=\tau\f$, the
     * initial conditions for the low-pass filter are calculated based on the current mean value and from then on,
     * regular filtering with the rational transfer function described by the coefficients b and a is performed.
     *
     * @param Vec input values (array of size N)
     * @param VectSize number of values in vector-valued signal
     * @param tau filter time constant \f$\tau\f$ in seconds (used for initialization)
     * @param Ts sampling time \f$T_\mathrm{s}\f$ in seconds (used for initialization)
     * @param b numerator coefficients
     * @param a denominator coefficients (without \f$a_0=1\f$)
     * @param state filter state (array of size N*2, will be modified)
     * @param out output array for filtered values (size N)
     */
    void FilterVec(const float Vec[], size_t VectSize, float tau, float Ts, const double b[3],
                          const double a[2], double state[], float out[]);

    VqfParam_t vParams;
    VqfState_t vState;
    VqfCoeff_t vCoeffs;
    uint64_t vPrevTimeStamp;
};

extern "C" {
#endif

// C prototypes

#ifdef __cplusplus
}
#endif

/** @} end group IMU */

#endif // __IMU_VQF_H__

