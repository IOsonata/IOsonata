/**-------------------------------------------------------------------------
@file	imu_icm20948.cpp

@brief	Implementation of an Inertial Measurement Unit for Invensense ICM-20948

@author	Hoang Nguyen Hoan
@date	Sept. 9, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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
#include "idelay.h"
#include "imu/imu_icm20948.h"
#include "sensors/agm_icm20948.h"

/* dmp3a.20x48-0.4.1 */

#define CFG_FIFO_SIZE                   (4222)

// data output control
#define DATA_OUT_CTL1			(4 * 16)
#define DATA_OUT_CTL2			(4 * 16 + 2)
#define DATA_INTR_CTL			(4 * 16 + 12)
#define FIFO_WATERMARK			(31 * 16 + 14)

// motion event control
#define MOTION_EVENT_CTL		(4 * 16 + 14)

// indicates to DMP which sensors are available
/*	1: gyro samples available
2: accel samples available
8: secondary samples available	*/
#define DATA_RDY_STATUS			(8 * 16 + 10)

// batch mode
#define BM_BATCH_CNTR			(27 * 16)
#define BM_BATCH_THLD			(19 * 16 + 12)
#define BM_BATCH_MASK			(21 * 16 + 14)

// sensor output data rate
#define ODR_ACCEL				(11 * 16 + 14)
#define ODR_GYRO				(11 * 16 + 10)
#define ODR_CPASS				(11 * 16 +  6)
#define ODR_ALS					(11 * 16 +  2)
#define ODR_QUAT6				(10 * 16 + 12)
#define ODR_QUAT9				(10 * 16 +  8)
#define ODR_PQUAT6				(10 * 16 +  4)
#define ODR_GEOMAG				(10 * 16 +  0)
#define ODR_PRESSURE			(11 * 16 + 12)
#define ODR_GYRO_CALIBR			(11 * 16 +  8)
#define ODR_CPASS_CALIBR		(11 * 16 +  4)

// sensor output data rate counter
#define ODR_CNTR_ACCEL			(9 * 16 + 14)
#define ODR_CNTR_GYRO			(9 * 16 + 10)
#define ODR_CNTR_CPASS			(9 * 16 +  6)
#define ODR_CNTR_ALS			(9 * 16 +  2)
#define ODR_CNTR_QUAT6			(8 * 16 + 12)
#define ODR_CNTR_QUAT9			(8 * 16 +  8)
#define ODR_CNTR_PQUAT6			(8 * 16 +  4)
#define ODR_CNTR_GEOMAG			(8 * 16 +  0)
#define ODR_CNTR_PRESSURE		(9 * 16 + 12)
#define ODR_CNTR_GYRO_CALIBR	(9 * 16 +  8)
#define ODR_CNTR_CPASS_CALIBR	(9 * 16 +  4)

// mounting matrix
#define CPASS_MTX_00            (23 * 16)
#define CPASS_MTX_01            (23 * 16 + 4)
#define CPASS_MTX_02            (23 * 16 + 8)
#define CPASS_MTX_10            (23 * 16 + 12)
#define CPASS_MTX_11            (24 * 16)
#define CPASS_MTX_12            (24 * 16 + 4)
#define CPASS_MTX_20            (24 * 16 + 8)
#define CPASS_MTX_21            (24 * 16 + 12)
#define CPASS_MTX_22            (25 * 16)

#define GYRO_SF					(19 * 16)
#define ACCEL_FB_GAIN			(34 * 16)
#define ACCEL_ONLY_GAIN			(16 * 16 + 12)

// bias calibration
#define GYRO_BIAS_X				(139 * 16 +  4)
#define GYRO_BIAS_Y				(139 * 16 +  8)
#define GYRO_BIAS_Z				(139 * 16 + 12)
#define GYRO_ACCURACY			(138 * 16 +  2)
#define GYRO_BIAS_SET			(138 * 16 +  6)
#define GYRO_LAST_TEMPR			(134 * 16)
#define GYRO_SLOPE_X			( 78 * 16 +  4)
#define GYRO_SLOPE_Y			( 78 * 16 +  8)
#define GYRO_SLOPE_Z			( 78 * 16 + 12)

#define ACCEL_BIAS_X            (110 * 16 +  4)
#define ACCEL_BIAS_Y            (110 * 16 +  8)
#define ACCEL_BIAS_Z            (110 * 16 + 12)
#define ACCEL_ACCURACY			(97 * 16)
#define ACCEL_CAL_RESET			(77 * 16)
#define ACCEL_VARIANCE_THRESH	(93 * 16)
#define ACCEL_CAL_RATE			(94 * 16 + 4)
#define ACCEL_PRE_SENSOR_DATA	(97 * 16 + 4)
#define ACCEL_COVARIANCE		(101 * 16 + 8)
#define ACCEL_ALPHA_VAR			(91 * 16)
#define ACCEL_A_VAR				(92 * 16)
#define ACCEL_CAL_INIT			(94 * 16 + 2)
#define ACCEL_CAL_SCALE_COVQ_IN_RANGE	(194 * 16)
#define ACCEL_CAL_SCALE_COVQ_OUT_RANGE	(195 * 16)
#define ACCEL_CAL_TEMPERATURE_SENSITIVITY	(194 * 16 + 4)
#define ACCEL_CAL_TEMPERATURE_OFFSET_TRIM	(194 * 16 + 12)

#define CPASS_BIAS_X            (126 * 16 +  4)
#define CPASS_BIAS_Y            (126 * 16 +  8)
#define CPASS_BIAS_Z            (126 * 16 + 12)
#define CPASS_ACCURACY			(37 * 16)
#define CPASS_BIAS_SET			(34 * 16 + 14)
#define MAR_MODE				(37 * 16 + 2)
#define CPASS_COVARIANCE		(115 * 16)
#define CPASS_COVARIANCE_CUR	(118 * 16 +  8)
#define CPASS_REF_MAG_3D		(122 * 16)
#define CPASS_CAL_INIT			(114 * 16)
#define CPASS_EST_FIRST_BIAS	(113 * 16)
#define MAG_DISTURB_STATE		(113 * 16 + 2)
#define CPASS_VAR_COUNT			(112 * 16 + 6)
#define CPASS_COUNT_7			( 87 * 16 + 2)
#define CPASS_MAX_INNO			(124 * 16)
#define CPASS_BIAS_OFFSET		(113 * 16 + 4)
#define CPASS_CUR_BIAS_OFFSET	(114 * 16 + 4)
#define CPASS_PRE_SENSOR_DATA	( 87 * 16 + 4)

// Compass Cal params to be adjusted according to sampling rate
#define CPASS_TIME_BUFFER		(112 * 16 + 14)
#define CPASS_RADIUS_3D_THRESH_ANOMALY	(112 * 16 + 8)

#define CPASS_STATUS_CHK		(25 * 16 + 12)

// 9-axis
#define MAGN_THR_9X				(80 * 16)
#define MAGN_LPF_THR_9X			(80 * 16 +  8)
#define QFB_THR_9X				(80 * 16 + 12)

// DMP running counter
#define DMPRATE_CNTR			(18 * 16 + 4)

// pedometer
#define PEDSTD_BP_B				(49 * 16 + 12)
#define PEDSTD_BP_A4			(52 * 16)
#define PEDSTD_BP_A3			(52 * 16 +  4)
#define PEDSTD_BP_A2			(52 * 16 +  8)
#define PEDSTD_BP_A1			(52 * 16 + 12)
#define PEDSTD_SB				(50 * 16 +  8)
#define PEDSTD_SB_TIME			(50 * 16 + 12)
#define PEDSTD_PEAKTHRSH		(57 * 16 +  8)
#define PEDSTD_TIML				(50 * 16 + 10)
#define PEDSTD_TIMH				(50 * 16 + 14)
#define PEDSTD_PEAK				(57 * 16 +  4)
#define PEDSTD_STEPCTR			(54 * 16)
#define PEDSTD_STEPCTR2			(58 * 16 +  8)
#define PEDSTD_TIMECTR			(60 * 16 +  4)
#define PEDSTD_DECI				(58 * 16)
#define PEDSTD_SB2				(60 * 16 + 14)
#define STPDET_TIMESTAMP		(18 * 16 +  8)
#define PEDSTEP_IND				(19 * 16 +  4)
#define PED_Y_RATIO				(17 * 16 +  0)

// SMD
#define SMD_VAR_TH              (141 * 16 + 12)
#define SMD_VAR_TH_DRIVE        (143 * 16 + 12)
#define SMD_DRIVE_TIMER_TH      (143 * 16 +  8)
#define SMD_TILT_ANGLE_TH       (179 * 16 + 12)
#define BAC_SMD_ST_TH           (179 * 16 +  8)
#define BAC_ST_ALPHA4           (180 * 16 + 12)
#define BAC_ST_ALPHA4A          (176 * 16 + 12)

// Wake on Motion
#define WOM_ENABLE              (64 * 16 + 14)
#define WOM_STATUS              (64 * 16 + 6)
#define WOM_THRESHOLD           (64 * 16)
#define WOM_CNTR_TH             (64 * 16 + 12)

// Activity Recognition
#define BAC_RATE                (48  * 16 + 10)
#define BAC_STATE               (179 * 16 +  0)
#define BAC_STATE_PREV          (179 * 16 +  4)
#define BAC_ACT_ON              (182 * 16 +  0)
#define BAC_ACT_OFF             (183 * 16 +  0)
#define BAC_STILL_S_F           (177 * 16 +  0)
#define BAC_RUN_S_F             (177 * 16 +  4)
#define BAC_DRIVE_S_F           (178 * 16 +  0)
#define BAC_WALK_S_F            (178 * 16 +  4)
#define BAC_SMD_S_F             (178 * 16 +  8)
#define BAC_BIKE_S_F            (178 * 16 + 12)
#define BAC_E1_SHORT            (146 * 16 +  0)
#define BAC_E2_SHORT            (146 * 16 +  4)
#define BAC_E3_SHORT            (146 * 16 +  8)
#define BAC_VAR_RUN             (148 * 16 + 12)
#define BAC_TILT_INIT           (181 * 16 +  0)
#define BAC_MAG_ON              (225 * 16 +  0)
#define BAC_PS_ON               (74  * 16 +  0)
#define BAC_BIKE_PREFERENCE     (173 * 16 +  8)
#define BAC_MAG_I2C_ADDR        (229 * 16 +  8)
#define BAC_PS_I2C_ADDR         (75  * 16 +  4)
#define BAC_DRIVE_CONFIDENCE    (144 * 16 +  0)
#define BAC_WALK_CONFIDENCE     (144 * 16 +  4)
#define BAC_SMD_CONFIDENCE      (144 * 16 +  8)
#define BAC_BIKE_CONFIDENCE     (144 * 16 + 12)
#define BAC_STILL_CONFIDENCE    (145 * 16 +  0)
#define BAC_RUN_CONFIDENCE      (145 * 16 +  4)
#define BAC_MODE_CNTR           (150 * 16)
#define BAC_STATE_T_PREV        (185 * 16 +  4)
#define BAC_ACT_T_ON            (184 * 16 +  0)
#define BAC_ACT_T_OFF           (184 * 16 +  4)
#define BAC_STATE_WRDBS_PREV    (185 * 16 +  8)
#define BAC_ACT_WRDBS_ON        (184 * 16 +  8)
#define BAC_ACT_WRDBS_OFF       (184 * 16 + 12)
#define BAC_ACT_ON_OFF          (190 * 16 +  2)
#define PREV_BAC_ACT_ON_OFF     (188 * 16 +  2)
#define BAC_CNTR                (48  * 16 +  2)

// Flip/Pick-up
#define FP_VAR_ALPHA            (245 * 16 +  8)
#define FP_STILL_TH             (246 * 16 +  4)
#define FP_MID_STILL_TH         (244 * 16 +  8)
#define FP_NOT_STILL_TH         (246 * 16 +  8)
#define FP_VIB_REJ_TH           (241 * 16 +  8)
#define FP_MAX_PICKUP_T_TH      (244 * 16 + 12)
#define FP_PICKUP_TIMEOUT_TH    (248 * 16 +  8)
#define FP_STILL_CONST_TH       (246 * 16 + 12)
#define FP_MOTION_CONST_TH      (240 * 16 +  8)
#define FP_VIB_COUNT_TH         (242 * 16 +  8)
#define FP_STEADY_TILT_TH       (247 * 16 +  8)
#define FP_STEADY_TILT_UP_TH    (242 * 16 + 12)
#define FP_Z_FLAT_TH_MINUS      (243 * 16 +  8)
#define FP_Z_FLAT_TH_PLUS       (243 * 16 + 12)
#define FP_DEV_IN_POCKET_TH     (76  * 16 + 12)
#define FP_PICKUP_CNTR          (247 * 16 +  4)
#define FP_RATE                 (240 * 16 + 12)

// Gyro FSR
#define GYRO_FULLSCALE          (72 * 16 + 12)

// Accel FSR
#define ACC_SCALE               (30 * 16 + 0)
#define ACC_SCALE2              (79 * 16 + 4)

// EIS authentication
#define EIS_AUTH_INPUT			(160 * 16 +   4)
#define EIS_AUTH_OUTPUT			(160 * 16 +   0)

// B2S
#define B2S_RATE                (48  * 16 +   8)
// mounting matrix
#define B2S_MTX_00              (208 * 16)
#define B2S_MTX_01              (208 * 16 + 4)
#define B2S_MTX_02              (208 * 16 + 8)
#define B2S_MTX_10              (208 * 16 + 12)
#define B2S_MTX_11              (209 * 16)
#define B2S_MTX_12              (209 * 16 + 4)
#define B2S_MTX_20              (209 * 16 + 8)
#define B2S_MTX_21              (209 * 16 + 12)
#define B2S_MTX_22              (210 * 16)

// Dmp3 orientation parameters (Q30) initialization
#define Q0_QUAT6				(33 * 16 + 0)
#define Q1_QUAT6				(33 * 16 + 4)
#define Q2_QUAT6				(33 * 16 + 8)
#define Q3_QUAT6				(33 * 16 + 12)

#define DMP_START_ADDRESS   ((unsigned short)0x1000)
#define DMP_MEM_BANK_SIZE   256
#define DMP_LOAD_START      0x90

#define DMP_CODE_SIZE 14301

#define DMP_ACCEL_SET			0x0080 //!< 0x8000 - calibrated accel if accel calibrated, raw accel otherwise
#define DMP_GYRO_SET			0x0040 //!< 0x4000 - raw gyro
#define DMP_CPASS_SET			0x0020 //!< 0x2000 - raw magnetic
#define DMP_ALS_SET				0x0010 //!< 0x1000 - ALS/proximity
#define DMP_QUAT6_SET			0x0008 //!< 0x0800 - game rotation vector
#define DMP_QUAT9_SET			0x0004 //!< 0x0400 - rotation vector with heading accuracy
#define DMP_PQUAT6_SET			0x0002 //!< 0x0200 - truncated game rotation vector for batching
#define DMP_GEOMAG_SET			0x0001 //!< 0x0100 - geomag rotation vector with heading accuracy
#define DMP_PRESSURE_SET		0x8000 //!< 0x0080 - pressure
#define DMP_GYRO_CALIBR_SET		0x4000 //!< 0x0040 - calibrated gyro
#define DMP_CPASS_CALIBR_SET	0x2000 //!< 0x0020 - calibrated magnetic
#define DMP_PED_STEPDET_SET		0x1000 //!< 0x0010 - timestamp when each step is detected
#define DMP_HEADER2_SET			0x0800 //!< 0x0008 - enable/disable data output in data output control register 2
#define DMP_PED_STEPIND_SET		0x0700 //!< 0x0007 - number of steps detected will be attached to the 3 least significant bits of header


static const uint8_t s_Dmp3Image[] = {
#include "imu/icm20948_img_dmp3a.h"
};

static const int s_DmpImageSize = sizeof(s_Dmp3Image);

static const float s_CfgMountingMatrix[9]= {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};

bool ImuIcm20948::Init(const ImuCfg_t &Cfg, AgmIcm20948 * const pIcm)
{
	if (pIcm == NULL)
	{
		return false;
	}

	vpIcm = pIcm;

	uint16_t regaddr = ICM20948_USER_CTRL;
	uint8_t d = vpIcm->Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN);
	//vpIcm->Write8((uint8_t*)&regaddr,	2, d);

	regaddr = ICM20948_FIFO_RST;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);

	if (vpIcm->UploadDMPImage((uint8_t*)s_Dmp3Image, DMP_CODE_SIZE, DMP_LOAD_START))
	{
		uint8_t dd[2];

		dd[0] = DMP_START_ADDRESS >> 8U;
		dd[1] = DMP_START_ADDRESS & 0xFFU;

		// Write DMP program start address
		regaddr = ICM20948_DMP_PROG_START_ADDRH;
		vpIcm->Write((uint8_t*)&regaddr, 2, dd, 2);

		Init(Cfg, pIcm, pIcm, pIcm);

		return true;
	}

	return false;
}

bool ImuIcm20948::Init(const ImuCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	if (pAccel == NULL)
	{
		return false;
	}
	uint16_t regaddr;
	uint8_t d;

	Imu::Init(Cfg, pAccel, pGyro, pMag);

	// Reset FIFO
#if 0
	regaddr = ICM20948_FIFO_RST;
	d = ICM20948_FIFO_RST_FIFO_RESET_MASK;
	Write8((uint8_t*)&regaddr, 2, d);
	regaddr = ICM20948_FIFO_EN_1;
	d = ICM20948_FIFO_EN_1_SLV_0_FIFO_EN;
	Write8((uint8_t*)&regaddr, 2, d);
#endif

	regaddr = ICM20948_FIFO_EN_2;
	d = 0xf;	// All
	Write8((uint8_t*)&regaddr, 2, d);

	vEvtHandler = Cfg.EvtHandler;


//	vpIcm = (AgmIcm20948*)pAccel;
	regaddr = ICM20948_INT_ENABLE;
	d = Read8((uint8_t*)&regaddr, 2);
	d |= ICM20948_INT_ENABLE_DMP_INT1_EN;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_INT_ENABLE_1;
	d = 0;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_INT_ENABLE_2;
	d = 1;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_INT_ENABLE_3;
	d = 1;
	Write8((uint8_t*)&regaddr, 2, d);


	return true;
}

bool ImuIcm20948::Enable()
{
	uint16_t regaddr = ICM20948_USER_CTRL;
	uint8_t d = vpIcm->Read8((uint8_t*)&regaddr, 2);

	d |= ICM20948_USER_CTRL_DMP_EN | ICM20948_USER_CTRL_FIFO_EN;
	vpIcm->Write8((uint8_t*)&regaddr,	2, d);

	return vpIcm->Enable();
}

void ImuIcm20948::Disable()
{
	vpIcm->Disable();
}

void ImuIcm20948::Reset()
{
	vpIcm->Reset();
}

IMU_FEATURE ImuIcm20948::Feature(IMU_FEATURE FeatureBit, bool bEnDis)
{
	if (FeatureBit & IMU_FEATURE_QUATERNION)
	{
		uint16_t f = DMP_QUAT9_SET;
		uint16_t m = DATA_OUT_CTL1;
		WriteDMP(m, (uint8_t*)&f, 2);
	}
	return Imu::Feature();
}

bool ImuIcm20948::Calibrate()
{
	return true;
}

void ImuIcm20948::SetAxisAlignmentMatrix(int8_t * const pMatrix)
{
}

bool ImuIcm20948::Compass(bool bEn)
{
	return true;
}

bool ImuIcm20948::Pedometer(bool bEn)
{
	return true;
}

bool ImuIcm20948::Quaternion(bool bEn, int NbAxis)
{
	Imu::Feature(IMU_FEATURE_QUATERNION, bEn);
	uint16_t f = DMP_QUAT9_SET;
	uint16_t m = DATA_OUT_CTL1;

	if (NbAxis <9)
	{
		f = DMP_QUAT6_SET;
	}

	WriteDMP(m, (uint8_t*)&f, 2);

	if (vbIntEn)
	{
		m = DATA_INTR_CTL;
		WriteDMP(m, (uint8_t*)&f, 2);
	}

	return true;
}

bool ImuIcm20948::Tap(bool bEn)
{
	return true;
}

bool ImuIcm20948::UpdateData()
{
	if (vEvtHandler != NULL)
	{
		vEvtHandler(this, DEV_EVT_DATA_RDY);
	}

	return true;
}

void ImuIcm20948::IntHandler()
{
	uint16_t addr = ICM20948_DMP_INT_STATUS;
	uint8_t d;

//	ReadDMP(addr, &d, 1);

	vpIcm->IntHandler();

	//printf("ImuIcm20948::IntHandler : %x\n", d);
}

int ImuIcm20948::ReadDMP(uint16_t MemAddr, uint8_t *pBuff, int Len)
{
	uint16_t regaddr = ICM20948_DMP_MEM_BANKSEL;

	Write8((uint8_t*)&regaddr, 2, MemAddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR;
	Write8((uint8_t*)&regaddr, 2, MemAddr & 0xFF);

	regaddr = ICM20948_DMP_MEM_RW;
	for (int i = 0; i < Len; i++)
	{
		pBuff[i] = Read8((uint8_t*)&regaddr, 2);
	}

	return Len;
}

int ImuIcm20948::WriteDMP(uint16_t MemAddr, uint8_t *pData, int Len)
{
	uint16_t regaddr = ICM20948_DMP_MEM_BANKSEL;

	Write8((uint8_t*)&regaddr, 2, MemAddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR;
	Write8((uint8_t*)&regaddr, 2, MemAddr & 0xFF);

	regaddr = ICM20948_DMP_MEM_RW;
	for (int i = 0; i < Len; i++)
	{
		Write8((uint8_t*)&regaddr, 2, pData[i]);
	}

	return Len;
}

