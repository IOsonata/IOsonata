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
#include <math.h>

#include "Devices/Drivers/Icm20948/Icm20948.h"
#include "Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseControl.h"
#include "Devices/Drivers/Icm20948/Icm20948AuxTransport.h"
#include "Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "Devices/Drivers/Icm20948/Icm20948Setup.h"
#include "Devices/SensorTypes.h"

#include "idelay.h"
#include "istddef.h"
#include "convutil.h"
#include "imu/imu_icm20948.h"
#include "sensors/agm_icm20948.h"
#include "sensors/agm_icm20948DMP.h"

/* dmp3a.20x48-0.4.1 */
#if 0
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

#endif

static const uint8_t s_Dmp3Image[] = {
#include "imu/icm20948_img_dmp3a.h"
};

static const int s_DmpImageSize = sizeof(s_Dmp3Image);

static const float s_CfgMountingMatrix[9]= {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};

int ImuIcm20948::InvnReadReg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	AgmIcm20948 *dev = (AgmIcm20948*)context;
//	return spi_master_transfer_rx(NULL, reg, rbuffer, rlen);
//	reg |= 0x80;
	int cnt = dev->Read(&reg, 1, rbuffer, (int)rlen);

	return cnt > 0 ? 0 : 1;
}

int ImuIcm20948::InvnWriteReg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	AgmIcm20948 *dev = (AgmIcm20948*)context;
//	return spi_master_transfer_tx(NULL, reg, wbuffer, wlen);

	int cnt = dev->Write(&reg, 1, (uint8_t*)wbuffer, (int)wlen);

	return cnt > 0 ? 0 : 1;
}

void ImuIcm20948::SensorEventHandler(void * context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	ImuIcm20948 *dev = (ImuIcm20948*)context;

	dev->UpdateData(sensortype, timestamp, data, arg);
}

#if 0
bool ImuIcm20948::Init(const ImuCfg_t &Cfg, AgmIcm20948 * const pIcm)
{
	if (pIcm == NULL)
	{
		return false;
	}

	vpIcm = pIcm;

	// Disable DMP & FIFO before FIFO can be reseted and DMP firmware loaded
	uint16_t regaddr = ICM20948_USER_CTRL;
	uint8_t d = vpIcm->Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN);
	vpIcm->Write8((uint8_t*)&regaddr,	2, d);

	// Reset FIFO

	regaddr = ICM20948_FIFO_RST;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);
	Write8((uint8_t*)&regaddr, 2, 0);

	// Upload DMP firmware
	if (vpIcm->UploadDMPImage((uint8_t*)s_Dmp3Image, DMP_CODE_SIZE, DMP_LOAD_START))
	{
		uint8_t dd[2];

		dd[0] = DMP_START_ADDRESS >> 8U;
		dd[1] = DMP_START_ADDRESS & 0xFFU;

		// Write DMP program start address
		regaddr = ICM20948_DMP_PROG_START_ADDRH;
		vpIcm->Write((uint8_t*)&regaddr, 2, dd, 2);

		Init(Cfg, pIcm, pIcm, pIcm);

		regaddr = ICM20948_USER_CTRL;
		d |= ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
		Write8((uint8_t*)&regaddr, 2, d);

		return true;
	}

	return false;
}
#endif

bool ImuIcm20948::Init(const ImuCfg_t &Cfg, AccelSensor * const pAccel, GyroSensor * const pGyro, MagSensor * const pMag)
{
	// Min require for IMU are accel & gyro, must be combo device
	if (pAccel == nullptr || pGyro == nullptr || (AgmIcm20948*)pAccel != (AgmIcm20948*)pGyro)
	{
		return false;
	}

	vpIcm = (AgmIcm20948*)pAccel;

	// Initialize INVN device data structure
	//inv_icm20948_reset_states(&vIcmDevice, &icm20948_serif);

	memset(&vInvnDev, 0, sizeof(inv_icm20948_t));
	vInvnDev.serif.context   = vpIcm;
	vInvnDev.serif.read_reg  = InvnReadReg;
	vInvnDev.serif.write_reg = InvnWriteReg;
	vInvnDev.serif.max_read  = 16; /* maximum number of bytes allowed per serial read */
	vInvnDev.serif.max_write = 16; /* maximum number of bytes allowed per serial write */
	vInvnDev.serif.is_spi = pAccel->InterfaceType() == DEVINTRF_TYPE_SPI;

	if (pMag)
	{
		// Initialize Mag data

		//inv_icm20948_register_aux_compass(&vIcmDevice, INV_ICM20948_COMPASS_ID_AK09916, (uint8_t)AK0991x_DEFAULT_I2C_ADDR);
		vInvnDev.secondary_state.compass_slave_id = HW_AK09916;
		vInvnDev.secondary_state.compass_chip_addr = AK09916_I2C_7BITS_DEVADDR;
		vInvnDev.secondary_state.compass_state = INV_ICM20948_COMPASS_INITED;

		// initialise mounting matrix of compass to identity akm9916
		vInvnDev.mounting_matrix_secondary_compass[0] = 1 ;
		vInvnDev.mounting_matrix_secondary_compass[4] = -1;
		vInvnDev.mounting_matrix_secondary_compass[8] = -1;
	}

	// Setup accel and gyro mounting matrix and associated angle for current board
	inv_icm20948_init_matrix(&vInvnDev);

	for (int i = 0; i < INV_ICM20948_SENSOR_MAX; i++) {
		inv_icm20948_set_matrix(&vInvnDev, s_CfgMountingMatrix, (inv_icm20948_sensor)i);
	}

	memset(&vInvnDev.base_state, 0, sizeof(vInvnDev.base_state));
	vInvnDev.base_state.pwr_mgmt_1 = ICM20948_PWR_MGMT_1_CLKSEL_AUTO;
	vInvnDev.base_state.pwr_mgmt_2 = ICM20948_PWR_MGMT_2_DISABLE_ALL;

	if (vInvnDev.serif.is_spi)
	{
		vInvnDev.base_state.serial_interface = SERIAL_INTERFACE_SPI;
		vInvnDev.base_state.user_ctrl = ICM20948_USER_CTRL_I2C_IF_DIS;
	}
	else
	{
		vInvnDev.base_state.serial_interface = SERIAL_INTERFACE_I2C;
	}

	uint16_t regaddr;
	uint16_t d;


	regaddr = ICM20948_USER_CTRL_REG;
	vInvnDev.base_state.user_ctrl = vpIcm->Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN);

	vpIcm->Write8((uint8_t*)&regaddr, 2, d | vInvnDev.base_state.user_ctrl);

//	inv_icm20948_wakeup_mems(&vInvnDev);

	vpIcm->Disable();


	bool res = vpIcm->InitDMP(ICM20948_DMP_PROG_START_ADDR, s_Dmp3Image, ICM20948_DMP_CODE_SIZE);

	if (res == false)
	{
		return false;
	}

	vInvnDev.base_state.firmware_loaded = 1;

	Imu::Init(Cfg, pAccel, pGyro, pMag);
	vEvtHandler = Cfg.EvtHandler;

	//inv_icm20948_initialize(&vInvnDev, s_Dmp3Image, sizeof(s_Dmp3Image));
	inv_icm20948_set_gyro_divider(&vInvnDev, FIFO_DIVIDER);       //Initial sampling rate 1125Hz/19+1 = 56Hz.
	inv_icm20948_set_accel_divider(&vInvnDev, FIFO_DIVIDER);      //Initial sampling rate 1125Hz/19+1 = 56Hz.

	// Init the sample rate to 56 Hz for BAC,STEPC and B2S
	dmp_icm20948_set_bac_rate(&vInvnDev, DMP_ALGO_FREQ_56);
	dmp_icm20948_set_b2s_rate(&vInvnDev, DMP_ALGO_FREQ_56);

	// FIFO Setup.
	inv_icm20948_write_single_mems_reg(&vInvnDev, REG_FIFO_CFG, BIT_SINGLE_FIFO_CFG); // FIFO Config. fixme do once? burst write?
	inv_icm20948_write_single_mems_reg(&vInvnDev, REG_FIFO_RST, 0x1f); // Reset all FIFOs.
	inv_icm20948_write_single_mems_reg(&vInvnDev, REG_FIFO_RST, 0x1e); // Keep all but Gyro FIFO in reset.
	inv_icm20948_write_single_mems_reg(&vInvnDev, REG_FIFO_EN, 0x0); // Slave FIFO turned off.
	inv_icm20948_write_single_mems_reg(&vInvnDev, REG_FIFO_EN_2, 0x0); // Hardware FIFO turned off.

	d = pAccel->Scale();

	switch (d)
	{
		case 2:
			vInvnDev.base_state.accel_fullscale = MPU_FS_2G;
			break;
		case 4:
			vInvnDev.base_state.accel_fullscale = MPU_FS_4G;
			break;
		case 8:
			vInvnDev.base_state.accel_fullscale = MPU_FS_8G;
			break;
		case 16:
			vInvnDev.base_state.accel_fullscale = MPU_FS_16G;
			break;
	}

	inv_icm20948_set_accel_fullscale(&vInvnDev, vInvnDev.base_state.accel_fullscale);

#if 0
#if 1
	regaddr = DATA_OUT_CTL1;
	d = 0;
	WriteDMP(regaddr, (uint8_t*)&d, 2);

	regaddr = DATA_OUT_CTL2;
	WriteDMP(regaddr, (uint8_t*)&d, 2);

	regaddr = DATA_INTR_CTL;
	WriteDMP(regaddr, (uint8_t*)&d, 2);

	regaddr = MOTION_EVENT_CTL;
	WriteDMP(regaddr, (uint8_t*)&d, 2);

	regaddr = DATA_RDY_STATUS;
	WriteDMP(regaddr, (uint8_t*)&d, 2);
#endif

	// Set FIFO watermark 80%
	uint16_t memaddr = FIFO_WATERMARK;
	d = EndianCvt16(800);
	WriteDMP(memaddr, (uint8_t*)&d, 2);

	regaddr = ICM20948_FIFO_MODE_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_MODE_SNAPSHOT); // blocking

	//regaddr = ICM20948_FIFO_EN_1_REG;
	//d = ICM20948_FIFO_EN_1_SLV_0_FIFO_EN;
//	Write8((uint8_t*)&regaddr, 2, d);

//	regaddr = ICM20948_FIFO_EN_2_REG;
//	d = ICM20948_FIFO_EN_2_TEMP_FIFO_EN;
//	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_INT_ENABLE_REG;
	d = Read8((uint8_t*)&regaddr, 2);
	d |= ICM20948_INT_ENABLE_DMP_INT1_EN;
	Write8((uint8_t*)&regaddr, 2, d);

	// Disable data ready interrupt
	regaddr = ICM20948_INT_ENABLE_1_REG;
	d = 0;
	Write8((uint8_t*)&regaddr, 2, d);

	// FIFO overflow interrupt enable
	regaddr = ICM20948_INT_ENABLE_2_REG;
	d = 1;
	Write8((uint8_t*)&regaddr, 2, d);

	// FIFO watermark enable
	regaddr = ICM20948_INT_ENABLE_3_REG;
	d = 1;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_USER_CTRL_REG;
	d = Read8((uint8_t*)&regaddr, 2);
	d |= ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_FIFO_CFG_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_CFG_SINGLE);

	regaddr = ICM20948_FIFO_CFG_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_CFG_SINGLE);

	regaddr = ICM20948_FIFO_RST_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);
	Write8((uint8_t*)&regaddr, 2, 0x1e);

	regaddr = ICM20948_FIFO_EN_1_REG;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_FIFO_EN_2_REG;
	Write8((uint8_t*)&regaddr, 2, 0);

#if 0
	uint32_t f = pGyro->SamplingFrequency();
	printf("f %d\n", f);

	uint32_t scale = pAccel->Scale();
	uint32_t scale2;

	printf("scale : %d\n", scale);
	switch (scale)
	{
	case 2:
		scale =  EndianCvt32(33554432L);  // 2^25
		scale2 = EndianCvt32(524288L);	// 2^19
		break;
	case 4:
		scale =  EndianCvt32(67108864L);  // 2^26
		scale2 = EndianCvt32(262144L);  // 2^18
		break;
	case 8:
		scale = EndianCvt32(134217728L);  // 2^27
		scale2 = EndianCvt32(131072L);  // 2^17
		break;
	case 16:
		scale = EndianCvt32(268435456L);  // 2^28
		scale2 = EndianCvt32(65536L);  // 2^16
		break;
	case 32:
		scale = EndianCvt32(536870912L);  // 2^29
		scale2 = EndianCvt32(32768L);  // 2^15
		break;
	}

	regaddr = ACC_SCALE;
	WriteDMP(regaddr, (uint8_t*)&scale, 4);

	regaddr = ACC_SCALE2;
	WriteDMP(regaddr, (uint8_t*)&scale2, 4);

	uint16_t div = Read16((uint8_t*)&regaddr, 2) & 0xFF0F;

	regaddr = ODR_ACCEL;
	WriteDMP(regaddr, (uint8_t*)&div, 2);

	uint32_t sens = pGyro->Sensitivity();
	printf("Sensitivity %d\n", sens);

	switch (sens) {
	case 4000:
		sens =  EndianCvt32(536870912L);  // 2^29
		break;
	case 2000:
		sens =  EndianCvt32(268435456L);  // 2^28
		break;
	case 1000:
		sens = EndianCvt32(134217728L);  // 2^27
		break;
	case 500:
		sens = EndianCvt32(67108864L);  // 2^26
		break;
	case 250:
		sens = EndianCvt32(33554432L);  // 2^25
		break;
	}

	regaddr = GYRO_FULLSCALE;
	WriteDMP(regaddr, (uint8_t*)&sens, 4);

	regaddr = DATA_OUT_CTL1;
	uint16_t x = 0xFFFF; // axel
	WriteDMP(regaddr, (uint8_t*)&x, 2);
#endif
#endif
//	x = 0x4000; // Gyro
//	WriteDMP(regaddr, (uint8_t*)&x, 2);

	return true;
}

bool ImuIcm20948::Enable()
{
	bool res = vpIcm->Enable();

	uint16_t regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d = vpIcm->Read8((uint8_t*)&regaddr, 2);

	d |= ICM20948_USER_CTRL_DMP_EN | ICM20948_USER_CTRL_FIFO_EN;
	vpIcm->Write8((uint8_t*)&regaddr, 2, d);

	int i = INV_SENSOR_TYPE_MAX;

	/* Disable all sensors */
	while(i-- > 0) {
		//inv_icm20948_set_sensor_period(vpIcmDevice, (inv_icm20948_sensor)i, 20);
		inv_icm20948_enable_sensor(&vInvnDev, (inv_icm20948_sensor)i, 1);
	}

	return res;
}

void ImuIcm20948::Disable()
{
	uint16_t regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d = vpIcm->Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN);

	vpIcm->Write8((uint8_t*)&regaddr, 2, d);

	vpIcm->Disable();
}

void ImuIcm20948::Reset()
{
	vpIcm->Reset();
}

IMU_FEATURE ImuIcm20948::Feature(IMU_FEATURE FeatureBit, bool bEnDis)
{
	if (FeatureBit & IMU_FEATURE_EULER)
	{

	}

	if (FeatureBit & IMU_FEATURE_QUATERNION)
	{
		uint16_t f = ICM20948_DMP_QUAT9_SET;
		uint16_t m = ICM20948_DMP_DATA_OUT_CTL1;
		WriteDMP(m, (uint8_t*)&f, 2);
	}

	if (FeatureBit & IMU_FEATURE_COMPASS)
	{

	}

	if (FeatureBit & IMU_FEATURE_GRAVITY)
	{

	}

	if (FeatureBit & IMU_FEATURE_EXTERNAL_ACCEL)
	{

	}

	if (FeatureBit & IMU_FEATURE_TAP)
	{

	}

	if (FeatureBit & IMU_FEATURE_ROTATION)
	{

	}

	if (FeatureBit & IMU_FEATURE_VIBRATION)
	{

	}

	if (FeatureBit & IMU_FEATURE_PEDOMETER)
	{

	}

	if (FeatureBit & IMU_FEATURE_CYCLING)
	{

	}

	return Imu::Feature(FeatureBit, bEnDis);
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
	uint16_t f = ICM20948_DMP_QUAT9_SET;
	uint16_t m = ICM20948_DMP_DATA_OUT_CTL1;

	if (NbAxis <9)
	{
		f = ICM20948_DMP_QUAT6_SET;
	}

	WriteDMP(m, (uint8_t*)&f, 2);

	if (vbIntEn)
	{
		m = ICM20948_DMP_DATA_INTR_CTL;
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

void ImuIcm20948::UpdateData(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	float raw_bias_data[6];
	inv_sensor_event_t event;
	//uint8_t sensor_id = convert_to_generic_ids[sensortype];

	memset((void *)&event, 0, sizeof(event));
	event.sensor = sensortype;
	event.timestamp = timestamp;
	switch (sensortype)
	{
	case INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED:
		memcpy(raw_bias_data, data, sizeof(raw_bias_data));
		memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
		memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
		memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED:
		memcpy(raw_bias_data, data, sizeof(raw_bias_data));
		memcpy(event.data.mag.vect, &raw_bias_data[0], sizeof(event.data.mag.vect));
		memcpy(event.data.mag.bias, &raw_bias_data[3], sizeof(event.data.mag.bias));
		memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_GYROSCOPE:
		memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
		memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_GRAVITY:
		memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
		event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
		break;
	case INV_ICM20948_SENSOR_LINEAR_ACCELERATION:
	case INV_ICM20948_SENSOR_ACCELEROMETER:
		memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
		memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
		//vAccData.X = event.data.acc.vect[0] * 256.0;
		//vAccData.Y = event.data.acc.vect[1] * 256.0;
		//vAccData.Z = event.data.acc.vect[2] * 256.0;
		//vAccData.Timestamp = timestamp;
		//printf("a %d : %d %d %d\r\n", vAccData.Timestamp, vAccData.X, vAccData.Y, vAccData.Z);
		break;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
		memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
		memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
		break;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
		break;
	case INV_ICM20948_SENSOR_ROTATION_VECTOR:
		memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));

#if 0
		vQuat.Q1 = event.data.quaternion.quat[0] * 16384.0;
		vQuat.Q2 = event.data.quaternion.quat[1] * 16384.0;
		vQuat.Q3 = event.data.quaternion.quat[2] * 16384.0;
		vQuat.Q4 = event.data.quaternion.quat[3] * 16384.0;
#else
		{
		float *d = (float*)data;
		float a = *(float*)arg;
		vQuat.Q1 = d[0];
		vQuat.Q2 = d[1];
		vQuat.Q3 = d[2];
		vQuat.Q4 = d[3];

		}
#endif
		vQuat.Timestamp = timestamp;
		break;
	case INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR:
	{
		uint8_t accel_accuracy;
		uint8_t gyro_accuracy;

		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));

		accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
		gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
#if 0
		event.data.quaternion.accuracy_flag = min(accel_accuracy, gyro_accuracy);
		vQuat.Q1 = event.data.quaternion.quat[0];// * 32768.0;
		vQuat.Q2 = event.data.quaternion.quat[1];// * 32768.0;
		vQuat.Q3 = event.data.quaternion.quat[2];// * 32768.0;
		vQuat.Q4 = event.data.quaternion.quat[3];// * 32768.0;
		vQuat.Timestamp = timestamp;
#endif
	}
		break;
	case INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON:
		memcpy(&(event.data.bac.event), data, sizeof(event.data.bac.event));
		break;
	case INV_ICM20948_SENSOR_FLIP_PICKUP:
	case INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR:
	case INV_ICM20948_SENSOR_STEP_DETECTOR:
	case INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION:
		event.data.event = true;
		break;
	case INV_ICM20948_SENSOR_B2S:
		event.data.event = true;
		memcpy(&(event.data.b2s.direction), data, sizeof(event.data.b2s.direction));
		break;
	case INV_ICM20948_SENSOR_STEP_COUNTER:
		memcpy(&(event.data.step.count), data, sizeof(event.data.step.count));
		break;
	case INV_ICM20948_SENSOR_ORIENTATION:
		//we just want to copy x,y,z from orientation data
		memcpy(&(event.data.orientation), data, 3*sizeof(float));
		break;
/*	case INV_ICM20948_SENSOR_RAW_ACCELEROMETER:
	case INV_ICM20948_SENSOR_RAW_GYROSCOPE:
		memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
		break;*/
	default:
		//((AgmInvnIcm20948*)vpAccel)->UpdateData(sensortype, timestamp, data, arg);
		//vpIcm->UpdateData(sensortype, timestamp, data, arg);
//		return;
		;
	}

	vpIcm->UpdateData(sensortype, timestamp, data, arg);

	if (vEvtHandler != NULL)
	{
		vEvtHandler(this, DEV_EVT_DATA_RDY);
	}
}

void ImuIcm20948::IntHandler()
{
#if 0
	inv_icm20948_poll_sensor(&vInvnDev, (void*)this, SensorEventHandler);
#else
	uint16_t regaddr = ICM20948_INT_STATUS_REG;
	uint8_t status[4];
	uint8_t d;

	//vpIcm->IntHandler();

	vpIcm->Read((uint8_t*)&regaddr, 2, status, 4);


	regaddr = ICM20948_DMP_INT_STATUS_REG;
	d = vpIcm->Read8((uint8_t*)&regaddr, 2);

	uint64_t t;
	regaddr = ICM20948_FIFO_COUNTH_REG;
	size_t cnt = vpIcm->Read16((uint8_t*)&regaddr, 2);
	cnt = EndianCvt16(cnt);

	if (vpTimer)
	{
		t = vpTimer->uSecond();
	}

	if (status[2])
	{
		printf("%x overflow %d\n", status[2], cnt);
		vpIcm->ResetFifo();
		vFifoHdr = vFifoHdr2 = 0;
		vFifoDataLen = 0;

		return;
	}

	regaddr = REG_FIFO_R_W;

	while (cnt >= ICM20948_FIFO_PAGE_SIZE)
	{
		if (vFifoDataLen > ICM20948_FIFO_PAGE_SIZE)
		{
			printf("VFifoLen : %d\n", vFifoDataLen);
		}
		int l = vpIcm->Read((uint8_t*)&regaddr, 2, &vFifo[vFifoDataLen], ICM20948_FIFO_PAGE_SIZE);

		int x = vFifoDataLen;
		vFifoDataLen += l;

		if (vFifoDataLen > ICM20948_FIFO_PAGE_SIZE * 2)
		{
			printf("-VFifoLen : %d %d %d\n", vFifoDataLen, x, l);
		}
		cnt -= l;

		uint8_t *p = vFifo;

		while (vFifoDataLen > 3)
		{
			if (vFifoHdr == 0 && vFifoHdr2 == 0)
			{
				int l = 0;
				// new packet
				vFifoHdr = ((uint16_t)p[0] << 8U) | ((uint16_t)p[1] & 0xFF);

				if (vFifoHdr & ~ICM20948_FIFO_HEADER_MASK)
				{
					vpIcm->ResetFifo();
					vFifoHdr = vFifoHdr2 = 0;
					vFifoDataLen = 0;
					cnt = 0;
					return;
				}

				l = 2;

				if (vFifoHdr & ICM20948_FIFO_HEADER_HEADER2)
				{
					vFifoHdr2 = ((uint16_t)p[2] << 8U) | ((uint16_t)p[3] & 0xFF);

					if (vFifoHdr2 & ~ICM20948_FIFO_HEADER2_MASK)
					{
						vpIcm->ResetFifo();
						vFifoHdr = vFifoHdr2 = 0;
						vFifoDataLen = 0;
						cnt = 0;
						return;
					}

					l += 2;
					vFifoHdr &= ~ICM20948_FIFO_HEADER_HEADER2;
				}
				vFifoDataLen -= l;
				p+= l;

//				if (vFifoDataLen > 0)
				{
//					memmove(vFifo, &vFifo[l], vFifoDataLen);
				}
	//				printf("Header %x %x\n", vFifoHdr, vFifoHdr2);
			}
			int l = ProcessDMPFifo(p, vFifoDataLen, t);
			if (l == 0)
			{
				return;
			}
			vFifoDataLen -= l;
			p += l;
		}
			if (vFifoDataLen > 0)
			{
				memmove(vFifo, p, vFifoDataLen);
				if (vFifoDataLen > ICM20948_FIFO_PAGE_SIZE)
				{
					printf("3-VFifoLen : %d\n", vFifoDataLen);
				}
			}

	}
#endif
}

int ImuIcm20948::ReadDMP(uint16_t MemAddr, uint8_t *pBuff, int Len)
{
	uint16_t regaddr = ICM20948_DMP_MEM_BANKSEL_REG;

	Write8((uint8_t*)&regaddr, 2, MemAddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
	Write8((uint8_t*)&regaddr, 2, MemAddr & 0xFF);

	regaddr = ICM20948_DMP_MEM_RW_REG;
	uint8_t *p = pBuff;

	while (Len > 0)
	{
		int l = min(16, Len);
		l = Read((uint8_t*)&regaddr, 2, p, l);
		p += l;
		Len -= l;
	}

	return Len;
}

int ImuIcm20948::WriteDMP(uint16_t MemAddr, uint8_t *pData, int Len)
{
	uint16_t regaddr = ICM20948_DMP_MEM_BANKSEL_REG;

	Write8((uint8_t*)&regaddr, 2, MemAddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
	Write8((uint8_t*)&regaddr, 2, MemAddr & 0xFF);

	regaddr = ICM20948_DMP_MEM_RW_REG;
	uint8_t *p = pData;

	while (Len > 0)
	{
		int l = min(16, Len);
		l = Write((uint8_t*)&regaddr, 2, p, l);
		p += l;
		Len -= l;
	}

	return Len;
}

size_t ImuIcm20948::ProcessDMPFifo(uint8_t *pFifo, size_t Len, uint64_t Timestamp)
{
	size_t cnt = 0;
	uint8_t *d = pFifo;//[ICM20948_FIFO_PAGE_SIZE];

	if (vFifoHdr & ICM20948_FIFO_HEADER_ACCEL)
	{
		if (Len < ICM20948_FIFO_HEADER_ACCEL_SIZE)
		{
			return cnt;
		}

		((AccelIcm20948*)vpAccel)->UpdateData(Timestamp,
											  (((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF)),
											  (((int16_t)d[2] << 8) | ((int16_t)d[3] & 0xFF)),
											  (((int16_t)d[4] << 8) | ((int16_t)d[5] & 0xFF)));

		d += ICM20948_FIFO_HEADER_ACCEL_SIZE;
		cnt += ICM20948_FIFO_HEADER_ACCEL_SIZE;
		Len -= ICM20948_FIFO_HEADER_ACCEL_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_ACCEL; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_GYRO)
	{
		if (Len < ICM20948_FIFO_HEADER_GYRO_SIZE)
		{
			return cnt;
		}

		((GyroIcm20948*)vpGyro)->UpdateData(Timestamp,
											(((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF)),
											(((int16_t)d[2] << 8) | ((int16_t)d[3] & 0xFF)),
											(((int16_t)d[4] << 8) | ((int16_t)d[5] & 0xFF)));

		// TODO : Process gyro bias
		float bias[3];

		bias[0] = (float)(((uint16_t)d[6] << 8) | ((uint16_t)d[7] & 0xFF)) / (float)(1<<20);
		bias[1] = (float)(((uint16_t)d[8] << 8) | ((uint16_t)d[9] & 0xFF)) / (float)(1<<20);
		bias[2] = (float)(((uint16_t)d[10] << 8) | ((uint16_t)d[11] & 0xFF)) / (float)(1<<20);

		vpGyro->SetCalibrationOffset(bias);
		//bias[0] /= (1<<20);
		//bias[1] /= (1<<20);
		//bias[2] /= (1<<20);

		//GyroSensor::vData.X += bias[0];// / (1<<4);
		//GyroSensor::vData.Y += bias[1];// / (1<<4);
		//GyroSensor::vData.Z += bias[2];// / (1<<4);


		if (bias[0] > 0)
		{
		//	g_Uart.printf("bias : %d %d %d\r\n", bias[0], bias[1], bias[2]);
		}
		d += ICM20948_FIFO_HEADER_GYRO_SIZE;
		cnt += ICM20948_FIFO_HEADER_GYRO_SIZE;
		Len -= ICM20948_FIFO_HEADER_GYRO_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_GYRO; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_CPASS)
	{
		if (Len < ICM20948_FIFO_HEADER_CPASS_SIZE)
		{
			return cnt;
		}

		((MagIcm20948*)vpMag)->UpdateData(Timestamp,
										  (((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF)),
										  (((int16_t)d[2] << 8) | ((int16_t)d[3] & 0xFF)),
										  (((int16_t)d[4] << 8) | ((int16_t)d[5] & 0xFF)));

		d += ICM20948_FIFO_HEADER_CPASS_SIZE;
		cnt += ICM20948_FIFO_HEADER_CPASS_SIZE;
		Len -= ICM20948_FIFO_HEADER_CPASS_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_CPASS; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_ALS)
	{
		if (Len < ICM20948_FIFO_HEADER_ALS_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_ALS_SIZE;
		cnt += ICM20948_FIFO_HEADER_ALS_SIZE;
		Len -= ICM20948_FIFO_HEADER_ALS_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_ALS; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_QUAT6)
	{
		if (Len < ICM20948_FIFO_HEADER_QUAT6_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_QUAT6_SIZE;
		cnt += ICM20948_FIFO_HEADER_QUAT6_SIZE;
		Len -= ICM20948_FIFO_HEADER_QUAT6_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_QUAT6; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_QUAT9)
	{
		if (Len < ICM20948_FIFO_HEADER_QUAT9_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_QUAT9_SIZE;
		cnt += ICM20948_FIFO_HEADER_QUAT9_SIZE;
		Len -= ICM20948_FIFO_HEADER_QUAT9_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_QUAT9; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_PEDO_QUAT6)
	{
		if (Len < ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE;
		cnt += ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE;
		Len -= ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_PEDO_QUAT6; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_GEOMAG)
	{
		if (Len < ICM20948_FIFO_HEADER_GEOMAG_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		cnt += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		Len -= ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_GEOMAG; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_PRESS_TEMP)
	{
		if (Len < ICM20948_FIFO_HEADER_PRESS_TEMP_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_PRESS_TEMP_SIZE;
		cnt += ICM20948_FIFO_HEADER_PRESS_TEMP_SIZE;
		Len -= ICM20948_FIFO_HEADER_PRESS_TEMP_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_PRESS_TEMP; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_CALIB_GYRO)
	{
		// Hardware unit scaled by 2^15
		// Although bit is set but no data, try to read data will corrupt fifo
#if 0
		if (Len < ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE;
		cnt += ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE;
		Len -= ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE;
#endif
		vFifoHdr &= ~ICM20948_FIFO_HEADER_CALIB_GYRO; // Clear bit
	}
	if (vFifoHdr & ICM20948_FIFO_HEADER_CALIB_CPASS)
	{
		if (Len < ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		cnt += ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		Len -= ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_CALIB_CPASS; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_STEP_DETECTOR)
	{
		// The unit is uT scaled by 2^16
		if (Len < ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE)
		{
			return cnt;
		}

		d += ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		cnt += ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		Len -= ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_STEP_DETECTOR; // Clear bit
	}

	if (vFifoHdr2 != 0)
	{
		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_ACCEL_ACCUR)
		{
			if (Len < ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE)
			{
				return cnt;
			}

			d += ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE;
			cnt += ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE;
			Len -= ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_ACCEL_ACCUR; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_GYRO_ACCUR)
		{
			if (Len < ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE)
			{
				return cnt;
			}

			d += ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE;
			cnt += ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE;
			Len -= ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_GYRO_ACCUR; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_CPASS_ACCUR)
		{
			if (Len < ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE)
			{
				return cnt;
			}

			d += ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			cnt += ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			Len -= ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_CPASS_ACCUR; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_FSYNC)
		{
			if (Len < ICM20948_FIFO_HEADER2_FSYNC_SIZE)
			{
				return cnt;
			}

			d += ICM20948_FIFO_HEADER2_FSYNC_SIZE;
			cnt += ICM20948_FIFO_HEADER2_FSYNC_SIZE;
			Len -= ICM20948_FIFO_HEADER2_FSYNC_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_FSYNC; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_PICKUP)
		{
			if (Len < ICM20948_FIFO_HEADER2_PICKUP_SIZE)
			{
				return cnt;
			}

			d += ICM20948_FIFO_HEADER2_PICKUP_SIZE;
			cnt += ICM20948_FIFO_HEADER2_PICKUP_SIZE;
			Len -= ICM20948_FIFO_HEADER2_PICKUP_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_PICKUP; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_ACTI_RECOG)
		{
			if (Len < ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE)
			{
				return cnt;
			}

			d += ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			cnt += ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			Len -= ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_ACTI_RECOG; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_SECOND_ONOFF)
		{
			if (Len < ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE)
			{
				return cnt;
			}

			d += ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE;
			cnt += ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE;
			Len -= ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_SECOND_ONOFF; // Clear bit
		}
	}

	if (Len < ICM20948_FIFO_FOOTER_SIZE)
	{
		vFifoHdr |= ICM20948_FIFO_HEADER_FOOTER;

		return cnt;
	}

	cnt += ICM20948_FIFO_FOOTER_SIZE;
	Len -= ICM20948_FIFO_FOOTER_SIZE;
	vFifoHdr &= ~ICM20948_FIFO_HEADER_FOOTER; // Clear bit

	vFifoHdr = vFifoHdr2 = 0;

	return cnt;
}

