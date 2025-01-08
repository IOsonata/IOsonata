/**-------------------------------------------------------------------------
@file	agm_invn_icm20948.cpp

@brief	Implementation of TDK ICM-20948 accel, gyro, mag sensor

This implementation wraps the Invensen SmartMotion driver

@author	Hoang Nguyen Hoan
@date	Dec. 24, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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
#include "Devices/Drivers/Icm20948/Icm20948.h"
#include "Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseControl.h"
#include "Devices/Drivers/Icm20948/Icm20948AuxTransport.h"
#include "Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "Devices/Drivers/Icm20948/Icm20948Setup.h"
//#include "Devices/Drivers/Ak0991x/Ak0991x.h"
#include "Devices/SensorTypes.h"
//#include "Devices/SensorConfig.h"
//#include "EmbUtils/InvScheduler.h"
//#include "EmbUtils/RingByteBuffer.h"
//#include "EmbUtils/Message.h"
//#include "EmbUtils/ErrorHelper.h"
//#include "EmbUtils/DataConverter.h"
//#include "EmbUtils/RingBuffer.h"
//#include "DynamicProtocol/DynProtocol.h"
//#include "DynamicProtocol/DynProtocolTransportUart.h"


#include "idelay.h"
#include "convutil.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "sensors/agm_invn_icm20948.h"

//#define ICM20948_WHO_AM_I_ID		0xEA

#define AK0991x_DEFAULT_I2C_ADDR	0x0C	/* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E	/* The secondary I2C address for AK0991x Magnetometers */

static const uint8_t s_Dmp3Image[] = {
#include "imu/icm20948_img_dmp3a.h"
};

static const float s_CfgMountingMatrix[9]= {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};
/*
static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
	INV_SENSOR_TYPE_ACCELEROMETER,
	INV_SENSOR_TYPE_GYROSCOPE,
	INV_SENSOR_TYPE_RAW_ACCELEROMETER,
	INV_SENSOR_TYPE_RAW_GYROSCOPE,
	INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
	INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
	INV_SENSOR_TYPE_BAC,
	INV_SENSOR_TYPE_STEP_DETECTOR,
	INV_SENSOR_TYPE_STEP_COUNTER,
	INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
	INV_SENSOR_TYPE_ROTATION_VECTOR,
	INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
	INV_SENSOR_TYPE_MAGNETOMETER,
	INV_SENSOR_TYPE_SMD,
	INV_SENSOR_TYPE_PICK_UP_GESTURE,
	INV_SENSOR_TYPE_TILT_DETECTOR,
	INV_SENSOR_TYPE_GRAVITY,
	INV_SENSOR_TYPE_LINEAR_ACCELERATION,
	INV_SENSOR_TYPE_ORIENTATION,
	INV_SENSOR_TYPE_B2S
};
*/

__attribute__((weak)) void inv_icm20948_sleep(int ms)
{
	msDelay(ms);
}

__attribute__((weak)) void inv_icm20948_sleep_us(int us)
{
	usDelay(us);
}

int AgmInvnIcm20948::InvnReadReg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	AgmInvnIcm20948 *dev = (AgmInvnIcm20948*)context;
//	return spi_master_transfer_rx(NULL, reg, rbuffer, rlen);
//	reg |= 0x80;
	int cnt = dev->Read(&reg, 1, rbuffer, (int)rlen);

	return cnt > 0 ? 0 : 1;
}

int AgmInvnIcm20948::InvnWriteReg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	AgmInvnIcm20948 *dev = (AgmInvnIcm20948*)context;
//	return spi_master_transfer_tx(NULL, reg, wbuffer, wlen);

	int cnt = dev->Write(&reg, 1, (uint8_t*)wbuffer, (int)wlen);

	return cnt > 0 ? 0 : 1;
}

bool AgmInvnIcm20948::Init(uint32_t DevAddr, DeviceIntrf *pIntrf, Timer *pTimer)
{
	//if (vbInitialized)
	if (Valid())
		return true;;

	if (pIntrf == NULL)
		return false;

	Interface(pIntrf);
	DeviceAddress(DevAddr);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	uint8_t d;
	struct inv_icm20948_serif icm20948_serif;

	icm20948_serif.context   = this;
	icm20948_serif.read_reg  = InvnReadReg;
	icm20948_serif.write_reg = InvnWriteReg;
	icm20948_serif.max_read  = 255; /* maximum number of bytes allowed per serial read */
	icm20948_serif.max_write = 255; /* maximum number of bytes allowed per serial write */
	icm20948_serif.is_spi = vpIntrf->Type() == DEVINTRF_TYPE_SPI;

	inv_icm20948_reset_states(&vIcmDevice, &icm20948_serif);

	inv_icm20948_register_aux_compass(&vIcmDevice, INV_ICM20948_COMPASS_ID_AK09916, (uint8_t)AK0991x_DEFAULT_I2C_ADDR);

	inv_icm20948_get_whoami(&vIcmDevice, &d);

	if (d != ICM20948_WHO_AM_I_ID)
	{
		return false;
	}

	Reset();

	DeviceID(d);
	Valid(true);


	// NOTE : require delay for reset to stabilize
	// the chip would not respond properly to motion detection
	usDelay(500000);

	// Setup accel and gyro mounting matrix and associated angle for current board
	inv_icm20948_init_matrix(&vIcmDevice);

	for (int i = 0; i < INV_ICM20948_SENSOR_MAX; i++) {
		inv_icm20948_set_matrix(&vIcmDevice, s_CfgMountingMatrix, (inv_icm20948_sensor)i);
	}

	inv_icm20948_initialize(&vIcmDevice, s_Dmp3Image, sizeof(s_Dmp3Image));
	/* Initialize auxiliary sensors */
	inv_icm20948_register_aux_compass( &vIcmDevice, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
	//rc = inv_icm20948_initialize_auxiliary(&vIcmDevice);

	// re-initialize base state structure
	inv_icm20948_init_structure(&vIcmDevice);

	vbInitialized  = true;

	return true;
}

bool AccelInvnIcm20948::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		return false;

	vData.Range = Range(ICM20948_ACC_ADC_RANGE);

	SamplingFrequency(Cfg.Freq);
	Scale(Cfg.Scale);
	FilterFreq(Cfg.FltrFreq);

	return true;
}

uint16_t AccelInvnIcm20948::Scale(uint16_t Value)
{
	enum mpu_accel_fs d;

	if (Value < 3)
	{
		d = MPU_FS_2G;
		Value = 2;
	}
	else if (Value < 6)
	{
		d = MPU_FS_4G;
		Value = 4;
	}
	else if (Value < 12)
	{
		d = MPU_FS_8G;
		Value = 8;
	}
	else
	{
		d = MPU_FS_16G;
		Value = 16;
	}


#if 1
	uint16_t regaddr = REG_ACCEL_CONFIG;
	Write8((uint8_t*)&regaddr, 2, d);
#else
	inv_icm20948_set_accel_fullscale(*this, d);
#endif
	return AccelSensor::Scale(Value);
}

uint32_t AccelInvnIcm20948::SamplingFrequency(uint32_t Freq)
{
	// ODR = 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])

	uint32_t div = (1125000 + (Freq >>1)) / Freq - 1;
	uint16_t d = EndianCvt16(div);
	uint16_t regaddr = REG_ACCEL_SMPLRT_DIV_1;

	Write16((uint8_t*)&regaddr, 2, d);

	div++;
//	inv_icm20948_ctrl_set_accel_quaternion_gain(s, div);
//	inv_icm20948_ctrl_set_accel_cal_params(s, div);
//	s->lLastHwSmplrtDividerAcc = div;
	return AccelSensor::SamplingFrequency(1125000 / div);
}

#define ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE					(1<<0)	// Enable accel DLPF

#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_MASK				(3<<1)	// Full scale select mask
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_2G				(0<<1)	// Full scale select 2g
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_4G				(1<<1)	// Full scale select 4g
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_8G				(2<<1)	// Full scale select 8g
#define ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_16G				(3<<1)	// Full scale select 16g

#define ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_MASK			(7<<3)	// Low pass filter config
#define ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS			3

uint32_t AccelInvnIcm20948::FilterFreq(uint32_t Freq)
{
	uint16_t regaddr = REG_ACCEL_CONFIG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_MASK | ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE);

	if (Freq == 0)
	{
		Freq = 1248000;
	}
	else if (Freq < 11000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (6 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 8300;	// NBW
	}
	else if (Freq < 23000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (5 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 17000;
	}
	else if (Freq < 50000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (4 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 34400;
	}
	else if (Freq < 110000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (3 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 68800;
	}
	else if (Freq < 240000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (2 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 136000;
	}
	else if (Freq < 470000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (1 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 265000;
	}
	else if (Freq < 1000000)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FCHOICE | (7 << ICM20948_ACCEL_CONFIG_ACCEL_DLPFCFG_BITPOS);
		Freq = 499000;
	}
	else
	{
		Freq = 1248000;
	}

	Write8((uint8_t*)&regaddr, 2, d);

	return AccelSensor::FilterFreq(Freq);
}

bool GyroInvnIcm20948::Init(const GyroSensorCfg_t &Cfg, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		return false;

	vData.Range = Range(ICM20948_GYRO_ADC_RANGE);
	Sensitivity(Cfg.Sensitivity);
	SamplingFrequency(Cfg.Freq);

	return true;
}

uint32_t GyroInvnIcm20948::Sensitivity(uint32_t Value)
{
	enum mpu_gyro_fs gfsr;

	if (Value < 325)
	{
		gfsr = MPU_FS_250dps;
		Value = 250;
	}
	else if (Value < 750)
	{
		gfsr = MPU_FS_500dps;
		Value = 500;
	}
	else if (Value < 1500)
	{
		gfsr = MPU_FS_1000dps;
		Value = 1000;
	}
	else
	{
		gfsr = MPU_FS_2000dps;
		Value = 2000;
	}

#if 1
	inv_icm20948_set_gyro_fullscale(*this, gfsr);
#else
	uint16_t regaddr = REG_GYRO_CONFIG_1;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~(3<<1);
	d |= (gfsr << 1);
	Write8((uint8_t*)&regaddr, 2, d);

#endif

	return GyroSensor::Sensitivity(Value);
}

uint32_t GyroInvnIcm20948::SamplingFrequency(uint32_t Freq)
{
	// ODR = 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])

	uint32_t div = (1100000 + (Freq >> 1)) / Freq - 1;
	uint16_t regaddr = REG_GYRO_SMPLRT_DIV;
	Write8((uint8_t*)&regaddr, 2, div);

	return GyroSensor::SamplingFrequency(1100000 / (div + 1));
}

uint32_t GyroInvnIcm20948::FilterFreq(uint32_t Freq)
{
	return GyroSensor::FilterFreq(Freq);
}

bool MagInvnIcm20948::Init(const MagSensorCfg_t &Cfg, DeviceIntrf *pIntrf, Timer *pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, pTimer) == false)
		return false;

#if 0
//	int rc = inv_icm20948_initialize_auxiliary(*this);
	//int rc = inv_icm20948_set_slave_compass_id(*this, ((struct inv_icm20948 *)*this)->secondary_state.compass_slave_id);

	int result = 0;

	struct inv_icm20948 *s = (struct inv_icm20948 *)*this;

	inv_icm20948_prevent_lpen_control(s);
	//activate_compass(s);
	s->s_compass_available = 1;
	//inv_icm20948_init_secondary(s);
	s->secondary_state.slv_reg[0].addr = REG_I2C_SLV0_ADDR;
	s->secondary_state.slv_reg[0].reg  = REG_I2C_SLV0_REG;
	s->secondary_state.slv_reg[0].ctrl = REG_I2C_SLV0_CTRL;
	s->secondary_state.slv_reg[0].d0   = REG_I2C_SLV0_DO;

    s->secondary_state.slv_reg[1].addr = REG_I2C_SLV1_ADDR;
	s->secondary_state.slv_reg[1].reg  = REG_I2C_SLV1_REG;
	s->secondary_state.slv_reg[1].ctrl = REG_I2C_SLV1_CTRL;
	s->secondary_state.slv_reg[1].d0   = REG_I2C_SLV1_DO;

    s->secondary_state.slv_reg[2].addr = REG_I2C_SLV2_ADDR;
	s->secondary_state.slv_reg[2].reg  = REG_I2C_SLV2_REG;
	s->secondary_state.slv_reg[2].ctrl = REG_I2C_SLV2_CTRL;
	s->secondary_state.slv_reg[2].d0   = REG_I2C_SLV2_DO;

	s->secondary_state.slv_reg[3].addr = REG_I2C_SLV3_ADDR;
	s->secondary_state.slv_reg[3].reg  = REG_I2C_SLV3_REG;
	s->secondary_state.slv_reg[3].ctrl = REG_I2C_SLV3_CTRL;
	s->secondary_state.slv_reg[3].d0   = REG_I2C_SLV3_DO;

	inv_icm20948_secondary_stop_channel(s, 0);

	// Set up the secondary I2C bus on 20630.
	//inv_icm20948_set_secondary(s);
	uint16_t regaddr = REG_I2C_MST_CTRL;
	uint8_t d = 0;
	Write8((uint8_t*)&regaddr, 2, BIT_I2C_MST_P_NSR);

	regaddr = REG_I2C_MST_ODR_CONFIG;
	Write8((uint8_t*)&regaddr, 2, MIN_MST_ODR_CONFIG);

	//Setup Compass
	result = inv_icm20948_setup_compass_akm(s);

	//Setup Compass mounting matrix into DMP
	result |= inv_icm20948_compass_dmp_cal(s, s->mounting_matrix, s->mounting_matrix_secondary_compass);

	if (result)
	{
		//desactivate_compass(s);
		s->s_compass_available = 0;

	}

	//result = inv_icm20948_sleep_mems(s);
	inv_icm20948_allow_lpen_control(s);

	MagSensor::Range(AK09916_ADC_RANGE);
	return result == 0;
#else
	//msDelay(200);

	struct inv_icm20948 *s = (struct inv_icm20948 *)*this;
	s->s_compass_available = 1;
	//inv_icm20948_init_secondary(s);
	s->secondary_state.slv_reg[0].addr = REG_I2C_SLV0_ADDR;
	s->secondary_state.slv_reg[0].reg  = REG_I2C_SLV0_REG;
	s->secondary_state.slv_reg[0].ctrl = REG_I2C_SLV0_CTRL;
	s->secondary_state.slv_reg[0].d0   = REG_I2C_SLV0_DO;

	//inv_icm20948_secondary_stop_channel(s, 0);
	uint16_t regaddr = REG_I2C_MST_CTRL;
	//Write8((uint8_t*)&regaddr, 2, 0);


//	regaddr = REG_I2C_MST_CTRL;
//	uint8_t d = 0;
	Write8((uint8_t*)&regaddr, 2, BIT_I2C_MST_P_NSR);

	regaddr = REG_I2C_MST_ODR_CONFIG;
	Write8((uint8_t*)&regaddr, 2, MIN_MST_ODR_CONFIG);

	if (MagAk09916::Init(Cfg, pIntrf, pTimer) == false)
	{
		return false;
	}

	((struct inv_icm20948 *)*this)->s_compass_available = 1;
	//int result = inv_icm20948_setup_compass_akm(*this);

	//Setup Compass mounting matrix into DMP
	int result = inv_icm20948_compass_dmp_cal(*this, ((struct inv_icm20948 *)*this)->mounting_matrix, ((struct inv_icm20948 *)*this)->mounting_matrix_secondary_compass);

#endif

	MagSensor::Range(AK09916_ADC_RANGE);

	return true;
}

AgmInvnIcm20948::AgmInvnIcm20948()
{
	vbInitialized  = false;
	vbDmpEnabled = false;
	vbSensorEnabled[0] = vbSensorEnabled[1] = vbSensorEnabled[2] = false;
	vType = SENSOR_TYPE_TEMP | SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO | SENSOR_TYPE_MAG;
	vFifoHdr = vFifoHdr2 = 0;
	vFifoDataLen = 0;
}

bool AgmInvnIcm20948::Enable()
{
	int i = INV_SENSOR_TYPE_MAX;

	/* Disable all sensors */
#if 1
	while(i-- > 0) {
		inv_icm20948_enable_sensor(&vIcmDevice, (inv_icm20948_sensor)i, 1);
	}
#else
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_ACCELEROMETER, 1);
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_GYROSCOPE, 1);
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, 1);
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_RAW_GYROSCOPE, 1);
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, 1);
	inv_icm20948_enable_sensor(&vIcmDevice, INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR, 1);

#endif
	return true;
}

void AgmInvnIcm20948::Disable()
{
	int i = INV_SENSOR_TYPE_MAX;

	/* Disable all sensors */
	while(i-- > 0) {
		inv_icm20948_enable_sensor(&vIcmDevice, (inv_icm20948_sensor)i, 0);
	}
}

void AgmInvnIcm20948::Reset()
{
	inv_icm20948_soft_reset(&vIcmDevice);
}

bool AgmInvnIcm20948::StartSampling()
{
	return true;
}

// Implement wake on motion
bool AgmInvnIcm20948::WakeOnEvent(bool bEnable, int Threshold)
{
    uint16_t regaddr;

	if (bEnable == true)
	{
		Reset();

		msDelay(2000);
	}
	else
	{
	}

	return true;
}

bool AgmInvnIcm20948::UpdateData()
{
#if 0
	inv_icm20948_poll_sensor(&vIcmDevice, (void*)this, SensorEventHandler);
	return true;
#else
	uint16_t regaddr = ICM20948_INT_STATUS_REG;
	uint8_t status[4];
	uint8_t d[20];
	uint64_t t;
	bool res = false;
	//uint8_t fifo[ICM20948_FIFO_PAGE_SIZE];
	//uint8_t *p = fifo;

	Read((uint8_t*)&regaddr, 2, status, 4);

	if (vpTimer)
	{
		t = vpTimer->uSecond();
	}

	if (status[0] & ICM20948_INT_STATUS_PLL_RDY_INT)
	{

	}

	if (status[0] & ICM20948_INT_STATUS_WOM_INT)
	{

	}
	if (status[0] & ICM20948_INT_STATUS_I2C_MIST_INT)
	{
		printf("ICM20948_INT_STATUS_I2C_MIST_INT\n");
	}
	if (status[0] & ICM20948_INT_STATUS_DMP_INT1 || status[2] || status[3])
	{
		regaddr = ICM20948_DMP_INT_STATUS_REG;
		uint16_t distatus;
		Read((uint8_t*)&regaddr, 2, (uint8_t*)&distatus, 1);
	//		Write16((uint8_t*)&regaddr, 2, distatus);

		if (distatus)// & (ICM20948_DMP_INT_STATUS_MSG_DMP_INT | ICM20948_DMP_INT_STATUS_MSG_DMP_INT_0))
		{
			// DMP msg
			//printf("distatus %x\n", distatus);
		}
	}
#if 0
		//printf("ICM20948_INT_STATUS_DMP_INT1\n");
//		if (status[2] || status[3])
		{
			regaddr = ICM20948_FIFO_COUNTH_REG;
			size_t cnt = Read16((uint8_t*)&regaddr, 2);
			cnt = EndianCvt16(cnt);
#if 0
			while (cnt > 0)
			{
				int l = min(cnt, min((size_t)ICM20948_FIFO_PAGE_SIZE, ICM20948_FIFO_SIZE_MAX - vFifoDataLen));
				if (l <= 0)
				{
					break;
				}
				regaddr = ICM20948_FIFO_R_W_REG;
				l = Read((uint8_t*)&regaddr, 2, &vFifo[vFifoDataLen], l);
				vFifoDataLen += l;
				cnt -= l;
			}
#endif
			regaddr = ICM20948_FIFO_R_W_REG;

			while (cnt > 2)
			{
				if (vFifoHdr == 0 && vFifoHdr2 == 0)
				{
					// New packet
					if (cnt < 4)
					{
						// Not enough data in fifo
						return false;
					}
					// Read headers from FIFO
					vFifoHdr = Read16((uint8_t*)&regaddr, 2);
					vFifoHdr = EndianCvt16(vFifoHdr);// | ICM20948_FIFO_HEADER_FOOTER;

					if (vFifoHdr == 0 || vFifoHdr & ~ICM20948_FIFO_HEADER_MASK)
					{
						// Bad header
						ResetFifo();
						return false;
					}

					//vFifoHdr &= ICM20948_FIFO_HEADER_MASK;
					//vFifoHdr = ((uint16_t)vFifo[0] << 8) | (vFifo[1] & 0xff) | ICM20948_FIFO_HEADER_FOOTER;

					//vFifoDataLen -= 2;
					cnt -= 2;
//					if (vFifoHdr & ~ICM20948_FIFO_HEADER_MASK)
					{
						// bad header
//						vFifoHdr = 0;
//						return false;
					}

					//printf("vFifoHdr %x\n", vFifoHdr);

					if (vFifoHdr & ICM20948_FIFO_HEADER_HEADER2)
					{
						vFifoHdr2 = Read16((uint8_t*)&regaddr, 2);
						vFifoHdr2 = EndianCvt16(vFifoHdr2);

						if (vFifoHdr2 & ~ICM20948_FIFO_HEADER2_MASK)
						{
							ResetFifo();
							return false;
						}
						//vFifoHdr2 &= ICM20948_FIFO_HEADER2_MASK;
						//vFifoHdr2 = ((uint16_t)vFifo[2] << 8) | (vFifo[3] & 0xff);
						//vFifoDataLen -= 2;
						cnt -= 2;
						vFifoHdr &= ~ICM20948_FIFO_HEADER_HEADER2; // Clear bit
					//	if (vFifoHdr2 & ~ICM20948_FIFO_HEADER2_MASK)
						{
							// bad header
					//		vFifoHdr = vFifoHdr2 = 0;

					//		return false;
						}
					}
					//printf("%x %x\n", vFifoHdr, vFifoHdr2);
				}
				else
				//if (cnt > 0 && (vFifoHdr != 0 || vFifoHdr2 != 0))
				{
					int l = min(cnt, (size_t)ICM20948_FIFO_PAGE_SIZE - vFifoDataLen);
					l = Read((uint8_t*)&regaddr, 2, &vFifo[vFifoDataLen], l);

					vFifoDataLen += l;
					cnt -= l;

					l = ProcessDMPFifo(vFifo, vFifoDataLen, t);
					if (l == 0)
					{
						return false;
					}
					vFifoDataLen -= l;
					if (vFifoDataLen > 0)
					{
						memmove(vFifo, &vFifo[l], vFifoDataLen);
					}
				}
			}
#if 0
#if 0
			if (cnt >= ICM20948_FIFO_MAX_PKT_SIZE)
			{
				ProcessDMPFifo(cnt, t);
			}
#else
			uint8_t fifo[cnt];
			uint8_t *p = fifo;
			regaddr = ICM20948_FIFO_R_W_REG;
			int len = cnt;

			if (cnt > 3)
			{
				uint16_t header = EndianCvt16(Read16((uint8_t*)&regaddr, 2));
				uint16_t header2 = 0;
				size_t pklen = CalcFifoPacketSize(header, 0x8000, s_FifoDataLenLookup1, s_NbFifoDataLenLookup1);

				if (header & ICM20948_FIFO_HEADER_HEADER2)
				{
					header2 = EndianCvt16(Read16((uint8_t*)&regaddr, 2));
					pklen += CalcFifoPacketSize(header2, 0x4000, s_FifoDataLenLookup2, s_NbFifoDataLenLookup2);
				}

				pklen += 2; // Footer

				if (pklen <= cnt)
				{
					while (pklen > 0)
					{
						size_t l = min(pklen, (size_t)ICM20948_FIFO_PAGE_SIZE);
						l = Read((uint8_t*)&regaddr, 2, p, l);
						cnt -= l;
						p += l;
						pklen -= l;
					}
				}
			}
#endif
#endif
			/*
			cnt = len;
			p = fifo;
			while (cnt > 0)
			{
				size_t l = ProcessDMPFifo(p, cnt, t);
				if (l == 0)
				{
					break;
				}
				p += l;
				cnt -= l;
			}
*/
		}

#endif

	if (vbDmpEnabled)
	{
		regaddr = ICM20948_FIFO_COUNTH_REG;
		size_t cnt = Read16((uint8_t*)&regaddr, 2);
		cnt = EndianCvt16(cnt);

		regaddr = ICM20948_FIFO_R_W_REG;
		uint8_t *p = &vFifo[vFifoDataLen];

		while (cnt > 0)
		{
			int l = min((size_t)ICM20948_FIFO_PAGE_SIZE, min(cnt, (size_t)ICM20948_FIFO_SIZE_MAX - vFifoDataLen));

			if (l == 0)
			{
				break;
			}
			l = Read((uint8_t*)&regaddr, 2, p, l);
			p += l;
			vFifoDataLen += l;
			cnt -= l;
		}

		while (vFifoDataLen > 2)
		{
			if (vFifoHdr == 0 && vFifoHdr2 == 0)
			{
				int l = 0;
				// new packet
				vFifoHdr = ((uint16_t)vFifo[0] << 8U) | ((uint16_t)vFifo[1] & 0xFF);

				if (vFifoHdr & ~ICM20948_FIFO_HEADER_MASK)
				{
					ResetFifo();
					return false;
				}

				l = 2;

				if (vFifoHdr & ICM20948_FIFO_HEADER_HEADER2)
				{
					vFifoHdr2 = ((uint16_t)vFifo[2] << 8U) | ((uint16_t)vFifo[3] & 0xFF);

					if (vFifoHdr2 & ~ICM20948_FIFO_HEADER2_MASK)
					{
						ResetFifo();
						return false;
					}

					l += 2;
				}
				vFifoDataLen -= l;

				if (vFifoDataLen > 0)
				{
					memmove(vFifo, &vFifo[l], vFifoDataLen);
				}
//				printf("Header %x %x\n", vFifoHdr, vFifoHdr2);
			}
			int l = ProcessDMPFifo(vFifo, vFifoDataLen, t);
			if (l == 0)
			{
				return false;
			}
			vFifoDataLen -= l;
			if (vFifoDataLen > 0)
			{
				memmove(vFifo, &vFifo[l], vFifoDataLen);
			}
		}
	}
	else
	{
		//res = MagIcm20948::UpdateData();

		if (status[1] & ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT)
		{
			regaddr = ICM20948_ACCEL_XOUT_H_REG;
			Read((uint8_t*)&regaddr, 2, (uint8_t*)d, 14);

			AccelSensor::vData.Timestamp = t;
			AccelSensor::vData.X = ((int16_t)(d[0] << 8) | d[1]);
			AccelSensor::vData.Y = ((int16_t)(d[2] << 8) | d[3]);
			AccelSensor::vData.Z = ((int16_t)(d[4] << 8) | d[5]);
			GyroSensor::vData.Timestamp = t;
			GyroSensor::vData.X = ((int16_t)(d[6] << 8) | d[7]);
			GyroSensor::vData.Y = ((int16_t)(d[8] << 8) | d[9]);
			GyroSensor::vData.Z = ((int16_t)(d[10] << 8) | d[11]);


			// TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
			int16_t t = ((int16_t)d[12] << 8) | d[13];
		//	TempSensor::vData.Temperature =  (((int16_t)d[12] << 8) | d[13]) * 100 / 33387 + 2100;
		//	TempSensor::vData.Timestamp = t;

			res = true;
		}
	}

	return res;
#endif
}

bool AgmInvnIcm20948::SelectBank(uint8_t BankNo)
{
	if (BankNo > 3 || vCurrBank == BankNo)
		return false;

	vCurrBank = BankNo;

	uint8_t regaddr = REG_BANK_SEL;

	return Write8(&regaddr, 1, (BankNo << 4) & 0xF0);
}

int AgmInvnIcm20948::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (CmdAddrLen == 2)
	{
		uint16_t *p = (uint16_t*)pCmdAddr;
		SelectBank(*p >> 7);
		CmdAddrLen--;
		*p &= 0x7f;
	}

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}

int AgmInvnIcm20948::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (CmdAddrLen == 2)
	{
		uint16_t *p = (uint16_t*)pCmdAddr;
		SelectBank(*p >> 7);
		CmdAddrLen--;
		*p &= 0x7f;
	}

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}

size_t AgmInvnIcm20948::ProcessDMPFifo(uint8_t *pFifo, size_t Len, uint64_t Timestamp)
{
	bool retval = false;
	size_t cnt = 0;
	uint16_t regaddr = ICM20948_FIFO_R_W_REG;
	uint8_t *d = pFifo;//[ICM20948_FIFO_PAGE_SIZE];

	if (vFifoHdr & ICM20948_FIFO_HEADER_ACCEL)
	{
		if (Len < ICM20948_FIFO_HEADER_ACCEL_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_ACCEL_SIZE);

		AccelSensor::vData.Timestamp = Timestamp;
		AccelSensor::vData.X = ((d[0] << 8) | (d[1] & 0xFF));// << 15;
		AccelSensor::vData.Y = ((d[2] << 8) | (d[3] & 0xFF));// << 15;
		AccelSensor::vData.Z = ((d[4] << 8) | (d[5] & 0xFF));// << 15;
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
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_GYRO_SIZE);

		GyroSensor::vData.Timestamp = Timestamp;
		GyroSensor::vData.X = ((uint16_t)d[0] << 8) | ((uint16_t)d[1] & 0xFF);
		GyroSensor::vData.Y = ((uint16_t)d[2] << 8) | ((uint16_t)d[3] & 0xFF);
		GyroSensor::vData.Z = ((uint16_t)d[4] << 8) | ((uint16_t)d[5] & 0xFF);

		// TODO : Process gyro bias
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
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_CPASS_SIZE);

		MagSensor::vData.Timestamp = Timestamp;
		MagSensor::vData.X = ((int16_t)d[0] << 8) | (d[1] & 0xFF);
		MagSensor::vData.Y = ((int16_t)d[2] << 8) | (d[3] & 0xFF);
		MagSensor::vData.Z = ((int16_t)d[4] << 8) | (d[5] & 0xFF);
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
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_ALS_SIZE);

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
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_QUAT6_SIZE);

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
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_QUAT9_SIZE);

		d += ICM20948_FIFO_HEADER_QUAT9_SIZE;
		cnt += ICM20948_FIFO_HEADER_QUAT9_SIZE;
		Len -= ICM20948_FIFO_HEADER_QUAT9_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_QUAT9; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_STEP_DETECTOR)
	{
		if (Len < ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE);

		d += ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		cnt += ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		Len -= ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_STEP_DETECTOR; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_GEOMAG)
	{
		if (Len < ICM20948_FIFO_HEADER_GEOMAG_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_GEOMAG_SIZE);

		d += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		cnt += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		Len -= ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_GEOMAG; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_PRESSURE)
	{
		if (Len < ICM20948_FIFO_HEADER_PRESSURE_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_PRESSURE_SIZE);
		d += ICM20948_FIFO_HEADER_PRESSURE_SIZE;
		cnt += ICM20948_FIFO_HEADER_PRESSURE_SIZE;
		Len -= ICM20948_FIFO_HEADER_PRESSURE_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_PRESSURE; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_CALIB_CPASS)
	{
		if (Len < ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE);
		d += ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		cnt += ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		Len -= ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_CALIB_CPASS; // Clear bit
	}

	if (vFifoHdr2 != 0)
	{
		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_ACCEL_ACCUR)
		{
			if (Len < ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE)
			{
				return cnt;
			}
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE);

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
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE);

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
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE);

			d += ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			cnt += ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			Len -= ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_CPASS_ACCUR; // Clear bit
		}

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_PICKUP)
		{
			if (Len < ICM20948_FIFO_HEADER2_PICKUP_SIZE)
			{
				return cnt;
			}
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_PICKUP_SIZE);

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
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE);

			d += ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			cnt += ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			Len -= ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_ACTI_RECOG; // Clear bit
		}
	}

	if (Len < ICM20948_FIFO_FOOTER_SIZE)
	{
	//printf("Footer size\n");
		return cnt;
	}
	//Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_FOOTER_SIZE);
	if (d[0] != 0 || d[1] != 0)
	{
		//printf("bad packet %x %x\n", d[0], d[1]);
	}

	cnt += ICM20948_FIFO_FOOTER_SIZE;
	Len -= ICM20948_FIFO_FOOTER_SIZE;
	vFifoHdr &= ~ICM20948_FIFO_HEADER_FOOTER; // Clear bit

//	printf("fh %x %x\n", vFifoHdr, vFifoHdr2);

	vFifoHdr = vFifoHdr2 = 0;
	//printf("h: %x %x : %x %x %x %x\n", header, header2, pFifo[0], pFifo[1], pFifo[2], pFifo[3]);

	return cnt;
}

void AgmInvnIcm20948::IntHandler()
{
#if 0
	inv_icm20948_poll_sensor(&vIcmDevice, (void*)this, SensorEventHandler);
#else
	UpdateData();
#endif
}

void AgmInvnIcm20948::ResetFifo()
{
	uint16_t regaddr;
	uint16_t cnt;

	regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2);
	Write8((uint8_t*)&regaddr, 2, d & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN));

	do {
		regaddr = ICM20948_FIFO_RST_REG;
		Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);
		Write8((uint8_t*)&regaddr, 2, 0x1e);
		msDelay(1);

		regaddr = ICM20948_FIFO_COUNTH_REG;
		cnt = EndianCvt16(Read16((uint8_t*)&regaddr, 2)) & 0x1FFF;
	} while (cnt != 0);

	regaddr = ICM20948_USER_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, d);
}

void AgmInvnIcm20948::SensorEventHandler(void * context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
	AgmInvnIcm20948 *dev = (AgmInvnIcm20948*)context;

	dev->UpdateData(sensortype, timestamp, data, arg);
}

void AgmInvnIcm20948::UpdateData(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
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
		{
		//memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
		//memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
			float *p = (float*)data;
			GyroSensor::vData.X = p[0] * 256.0;
			GyroSensor::vData.Y = p[1] * 256.0;
			GyroSensor::vData.Z = p[2] * 256.0;
			GyroSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_GRAVITY:
		{
			memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
			event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
			float *p = (float*)data;
			AccelSensor::vData.X = p[0] * 256.0;
			AccelSensor::vData.Y = p[1] * 256.0;
			AccelSensor::vData.Z = p[2] * 256.0;
			AccelSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_LINEAR_ACCELERATION:
	case INV_ICM20948_SENSOR_ACCELEROMETER:
		{
			float *p = (float*)data;
			//memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
			//memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
			AccelSensor::vData.X = p[0] * 256.0;
			AccelSensor::vData.Y = p[1] * 256.0;
			AccelSensor::vData.Z = p[2] * 256.0;
			AccelSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD:
		{
			float *p = (float*)data;
			//memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
			//memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
			MagSensor::vData.X = p[0] * 256.0;
			MagSensor::vData.Y = p[1] * 256.0;
			MagSensor::vData.Z = p[2] * 256.0;
			MagSensor::vData.Timestamp = timestamp;
		}
		break;
	case INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR:
	case INV_ICM20948_SENSOR_ROTATION_VECTOR:
		memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
		break;
	case INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR:
	{
		uint8_t accel_accuracy;
		uint8_t gyro_accuracy;

		memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));

		accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
		gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();

		event.data.quaternion.accuracy_flag = min(accel_accuracy, gyro_accuracy);
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
	case INV_ICM20948_SENSOR_RAW_ACCELEROMETER:
	case INV_ICM20948_SENSOR_RAW_GYROSCOPE:
		memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
		break;
	default:
		return;
	}
}

int AgmInvnIcm20948::Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint16_t regaddr = REG_USER_CTRL;
		uint8_t userctrl = Read8((uint8_t*)&regaddr, 2) | BIT_I2C_MST_EN;

#if 1
		uint8_t d[4];

		regaddr = REG_I2C_SLV0_ADDR;

		d[0] = (DevAddr & 0x7f) | INV_MPU_BIT_I2C_READ;
		d[1] = *pCmdAddr;

		while (BuffLen > 0)
		{
			int cnt = min(15, BuffLen);

			d[2] = INV_MPU_BIT_SLV_EN | (cnt & 0xf);

			Write((uint8_t*)&regaddr, 2, d, 3);

			regaddr = REG_USER_CTRL;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			msDelay(60);

			Write8((uint8_t*)&regaddr, 2, userctrl & ~BIT_I2C_MST_EN);

			regaddr = REG_I2C_SLV0_DO;
			cnt = Read((uint8_t*)&regaddr, 1, pBuff, cnt);
			if (cnt <= 0)
				break;

			pBuff += cnt;
			BuffLen -= cnt;
			retval += cnt;
			d[1] += cnt;
		}
#else
		regaddr = ICM20948_I2C_SLV0_ADDR;
		Write8((uint8_t*)&regaddr, 2, (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD);

		regaddr = ICM20948_I2C_SLV0_REG;
		Write8((uint8_t*)&regaddr, 2, *pCmdAddr);

		regaddr = ICM20948_I2C_SLV0_CTRL;
		Write8((uint8_t*)&regaddr, 2, ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN | (1 & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LENG_MASK));

		regaddr = ICM20948_USER_CTRL;
		Write8((uint8_t*)&regaddr, 2, userctrl);

		msDelay(100);

		Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);
		regaddr = ICM20948_EXT_SLV_SENS_DATA_00;
		int cnt = Read((uint8_t*)&regaddr, 2, pBuff, 1);

		BuffLen -= cnt;

		regaddr = ICM20948_I2C_SLV0_CTRL;
		Write8((uint8_t*)&regaddr, 2, 0);


#endif
	}
	else
	{
		retval = vpIntrf->Read(DevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}

	return retval;
}

int AgmInvnIcm20948::Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint16_t regaddr = REG_USER_CTRL;
		uint8_t d[8];
		uint8_t userctrl = Read8((uint8_t*)&regaddr, 2) | BIT_I2C_MST_EN;

		regaddr = REG_I2C_SLV0_ADDR;
		Write8((uint8_t*)&regaddr, 2, (DevAddr & 0x7F));

		d[0] = *pCmdAddr;
		d[1] = 1;// | ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN;	// Length : Write is done 1 byte at a time

		while (DataLen > 0)
		{
			d[2] = *pData;

			regaddr = REG_I2C_SLV0_REG;
			Write((uint8_t*)&regaddr, 2, d, 3);

			regaddr = REG_USER_CTRL;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			msDelay(60);

			Write8((uint8_t*)&regaddr, 2, userctrl & ~BIT_I2C_MST_EN);

			d[0]++;
			pData++;
			DataLen--;
			retval++;
		}
	}
	else
	{
		retval = vpIntrf->Write(DevAddr, pCmdAddr, CmdAddrLen, pData, DataLen);
	}

	return retval;
}

#if 0
void sensor_event(const inv_sensor_event_t * event, void * arg){
	/* arg will contained the value provided at init time */
	(void)arg;

	/*
	* Encode sensor event and sent to host over UART through IddWrapper protocol
	*/
	static DynProtocolEdata_t async_edata; /* static to take on .bss */
	static uint8_t async_buffer[256]; /* static to take on .bss */
	uint16_t async_bufferLen;

	async_edata.sensor_id = event->sensor;
	async_edata.d.async.sensorEvent.status = DYN_PRO_SENSOR_STATUS_DATA_UPDATED;
	convert_sensor_event_to_dyn_prot_data(event, &async_edata.d.async.sensorEvent.vdata);

	if(DynProtocol_encodeAsync(&protocol,
		DYN_PROTOCOL_EID_NEW_SENSOR_DATA, &async_edata,
		async_buffer, sizeof(async_buffer), &async_bufferLen) != 0) {
			goto error_dma_buf;
	}

	DynProTransportUart_tx(&transport, async_buffer, async_bufferLen);
	return;

error_dma_buf:
	INV_MSG(INV_MSG_LEVEL_WARNING, "sensor_event_cb: encode error, frame dropped");

	return;
}
#endif

