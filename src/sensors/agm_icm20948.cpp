/**-------------------------------------------------------------------------
@file	agm_icm20948.cpp

@brief	Implementation of TDK ICM-20948 accel, gyro, mag sensor

@author	Hoang Nguyen Hoan
@date	Nov. 5, 2018

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
#include "Devices/Drivers/Icm20948/Icm20948.h"
#include "Devices/Drivers/Icm20948/Icm20948Defs.h"
#include "Devices/Drivers/Icm20948/Icm20948Dmp3Driver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseDriver.h"
#include "Devices/Drivers/Icm20948/Icm20948DataBaseControl.h"
#include "Devices/Drivers/Icm20948/Icm20948AuxTransport.h"
#include "Devices/Drivers/Icm20948/Icm20948MPUFifoControl.h"
#include "Devices/Drivers/Icm20948/Icm20948Setup.h"
#include "Devices/SensorTypes.h"

#include "istddef.h"
#include "convutil.h"
#include "idelay.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "coredev/uart.h"
#include "sensors/agm_icm20948.h"
#include "sensors/agm_icm20948DMP.h"

extern UART g_Uart;

typedef struct {
	size_t Len;
} FifoDataLen_t;

static const size_t s_FifoDataLenLookup1[] = {
	ICM20948_FIFO_HEADER_ACCEL_SIZE, ICM20948_FIFO_HEADER_GYRO_SIZE,
	ICM20948_FIFO_HEADER_CPASS_SIZE, ICM20948_FIFO_HEADER_ALS_SIZE,
	ICM20948_FIFO_HEADER_QUAT6_SIZE, ICM20948_FIFO_HEADER_QUAT9_SIZE,
	ICM20948_FIFO_HEADER_PEDO_QUAT6_SIZE, ICM20948_FIFO_HEADER_GEOMAG_SIZE,
	ICM20948_FIFO_HEADER_PRESS_TEMP_SIZE, ICM20948_FIFO_HEADER_CALIB_GYRO_SIZE,
	ICM20948_FIFO_HEADER_CALIB_CPASS_SIZE, ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE
};
static const size_t s_NbFifoDataLenLookup1 = sizeof(s_FifoDataLenLookup1) / sizeof(size_t);

static const size_t s_FifoDataLenLookup2[] = {
	ICM20948_FIFO_HEADER2_ACCEL_ACCUR_SIZE, ICM20948_FIFO_HEADER2_GYRO_ACCUR_SIZE,
	ICM20948_FIFO_HEADER2_CPASS_ACCUR_SIZE, ICM20948_FIFO_HEADER2_FSYNC_SIZE,
	ICM20948_FIFO_HEADER2_PICKUP_SIZE, 0, 0, ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE,
	2
};
static const size_t s_NbFifoDataLenLookup2 = sizeof(s_FifoDataLenLookup2) / sizeof(size_t);

static const uint8_t s_Dmp3Image[] = {
#include "imu/icm20948_img_dmp3a.h"
};
static const int s_DmpImageSize = sizeof(s_Dmp3Image);

static const float s_CfgMountingMatrix[9]= {
	1.f, 0, 0,
	0, 1.f, 0,
	0, 0, 1.f
};

inv_icm20948_t vIcmDevice;
int InvnReadReg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	AgmIcm20948 *dev = (AgmIcm20948*)context;
//	return spi_master_transfer_rx(NULL, reg, rbuffer, rlen);
//	reg |= 0x80;
	int cnt = dev->Read(&reg, 1, rbuffer, (int)rlen);

	return cnt > 0 ? 0 : 1;
}

int InvnWriteReg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	AgmIcm20948 *dev = (AgmIcm20948*)context;
//	return spi_master_transfer_tx(NULL, reg, wbuffer, wlen);

	int cnt = dev->Write(&reg, 1, (uint8_t*)wbuffer, (int)wlen);

	return cnt > 0 ? 0 : 1;
}

bool AgmIcm20948::Init(uint32_t DevAddr, DeviceIntrf * const pIntrf, uint8_t Inter, DEVINTR_POL IntPol, Timer * const pTimer)
{
	if (Valid())
		return true;;

	if (pIntrf == NULL)
		return false;

	Interface(pIntrf);

	vCurrBank = -1;

	// Read chip id
	uint16_t regaddr = ICM20948_WHO_AM_I_REG;
	uint8_t d;

	if (DevAddr == ICM20948_I2C_DEV_ADDR0 || DevAddr == ICM20948_I2C_DEV_ADDR1)
	{
		if (pIntrf->Type() != DEVINTRF_TYPE_I2C)
		{
			// Device address is set to I2C but interface is not
			return false;
		}

		DeviceAddress(DevAddr);
		d = Read8((uint8_t*)&regaddr, 2);

		if (d != ICM20948_WHO_AM_I_ID)
		{
			return false;
		}
	}
	else if (pIntrf->Type() == DEVINTRF_TYPE_I2C)
	{
		// Interface is I2C but device address is not set.
		// Detect device
		DeviceAddress(ICM20948_I2C_DEV_ADDR0);
		d = Read8((uint8_t*)&regaddr, 2);

		if (d != ICM20948_WHO_AM_I_ID)
		{
			// Try alternate address
			DeviceAddress(ICM20948_I2C_DEV_ADDR1);
			d = Read8((uint8_t*)&regaddr, 2);
			if (d != ICM20948_WHO_AM_I_ID)
			{
				return false;
			}
		}
	}
	else
	{
		// Not I2C interface
		DeviceAddress(DevAddr);
		d = Read8((uint8_t*)&regaddr, 2);

		if (d != ICM20948_WHO_AM_I_ID)
		{
			return false;
		}
	}

	// Device found, save device id
	DeviceID(d);

	if (pTimer != NULL)
	{
		vpTimer = pTimer;
	}

	Valid(true);

	Reset();

	struct inv_icm20948_serif icm20948_serif;

	//inv_icm20948_reset_states(&vIcmDevice, &icm20948_serif);
	memset(&vIcmDevice, 0, sizeof(inv_icm20948_t));
	vIcmDevice.serif.context   = this;
	vIcmDevice.serif.read_reg  = InvnReadReg;
	vIcmDevice.serif.write_reg = InvnWriteReg;
	vIcmDevice.serif.max_read  = 16; /* maximum number of bytes allowed per serial read */
	vIcmDevice.serif.max_write = 16; /* maximum number of bytes allowed per serial write */
	vIcmDevice.serif.is_spi = vpIntrf->Type() == DEVINTRF_TYPE_SPI;

	//inv_icm20948_register_aux_compass(&vIcmDevice, INV_ICM20948_COMPASS_ID_AK09916, (uint8_t)AK0991x_DEFAULT_I2C_ADDR);
	vIcmDevice.secondary_state.compass_slave_id = HW_AK09916;

#define AK0991x_DEFAULT_I2C_ADDR	0x0C	/* The default I2C address for AK0991x Magnetometers */

	vIcmDevice.secondary_state.compass_chip_addr = AK0991x_DEFAULT_I2C_ADDR;
	vIcmDevice.secondary_state.compass_state = INV_ICM20948_COMPASS_INITED;
	/* initialise mounting matrix of compass to identity akm9916 */
	vIcmDevice.mounting_matrix_secondary_compass[0] = 1 ;
	vIcmDevice.mounting_matrix_secondary_compass[4] = -1;
	vIcmDevice.mounting_matrix_secondary_compass[8] = -1;

	msDelay(500);

	// Setup accel and gyro mounting matrix and associated angle for current board
	inv_icm20948_init_matrix(&vIcmDevice);

	for (int i = 0; i < INV_ICM20948_SENSOR_MAX; i++) {
		inv_icm20948_set_matrix(&vIcmDevice, s_CfgMountingMatrix, (inv_icm20948_sensor)i);
	}

#if 1
#if 0
//	inv_icm20948_initialize(&vIcmDevice, s_Dmp3Image, sizeof(s_Dmp3Image));

	if(vIcmDevice.serif.is_spi) {
		/* Hardware initialization */
		// No image to be loaded from flash, no pointer to pass.
		if (inv_icm20948_initialize_lower_driverx(&vIcmDevice, SERIAL_INTERFACE_SPI, s_Dmp3Image, ICM20948_DMP_CODE_SIZE)) {
			return false;
		}
	}
	else {
		/* Hardware initialization */
		// No image to be loaded from flash, no pointer to pass.
		if (inv_icm20948_initialize_lower_driverx(&vIcmDevice, SERIAL_INTERFACE_I2C, s_Dmp3Image, ICM20948_DMP_CODE_SIZE)) {
			return false;
		}
	}

#else
	{
		int result = 0;
		static unsigned char data;
		// set static variable
		vIcmDevice.sAllowLpEn = 1;
		vIcmDevice.s_compass_available = 0;
		// ICM20948 do not support the proximity sensor for the moment.
		// s_proximity_available variable is nerver changes
		vIcmDevice.s_proximity_available = 0;

		// Set varialbes to default values
		memset(&vIcmDevice.base_state, 0, sizeof(vIcmDevice.base_state));
		vIcmDevice.base_state.pwr_mgmt_1 = ICM20948_PWR_MGMT_1_CLKSEL_AUTO;
		vIcmDevice.base_state.pwr_mgmt_2 = ICM20948_PWR_MGMT_2_DISABLE_ALL;//BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY | BIT_PWR_PRESSURE_STBY;
		vIcmDevice.base_state.serial_interface = vIcmDevice.serif.is_spi ? SERIAL_INTERFACE_SPI : SERIAL_INTERFACE_I2C;
		//result |= inv_icm20948_read_mems_reg(&vIcmDevice, REG_USER_CTRL, 1, &vIcmDevice.base_state.user_ctrl);

		vIcmDevice.base_state.user_ctrl = 0;

		//result |= inv_icm20948_wakeup_mems(&vIcmDevice);
		regaddr = ICM20948_PWR_MGMT_1_REG;
		Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_CLKSEL_AUTO);

		regaddr = ICM20948_PWR_MGMT_2_REG;
		Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_2_DISABLE_ALL);

		//result |= inv_icm20948_read_mems_reg(&vIcmDevice, REG_WHO_AM_I, 1, &data);

		/* secondary cycle mode should be set all the time */
		data = BIT_I2C_MST_CYCLE|BIT_ACCEL_CYCLE|BIT_GYRO_CYCLE;

		// Set default mode to low power mode
		result |= inv_icm20948_set_lowpower_or_highperformance(&vIcmDevice, 0);

		// Disable Ivory DMP.
		if(vIcmDevice.base_state.serial_interface == SERIAL_INTERFACE_SPI)
			vIcmDevice.base_state.user_ctrl = ICM20948_USER_CTRL_I2C_IF_DIS;
		else
			vIcmDevice.base_state.user_ctrl = 0;

//		result |= inv_icm20948_write_single_mems_reg(&vIcmDevice, REG_USER_CTRL, vIcmDevice.base_state.user_ctrl);

		regaddr = ICM20948_USER_CTRL_REG;
		Write8((uint8_t*)&regaddr, 2, vIcmDevice.base_state.user_ctrl);
#if 1
		bool res = InitDMP(ICM20948_DMP_PROG_START_ADDR, s_Dmp3Image, ICM20948_DMP_CODE_SIZE);
		if (res)
#else
		//Setup Ivory DMP.
		result |= inv_icm20948_load_firmware(&vIcmDevice, s_Dmp3Image, ICM20948_DMP_CODE_SIZE);
		if(result)
			return result;
		else
#endif
			vIcmDevice.base_state.firmware_loaded = 1;
		//result |= inv_icm20948_set_dmp_address(&vIcmDevice);
		// Turn off all sensors on DMP by default.
		//result |= dmp_set_data_output_control1(0);   // FIXME in DMP, these should be off by default.
//		result |= dmp_icm20948_reset_control_registers(&vIcmDevice);
/*
		unsigned char dat[4]={0};

		//reset data output control registers
		result = inv_icm20948_write_memsx(&vIcmDevice, ICM20948_DMP_DATA_OUT_CTL1, 2, &dat[0]);
		result += inv_icm20948_write_memsx(&vIcmDevice, ICM20948_DMP_DATA_OUT_CTL2, 2, &dat[0]);

		//reset data interrupt control register
		result += inv_icm20948_write_memsx(&vIcmDevice, ICM20948_DMP_DATA_INTR_CTL, 2, &dat[0]);

		//reset motion event control register
		result += inv_icm20948_write_memsx(&vIcmDevice, ICM20948_DMP_MOTION_EVENT_CTL, 2, &dat[0]);

		//reset data ready status register
		result += inv_icm20948_write_memsx(&vIcmDevice, ICM20948_DMP_DATA_RDY_STATUS, 2, &dat[0]);

		//if((power_state & CHIP_AWAKE) == 0)   // Wake up chip since it is asleep
		//	result = inv_icm20948_set_chip_power_state(&vIcmDevice, CHIP_AWAKE, 1);

		//result |= inv_icm20948_set_chip_power_state(&vIcmDevice, CHIP_LP_ENABLE, 0);
*/
		ResetDMPCtrlReg();
		vIcmDevice.lLastBankSelected = -1;

		// set FIFO watermark to 80% of actual FIFO size
//		result |= dmp_icm20948_set_FIFO_watermark(&vIcmDevice, 800);
		// Fifo watermark 80%
		uint16_t val = EndianCvt16(800);
		WriteDMP(ICM20948_DMP_FIFO_WATERMARK, (uint8_t*)&val, 2);

		// Enable Interrupts.
		data = 0x2;
//		result |= inv_icm20948_write_mems_reg(&vIcmDevice, REG_INT_ENABLE, 1, &data); // Enable DMP Interrupt

		data = 0x1;
		//result |= inv_icm20948_write_mems_reg(&vIcmDevice, REG_INT_ENABLE_2, 1, &data); // Enable FIFO Overflow Interrupt

		// TRACKING : To have accelerometers datas and the interrupt without gyro enables.
		data = 0XE4;
		//result |= inv_icm20948_write_mems_reg(&vIcmDevice, REG_SINGLE_FIFO_PRIORITY_SEL, 1, &data);
		// Undocumented value
		regaddr = ICM20948_SINGLE_FIFO_PRIORITY_SEL;
		Write8((uint8_t*)&regaddr, 2, ICM20948_SINGLE_FIFO_PRIORITY_SEL_0XE4);

		// Disable HW temp fix
		inv_icm20948_read_mems_reg(&vIcmDevice, REG_HW_FIX_DISABLE,1,&data);
		data |= 0x08;
		inv_icm20948_write_mems_reg(&vIcmDevice, REG_HW_FIX_DISABLE,1,&data);

		// Setup MEMs properties.
		vIcmDevice.base_state.accel_averaging = 1; //Change this value if higher sensor sample avergaing is required.
		vIcmDevice.base_state.gyro_averaging = 1;  //Change this value if higher sensor sample avergaing is required.
		inv_icm20948_set_gyro_divider(&vIcmDevice, FIFO_DIVIDER);       //Initial sampling rate 1125Hz/19+1 = 56Hz.
		//inv_icm20948_set_accel_divider(&vIcmDevice, FIFO_DIVIDER);      //Initial sampling rate 1125Hz/19+1 = 56Hz.

		// Init the sample rate to 56 Hz for BAC,STEPC and B2S
		dmp_icm20948_set_bac_rate(&vIcmDevice, DMP_ALGO_FREQ_56);
		dmp_icm20948_set_b2s_rate(&vIcmDevice, DMP_ALGO_FREQ_56);

		// FIFO Setup.
		result |= inv_icm20948_write_single_mems_reg(&vIcmDevice, REG_FIFO_CFG, BIT_SINGLE_FIFO_CFG); // FIFO Config. fixme do once? burst write?
		result |= inv_icm20948_write_single_mems_reg(&vIcmDevice, REG_FIFO_RST, 0x1f); // Reset all FIFOs.
		result |= inv_icm20948_write_single_mems_reg(&vIcmDevice, REG_FIFO_RST, 0x1e); // Keep all but Gyro FIFO in reset.
		result |= inv_icm20948_write_single_mems_reg(&vIcmDevice, REG_FIFO_EN, 0x0); // Slave FIFO turned off.
		result |= inv_icm20948_write_single_mems_reg(&vIcmDevice, REG_FIFO_EN_2, 0x0); // Hardware FIFO turned off.

		vIcmDevice.base_state.lp_en_support = 1;

		//if(vIcmDevice.base_state.lp_en_support == 1)
			inv_icm20948_set_chip_power_state(&vIcmDevice, CHIP_LP_ENABLE, 1);

		result |= inv_icm20948_sleep_mems(&vIcmDevice);

//		return true;
	}
#endif
	vbDmpEnabled = true;

#else
	uint8_t userctrl = ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
	uint8_t lpconfig = 0;//ICM20948_LP_CONFIG_ACCEL_CYCLE | ICM20948_LP_CONFIG_GYRO_CYCLE;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		// in SPI mode, use i2c master mode to access Mag device (AK09916)
		userctrl |= ICM20948_USER_CTRL_I2C_IF_DIS;
		lpconfig |= ICM20948_LP_CONFIG_I2C_MST_CYCLE;
	}


	// NOTE : require delay for reset to stabilize
	// the chip would not respond properly to motion detection
	msDelay(50);

	regaddr = ICM20948_PWR_MGMT_1_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_CLKSEL_AUTO);

	regaddr = ICM20948_PWR_MGMT_2_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_2_DISABLE_ALL);

	regaddr = ICM20948_USER_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, userctrl);

	regaddr = ICM20948_ODR_ALIGN_EN_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_ODR_ALIGN_EN_ODR_ALIGN_EN);

	// Upload DMP
	InitDMP(ICM20948_DMP_PROG_START_ADDR, s_Dmp3Image, ICM20948_DMP_CODE_SIZE);

	ResetDMPCtrlReg();
	ResetFifo();

	regaddr = ICM20948_HWTEMP_FIX_DISABLE_REG;
	d = Read8((uint8_t*)&regaddr, 2) | 0x08;
	Write8((uint8_t*)&regaddr, 2, d);//ICM20948_HWTEMP_FIX_DISABLE_DIS);

	// Fifo watermark 80%
	uint16_t val = EndianCvt16(800);
	WriteDMP(ICM20948_DMP_FIFO_WATERMARK, (uint8_t*)&val, 2);

	regaddr = ICM20948_FIFO_CFG_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_CFG_SINGLE);

	// Undocumented value
	regaddr = ICM20948_SINGLE_FIFO_PRIORITY_SEL;
	Write8((uint8_t*)&regaddr, 2, ICM20948_SINGLE_FIFO_PRIORITY_SEL_0XE4);
#endif

	// ICM20948 has only 1 interrupt pin. Don't care the value
	if (Inter)
	{
		regaddr = ICM20948_INT_PIN_CFG_REG;

		if (IntPol == DEVINTR_POL_HIGH)
		{
			d = 0;
		}
		else
		{
			d = 0 | ICM20948_INT_PIN_CFG_INT1_ACTL;
		}
		Write8((uint8_t*)&regaddr, 2, d);

		regaddr = ICM20948_INT_ENABLE_REG;
		d = ICM20948_INT_ENABLE_DMP_INT1_EN;
		Write8((uint8_t*)&regaddr, 2, d);

		//regaddr = ICM20948_INT_ENABLE_1_REG;
		//d = ICM20948_INT_ENABLE_1_RAW_DATA_0_DRY_EN;
		//Write8((uint8_t*)&regaddr, 2, d);

		regaddr = ICM20948_INT_ENABLE_2_REG;
		Write8((uint8_t*)&regaddr, 2, ICM20948_INT_ENABLE_2_FIFO_OVERFLOW_EN);

		regaddr = ICM20948_INT_ENABLE_3_REG;
		Write8((uint8_t*)&regaddr, 2, ICM20948_INT_ENABLE_3_FIFO_WM_EN);

	}
	vIcmDevice.lLastBankSelected = -1;

	return true;
}

bool AccelIcm20948::Init(const AccelSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint16_t regaddr;
	uint8_t d[10];

	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
		return false;

	vData.Range = Range(ICM20948_ACC_ADC_RANGE);

	SamplingFrequency(Cfg.Freq);
	Scale(Cfg.Scale);
	FilterFreq(Cfg.FltrFreq);

	// Read manufacture trim offset
	regaddr = ICM20948_XA_OFFS_H_REG;
	Device::Read((uint8_t*)&regaddr, 2, d, 6);

	float offs[3];

	offs[0] = (float)(((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF)) * ICM20948_ACCEL_TRIM_SCALE;
	offs[1] = (float)(((int16_t)d[2] << 8) | ((int16_t)d[3] & 0xFF)) * ICM20948_ACCEL_TRIM_SCALE;
	offs[2] = (float)(((int16_t)d[4] << 8) | ((int16_t)d[5] & 0xFF)) * ICM20948_ACCEL_TRIM_SCALE;

	SetCalibrationOffset(offs);

	return true;
}

uint16_t AccelIcm20948::Scale(uint16_t Value)
{
	uint16_t regaddr = ICM20948_ACCEL_CONFIG_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_MASK;
	int32_t scale, scale2;

	if (Value < 3)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_2G;
		Value = 2;
		scale = (1 << 25);  // 33554432L
		scale2 = (1 << 19);	// 524288L
	}
	else if (Value < 6)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_4G;
		Value = 4;
		scale =  (1 << 26);	// 67108864L
		scale2 = (1 << 18);	// 262144L
	}
	else if (Value < 12)
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_8G;
		Value = 8;
		scale = (1 << 27);  // 134217728L
		scale2 = (1 << 17);	// 131072L
	}
	else
	{
		d |= ICM20948_ACCEL_CONFIG_ACCEL_FS_SEL_16G;
		Value = 16;
		scale = (1 << 28);  // 268435456L
		scale2 = (1 << 16);	// 65536L
	}

	Write8((uint8_t*)&regaddr, 2, d);

	/**
	* Sets scale in DMP to convert accel data to 1g=2^25 regardless of fsr.
	* @param[in] fsr for accel parts
	2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.

	For 2g parts, 2g = 2^15 -> 1g = 2^14,.
	DMP takes raw accel data and left shifts by 16 bits, so 1g=2^14 (<<16) becomes 1g=2^30, to make 1g=2^25, >>5 bits.
	In Q-30 math, >> 5 equals multiply by 2^25 = 33554432.

	For 8g parts, 8g = 2^15 -> 1g = 2^12.
	DMP takes raw accel data and left shifts by 16 bits, so 1g=2^12 (<<16) becomes 1g=2^28, to make 1g=2^25, >>3bits.
	In Q-30 math, >> 3 equals multiply by 2^27 = 134217728.
	*/
	regaddr = ICM20948_DMP_ACC_SCALE;
	scale = EndianCvt32(scale);
	((AgmIcm20948*)this)->WriteDMP(regaddr, (uint8_t*)&scale, 4);

	/**
	* According to input fsr, a scale factor will be set at memory location ACC_SCALE2
	* to convert calibrated accel data to 16-bit format same as what comes out of MPU register.
	* It is a reverse scaling of the scale factor written to ACC_SCALE.
	* @param[in] fsr for accel parts
	2: 2g. 4: 4g. 8: 8g. 16: 16g. 32: 32g.
	*/
	regaddr = ICM20948_DMP_ACC_SCALE2;
	scale2 = EndianCvt32(scale2);
	((AgmIcm20948*)this)->WriteDMP(regaddr, (uint8_t*)&scale2, 4);

	return AccelSensor::Scale(Value);
}

uint32_t AccelIcm20948::SamplingFrequency(uint32_t Freq)
{
	// ODR = 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0])

	uint32_t div = (1125000 + (Freq >> 1))/ Freq - 1;
	div = EndianCvt16(div);

	uint16_t regaddr = ICM20948_ACCEL_SMPLRT_DIV_1_REG;
	Write16((uint8_t*)&regaddr, 2, div);

	return AccelSensor::SamplingFrequency(1125000 / (1 + div));
}

uint32_t AccelIcm20948::FilterFreq(uint32_t Freq)
{
	uint16_t regaddr = ICM20948_ACCEL_CONFIG_REG;
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

bool AccelIcm20948::Enable()
{
	uint16_t regaddr = ICM20948_FIFO_EN_2_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) | ICM20948_FIFO_EN_2_ACCEL_FIFO_EN;

	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_PWR_MGMT_2_REG;
	d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK;

	Write8((uint8_t*)&regaddr, 2, d);


	return true;
}

bool GyroIcm20948::Init(const GyroSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
		return false;

	vData.Range = Range(ICM20948_GYRO_ADC_RANGE);
	SamplingFrequency(Cfg.Freq);
	Sensitivity(Cfg.Sensitivity);
	FilterFreq(Cfg.FltrFreq);

	// Read manufacture trim offset
	uint16_t regaddr = ICM20948_XG_OFFS_USRH_REG;
	uint8_t d[10];
	Device::Read((uint8_t*)&regaddr, 2, d, 6);

	float offs[3];

	offs[0] = (float)(((int16_t)d[0] << 8) | ((int16_t)d[1] & 0xFF)) * ICM20948_GYRO_TRIM_SCALE;
	offs[1] = (float)(((int16_t)d[2] << 8) | ((int16_t)d[3] & 0xFF)) * ICM20948_GYRO_TRIM_SCALE;
	offs[2] = (float)(((int16_t)d[4] << 8) | ((int16_t)d[5] & 0xFF)) * ICM20948_GYRO_TRIM_SCALE;

	SetCalibrationOffset(offs);

	return true;
}

uint32_t GyroIcm20948::Sensitivity(uint32_t Value)
{
	uint16_t regaddr = ICM20948_GYRO_CONFIG_1_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_MASK;
	int32_t scale;

	if (Value < 325)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_250DPS;
		Value = 250;
		scale = (1 << 25); // 33554432L
	}
	else if (Value < 750)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_500DPS;
		Value = 500;
		scale = (1 << 26); // 67108864L
	}
	else if (Value < 1500)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_1000DPS;
		Value = 1000;
		scale = (1 << 27); // 134217728L
	}
	else
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_2000DPS;
		Value = 2000;
		scale = (1 << 28); // 268435456L
	}

	Write8((uint8_t*)&regaddr, 2, d);

	/**
	* Sets scale in DMP to convert gyro data to 4000dps=2^30 regardless of fsr.
	* @param[in] fsr for gyro parts
	4000: 4000dps. 2000: 2000dps. 1000: 1000dps. 500: 500dps. 250: 250dps.

	For 4000dps parts, 4000dps = 2^15.
	DMP takes raw gyro data and left shifts by 16 bits, so (<<16) becomes 4000dps=2^31, to make 4000dps=2^30, >>1 bit.
	In Q-30 math, >> 1 equals multiply by 2^29 = 536870912.

	For 2000dps parts, 2000dps = 2^15.
	DMP takes raw gyro data and left shifts by 16 bits, so (<<16) becomes 2000dps=2^31, to make 4000dps=2^30, >>2 bits.
	In Q-30 math, >> 2 equals multiply by 2^28 = 268435456.
	*/
	regaddr = ICM20948_DMP_GYRO_FULLSCALE;
	scale = EndianCvt32(scale);
	((AgmIcm20948*)this)->WriteDMP(regaddr, (uint8_t*)&scale, 4);

	//inv_icm20948_set_gyro_fullscale(&vIcmDevice, d);

	return GyroSensor::Sensitivity(Value);
}

uint32_t GyroIcm20948::SamplingFrequency(uint32_t Freq)
{
	// ODR = 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])

	uint32_t div = (1100000 + (Freq >> 1)) / Freq - 1;
	uint16_t regaddr = ICM20948_GYRO_SMPLRT_DIV_REG;
	Write8((uint8_t*)&regaddr, 2, div);

	// gyro_level should be set to 4 regardless of fullscale, due to the addition of API dmp_icm20648_set_gyro_fsr()
	// 4 = ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_1000DPS
	uint8_t tbpll;
	regaddr = ICM20948_TIMEBASE_CORRECTION_PLL_REG;
	tbpll = Read8((uint8_t*)&regaddr, 2);

	const uint64_t MagicConstant = 264446880937391ULL;
	const uint64_t MagicConstantScale = 100000ULL;
	uint64_t res;
	int32_t gyrosf;

	if (tbpll & 0x80)
	{
		res = (MagicConstant * (long long)(1ULL << ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_1000DPS) * (1 + div) / (1270 - (tbpll & 0x7F)) / MagicConstantScale);
	}
	else
	{
		res = (MagicConstant * (long long)(1ULL << ICM20948_GYRO_CONFIG_1_GYRO_FS_SEL_1000DPS) * (1 + div) / (1270 + tbpll) / MagicConstantScale);
	}

	/*
	    In above deprecated FP version, worst case arguments can produce a result that overflows a signed long.
	    Here, for such cases, we emulate the FP behavior of setting the result to the maximum positive value, as
	    the compiler's conversion of a u64 to an s32 is simple truncation of the u64's high half, sadly....
	*/
	if  (res > 0x7FFFFFFF)
		gyrosf = EndianCvt32(0x7FFFFFFF);
	else
		gyrosf = EndianCvt32((int32_t)res);

	regaddr = ICM20948_DMP_GYRO_SF;
	((AgmIcm20948*)this)->WriteDMP(regaddr, (uint8_t*)&gyrosf, 4);

	return GyroSensor::SamplingFrequency(1100000 / (div + 1));
}

uint32_t GyroIcm20948::FilterFreq(uint32_t Freq)
{
	uint16_t regaddr = ICM20948_GYRO_CONFIG_1_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_MASK | ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE);

	if (Freq == 0)
	{
		Freq = 12316000;
	}
	else if (Freq < 11000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (6 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 8900;	// NBW
	}
	else if (Freq < 23000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (5 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 17800;
	}
	else if (Freq < 50000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (4 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 35900;
	}
	else if (Freq < 110000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (3 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 73300;
	}
	else if (Freq < 150000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (2 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 154300;
	}
	else if (Freq < 190000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (1 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 187600;
	}
	else if (Freq < 360000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (0 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 229800;
	}
	else if (Freq < 1000000)
	{
		d |= ICM20948_GYRO_CONFIG_1_GYRO_FCHOICE | (7 << ICM20948_GYRO_CONFIG_1_GYRO_DLPFCFG_BITPOS);
		Freq = 376500;
	}
	else
	{
		Freq = 12316000;
	}

	Write8((uint8_t*)&regaddr, 2, d);

	return GyroSensor::FilterFreq(Freq);
}

bool GyroIcm20948::Enable()
{
	uint16_t regaddr = ICM20948_PWR_MGMT_2_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK;

	Write8((uint8_t*)&regaddr, 2, d);

	return true;
}

bool MagIcm20948::Init(const MagSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	uint16_t regaddr;
	uint8_t d;

	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
		return false;

	msDelay(200);

	regaddr = ICM20948_I2C_MST_CTRL_REG;
	d = ICM20948_I2C_MST_CTRL_I2C_MST_P_NSR | 7;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_I2C_MST_ODR_CONFIG_REG;
	Write8((uint8_t*)&regaddr, 2, 4);

	if (MagAk09916::Init(Cfg, pIntrf, pTimer) == false)
	{
		return false;
	}

	float g[3][3] = {
		1.0, 0.0, 0.0,
		0.0, -1.0, 0.0,
		0.0, 0.0, -1.0 };
	float offs[3] = {0.0, 0.0, 0.0};

	SetCalibration(g, offs);

//	regaddr = ICM20948_FIFO_EN_1;
//	d = Read8((uint8_t*)&regaddr, 2) | ICM20948_FIFO_EN_1_SLV_0_FIFO_EN;
//	Write8((uint8_t*)&regaddr, 2, d);


	return true;
}

AgmIcm20948::AgmIcm20948()
{
	vbDmpEnabled = false;
	vbSensorEnabled[0] = vbSensorEnabled[1] = vbSensorEnabled[2] = false;
	vType = SENSOR_TYPE_TEMP | SENSOR_TYPE_ACCEL | SENSOR_TYPE_GYRO | SENSOR_TYPE_MAG;
	vFifoHdr = vFifoHdr2 = 0;
	vFifoDataLen = 0;
}

bool AgmIcm20948::Enable()
{
	uint8_t fifoen = 0;
	uint8_t d, userctrl;
	uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;
#if 0
	regaddr = ICM20948_USER_CTRL_REG;
	userctrl = Read8((uint8_t*)&regaddr, 2);
	Write8((uint8_t*)&regaddr, 2, userctrl & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN));

	ResetFifo();

	regaddr = ICM20948_PWR_MGMT_1_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_CLKSEL_AUTO);

	msDelay(100);

	regaddr = ICM20948_PWR_MGMT_2_REG;
	d = Read8((uint8_t*)&regaddr, 2);

	uint16_t dout = ICM20948_DMP_QUAT6_SET | ICM20948_DMP_QUAT9_SET | ICM20948_DMP_PRESSURE_SET;

	if (vbSensorEnabled[ICM20948_ACCEL_IDX])
	{
//		AccelIcm20948::Enable();
		d &= ~ICM20948_PWR_MGMT_2_DISABLE_ACCEL_MASK;
		fifoen |= ICM20948_FIFO_EN_2_ACCEL_FIFO_EN;

		dout |= ICM20948_DMP_ACCEL_SET;
	}

	if (vbSensorEnabled[ICM20948_GYRO_IDX])
	{
		d &= ~ICM20948_PWR_MGMT_2_DISABLE_GYRO_MASK;
		fifoen |= (ICM20948_FIFO_EN_2_GYRO_X_FIFO_EN | ICM20948_FIFO_EN_2_GYRO_Y_FIFO_EN |
				   ICM20948_FIFO_EN_2_GYRO_Z_FIFO_EN);
		dout |= ICM20948_DMP_GYRO_SET;
	}
	regaddr = ICM20948_FIFO_EN_2_REG;
	Write8((uint8_t*)&regaddr, 2, fifoen);

	regaddr = ICM20948_PWR_MGMT_2_REG;
	Write8((uint8_t*)&regaddr, 2, d);

	if (vbSensorEnabled[ICM20948_MAG_IDX])
	{
		regaddr = ICM20948_FIFO_EN_1_REG;
//		Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_EN_1_SLV_0_FIFO_EN);
		dout |= ICM20948_DMP_CPASS_SET;
	}


	regaddr = ICM20948_USER_CTRL_REG;
	//d = Read8((uint8_t*)&regaddr, 2);
	userctrl |= ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
	Write8((uint8_t*)&regaddr, 2, userctrl);


#endif
	uint16_t dout = ICM20948_DMP_QUAT6_SET | ICM20948_DMP_QUAT9_SET | ICM20948_DMP_PRESSURE_SET | ICM20948_DMP_ACCEL_SET | ICM20948_DMP_GYRO_SET;

	dout = EndianCvt16(dout);
	regaddr = ICM20948_DMP_DATA_OUT_CTL1;
	WriteDMP(regaddr, (uint8_t*)&dout, 2);

	int i = INV_SENSOR_TYPE_MAX;

	while(i-- > 0) {
		inv_icm20948_enable_sensor(&vIcmDevice, (inv_icm20948_sensor)i, 1);
	}


	return true;
}

void AgmIcm20948::Disable()
{
	uint16_t regaddr;
	uint8_t d;

	// Disable Accel & Gyro
	regaddr = ICM20948_PWR_MGMT_2_REG;
	d = ICM20948_PWR_MGMT_2_DISABLE_ALL;
	Write8((uint8_t*)&regaddr, 2, d);

	regaddr = ICM20948_USER_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_LP_CONFIG_REG;
	Write8((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_PWR_MGMT_1_REG;
	d = ICM20948_PWR_MGMT_1_TEMP_DIS | ICM20948_PWR_MGMT_1_CLKSEL_STOP | ICM20948_PWR_MGMT_1_LP_EN;// | ICM20948_PWR_MGMT_1_SLEEP;// | ICM20948_PWR_MGMT_1_CLKSEL_STOP;
	Write8((uint8_t*)&regaddr, 2, d);
}

void AgmIcm20948::PowerOff()
{
	uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;
	uint8_t d = ICM20948_PWR_MGMT_1_TEMP_DIS | ICM20948_PWR_MGMT_1_SLEEP | ICM20948_PWR_MGMT_1_CLKSEL_STOP;
	Write8((uint8_t*)&regaddr, 2, d);
}

void AgmIcm20948::Reset()
{
	uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;

	Write8((uint8_t*)&regaddr, 2, ICM20948_PWR_MGMT_1_DEVICE_RESET);
//	regaddr = ICM20948_FIFO_RST_REG;
//	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);

}

void AgmIcm20948::ResetDMPCtrlReg()
{
	uint16_t regaddr = ICM20948_DMP_DATA_OUT_CTL1;
	uint8_t d[2] = {0, 0};

	WriteDMP(ICM20948_DMP_DATA_OUT_CTL1, d, 2);
	WriteDMP(ICM20948_DMP_DATA_OUT_CTL2, d, 2);
	WriteDMP(ICM20948_DMP_DATA_INTR_CTL, d, 2);
	WriteDMP(ICM20948_DMP_MOTION_EVENT_CTL, d, 2);
	WriteDMP(ICM20948_DMP_DATA_RDY_STATUS, d, 2);
}


bool AgmIcm20948::StartSampling()
{
	return true;
}

// Implement wake on motion
bool AgmIcm20948::WakeOnEvent(bool bEnable, int Threshold)
{
    uint16_t regaddr = ICM20948_INT_ENABLE_REG;
    uint8_t d = Read8((uint8_t*)&regaddr, 2);

	if (bEnable == true)
	{
	    Write8((uint8_t*)&regaddr, 2, d | ICM20948_INT_ENABLE_WOM_INT_EN);

	    regaddr = ICM20948_ACCEL_WOM_THR_REG;
	    Write8((uint8_t*)&regaddr, 2, Threshold);

	    regaddr = ICM20948_ACCEL_INTEL_CTRL_REG;
	    Write8((uint8_t*)&regaddr, 2, ICM20948_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN |
	    							  ICM20948_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE_CMPPREV);
	}
	else
	{
	    Write8((uint8_t*)&regaddr, 2, d & ~ICM20948_INT_ENABLE_WOM_INT_EN);

	    regaddr = ICM20948_ACCEL_INTEL_CTRL_REG;
	    Write8((uint8_t*)&regaddr, 2, 0);
	}

	return true;
}
/*
// Accel low pass frequency
uint32_t AgmIcm20948::FilterFreq(uint32_t Freq)
{
	return AccelSensor::FilterFreq();
}

// Accel scale
uint16_t AgmIcm20948::Scale(uint16_t Value)
{
	return AccelSensor::Scale();
}

// Gyro scale
uint32_t AgmIcm20948::Sensitivity(uint32_t Value)
{

	return GyroSensor::Sensitivity();
}
*/
void AgmIcm20948::ResetFifo(void)
{
	uint16_t regaddr;
	uint16_t cnt;

	regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2);
	Write8((uint8_t*)&regaddr, 2, d & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN));

	do {
		regaddr = ICM20948_FIFO_RST_REG;
		Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);
		Write8((uint8_t*)&regaddr, 2, ~ICM20948_FIFO_RST_FIFO_RESET_MASK);//0x1e);
		msDelay(1);

		regaddr = ICM20948_FIFO_COUNTH_REG;
		cnt = EndianCvt16(Read16((uint8_t*)&regaddr, 2)) & 0x1FFF;
	} while (cnt != 0);

	regaddr = ICM20948_INT_STATUS_2_REG;
	Write16((uint8_t*)&regaddr, 2, 0);

	regaddr = ICM20948_USER_CTRL_REG;
	Write8((uint8_t*)&regaddr, 2, d);
}

size_t AgmIcm20948::CalcFifoPacketSize(uint16_t Header, uint16_t Mask, const size_t *pLookup, size_t LookupSize)
{
	size_t cnt = 0;
	uint16_t bit = Mask;

	for (int i = 0; i < LookupSize && Mask != 0; i++)
	{
		if (Header & Mask)
		{
			cnt += pLookup[i];
		}
		Mask >>= 1;
	}

	return cnt;
}

size_t AgmIcm20948::ProcessDMPFifo(uint8_t *pFifo, size_t Len, uint64_t Timestamp)
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
		uint16_t bias[3];

		bias[0] = ((uint16_t)d[6] << 8) | ((uint16_t)d[7] & 0xFF);
		bias[1] = ((uint16_t)d[8] << 8) | ((uint16_t)d[9] & 0xFF);
		bias[2] = ((uint16_t)d[10] << 8) | ((uint16_t)d[11] & 0xFF);

		GyroSensor::vData.X += bias[0] * (1<<4);
		GyroSensor::vData.Y += bias[1] * (1<<4);
		GyroSensor::vData.Z += bias[2] * (1<<4);

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
		// Byte[0]: Dummy, Byte[2:1]: Ch0DATA, Byte[4:3]: Ch1DATA, Byte[6:5]: PDATA, Byte[7]: Dummy
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
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_QUAT9_SIZE);

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
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_STEP_DETECTOR_SIZE);

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
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_GEOMAG_SIZE);

		d += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		cnt += ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		Len -= ICM20948_FIFO_HEADER_GEOMAG_SIZE;
		vFifoHdr &= ~ICM20948_FIFO_HEADER_GEOMAG; // Clear bit
	}

	if (vFifoHdr & ICM20948_FIFO_HEADER_PRESS_TEMP)
	{
		// Byte [2:0]: Pressure data, Byte [5:3]: Temperature data
		if (Len < ICM20948_FIFO_HEADER_PRESS_TEMP_SIZE)
		{
			return cnt;
		}
//		Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER_PRESSURE_SIZE);
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
		// The unit is uT scaled by 2^16
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

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_FSYNC)
		{
			if (Len < ICM20948_FIFO_HEADER2_FSYNC_SIZE)
			{
				return cnt;
			}
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_PICKUP_SIZE);

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

		if (vFifoHdr2 & ICM20948_FIFO_HEADER2_SECOND_ONOFF)
		{
			if (Len < ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE)
			{
				return cnt;
			}
//			Read((uint8_t*)&regaddr, 2, d, ICM20948_FIFO_HEADER2_ACTI_RECOG_SIZE);

			d += ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE;
			cnt += ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE;
			Len -= ICM20948_FIFO_HEADER2_SECOND_ONOFF_SIZE;
			vFifoHdr2 &= ~ICM20948_FIFO_HEADER2_SECOND_ONOFF; // Clear bit
		}
	}

	if (Len < ICM20948_FIFO_FOOTER_SIZE)
	{
		vFifoHdr |= ICM20948_FIFO_HEADER_FOOTER;
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

void AgmIcm20948::UpdateData(SENSOR_TYPE Type, uint64_t Timestamp, uint8_t * const pData)
{
	switch (Type)
	{
		case SENSOR_TYPE_ACCEL:
			AccelSensor::vData.Timestamp = Timestamp;
			AccelSensor::vData.X = ((pData[0] << 8) | (pData[1] & 0xFF));
			AccelSensor::vData.Y = ((pData[2] << 8) | (pData[3] & 0xFF));
			AccelSensor::vData.Z = ((pData[4] << 8) | (pData[5] & 0xFF));
			break;
		case SENSOR_TYPE_GYRO:
			GyroSensor::vData.Timestamp = Timestamp;
			GyroSensor::vData.X = ((pData[0] << 8) | (pData[1] & 0xFF));
			GyroSensor::vData.Y = ((pData[2] << 8) | (pData[3] & 0xFF));
			GyroSensor::vData.Z = ((pData[4] << 8) | (pData[5] & 0xFF));
			break;
		case SENSOR_TYPE_MAG:
			MagSensor::vData.Timestamp = Timestamp;
			MagSensor::vData.X = ((int16_t)pData[0] << 8) | (pData[1] & 0xFF);
			MagSensor::vData.Y = ((int16_t)pData[2] << 8) | (pData[3] & 0xFF);
			MagSensor::vData.Z = ((int16_t)pData[4] << 8) | (pData[5] & 0xFF);
			break;
	}
}

void AgmIcm20948::UpdateData(enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
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

static size_t s_FifoProcessCnt = 0;
static size_t s_BadHdrCnt = 0;

bool AgmIcm20948::UpdateData()
{
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
	if (status[0] & ICM20948_INT_STATUS_DMP_INT1 || status[3])
	{
		regaddr = ICM20948_DMP_INT_STATUS_REG;
		uint16_t distatus;
		Read((uint8_t*)&regaddr, 2, (uint8_t*)&distatus, 1);

		//g_Uart.printf("DMP int %x\r\n", distatus);
	//		Write16((uint8_t*)&regaddr, 2, distatus);

		if (distatus)// & (ICM20948_DMP_INT_STATUS_MSG_DMP_INT | ICM20948_DMP_INT_STATUS_MSG_DMP_INT_0))
		{
			// DMP msg
			//printf("distatus %x\n", distatus);
		}
	}

	if (status[2])
	{
		g_Uart.printf("Fifo overflow\r\n");
		ResetFifo();
		vFifoDataLen = 0;
		vFifoHdr = vFifoHdr2 = 0;
	}

#if 1
	if (vbDmpEnabled)
	{
		s_FifoProcessCnt++;

		regaddr = ICM20948_FIFO_COUNTH_REG;
		size_t cnt = Read16((uint8_t*)&regaddr, 2);
		cnt = EndianCvt16(cnt);

		regaddr = ICM20948_FIFO_R_W_REG;
		uint8_t *p = &vFifo[vFifoDataLen];

		while (cnt > 0)//ICM20948_FIFO_PAGE_SIZE)
		{
			int l = min((size_t)ICM20948_FIFO_PAGE_SIZE, min(cnt, sizeof(vFifo) - vFifoDataLen));

			if (l == 0)
			{
				break;
			}
			l = Read((uint8_t*)&regaddr, 2, p, l);
			p += l;
			vFifoDataLen += l;
			cnt -= l;
		}

		p = vFifo;

		while (vFifoDataLen > 0)
		{
			if (vFifoHdr == 0 && vFifoHdr2 == 0)
			{
				int l = 0;

				// new packet
				vFifoHdr = ((uint16_t)p[0] << 8U) | ((uint16_t)p[1] & 0xFF);

				if ((vFifoHdr & ~ICM20948_FIFO_HEADER_MASK))
				{
					s_BadHdrCnt++;
					g_Uart.printf("Bad hdr %x %d %d %d\r\n", vFifoHdr, s_BadHdrCnt, s_FifoProcessCnt, vFifoDataLen);
					ResetFifo();
					vFifoDataLen = 0;
					vFifoHdr = 0;
					cnt = 0;
					return false;
				}

				//vFifoHdr |= ICM20948_FIFO_HEADER_FOOTER;
				l = 2;

				if (vFifoHdr & ICM20948_FIFO_HEADER_HEADER2)
				{
					if (vFifoDataLen < 4)
					{
						vFifoHdr = 0;
						return false;
					}
					vFifoHdr2 = ((uint16_t)p[2] << 8U) | ((uint16_t)p[3] & 0xFF);

					if (vFifoHdr2 & ~ICM20948_FIFO_HEADER2_MASK)
					{
						s_BadHdrCnt++;
						g_Uart.printf("Bad hdr2 %x %d %d\r\n", vFifoHdr2, s_BadHdrCnt, s_FifoProcessCnt);
						ResetFifo();
						vFifoDataLen = 0;
						vFifoHdr = vFifoHdr2 = 0;
						cnt = 0;
						return false;
					}

					l += 2;
					vFifoHdr &= ~ICM20948_FIFO_HEADER_HEADER2;
				}
				vFifoDataLen -= l;

				p += l;

			}
			int l = ProcessDMPFifo(p, vFifoDataLen, t);
			if (l == 0)
			{
				break;//return false;
			}
			vFifoDataLen -= l;
			p += l;
		}
		if (vFifoDataLen > 0 && p != vFifo)
		{
			memmove(vFifo, p, vFifoDataLen);
		}

	}
	else
#endif
	{
		//res = MagIcm20948::UpdateData();

		if (status[1] & ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT)
		{
			regaddr = ICM20948_ACCEL_XOUT_H_REG;
			Read((uint8_t*)&regaddr, 2, (uint8_t*)d, 14);

			AccelSensor::vData.Timestamp = t;
			AccelSensor::vData.X = ((int16_t)(d[0] << 8) | ((int16_t)d[1] & 0xFF));
			AccelSensor::vData.Y = ((int16_t)(d[2] << 8) | ((int16_t)d[3] & 0xFF));
			AccelSensor::vData.Z = ((int16_t)(d[4] << 8) | ((int16_t)d[5] & 0xFF));
			GyroSensor::vData.Timestamp = t;
			GyroSensor::vData.X = ((int16_t)(d[6] << 8) | ((int16_t)d[7] & 0xFF));
			GyroSensor::vData.Y = ((int16_t)(d[8] << 8) | ((int16_t)d[9] & 0xFF));
			GyroSensor::vData.Z = ((int16_t)(d[10] << 8) | ((int16_t)d[11] & 0xFF));


			//MagIcm20948::UpdateData();

			// TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC
			int16_t t = ((int16_t)d[12] << 8) | d[13];
			TempSensor::vData.Temperature =  (((int16_t)d[12] << 8) | ((int16_t)d[13] & 0xFF)) * 100 / 33387 + 2100;
			TempSensor::vData.Timestamp = t;

			res = true;
		}
	}

	return res;
}

int AgmIcm20948::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (CmdAddrLen == 2)
	{
		SelectBank(*(uint16_t*)pCmdAddr >> 7);
		CmdAddrLen--;
	}

	*(uint16_t*)pCmdAddr &= 0x7F;

	return Device::Read(pCmdAddr, CmdAddrLen, pBuff, BuffLen);
}


int AgmIcm20948::Write(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	if (CmdAddrLen == 2)
	{
		SelectBank(*(uint16_t*)pCmdAddr >> 7);
		CmdAddrLen--;
	}

	*(uint16_t*)pCmdAddr &= 0x7F;

	return Device::Write(pCmdAddr, CmdAddrLen, pData, DataLen);
}

int AgmIcm20948::Read(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint16_t regaddr = ICM20948_USER_CTRL_REG;
		uint8_t userctrl = Read8((uint8_t*)&regaddr, 2) | ICM20948_USER_CTRL_I2C_MST_EN;
		uint8_t d[4];

		regaddr = ICM20948_I2C_SLV0_ADDR_REG;
		Write8((uint8_t*)&regaddr, 2, (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD);
		//d[0] = (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_RD;
		d[0] = *pCmdAddr;

		while (BuffLen > 0)
		{
			int cnt = min(ICM20948_I2C_SLV_MAXLEN, BuffLen);

			d[1] = ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN | (cnt & ICM20948_I2C_SLV0_CTRL_I2C_SLV0_LEN_MASK);

			regaddr = ICM20948_I2C_SLV0_REG_REG;
			Write((uint8_t*)&regaddr, 2, d, 2);

			// Start transfer
			regaddr = ICM20948_USER_CTRL_REG;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			msDelay(3);

			Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);

			regaddr = ICM20948_EXT_SLV_SENS_DATA_00_REG;
			cnt = Read((uint8_t*)&regaddr, 1, pBuff, cnt);
			if (cnt <= 0)
				break;

			pBuff += cnt;
			BuffLen -= cnt;
			retval += cnt;
			d[1] += cnt;
		}
	}
	else
	{
		retval = vpIntrf->Read(DevAddr, pCmdAddr, CmdAddrLen, pBuff, BuffLen);
	}

	return retval;
}

int AgmIcm20948::Write(uint32_t DevAddr, uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pData, int DataLen)
{
	int retval = 0;

	if (vpIntrf->Type() == DEVINTRF_TYPE_SPI)
	{
		uint16_t regaddr = ICM20948_USER_CTRL_REG;
		uint8_t d[8];
		uint8_t userctrl = Read8((uint8_t*)&regaddr, 2) | ICM20948_USER_CTRL_I2C_MST_EN;

		regaddr = ICM20948_I2C_SLV0_ADDR_REG;
		Write8((uint8_t*)&regaddr, 2, (DevAddr & ICM20948_I2C_SLV0_ADDR_I2C_ID_0_MASK) | ICM20948_I2C_SLV0_ADDR_I2C_SLV0_WR);

		d[0] = *pCmdAddr;
		d[1] = 1 | ICM20948_I2C_SLV0_CTRL_I2C_SLV0_EN;	// Length : Write is done 1 byte at a time

		while (DataLen > 0)
		{
			d[2] = *pData;

			regaddr = ICM20948_I2C_SLV0_REG_REG;
			Write((uint8_t*)&regaddr, 2, d, 3);

			regaddr = ICM20948_USER_CTRL_REG;
			Write8((uint8_t*)&regaddr, 2, userctrl);

			// Delay require for transfer to complete
			msDelay(2);

			Write8((uint8_t*)&regaddr, 2, userctrl & ~ICM20948_USER_CTRL_I2C_MST_EN);

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

bool AgmIcm20948::SelectBank(uint8_t BankNo)
{
	if (BankNo > 3 || vCurrBank == BankNo)
		return false;

	vCurrBank = BankNo;

	uint8_t regaddr = ICM20948_REG_BANK_SEL_REG;

	return Write8(&regaddr, 1, (BankNo << ICM20948_REG_BANK_SEL_USER_BANK_BITPOS) & ICM20948_REG_BANK_SEL_USER_BANK_MASK);
}

static uint64_t g_Dt = 0;
static uint64_t g_PrevT = 0;
static uint64_t g_IntDt = 0;

void AgmIcm20948::IntHandler()
{
	uint64_t t = vpTimer->uSecond();

	UpdateData();

	if (g_Dt == 0)
	{
		g_Dt = vpTimer->uSecond() - t;
	}
	else
	{
		g_Dt = (g_Dt + vpTimer->uSecond() - t) >> 1;
	}

	g_IntDt = t - g_PrevT;
	g_PrevT = t;
	//status = Read8((uint8_t*)&regaddr, 2);
	//printf("- status %x\n", status);
	//Write8((uint8_t*)&regaddr, 2, status);

#if 0
	int cnt = Read((uint8_t*)&regaddr, 2, istatus, 4);
	//printf("%x %x %x %x\n", istatus[0], istatus[1], istatus[2], istatus[3]);

	if (istatus[0] & ICM20948_INT_STATUS_I2C_MIST_INT)
	{
		printf("ICM20948_INT_STATUS_I2C_MIST_INT\n");
	}
	if (istatus[0] & ICM20948_INT_STATUS_DMP_INT1)
	{
		regaddr = ICM20948_DMP_INT_STATUS_REG;
		uint16_t distatus;
		Read((uint8_t*)&regaddr, 2, (uint8_t*)&distatus, 1);
//		Write16((uint8_t*)&regaddr, 2, distatus);

		if (distatus)// & (ICM20948_DMP_INT_STATUS_MSG_DMP_INT | ICM20948_DMP_INT_STATUS_MSG_DMP_INT_0))
		{
			// DMP msg
			printf("distatus %x\n", distatus);
		}
	}
	uint32_t drdy = (istatus[1] & ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT) |
			(istatus[2] & ICM20948_INT_STATUS_2_FIFO_OVERFLOW_INT_MASK) |
			(istatus[3] & ICM20948_INT_STATUS_3_FIFO_WM_INT_MASK);
	if (drdy)
	{
		regaddr = ICM20948_FIFO_COUNTH_REG;
		uint16_t cnt;
		Read((uint8_t*)&regaddr, 2, (uint8_t*)&cnt, 2);
		//printf("cnt = %d : ", cnt);
		cnt = EndianCvt16(cnt);

//		printf("%d\n", cnt);
		if (cnt > 2)
		{
			regaddr = ICM20948_FIFO_R_W_REG;
//			uint16_t h = Read16((uint8_t*)&regaddr, 2);
			uint8_t dd[16];
		//	dd [0] = Read8((uint8_t*)&regaddr, 2);
		//	dd [1] = Read8((uint8_t*)&regaddr, 2);

//			Read((uint8_t*)&regaddr, 2, dd, 16);
			regaddr = ICM20948_FIFO_COUNTH_REG;
			//uint16_t cnt1 = Read16((uint8_t*)&regaddr, 2);
			//printf("cnt = %d : ", cnt);
			//cnt1 = EndianCvt16(cnt1);
	//		printf("%d Fifo header=%x %x %d\n", cnt, dd[0], dd[1], cnt1);

		}
	}

#if 1
//	if (istatus[1] & ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT)
	{
//		printf("ICM20948_INT_STATUS_1_RAW_DATA_0_RDY_INT\n");
		UpdateData();
	}

	if (istatus[2] & ICM20948_INT_STATUS_2_FIFO_OVERFLOW_INT_MASK)
	{
	//	printf("FIFO ovr Int ");
		regaddr = ICM20948_FIFO_COUNTH_REG;
		//uint16_t cnt = Read16((uint8_t*)&regaddr, 2);
		cnt = EndianCvt16(cnt);

	//	printf("cnt = %d\r\n", cnt);
	}

	if (istatus[3] & ICM20948_INT_STATUS_3_FIFO_WM_INT_MASK)
	{
		//printf("WM Int ");
		regaddr = ICM20948_FIFO_COUNTH_REG;
		//uint16_t cnt = Read16((uint8_t*)&regaddr, 2);
		//cnt = EndianCvt16(cnt);

		//printf("cnt = %d\r\n", cnt);

#if 0
		for (int i = 0; i < cnt; i++)
		{
			regaddr = ICM20948_FIFO_R_W;
			uint8_t d = Read8((uint8_t*)&regaddr, 2);
			//printf("Fifo d=%x\n", d);
		}
#else
		uint8_t dd[cnt];
		memset(dd, 0, cnt);
		regaddr = ICM20948_FIFO_R_W_REG;
	//	int l = Read((uint8_t*)&regaddr, 2, dd, cnt);
		//printf("l:%d %x %x %x %x\n", l, dd[0], dd[1], dd[2], dd[3]);
#endif

	}
#endif

	//regaddr = ICM20948_INT_STATUS_REG;
	//cnt = Read((uint8_t*)&regaddr, 2, istatus, 4);

//	printf("x- %x %x %x %x\n", istatus[0], istatus[1], istatus[2], istatus[3]);
	//istatus[0] = istatus[1] = istatus[2] = istatus[3] = 0;
	//cnt = Write((uint8_t*)&regaddr, 2, istatus, 4);
#endif
}

int AgmIcm20948::ReadDMP(uint16_t MemAddr, uint8_t *pBuff, int Len)
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

int AgmIcm20948::WriteDMP(uint16_t MemAddr, uint8_t *pData, int Len)
{
	uint16_t regaddr = ICM20948_DMP_MEM_BANKSEL_REG;

	Write8((uint8_t*)&regaddr, 2, MemAddr >> 8);

	MemAddr &= 0xFF;
	uint8_t *p = pData;

	while (Len > 0)
	{
		regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
		Write8((uint8_t*)&regaddr, 2, MemAddr);

		regaddr = ICM20948_DMP_MEM_RW_REG;

		int l = min(16, Len);
		l = Write((uint8_t*)&regaddr, 2, p, l);
		p += l;
		Len -= l;
		MemAddr += l;
	}

	return Len;
}

/**
 * @brief	Initialize sensor (require implementation).
 *
 * @param 	CfgData : Reference to configuration data
 * @param	pIntrf 	: Pointer to interface to the sensor.
 * 					  This pointer will be kept internally
 * 					  for all access to device.
 * 					  DONOT delete this object externally
 * @param	pTimer	: Pointer to timer for retrieval of time stamp
 * 					  This pointer will be kept internally
 * 					  for all access to device.
 * 					  DONOT delete this object externally
 *
 * @return
 * 			- true	: Success
 * 			- false	: Failed
 */
bool AgmIcm20948::Init(const TempSensorCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (Init(Cfg.DevAddr, pIntrf, Cfg.Inter, Cfg.IntPol, pTimer) == false)
		return false;

	uint16_t regaddr = ICM20948_PWR_MGMT_1_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2);

	// Enable temperature sensor
	d &= ~ICM20948_PWR_MGMT_1_TEMP_DIS;
	Write8((uint8_t*)&regaddr, 2, d);

	return true;
}

bool AgmIcm20948::InitDMP(uint16_t DmpStartAddr, const uint8_t * const pDmpImage, int Len)
{
	if (pDmpImage == NULL || Len == 0)
		return false;

	// Disable DMP & FIFO before FIFO can be reseted and DMP firmware loaded
	uint16_t regaddr = ICM20948_USER_CTRL_REG;
	uint8_t d = Read8((uint8_t*)&regaddr, 2) & ~(ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN);
	Write8((uint8_t*)&regaddr,	2, d);
#if 0
	// Reset FIFO
	regaddr = ICM20948_FIFO_RST_REG;
	Write8((uint8_t*)&regaddr, 2, ICM20948_FIFO_RST_FIFO_RESET_MASK);
	Write8((uint8_t*)&regaddr, 2, 0);
#else
	ResetFifo();
#endif

	// load external image
	bool res = UploadDMPImage(pDmpImage, Len);

	if (res)
	{
		DmpStartAddr = EndianCvt16(DmpStartAddr);

		// Write DMP program start address
		regaddr = ICM20948_DMP_PROG_START_ADDRH_REG;
		Write16((uint8_t*)&regaddr, 2, DmpStartAddr);

		regaddr = ICM20948_USER_CTRL_REG;
		d |= ICM20948_USER_CTRL_FIFO_EN | ICM20948_USER_CTRL_DMP_EN;
		Write8((uint8_t*)&regaddr, 2, d);

		vbDmpEnabled = true;

		return true;
	}

	return false;
}

bool AgmIcm20948::UploadDMPImage(const uint8_t * const pDmpImage, int Len)
{
	int len = Len, l = 0;
	uint8_t *p = (uint8_t*)pDmpImage;
	uint16_t regaddr;
	uint16_t memaddr = ICM20948_DMP_LOAD_MEM_START_ADDR;
	size_t psize = ICM20948_DMP_MEM_BANK_SIZE;
	
	if (Interface()->Type() == DEVINTRF_TYPE_I2C)
	{
		psize = ICM20948_FIFO_PAGE_SIZE;
	}
			
	regaddr = ICM20948_PWR_MGMT_1_REG;
	uint8_t pwrstate;

	pwrstate = Read8((uint8_t*)&regaddr, 2);

	// make sure it is on full power
	Write8((uint8_t*)&regaddr, 2, pwrstate & ~(ICM20948_PWR_MGMT_1_LP_EN | ICM20948_PWR_MGMT_1_SLEEP));

	regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);

	l = min(len, ICM20948_FIFO_PAGE_SIZE - (memaddr % ICM20948_FIFO_PAGE_SIZE));

	while (len > 0)
	{
		regaddr = ICM20948_DMP_MEM_RW_REG;
		l = Write((uint8_t*)&regaddr, 2, p, l);

		p += l;
		memaddr += l;
		len -= l;

		regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

		regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);

		l = min(len, ICM20948_FIFO_PAGE_SIZE);
	}

	len = Len;
	p = (uint8_t*)pDmpImage;
	memaddr = ICM20948_DMP_LOAD_MEM_START_ADDR;

	// Verify

	regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

	regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
	Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);
	l = min(len, ICM20948_FIFO_PAGE_SIZE - (memaddr % ICM20948_FIFO_PAGE_SIZE));

	while (len > 0)
	{
		uint8_t m[ICM20948_FIFO_PAGE_SIZE];

		regaddr = ICM20948_DMP_MEM_RW_REG;
		Read((uint8_t*)&regaddr, 2, m, l);

		if (memcmp(p, m, l) != 0)
		{
			g_Uart.printf("failed loading DMP\r\n");
			return false;
		}

		p += l;
		memaddr += l;
		len -= l;

		regaddr = ICM20948_DMP_MEM_BANKSEL_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr >> 8);

		regaddr = ICM20948_DMP_MEM_STARTADDR_REG;
		Write8((uint8_t*)&regaddr, 2, memaddr & 0xFF);

		l = min(len, ICM20948_FIFO_PAGE_SIZE);
	}

	// Restore power state
	Write8((uint8_t*)&regaddr, 2, pwrstate);

	return true;
}
