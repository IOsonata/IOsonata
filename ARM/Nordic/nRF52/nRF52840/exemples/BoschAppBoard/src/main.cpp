/*
 * main.cpp
 *
 *  Created on: Jul. 25, 2024
 *      Author: hoan
 */
#include <string.h>
#include <math.h>

#include "idelay.h"
#include "iopinctrl.h"
#include "coredev/spi.h"
#include "coredev/i2c.h"
#include "pwrmgnt/pm_bq25120a.h"
#include "sensors/ag_bmi323.h"

#include "bmi323.h"
#include "common.h"
#include "board.h"

static const IOPinCfg_t s_IOPins[] = {
    {LED_RED_PORT, LED_RED_PIN, LED_RED_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {LED_GREEN_PORT, LED_GREEN_PIN, LED_GREEN_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {LED_BLUE_PORT, LED_BLUE_PIN, LED_BLUE_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {VDD_EN_PORT, VDD_EN_PIN, VDD_EN_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {VDDIO_EN_PORT, VDDIO_EN_PIN, VDDIO_EN_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {LS_EN_PORT, LS_EN_PIN, LS_EN_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const int s_IOPinsCnt = sizeof(s_IOPins) / sizeof(IOPinCfg_t);

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt);

const static TimerCfg_t s_TimerCfg = {
    .DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 1,
	.EvtHandler = TimerHandler,
};

Timer g_Timer;

static const IOPinCfg_t s_SpiPins[] = {
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{SPI_BMI323_CS_PORT, SPI_BMI323_CS_PIN, SPI_BMI323_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
	.Rate = 4000000,   // Speed in Hz
	.DataSize = 8,      // Data Size
	.MaxRetry = 5,      // Max retries
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK, // Data phase
	.ClkPol = SPICLKPOL_HIGH,         // clock polarity
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = true,	// DMA
	.bIntEn = false,
	.IntPrio = 6,      // Interrupt priority
	.EvtCB = NULL
};

SPI g_Spi;

//********** I2C **********
static const IOPinCfg_t s_I2cPins[] = {
	{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
	{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},
};

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,			// I2C device number
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = 100000,	// Rate
	.MaxRetry = 5,			// Retry
	.AddrType = I2CADDR_TYPE_NORMAL,
	0,			// Number of slave addresses
	{0,},		// Slave addresses
	true,	// DMA
	false,		// Use interrupt
	7,			// Interrupt prio
	NULL		// Event callback
};

I2C g_I2c;

static const PwrMgntVoutCfg_t s_PmVoutCfg[] = {
	{
		.mVout = 3300,
		.mAlimit = 100,
	},
	{
		.mVout = 3300,
		.mAlimit = 100,
	},
};

PwrMgntCfg_t s_PmicCfg = {
	.DevAddr = BQ25120A_I2C_7BITS_DEVADDR,
	.pVout = (PwrMgntVoutCfg_t*)&s_PmVoutCfg,
	.NbVout = sizeof(s_PmVoutCfg) / sizeof(PwrMgntVoutCfg_t),
	.VEndChrg = 4200,
	.ChrgCurr = 500,
};

PmBq25120a g_Pmic;

static const AccelSensorCfg_t s_AccelCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 100000,
	.Scale = 2,
	.FltrFreq = 0,
	.bInter = true,
	.IntPol = DEVINTR_POL_LOW,
};

static const GyroSensorCfg_t s_GyroCfg = {
	.DevAddr = 0,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 50000,
	.Sensitivity = 10,
	.FltrFreq = 200,
};

AgBmi323 g_MotSensor;
AccelSensor *g_pAccel = NULL;
GyroSensor *g_pGyro = NULL;

uint32_t g_DT = 0;
static uint32_t g_TPrev = 0;

void TimerHandler(TimerDev_t *pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    }
}

extern "C" {

/*!
 * @brief This internal API reads I2C function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
BMI3_INTF_RET_TYPE bmi3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{

}

/*!
 * @brief This internal API writes I2C function map to COINES platform
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     len      : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks.
 *
 * @return Status of execution.
 */
BMI3_INTF_RET_TYPE bmi3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{

}

/*!
 * @brief This internal API reads SPI function map to COINES platform
 */
BMI3_INTF_RET_TYPE bmi3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    if (g_Spi.Read(device_addr, &reg_addr, 1, reg_data, len) > 0)
    {
    	return 0;
    }

    return -1;

#if 0

    (void)intf_ptr;

    return coines_read_spi(COINES_SPI_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)len);
#endif
}

/*!
 * @brief This internal API writes SPI function map to COINES platform
 */
BMI3_INTF_RET_TYPE bmi3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    if (g_Spi.Write(device_addr, &reg_addr, 1, (uint8_t*)reg_data, len) > 0)
    {
    	return 0;
    }

    return -1;

#if 0
    (void)intf_ptr;

    return coines_write_spi(COINES_SPI_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
#endif
}

}	// exern "C"

/*!
 * @brief This internal API is used to set configurations for accel.
 */
static int8_t set_accel_config(struct bmi3_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi3_sens_config config;

    /* Structure to map interrupt */
    struct bmi3_map_int map_int = { 0 };

    /* Configure the type of feature. */
    config.type = BMI323_ACCEL;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(&config, 1, dev);
    bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

    if (rslt == BMI323_OK)
    {
        map_int.acc_drdy_int = BMI3_INT1;

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi323_map_interrupt(map_int, dev);
        bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

        if (rslt == BMI323_OK)
        {
            /* NOTE: The user can change the following configuration parameters according to their requirement. */
            /* Output Data Rate. By default ODR is set as 100Hz for accel. */
            config.cfg.acc.odr = BMI3_ACC_ODR_100HZ;

            /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
            config.cfg.acc.range = BMI3_ACC_RANGE_2G;

            /* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
            config.cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;

            /* Set number of average samples for accel. */
            config.cfg.acc.avg_num = BMI3_ACC_AVG64;

            /* Enable the accel mode where averaging of samples
             * will be done based on above set bandwidth and ODR.
             * Note : By default accel is disabled. The accel will get enable by selecting the mode.
             */
            config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

            /* Set the accel configurations. */
            rslt = bmi323_set_sensor_config(&config, 1, dev);
            bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);
        }
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
	/*! Earth's gravity in m/s^2 */
	#define GRAVITY_EARTH  (9.80665f)

	double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

bool HardwareInit()
{
	bool res;

	g_I2c.Init(s_I2cCfg);

	g_Pmic.Init(s_PmicCfg, &g_I2c);

	IOPinCfg(s_IOPins, s_IOPinsCnt);

	IOPinSet(LED_BLUE_PORT, LED_BLUE_PIN);
	IOPinSet(VDD_EN_PORT, VDD_EN_PIN);
	IOPinSet(LS_EN_PORT, LS_EN_PIN);
	IOPinSet(VDDIO_EN_PORT, VDDIO_EN_PIN);

	g_Timer.Init(s_TimerCfg);

	res = g_Spi.Init(s_SpiCfg);

	msDelay(10);

/*
	res = g_MotSensor.Init(s_AccelCfg, &g_Spi, &g_Timer);
	if (res == true)
	{
		g_pAccel = &g_MotSensor;
	}
*/
	return res;
}

int main()
{
	HardwareInit();
    /* Sensor initialization configuration. */
    struct bmi3_dev dev = { 0 };

    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Variable to define limit to print accel data. */
    uint16_t limit = 100;

    /* Create an instance of sensor data structure. */
    struct bmi3_sensor_data sensor_data = { 0 };

    /* Initialize the interrupt status of accel. */
    uint16_t int_status = 0;

    uint8_t indx = 0;
    float x = 0, y = 0, z = 0;

    /* Structure to define accelerometer configuration. */
    struct bmi3_sens_config config = { 0 };

    /* Select accel sensor. */
    sensor_data.type = BMI323_ACCEL;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_SPI_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    /* Initialize bmi323. */
    rslt = bmi323_init(&dev);
    bmi3_error_codes_print_result("bmi323_init", rslt);

    if (rslt == BMI323_OK)
    {
        /* Accel configuration settings. */
        rslt = set_accel_config(&dev);

        if (rslt == BMI323_OK)
        {
            rslt = bmi323_get_sensor_config(&config, 1, &dev);
            bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);
        }

        if (rslt == BMI323_OK)
        {
            printf("\nData set, Range, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z\n\n");

            while (indx <= limit)
            {
                /* To get the status of accel data ready interrupt. */
                rslt = bmi323_get_int1_status(&int_status, &dev);
                bmi3_error_codes_print_result("bmi323_get_int1_status", rslt);

                /* To check the accel data ready interrupt status and print the status for 100 samples. */
                if (int_status & BMI3_INT_STATUS_ACC_DRDY)
                {
                    /* Get accelerometer data for x, y and z axis. */
                    rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev);
                    bmi3_error_codes_print_result("Get sensor data", rslt);

                    /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                    x = lsb_to_mps2(sensor_data.sens_data.acc.x, 2, dev.resolution);
                    y = lsb_to_mps2(sensor_data.sens_data.acc.y, 2, dev.resolution);
                    z = lsb_to_mps2(sensor_data.sens_data.acc.z, 2, dev.resolution);

                    /* Print the data in m/s2. */
                    printf("%d, %d, %d, %d, %d, %4.2f, %4.2f, %4.2f\n",
                           indx,
                           config.cfg.acc.range,
                           sensor_data.sens_data.acc.x,
                           sensor_data.sens_data.acc.y,
                           sensor_data.sens_data.acc.z,
                           x,
                           y,
                           z);

                    indx++;
                }
            }
        }
    }

    bmi3_coines_deinit();

    return rslt;

#if 0
	AccelSensorRawData_t arawdata;
	AccelSensorData_t accdata;
	GyroSensorRawData_t grawdata;
	GyroSensorData_t gyrodata;

	memset(&arawdata, 0, sizeof(AccelSensorRawData_t));
	memset(&accdata, 0, sizeof(AccelSensorData_t));
	memset(&gyrodata, 0, sizeof(GyroSensorData_t));


	uint32_t prevt = 0;
	int cnt = 10;
	while (1)
	{
//		uint32_t t = g_Timer.uSecond();

		//NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
		//__WFE();
		g_MotSensor.UpdateData();

		uint32_t dt = arawdata.Timestamp - prevt;
		prevt = arawdata.Timestamp;

		g_MotSensor.Read(arawdata);

		if (g_pGyro)
		{
			g_pGyro->Read(grawdata);
		}

		g_MotSensor.Read(accdata);
		//g_Imu.Read(quat);

		if (cnt-- < 0)
		{
			cnt = 10;
			uint32_t t = accdata.Timestamp;
			//printf("Accel %d %d: %d %d %d\r\n", (uint32_t)g_DT, (uint32_t)dt, arawdata.X, arawdata.Y, arawdata.Z);
			printf("Accel %d %d: %f %f %f %u\r\n", (uint32_t)g_DT, (uint32_t)dt, accdata.X, accdata.Y, accdata.Z, t);
			//printf("Quat %d %d: %f %f %f %f\r\n", (uint32_t)g_DT, (uint32_t)dt, quat.Q1, quat.Q2, quat.Q3, quat.Q4);
		}
	}
#endif
}


