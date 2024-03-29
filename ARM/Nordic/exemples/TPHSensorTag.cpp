/**-------------------------------------------------------------------------
@example	TPHSensorTag.cpp


@brief	Environmental Sensor BLE demo (Supports BME280, BME680, MS8607).

This application demo shows BLE non-connectable using EHAL library. It advertises
Temperature, Pressure, Humidity (TPH) data in BLE manufacturer specific data.
Support I2C and SPI interface


NOTE : The BME680 Air Quality Index is undocumented.  It requires the library
Bosch Sensortec Environmental Cluster (BSEC) Software. Go to
https://github.com/boschsensortec/Bosch-BSEC2-Library. Clone it to `external/BSEC`
as indicated on the folder tree.

The BSEC library must be initialized in the main application prior to initializing
this driver by calling the function

bsec_library_return_t res = bsec_init();

@author Hoang Nguyen Hoan
@date	May 8, 2017

@license

MIT License

Copyright (c) 2017 I-SYST inc. All rights reserved.

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

#include <string.h>

#include "istddef.h"
#include "bluetooth/bt_app.h"
//#ifndef NRFXLIB_SDC
//#include "app_util_platform.h"
//#include "app_scheduler.h"
//#include "ble_app_nrf5.h"
//#endif

#include "sensors/bsec_interface.h"

#include "blueio_board.h"
#include "coredev/uart.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "sensors/tph_bme280.h"
#include "sensors/tph_ms8607.h"
#include "sensors/tphg_bme680.h"
#include "bluetooth/bt_hci.h"
#include "timer_nrfx.h"
#ifdef NRF51
//#include "timer_nrf_app_timer.h"
#else
#include "adc_nrf52_saadc.h"
#endif
#include "board.h"
#include "idelay.h"

#define DEVICE_NAME                     "EnvSensorTag"                            /**< Name of device. Will be included in the advertising data. */

#define EVIM
#ifdef NEBLINA_MODULE
#define TPH_BME280
#else
//#define TPH_BME280
#define TPH_BME680
#endif

//#ifdef NRF52
// Use timer to update data
// NOTE :	RTC timer 0 used by radio, RTC Timer 1 used by SDK
//			Only RTC timer 2 is usable with Softdevice for nRF52, not avail on nRF51
//
#define USE_TIMER_UPDATE
//#endif

#define APP_ADV_INTERVAL_MSEC		1000 //                MSEC_TO_UNITS(1000, UNIT_0_625_MS)             /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#ifdef USE_TIMER_UPDATE
// Use timer to update date
#define APP_ADV_TIMEOUT_MSEC      0                                         /**< The advertising timeout (in units of seconds). */
#else
// Use advertisement timeout to update data
#define APP_ADV_TIMEOUT_MSEC      	120000 //MSEC_TO_UNITS(120000, UNIT_10_MS)           /**< The advertising timeout (in units of seconds). */
#endif

void TimerHandler(TIMER * const pTimer, uint32_t Evt);

uint8_t g_AdvDataBuff[10] = {
	BLEADV_MANDATA_TYPE_TPH,
};

BleAdvManData_t &g_AdvData = *(BleAdvManData_t*)g_AdvDataBuff;

// Evironmental Sensor Data to advertise
BleAdvManData_TphSensor_t &g_TPHData = *(BleAdvManData_TphSensor_t *)g_AdvData.Data;
BleAdvManData_AqSensor_t &g_GasData = *(BleAdvManData_AqSensor_t *)g_AdvData.Data;
BLUEIO_DATA_BAT &g_AdvBat = *(BLUEIO_DATA_BAT *)g_AdvData.Data;

const static TIMER_CFG s_TimerCfg = {
    .DevNo = 2,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,			// 0 => Default highest frequency
	.IntPrio = 6,
	.EvtHandler = TimerHandler
};

Timer g_Timer;

const BtAppCfg_t s_BtDevCfg = {
	BTAPP_ROLE_BROADCASTER,
	0, 						// Number of central link
	1, 						// Number of peripheral link
	DEVICE_NAME,                 // Device name
	ISYST_BLUETOOTH_ID,     // PnP Bluetooth/USB vendor id
	1,                      // PnP Product ID
	0,						// Pnp prod version
	0,						// Appearance
	NULL,					// Enable device information service (DIS)
	false,
	(uint8_t*)&g_AdvDataBuff,   // Manufacture specific data to advertise
	sizeof(g_AdvDataBuff),      // Length of manufacture specific data
	NULL,
	0,
	BTGAP_SECTYPE_NONE,    // Secure connection type
	BTAPP_SECEXCHG_NONE,   // Security key exchange
	NULL,      				// Service uuids to advertise
	0, 						// Total number of uuids
	APP_ADV_INTERVAL_MSEC,       // Advertising interval in msec
	APP_ADV_TIMEOUT_MSEC,	// Advertising timeout in sec
	0,//MSEC_TO_UNITS(1000, UNIT_0_625_MS) ,   // Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	0,
	0,
	BLUEIO_LED1_PORT,		// Led port nuber
	BLUEIO_LED1_PIN,     // Led pin number
	0,
	0, 		// Tx power
	NULL						// RTOS Softdevice handler
};

// Motsai Neblina V2 module uses SPI interface
#ifdef NEBLINA_MODULE
static const IOPINCFG gsSpiBoschPin[] = {
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP,
     IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
    {BME280_CS_PORT, BME280_CS_PIN, BME280_CS_PINOP,
     IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
};

static const SPICFG s_SpiCfg = {
    SPI_DEVNO,
    SPIMODE_MASTER,
    gsSpiBoschPin,
    sizeof( gsSpiBoschPin ) / sizeof( IOPINCFG ),
    8000000,   // Speed in Hz
    8,      // Data Size
    5,      // Max retries
    SPIDATABIT_MSB,
    SPIDATAPHASE_SECOND_CLK, // Data phase
    SPICLKPOL_LOW,         // clock polarity
    SPICSEL_AUTO,
    6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    nullptr
};

SPI g_Spi;

DeviceIntrf *g_pIntrf = &g_Spi;

#else

// Configure I2C interface
static const IOPinCfg_t s_I2cPins[] = {
#if defined(TPH_BME280) || defined(TPH_BME680)
		{I2C0_SDA_PORT, I2C0_SDA_PIN, I2C0_SDA_PINOP, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{I2C0_SCL_PORT, I2C0_SCL_PIN, I2C0_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
#else
		// Custom board with MS8607
		{0, 4, 0, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{0, 3, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
#endif
};

static const I2CCfg_t s_I2cCfg = {
	0,			// I2C device number
	I2CTYPE_STANDARD,
	I2CMODE_MASTER,
	s_I2cPins,
	sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	100000,	// Rate
	5,			// Retry
	I2CADDR_TYPE_NORMAL,
	0,			// Number of slave addresses
	{0,},		// Slave addresses
	true,
	false,		// Use interrupt
	6,			// Interrupt prio
	NULL		// Event callback
};

// I2C interface instance
I2C g_I2c;

DeviceIntrf *g_pIntrf = &g_I2c;
#endif

static const IOPinCfg_t s_GpioPins[] = {
	{BLUEIO_BUT1_PORT, BLUEIO_BUT1_PIN, BLUEIO_BUT1_PINOP,	// Button 1
	 IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{BLUEIO_BUT2_PORT, BLUEIO_BUT2_PIN, BLUEIO_BUT2_PINOP,	// Button 2
	 IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_IMU_INT_PORT, BLUEIO_TAG_EVIM_IMU_INT_PIN, BLUEIO_TAG_EVIM_IMU_INT_PINOP, 			// MPU9250 Interrupt
	 IOPINDIR_INPUT, IOPINRES_PULLDOWN, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_LED2R_PORT, BLUEIO_TAG_EVIM_LED2R_PIN, BLUEIO_TAG_EVIM_LED2R_PINOP,		// RGB LED2 Red
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BLUEIO_TAG_EVIM_LED2G_PORT, BLUEIO_TAG_EVIM_LED2G_PIN, BLUEIO_TAG_EVIM_LED2G_PINOP,	// RGB LED2 Green
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
	{BLUEIO_TAG_EVIM_LED2B_PORT, BLUEIO_TAG_EVIM_LED2B_PIN, BLUEIO_TAG_EVIM_LED2B_PINOP,	// RGB LED2 Blue
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
//	    {BLUEIO_TAG_EVIM_SPI2_SCK_PORT, BLUEIO_TAG_EVIM_SPI2_SCK_PIN, BLUEIO_TAG_EVIM_SPI2_SCK_PINOP,
//	     IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const int s_NbGpioPins = sizeof(s_GpioPins) / sizeof(IOPinCfg_t);

// Configure environmental sensor
static TPHSensorCfg_t s_TphSensorCfg = {
#ifdef NEBLINA_MODULE
    0,      // SPI CS index 0 connected to BME280
#else
	BME680_I2C_DEV_ADDR0,   // I2C device address
#endif
	SENSOR_OPMODE_SINGLE,
	10,						// Sampling frequency in mHz
	2,
	1,
	1,
	1
};

static const GasSensorHeater_t s_HeaterProfile[] = {
	{ 375, 125 },
};

static const GasSensorCfg_t s_GasSensorCfg = {
	BME680_I2C_DEV_ADDR0,	// Device address
	SENSOR_OPMODE_SINGLE,	// Operating mode
	10,
	sizeof(s_HeaterProfile) / sizeof(GasSensorCfg_t),
	s_HeaterProfile
};

// Environmental sensor instance
TphgBme680 g_Bme680Sensor;
TphBme280 g_Bme280Sensor;
TphMS8607 g_MS8607Sensor;


#ifdef TPH_BME280
TphSensor &g_TphSensor = g_Bme280Sensor;
#elif defined(TPH_BME680)
TphSensor &g_TphSensor = g_Bme680Sensor;
#else
TphSensor &g_TphSensor = g_MS8607Sensor;
#endif

GasSensor &g_GasSensor = g_Bme680Sensor;

#ifdef NRF52_SERIES
// Define available voltage sources
static const AdcRefVolt_t s_RefVolt[] = {
	{.Type = ADC_REFVOLT_TYPE_INTERNAL, .Voltage = 0.6 },
};

static const int s_NbRefVolt = sizeof(s_RefVolt) / sizeof(AdcRefVolt_t);

#define ADC_CFIFO_SIZE		CFIFO_TOTAL_MEMSIZE(200, sizeof(AdcData_t))

void ADCEventHandler(Device *pAdcDev, DEV_EVT Evt);

static uint8_t s_AdcFifoMem[ADC_CFIFO_SIZE];

// Define ADC device
static const AdcCfg_t s_AdcCfg = {
	.Mode = ADC_CONV_MODE_SINGLE,
	.pRefVolt = s_RefVolt,
	.NbRefVolt = s_NbRefVolt,
	.DevAddr = 0,
	.Resolution = 10,
	.Rate = 8000,
	.OvrSample = 0,
	.bInterrupt = true,
	.IntPrio = 6,
	.EvtHandler = ADCEventHandler
};

AdcnRF52 g_Adc;

// Define ADC channel
static const AdcChanCfg_t s_ChanCfg[] = {
	{
		.Chan = 0,
		.RefVoltIdx = 0,
		.Type = ADC_CHAN_TYPE_SINGLE_ENDED,
		.Gain = 3,//5,//1 << 8,
		.AcqTime = 10,
		.BurstMode = true,
#ifdef EVIM
		.PinP = { .PinNo = 2, .Conn = ADC_PIN_CONN_NONE },
#else
		.PinP = { .PinNo = 8, .Conn = ADC_PIN_CONN_NONE },
#endif
	},
};

static const int s_NbChan = sizeof(s_ChanCfg) / sizeof(AdcChanCfg_t);
volatile bool g_bDataReady = false;
BLUEIO_DATA_BAT g_BatData;

void ADCEventHandler(Device *pAdcDev, DEV_EVT Evt)
{
	if (Evt == DEV_EVT_DATA_RDY)
	{
		g_bDataReady = true;
		int cnt = 0;

		AdcData_t df[s_NbChan];
		cnt = g_Adc.Read(df, s_NbChan);
		if (cnt > 0)
		{
//			g_Uart.printf("%d ADC[0] = %.2fV, ADC[1] = %.2fV, ADC[2] = %.2fV, ADC[3] = %.2fV\r\n",
//					df[0].Timestamp, df[0].Data, df[1].Data, df[2].Data, df[3].Data);
#ifdef EVIM
			uint8_t level = 100 * ((df->Data * 2.0) - 1.75)/ 1.25;
			g_BatData.Voltage = (int32_t)(df->Data * 2000.0);
#else
			uint8_t level = 100 * (df->Data - 1.75)/ 1.25;
			g_BatData.Voltage = (int32_t)(df->Data * 1000.0);
#endif
			g_BatData.Level = level;
		}

		g_Adc.Disable();
	}
}
#endif

void ReadPTHData()
{
	static uint32_t gascnt = 0;
	TPHSensorData_t data;
	GasSensorData_t gdata;
#if 1

	g_I2c.Enable();

	g_TphSensor.Read(data);


/*
	if (g_TphSensor.DeviceID() == BME680_ID && (gascnt & 0x3) == 0)
	{
		g_GasSensor.Read(gdata);
	}
*/
	if ((gascnt & 0xf) == 0)
	{
#ifdef NRF52_SERIES
		g_Adc.Enable();
		g_Adc.OpenChannel(s_ChanCfg, s_NbChan);
		g_Adc.StartConversion();

		g_AdvData.Type = BLEADV_MANDATA_TYPE_BAT;

		memcpy(&g_AdvBat, &g_BatData, sizeof(BLUEIO_DATA_BAT));
#endif
	}
	else if ((gascnt & 0x3) == 0)
	{
		BleAdvManData_AqSensor_t gas;

		g_GasSensor.Read(gdata);

		g_AdvData.Type = BLEADV_MANDATA_TYPE_GAS;
		gas.GasRes = gdata.GasRes[gdata.MeasIdx];
		gas.AirQIdx = gdata.AirQualIdx;

		memcpy(&g_GasData, &gas, sizeof(BleAdvManData_AqSensor_t));

		g_TphSensor.StartSampling();
	}
	else
	{
		g_AdvData.Type = BLEADV_MANDATA_TYPE_TPH;

		// NOTE : M0 does not access unaligned data
		// use local 4 bytes align stack variable then mem copy
		// skip timestamp as advertising pack is limited in size
		memcpy(&g_TPHData, ((uint8_t*)&data) + sizeof(data.Timestamp), sizeof(BleAdvManData_TphSensor_t));
	}

//	g_TphSensor.StartSampling();

	g_I2c.Disable();

#ifdef NRF52_SERIES
	g_Adc.Enable();
	g_Adc.OpenChannel(s_ChanCfg, s_NbChan);
	g_Adc.StartConversion();
#endif

#endif
	// Update advertisement data
	BtAppAdvManDataSet(g_AdvDataBuff, sizeof(g_AdvDataBuff), NULL, 0);
//	BleAppAdvManDataSet((uint8_t*)&gascnt, sizeof(gascnt), NULL, 0);

	gascnt++;
}

void TimerHandler(TIMER * const pTimer, uint32_t Evt)
{
    if (Evt & TIMER_EVT_TRIGGER(0))
    {
    	// SDK15 no longer allow updating advertisement data dynamically
    	// Have to stop and restart advertisement
		//BleAppAdvStop();
    	ReadPTHData();
		//BleAppAdvStart(BLEAPP_ADVMODE_FAST);
    }
}

void AppTimerHandler(Timer *pTimer, int TrigNo, void *pContext)
{
	if (TrigNo == 0)
	{
		ReadPTHData();
//		app_sched_event_put(pContext, sizeof(uint32_t), SchedAdvData);
	}
}

/*
void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
#ifndef USE_TIMER_UPDATE
    if (p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT)
    {
    	// Update environmental sensor data every time advertisement timeout
    	// for re-advertisement
   // 	ReadPTHData();
    }
#endif
}
*/
void BtAppAdvTimeoutHandler()
{
	ReadPTHData();
	BtAdvStart();//BLEAPP_ADVMODE_FAST);
}

void HardwareInit()
{
	// Set this only if nRF is power at 2V or more
	//nrf_power_dcdcen_set(true);
	NRF_POWER->DCDCEN = 1;

    IOPinCfg(s_GpioPins, s_NbGpioPins);

	IOPinClear(0, BLUEIO_TAG_BME680_LED2_BLUE_PIN);
	IOPinClear(0, BLUEIO_TAG_BME680_LED2_GREEN_PIN);
	IOPinClear(0, BLUEIO_TAG_BME680_LED2_RED_PIN);

	g_Timer.Init(s_TimerCfg);

	// Initialize I2C
#ifdef NEBLINA_MODULE
    g_Spi.Init(s_SpiCfg);
#else
    g_I2c.Init(s_I2cCfg);
#endif

	bsec_library_return_t bsec_status;

	// NOTE : For BME680 air quality calculation, this library is require to be initialized
	// before initializing the sensor driver.
	bsec_status = bsec_init();

	if (bsec_status != BSEC_OK)
	{
		printf("BSEC init failed\r\n");

		return;
	}

	// Inititalize sensor
    g_TphSensor.Init(s_TphSensorCfg, g_pIntrf, &g_Timer);

//    g_TphSensor.Disable();

//	g_I2c.Disable();

//	while(1) __WFE();


	if (g_TphSensor.DeviceID() == BME680_ID)
    {
    	g_GasSensor.Init(s_GasSensorCfg, g_pIntrf, NULL);
    }


    g_TphSensor.StartSampling();

	usDelay(300000);

    // Update sensor data
    TPHSensorData_t tphdata;

    g_TphSensor.Read(tphdata);

    if (g_TphSensor.DeviceID() == BME680_ID)
    {
		GasSensorData_t gdata;
		g_GasSensor.Read(gdata);
    }

	g_TphSensor.StartSampling();

	g_AdvData.Type = BLEADV_MANDATA_TYPE_TPH;
	// Do memcpy to adv data. Due to byte alignment, cannot read directly into
	// adv data
	memcpy(g_AdvData.Data, ((uint8_t*)&tphdata) + sizeof(tphdata.Timestamp), sizeof(BleAdvManData_TphSensor_t));


	g_I2c.Disable();

#ifdef NRF52_SERIES
	g_Adc.Init(s_AdcCfg);
	g_Adc.OpenChannel(s_ChanCfg, s_NbChan);
	g_Adc.StartConversion();
#endif

#ifdef USE_TIMER_UPDATE
	// Only with SDK14

	uint64_t period = g_Timer.EnableTimerTrigger(0, 500UL, TIMER_TRIG_TYPE_CONTINUOUS);
#endif
}

int main()
{
    HardwareInit();

    BtAppInit(&s_BtDevCfg);//, true);

	//uint64_t period = g_Timer.EnableTimerTrigger(0, 500UL, TIMER_TRIG_TYPE_CONTINUOUS, AppTimerHandler);

    BtAppRun();

	return 0;
}

