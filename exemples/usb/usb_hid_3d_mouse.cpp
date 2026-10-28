/**-------------------------------------------------------------------------
@example	usb_hid_3d_mouse.cpp

@brief	USB HID 3D mouse demo using a Bosch BMI323

The BMI323 accelerometer drives X, Y and Z translation. The gyroscope drives
Rx, Ry and Rz rotation. Startup samples establish the stationary center.

@author	Hoang Nguyen Hoan
@date	Sep. 10, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <limits.h>
#include <stdint.h>
#include <string.h>

#include "idelay.h"
#include "iopinctrl.h"
#include "coredev/spi.h"
#include "coredev/timer.h"
#include "sensors/ag_bmi323.h"
#include "app_evt_handler.h"
#include "usb/usb.h"
#include "usb/usbd_hid.h"

#include "board.h"

// board.h can select another controller
#ifndef USB_DEVNO
#define USB_DEVNO			0
#endif
#define HID_STR_INTERFACE	4U
#define HID_CENTER_SAMPLES	64U
#define HID_ACCEL_DEAD_ZONE	700
#define HID_GYRO_DEAD_ZONE	80
#define HID_ACCEL_DIVISOR	16
#define HID_GYRO_DIVISOR		4

static const IOPinCfg_t s_PowerPins[] = {
	{BMI323_VDD_EN_PORT, BMI323_VDD_EN_PIN, BMI323_VDD_EN_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BMI323_VDDIO_EN_PORT, BMI323_VDDIO_EN_PIN, BMI323_VDDIO_EN_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BMI323_LS_EN_PORT, BMI323_LS_EN_PIN, BMI323_LS_EN_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const IOPinCfg_t s_SpiPins[] = {
	{BMI323_SPI_SCK_PORT, BMI323_SPI_SCK_PIN, BMI323_SPI_SCK_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BMI323_SPI_MISO_PORT, BMI323_SPI_MISO_PIN, BMI323_SPI_MISO_PINOP,
	 IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BMI323_SPI_MOSI_PORT, BMI323_SPI_MOSI_PIN, BMI323_SPI_MOSI_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{BMI323_SPI_CS_PORT, BMI323_SPI_CS_PIN, BMI323_SPI_CS_PINOP,
	 IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const SPICfg_t s_SpiCfg = {
	.DevNo = BMI323_SPI_DEVNO,
	.Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(s_SpiPins[0]),
	.Rate = 4000000U,
	.DataSize = 8U,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH,
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = true,
	.bIntEn = false,
	.IntPrio = 6,
	.DummyByte = 0U,
	.EvtCB = nullptr,
};

static const TimerCfg_t s_TimerCfg = {
	.DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0U,
	.IntPrio = 1,
	.EvtHandler = nullptr,
	.bTickInt = false,
};

static const AccelSensorCfg_t s_AccelCfg = {
	.DevAddr = 0U,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 100000U,
	.Scale = 2U,
	.FltrFreq = 0U,
	.Inter = 0U,
	.IntPol = DEVINTR_POL_LOW,
	.IntHandler = nullptr,
	.bFifoEn = false,
};

static const GyroSensorCfg_t s_GyroCfg = {
	.DevAddr = 0U,
	.OpMode = SENSOR_OPMODE_CONTINUOUS,
	.Freq = 100000U,
	.Sensitivity = 500U,
	.FltrFreq = 0U,
	.Inter = 0U,
	.IntPol = DEVINTR_POL_LOW,
	.bFifoEn = false,
};

static SPI s_Spi;
static Timer s_Timer;
static AgBmi323 s_Imu;
static int HidGetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	uint8_t *pBuffer, uint16_t BufferSize, void *pContext);

static UsbdHid s_Hid;
static UsbHid3dMouseReport_t s_Report;
static int32_t s_AccelCenter[3];
static int32_t s_GyroCenter[3];

alignas(4) static uint8_t s_HidRxBuffer[USB_INT_INTRF_PKT_BLKSIZE];
alignas(4) static uint8_t s_HidTxBuffer[USB_INT_INTRF_PKT_BLKSIZE];

static const UsbdHidCfg_t s_HidCfg = {
	.DevNo = USB_DEVNO,
	.InterfaceString = HID_STR_INTERFACE,
	.pRxBuffer = s_HidRxBuffer,
	.pTxBuffer = s_HidTxBuffer,
	.GetReportCB = HidGetReport,
};

// Application event queue memory, replaces the 4 event library default. The
// USB controller port queues its deferred endpoint events there.
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0007,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata HID 3D Mouse",
	.pFuncName = "HID 3D Mouse",
	.IntPrio = 6,
	.MaxPower = 100,
};

static int HidGetReport(USB_HID_REPTYPE Type, uint8_t ReportId,
	uint8_t *pBuffer, uint16_t BufferSize, void *)
{
	if (Type != USB_HID_REPTYPE_INPUT || ReportId != 0U ||
		BufferSize < sizeof(s_Report))
	{
		return -1;
	}
	memcpy(pBuffer, &s_Report, sizeof(s_Report));
	return sizeof(s_Report);
}

static bool ImuInit()
{
	IOPinCfg(s_PowerPins, sizeof(s_PowerPins) / sizeof(s_PowerPins[0]));
	IOPinSet(BMI323_VDD_EN_PORT, BMI323_VDD_EN_PIN);
	IOPinSet(BMI323_LS_EN_PORT, BMI323_LS_EN_PIN);
	IOPinSet(BMI323_VDDIO_EN_PORT, BMI323_VDDIO_EN_PIN);
	msDelay(10U);

	if (!s_Timer.Init(s_TimerCfg) || !s_Spi.Init(s_SpiCfg) ||
		!s_Imu.Init(s_AccelCfg, &s_Spi, &s_Timer) ||
		!s_Imu.Init(s_GyroCfg, &s_Spi, &s_Timer))
	{
		return false;
	}

	int64_t accel[3] = {};
	int64_t gyro[3] = {};
	uint32_t count = 0U;
	while (count < HID_CENTER_SAMPLES)
	{
		if (s_Imu.UpdateData())
		{
			AccelSensorRawData_t a = {};
			GyroSensorRawData_t g = {};
			s_Imu.Read(a);
			s_Imu.Read(g);
			accel[0] += a.X;
			accel[1] += a.Y;
			accel[2] += a.Z;
			gyro[0] += g.X;
			gyro[1] += g.Y;
			gyro[2] += g.Z;
			count++;
		}
		msDelay(1U);
	}
	for (int i = 0; i < 3; i++)
	{
		s_AccelCenter[i] = (int32_t)(accel[i] / HID_CENTER_SAMPLES);
		s_GyroCenter[i] = (int32_t)(gyro[i] / HID_CENTER_SAMPLES);
	}
	return true;
}

static int16_t HidAxis(int32_t Sample, int32_t Center, int32_t DeadZone,
					   int32_t Divisor)
{
	int32_t value = Sample - Center;
	if (value > DeadZone)
	{
		value -= DeadZone;
	}
	else if (value < -DeadZone)
	{
		value += DeadZone;
	}
	else
	{
		return 0;
	}
	value /= Divisor;
	if (value > INT16_MAX) value = INT16_MAX;
	if (value < -INT16_MAX) value = -INT16_MAX;
	return (int16_t)value;
}

static bool HidReportUpdate()
{
	if (!s_Imu.UpdateData())
	{
		return false;
	}
	AccelSensorRawData_t accel = {};
	GyroSensorRawData_t gyro = {};
	s_Imu.Read(accel);
	s_Imu.Read(gyro);
	s_Report.X = HidAxis(accel.X, s_AccelCenter[0], HID_ACCEL_DEAD_ZONE,
		HID_ACCEL_DIVISOR);
	s_Report.Y = HidAxis(accel.Y, s_AccelCenter[1], HID_ACCEL_DEAD_ZONE,
		HID_ACCEL_DIVISOR);
	s_Report.Z = HidAxis(accel.Z, s_AccelCenter[2], HID_ACCEL_DEAD_ZONE,
		HID_ACCEL_DIVISOR);
	s_Report.Rx = HidAxis(gyro.X, s_GyroCenter[0], HID_GYRO_DEAD_ZONE,
		HID_GYRO_DIVISOR);
	s_Report.Ry = HidAxis(gyro.Y, s_GyroCenter[1], HID_GYRO_DEAD_ZONE,
		HID_GYRO_DIVISOR);
	s_Report.Rz = HidAxis(gyro.Z, s_GyroCenter[2], HID_GYRO_DEAD_ZONE,
		HID_GYRO_DIVISOR);
	return true;
}

int main()
{
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)) ||
		!ImuInit() || !UsbInit(&s_UsbCfg) || !s_Hid.Init3dMouse(s_HidCfg))
	{
		return -1;
	}
	(void)UsbEnable(USB_DEVNO);

	bool suspended = false;
	while (1)
	{
		AppEvtHandlerExec();
		const bool nowSuspended = UsbSuspended(USB_DEVNO);
		suspended = nowSuspended;
		if (!suspended && HidReportUpdate())
		{
			(void)s_Hid.Tx(0,
				reinterpret_cast<const uint8_t *>(&s_Report), sizeof(s_Report));
		}
		msDelay(1U);
	}
	return 0;
}

