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
#include "usb/usb.h"
#include "usb/usbd_hid.h"

#include "board.h"

#define USB_DEVNO			0
#define HID_STR_INTERFACE	4U
#define HID_CENTER_SAMPLES	64U
#define HID_ACCEL_DEAD_ZONE	700
#define HID_GYRO_DEAD_ZONE	80
#define HID_ACCEL_DIVISOR	16
#define HID_GYRO_DIVISOR		4

#pragma pack(push, 1)
typedef struct __Hid_3d_Mouse_Report {
	int16_t X;
	int16_t Y;
	int16_t Z;
	int16_t Rx;
	int16_t Ry;
	int16_t Rz;
} Hid3dMouseReport_t;

#pragma pack(pop)

static_assert(sizeof(Hid3dMouseReport_t) == 12U,
	"The six 16-bit axes must form a 12-byte report");

static const uint8_t s_ReportDesc[] = {
	0x05U, 0x01U,			// Usage Page (Generic Desktop)
	0x09U, 0x08U,			// Usage (Multi-axis Controller)
	0xA1U, 0x01U,			// Collection (Application)
	0x16U, 0x01U, 0x80U,	// Logical Minimum (-32767)
	0x26U, 0xFFU, 0x7FU,	// Logical Maximum (32767)
	0x75U, 0x10U,			// Report Size (16)
	0x95U, 0x06U,			// Report Count (6)
	0x09U, 0x30U,			// Usage (X)
	0x09U, 0x31U,			// Usage (Y)
	0x09U, 0x32U,			// Usage (Z)
	0x09U, 0x33U,			// Usage (Rx)
	0x09U, 0x34U,			// Usage (Ry)
	0x09U, 0x35U,			// Usage (Rz)
	0x81U, 0x06U,			// Input (Data, Variable, Relative)
	0xC0U,
};

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

static SPI g_Spi;
static Timer g_Timer;
static AgBmi323 g_Imu;
static bool HidReportRequest(const UsbSetupData_t *pSetup,
							 UsbCtrlStage_t Stage, uint8_t **ppData,
							 uint16_t *pLength);

class Hid3dMouse final : public UsbdHid {
public:
	bool Control(const UsbSetupData_t *pSetup, UsbCtrlStage_t Stage,
				 uint8_t **ppData, uint16_t *pLength) override {
		if (pSetup != nullptr &&
			(pSetup->bmRequestType & USB_REQTYPE_MASK_TYPE) ==
				USB_REQTYPE_CLASS &&
			pSetup->bRequest == USB_HID_REQ_GET_REPORT)
		{
			return HidReportRequest(pSetup, Stage, ppData, pLength);
		}
		return UsbdHid::Control(pSetup, Stage, ppData, pLength);
	}
};

static Hid3dMouse g_Hid;
static Hid3dMouseReport_t s_Report;
static int32_t s_AccelCenter[3];
static int32_t s_GyroCenter[3];

static bool HidReportRequest(const UsbSetupData_t *pSetup,
							 UsbCtrlStage_t Stage, uint8_t **ppData,
							 uint16_t *pLength)
{
	if (pSetup == nullptr || pLength == nullptr ||
		pSetup->bRequest != USB_HID_REQ_GET_REPORT ||
		(pSetup->wValue & USB_HID_REPTYPE_MASK) != USB_HID_REPTYPE_INPUT ||
		(pSetup->wValue & USB_HID_REPID_MASK) != 0U)
	{
		return false;
	}
	if (Stage == USB_CTRL_SETUP)
	{
		if (ppData == nullptr)
		{
			return false;
		}
		*ppData = reinterpret_cast<uint8_t *>(&s_Report);
		*pLength = sizeof(s_Report);
	}
	return true;
}

static const UsbdHidCfg_t s_HidCfg = {
	.DevNo = USB_DEVNO,
	.pReportDesc = s_ReportDesc,
	.ReportDescLength = sizeof(s_ReportDesc),
	.BcdHid = 0U,
	.FsMps = sizeof(Hid3dMouseReport_t),
	.HsMps = sizeof(Hid3dMouseReport_t),
	.FsInterval = 1U,
	.HsInterval = 4U,
	.SubClass = USB_HID_SUBCLASS_NONE,
	.Protocol = USB_HID_PROT_NONE,
	.CountryCode = 0U,
	.InterfaceString = HID_STR_INTERFACE,
	.RxHandler = nullptr,
	.TxHandler = nullptr,
	.pContext = nullptr,
};

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x0007,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata HID 3D Mouse",
	.pSerial = nullptr,
	.pFuncName = "HID 3D Mouse",
	.IntPrio = 6,
	.DeviceClass = USB_DEVCLASS_NONE,
	.DeviceSubClass = 0U,
	.DeviceProtocol = 0U,
	.bSelfPowered = false,
	.bRemoteWakeup = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

static bool ImuInit()
{
	IOPinCfg(s_PowerPins, sizeof(s_PowerPins) / sizeof(s_PowerPins[0]));
	IOPinSet(BMI323_VDD_EN_PORT, BMI323_VDD_EN_PIN);
	IOPinSet(BMI323_LS_EN_PORT, BMI323_LS_EN_PIN);
	IOPinSet(BMI323_VDDIO_EN_PORT, BMI323_VDDIO_EN_PIN);
	msDelay(10U);

	if (!g_Timer.Init(s_TimerCfg) || !g_Spi.Init(s_SpiCfg) ||
		!g_Imu.Init(s_AccelCfg, &g_Spi, &g_Timer) ||
		!g_Imu.Init(s_GyroCfg, &g_Spi, &g_Timer))
	{
		return false;
	}

	int64_t accel[3] = {};
	int64_t gyro[3] = {};
	uint32_t count = 0U;
	while (count < HID_CENTER_SAMPLES)
	{
		if (g_Imu.UpdateData())
		{
			AccelSensorRawData_t a = {};
			GyroSensorRawData_t g = {};
			g_Imu.Read(a);
			g_Imu.Read(g);
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
	if (!g_Imu.UpdateData())
	{
		return false;
	}
	AccelSensorRawData_t accel = {};
	GyroSensorRawData_t gyro = {};
	g_Imu.Read(accel);
	g_Imu.Read(gyro);
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
	if (!ImuInit() || !UsbInit(&s_UsbCfg) || !g_Hid.Init(s_HidCfg))
	{
		return -1;
	}
	(void)UsbEnable(USB_DEVNO);

	bool suspended = false;
	while (1)
	{
		UsbProcess(USB_DEVNO);
		const bool nowSuspended = UsbSuspended(USB_DEVNO);
		if (nowSuspended != suspended)
		{
			suspended = nowSuspended;
			if (suspended) g_Hid.Suspend();
			else (void)g_Hid.Resume();
		}
		if (!suspended && g_Hid.TxReady() && HidReportUpdate())
		{
			(void)g_Hid.SendReport(
				reinterpret_cast<const uint8_t *>(&s_Report), sizeof(s_Report));
		}
		msDelay(1U);
	}
	return 0;
}
