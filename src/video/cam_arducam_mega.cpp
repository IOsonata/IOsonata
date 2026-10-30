/**-------------------------------------------------------------------------
@file	cam_arducam_mega.cpp

@brief	ArduCAM Mega SPI camera module driver

The frame is captured in the frame memory of the module, then read over SPI
in bursts of any length.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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
#include "idelay.h"
#include "istddef.h"
#include "video/cam_arducam_mega.h"

// Status polling, 1 s bound. A sensor reset loads the whole sensor setup.
#define ARDUCAM_MEGA_POLL_US			100
#define ARDUCAM_MEGA_POLL_MAX			10000

// Bound of the wait for the module to start after power up, 100 ms specified
#define ARDUCAM_MEGA_BOOT_POLL_MAX		2000

#define ARDUCAM_MEGA_TRANSFER_POLL_MAX	100000

// Sensor bits of the resolution table
#define ARDUCAM_MEGA_RES_2MP			(1<<0)
#define ARDUCAM_MEGA_RES_3MP			(1<<1)
#define ARDUCAM_MEGA_RES_5MP			(1<<2)
#define ARDUCAM_MEGA_RES_OLD_3MP		(1<<3)
#define ARDUCAM_MEGA_RES_OLD_5MP		(1<<4)
#define ARDUCAM_MEGA_RES_CUR			(ARDUCAM_MEGA_RES_2MP | ARDUCAM_MEGA_RES_3MP | ARDUCAM_MEGA_RES_5MP)
#define ARDUCAM_MEGA_RES_OLD			(ARDUCAM_MEGA_RES_OLD_3MP | ARDUCAM_MEGA_RES_OLD_5MP)

#define ARDUCAM_MEGA_RES_NONE			0xFF	//!< Resolution register not written

typedef struct __ArduCam_Mega_Res {
	uint16_t Width;
	uint16_t Height;
	uint8_t Code;		//!< Resolution code of the current sensors
	uint8_t OldCode;	//!< Resolution code of the older sensors
	uint8_t Sensors;	//!< ARDUCAM_MEGA_RES_ sensors supporting it
} ArduCamMegaRes_t;

typedef struct __ArduCam_Mega_Sensor_Reg {
	uint16_t Reg;
	uint16_t Val;
} ArduCamMegaSensorReg_t;

static const ArduCamMegaRes_t s_ArduCamMegaRes[] = {
	{   96,   96, 0x00, 0x0A, ARDUCAM_MEGA_RES_CUR | ARDUCAM_MEGA_RES_OLD },
	{  128,  128, 0x01, 0x0B, ARDUCAM_MEGA_RES_CUR | ARDUCAM_MEGA_RES_OLD },
	{  160,  120, 0x02, 0x00, ARDUCAM_MEGA_RES_CUR },
	{  320,  240, 0x03, 0x01, ARDUCAM_MEGA_RES_CUR | ARDUCAM_MEGA_RES_OLD },
	{  320,  320, 0x04, 0x0C, ARDUCAM_MEGA_RES_CUR | ARDUCAM_MEGA_RES_OLD },
	{  640,  480, 0x05, 0x02, ARDUCAM_MEGA_RES_CUR | ARDUCAM_MEGA_RES_OLD },
	{  800,  600, 0x06, 0x00, ARDUCAM_MEGA_RES_CUR },
	{ 1024,  768, 0x07, 0x00, ARDUCAM_MEGA_RES_CUR },
	{ 1280,  720, 0x08, 0x04, ARDUCAM_MEGA_RES_CUR | ARDUCAM_MEGA_RES_OLD },
	{ 1280, 1024, 0x09, 0x00, ARDUCAM_MEGA_RES_CUR },
	{ 1600, 1200, 0x0A, 0x06, ARDUCAM_MEGA_RES_CUR | ARDUCAM_MEGA_RES_OLD },
	{ 1920, 1080, 0x0B, 0x07, ARDUCAM_MEGA_RES_3MP | ARDUCAM_MEGA_RES_5MP | ARDUCAM_MEGA_RES_OLD },
	{ 2048, 1536, 0x0C, 0x08, ARDUCAM_MEGA_RES_3MP | ARDUCAM_MEGA_RES_5MP | ARDUCAM_MEGA_RES_OLD_3MP },
	{ 2592, 1944, 0x0D, 0x09, ARDUCAM_MEGA_RES_5MP | ARDUCAM_MEGA_RES_OLD_5MP },
};

static const int s_NbArduCamMegaRes = sizeof(s_ArduCamMegaRes) / sizeof(ArduCamMegaRes_t);

// Low power of the 0x83 and 0x84 sensors : registers of the sensor written
// through the debug registers, values of the vendor library. The last
// register is 1 for low power, 0 for normal.
static const ArduCamMegaSensorReg_t s_ArduCamMegaLowPower[] = {
	{ 0x0028, 0xD000 },
	{ 0x002A, 0x107E },
	{ 0x0F12, 0 },
};

static const int s_NbArduCamMegaLowPower = sizeof(s_ArduCamMegaLowPower) / sizeof(ArduCamMegaSensorReg_t);

CamArducamMega::CamArducamMega()
{
	vbTransferPending = false;
	vbTransferDone = false;
	vTransferCount = 0;
	vbTransferFailed = false;
	vSensorId = 0;
	vResMask = 0;
	vResIdx = 0;
	vRegFmt = 0;
	vRegRes = ARDUCAM_MEGA_RES_NONE;
	vbRun = false;
	vbCapture = false;
	vbFirstBurst = true;
	vRemain = 0;
}

bool CamArducamMega::Init(const VideoDevCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (pIntrf == nullptr || (vbTransferPending && !vbTransferDone))
	{
		return false;
	}

	DevIntrf_t *intrf = *pIntrf;
	if (intrf->bIntEn && intrf->EvtCB == nullptr)
	{
		return false;
	}
	vbTransferPending = false;
	vbTransferFailed = false;
	Valid(false);
	Interface(pIntrf);
	DeviceAddress(Cfg.DevAddr);
	vpTimer = pTimer;
	SetEvtHandler(Cfg.EvtHandler);

	// Wait for the module to start, no answer when it is not there
	bool ready = false;

	for (int i = 0; i < ARDUCAM_MEGA_BOOT_POLL_MAX && ready == false; i++)
	{
		ready = (RegRead(ARDUCAM_MEGA_STATUS_REG) & ARDUCAM_MEGA_STATUS_SENSOR_MASK) == ARDUCAM_MEGA_STATUS_SENSOR_IDLE;
		if (ready == false)
		{
			usDelay(ARDUCAM_MEGA_POLL_US);
		}
	}

	if (ready == false || SensorReset() == false)
	{
		return false;
	}

	uint8_t id = RegRead(ARDUCAM_MEGA_SENSOR_ID_REG);

	switch (id)
	{
		case ARDUCAM_MEGA_SENSOR_ID_5MP_1:
		case ARDUCAM_MEGA_SENSOR_ID_5MP_2:
			vResMask = ARDUCAM_MEGA_RES_OLD_5MP;
			break;
		case ARDUCAM_MEGA_SENSOR_ID_3MP_1:
		case ARDUCAM_MEGA_SENSOR_ID_3MP_2:
			vResMask = ARDUCAM_MEGA_RES_OLD_3MP;
			break;
		case ARDUCAM_MEGA_SENSOR_ID_5MP:
			vResMask = ARDUCAM_MEGA_RES_5MP;
			break;
		case ARDUCAM_MEGA_SENSOR_ID_3MP:
			vResMask = ARDUCAM_MEGA_RES_3MP;
			break;
		case ARDUCAM_MEGA_SENSOR_ID_2MP:
			vResMask = ARDUCAM_MEGA_RES_2MP;
			break;
		default:
			return false;
	}

	vSensorId = id;

	uint64_t devid = id;

	devid |= (uint64_t)RegRead(ARDUCAM_MEGA_FPGA_VERSION_REG) << 8;
	devid |= (uint64_t)(RegRead(ARDUCAM_MEGA_YEAR_REG) & 0x3F) << 16;
	devid |= (uint64_t)(RegRead(ARDUCAM_MEGA_MONTH_REG) & 0x0F) << 24;
	devid |= (uint64_t)(RegRead(ARDUCAM_MEGA_DAY_REG) & 0x1F) << 32;
	DeviceID(devid);

	if (CtrlWrite(ARDUCAM_MEGA_DBG_DEVADDR_REG, ARDUCAM_MEGA_DBG_DEVADDR) == false)
	{
		return false;
	}

	Mode(Cfg.OpMode);
	Format(VIDEO_STREAM_OUT, Cfg.OutFmt);

	Valid(true);

	return true;
}

bool CamArducamMega::Enable()
{
	if (vSensorId == 0)
	{
		return false;
	}

	return LowPower(false);
}

void CamArducamMega::Disable()
{
	if (vSensorId == 0)
	{
		return;
	}

	Stop();
	LowPower(true);
}

void CamArducamMega::Reset()
{
	Stop();
	SensorReset();

	vRegFmt = 0;
	vRegRes = ARDUCAM_MEGA_RES_NONE;
	vRemain = 0;
	vFrameLen = 0;
}

bool CamArducamMega::Start()
{
	if (Valid() == false)
	{
		return false;
	}

	uint8_t fmt;

	switch (vFmt[VIDEO_STREAM_OUT].Fmt)
	{
		case VIDEO_FMT_RGB565:
			fmt = ARDUCAM_MEGA_FORMAT_RGB565;
			break;
		case VIDEO_FMT_YUV422:
			fmt = ARDUCAM_MEGA_FORMAT_YUV;
			break;
		default:
			fmt = ARDUCAM_MEGA_FORMAT_JPEG;
	}

	const ArduCamMegaRes_t &r = s_ArduCamMegaRes[vResIdx];
	uint8_t res = vSensorId < ARDUCAM_MEGA_SENSOR_ID_5MP ? r.OldCode : r.Code;

	if (vOpMode == VIDEO_OPMODE_CONTINUOUS)
	{
		res |= ARDUCAM_MEGA_RESOLUTION_VIDEO;
	}

	if (fmt != vRegFmt)
	{
		if (CtrlWrite(ARDUCAM_MEGA_FORMAT_REG, fmt) == false)
		{
			return false;
		}
		vRegFmt = fmt;

		// The vendor library writes the resolution again after a format change
		vRegRes = ARDUCAM_MEGA_RES_NONE;
	}

	if (res != vRegRes)
	{
		if (CtrlWrite(ARDUCAM_MEGA_RESOLUTION_REG, res) == false)
		{
			return false;
		}
		vRegRes = res;
	}

	vFrameLen = 0;
	vRemain = 0;
	vbRun = true;

	return StartFrame();
}

void CamArducamMega::Stop()
{
	vbRun = false;
	vbCapture = false;
}

bool CamArducamMega::UpdateData()
{
	if (vbCapture == false)
	{
		return false;
	}

	if ((RegRead(ARDUCAM_MEGA_STATUS_REG) & ARDUCAM_MEGA_STATUS_CAPTURE_DONE) == 0)
	{
		return false;
	}

	vbCapture = false;

	uint32_t len = RegRead(ARDUCAM_MEGA_FIFO_SIZE_REG);

	len |= (uint32_t)RegRead(ARDUCAM_MEGA_FIFO_SIZE_REG + 1) << 8;
	len |= (uint32_t)RegRead(ARDUCAM_MEGA_FIFO_SIZE_REG + 2) << 16;

	if (len == 0)
	{
		if (vbRun && vOpMode == VIDEO_OPMODE_CONTINUOUS)
		{
			StartFrame();
		}

		return false;
	}

	vFrameLen = len;
	vRemain = len;
	vbFirstBurst = true;
	vFrameTime = vpTimer != nullptr ? vpTimer->uSecond() : 0;

	EvtHandler(DEV_EVT_DATA_RDY);

	return true;
}

int CamArducamMega::ReadFrame(uint8_t *pBuff, int Len)
{
	int cnt = 0;
	int n;
	uint8_t cmd[2] = { ARDUCAM_MEGA_BURST_FIFO_READ, 0 };

	if (pBuff == nullptr || vRemain == 0 || Len <= 0)
	{
		return 0;
	}

	if ((uint32_t)Len > vRemain)
	{
		Len = (int)vRemain;
	}

	while (cnt < Len)
	{
		// Keep DMA storage in the camera until its completion, including a
		// late completion after timeout. The first burst alone has a dummy.
		n = ReadTransfer(cmd, vbFirstBurst ? 2 : 1, pBuff + cnt, Len - cnt);
		if (n <= 0)
		{
			break;
		}
		vbFirstBurst = false;
		vRemain -= n;
		cnt += n;
	}

	if (vRemain == 0)
	{
		vFrameLen = 0;

		if (vbRun && vOpMode == VIDEO_OPMODE_CONTINUOUS)
		{
			StartFrame();
		}
	}

	return cnt;
}

const VideoFmt_t &CamArducamMega::Format(VIDEO_STREAM Stream, const VideoFmt_t &Fmt)
{
	if (Stream != VIDEO_STREAM_OUT)
	{
		// No input stream
		return vFmt[Stream];
	}

	VideoFmt_t &f = vFmt[VIDEO_STREAM_OUT];

	switch (Fmt.Fmt)
	{
		case VIDEO_FMT_JPEG:
		case VIDEO_FMT_RGB565:
		case VIDEO_FMT_YUV422:
			f.Fmt = Fmt.Fmt;
			break;
		default:
			if (f.Fmt == VIDEO_FMT_NONE)
			{
				f.Fmt = VIDEO_FMT_JPEG;
			}
	}

	// Closest resolution supported by the sensor
	uint32_t best = UINT32_MAX;

	for (int i = 0; i < s_NbArduCamMegaRes; i++)
	{
		const ArduCamMegaRes_t &r = s_ArduCamMegaRes[i];

		if (r.Sensors & vResMask)
		{
			int32_t dw = (int32_t)r.Width - Fmt.Width;
			int32_t dh = (int32_t)r.Height - Fmt.Height;
			uint32_t d = (uint32_t)(dw < 0 ? -dw : dw) + (uint32_t)(dh < 0 ? -dh : dh);

			if (d < best)
			{
				best = d;
				vResIdx = i;
			}
		}
	}

	if (best != UINT32_MAX)
	{
		f.Width = s_ArduCamMegaRes[vResIdx].Width;
		f.Height = s_ArduCamMegaRes[vResIdx].Height;
	}

	// Frame rate not adjustable
	f.FrameRate = 0;

	return f;
}

int CamArducamMega::Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen)
{
	if (pCmdAddr == nullptr || CmdAddrLen <= 0)
	{
		return 0;
	}

	uint8_t cmd[2] = { (uint8_t)(pCmdAddr[0] & ~ARDUCAM_MEGA_REG_WR), 0 };

	return ReadTransfer(cmd, 2, pBuff, BuffLen);
}

int CamArducamMega::Write(uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData, int DataLen)
{
	int n;

	if (pCmdAddr == nullptr || CmdAddrLen <= 0 || pData == nullptr || DataLen <= 0 ||
		DataLen >= (int)sizeof(vTransferBuff) || !TransferReady())
	{
		return 0;
	}
	vTransferBuff[0] = pCmdAddr[0] | ARDUCAM_MEGA_REG_WR;
	memcpy(vTransferBuff + 1, pData, DataLen);
	vbTransferPending = true;
	vbTransferDone = false;
	n = TransferWait(vpIntrf->Tx(vDevAddr, vTransferBuff, DataLen + 1));
	return n > 1 ? n - 1 : 0;
}

// The owner of the SPI supplies the callback; the camera never replaces it.
void CamArducamMega::IntrfEvent(DEVINTRF_EVT Evt, int Len)
{
	if (!vbTransferPending)
	{
		return;
	}
	if (Evt == DEVINTRF_EVT_COMPLETED || Evt == DEVINTRF_EVT_TX_TIMEOUT ||
		Evt == DEVINTRF_EVT_RX_TIMEOUT)
	{
		vTransferCount = Evt == DEVINTRF_EVT_COMPLETED ? Len : 0;
		if (Evt != DEVINTRF_EVT_COMPLETED)
		{
			vbTransferFailed = true;
		}
		vbTransferDone = true;
	}
}

bool CamArducamMega::TransferReady()
{
	return !vbTransferFailed && !vbTransferPending;
}

int CamArducamMega::TransferWait(int Result)
{
	if (Result < 0)
	{
		for (int i = 0; i < ARDUCAM_MEGA_TRANSFER_POLL_MAX && !vbTransferDone; i++)
		{
			usDelay(1);
		}
		if (!vbTransferDone)
		{
			// Keep the buffer reserved. Reinitialization is allowed only after
			// completion; returning must not expose caller memory to late DMA.
			vbTransferFailed = true;
			return 0;
		}
		Result = vTransferCount;
	}
	vbTransferPending = false;
	return Result;
}

int CamArducamMega::ReadTransfer(const uint8_t *pCmd, int CmdLen, uint8_t *pBuff, int Len)
{
	DevIntrf_t *intrf = *vpIntrf;
	int count;

	if (pBuff == nullptr || Len <= 0 || !TransferReady())
	{
		return 0;
	}
	if (!intrf->bIntEn)
	{
		return vpIntrf->Read(vDevAddr, pCmd, CmdLen, pBuff, Len);
	}
	Len = min(Len, (int)sizeof(vTransferBuff));
	vbTransferPending = true;
	vbTransferDone = false;
	count = TransferWait(vpIntrf->Read(vDevAddr, pCmd, CmdLen, vTransferBuff, Len));
	if (count > 0 && count <= Len)
	{
		memcpy(pBuff, vTransferBuff, count);
		return count;
	}
	return 0;
}

uint8_t CamArducamMega::RegRead(uint8_t Reg)
{
	return Read8(&Reg, 1);
}

bool CamArducamMega::RegWrite(uint8_t Reg, uint8_t Val)
{
	return Write8(&Reg, 1, Val);
}

// Write a register forwarded to the sensor, wait until the module is done
bool CamArducamMega::CtrlWrite(uint8_t Reg, uint8_t Val)
{
	if (RegWrite(Reg, Val) == false)
	{
		return false;
	}

	return WaitStatus(ARDUCAM_MEGA_STATUS_SENSOR_MASK, ARDUCAM_MEGA_STATUS_SENSOR_IDLE);
}

// Level code : 0 default, +n is 2n - 1, -n is 2n
bool CamArducamMega::CtrlLevel(uint8_t Reg, int Level, int Max)
{
	if (Level > Max)
	{
		Level = Max;
	}
	else if (Level < -Max)
	{
		Level = -Max;
	}

	uint8_t code = Level > 0 ? (uint8_t)(Level * 2 - 1) : (uint8_t)(-Level * 2);

	return CtrlWrite(Reg, code);
}

bool CamArducamMega::AutoCtrl(uint8_t Sel, bool bEn)
{
	if (CtrlWrite(ARDUCAM_MEGA_AUTO_CTRL_REG, Sel | (bEn ? ARDUCAM_MEGA_AUTO_CTRL_EN : 0)) == false)
	{
		return false;
	}

	if (vSensorId >= ARDUCAM_MEGA_SENSOR_ID_5MP)
	{
		return WaitStatus(ARDUCAM_MEGA_STATUS_AUTO_BUSY, 0);
	}

	return true;
}

bool CamArducamMega::WaitStatus(uint8_t Mask, uint8_t Val)
{
	for (int i = 0; i < ARDUCAM_MEGA_POLL_MAX; i++)
	{
		if ((RegRead(ARDUCAM_MEGA_STATUS_REG) & Mask) == Val)
		{
			return true;
		}
		usDelay(ARDUCAM_MEGA_POLL_US);
	}

	return false;
}

bool CamArducamMega::LowPower(bool bEn)
{
	if (vSensorId == ARDUCAM_MEGA_SENSOR_ID_5MP_2 || vSensorId == ARDUCAM_MEGA_SENSOR_ID_3MP_2)
	{
		for (int i = 0; i < s_NbArduCamMegaLowPower; i++)
		{
			uint16_t reg = s_ArduCamMegaLowPower[i].Reg;
			uint16_t val = i < s_NbArduCamMegaLowPower - 1 ? s_ArduCamMegaLowPower[i].Val : (bEn ? 1 : 0);

			if (CtrlWrite(ARDUCAM_MEGA_DBG_REGADDR_HI_REG, reg >> 8) == false ||
				CtrlWrite(ARDUCAM_MEGA_DBG_REGADDR_LO_REG, reg & 0xFF) == false ||
				CtrlWrite(ARDUCAM_MEGA_DBG_VAL_HI_REG, val >> 8) == false ||
				CtrlWrite(ARDUCAM_MEGA_DBG_VAL_LO_REG, val & 0xFF) == false)
			{
				return false;
			}
		}

		return true;
	}

	if (vSensorId >= ARDUCAM_MEGA_SENSOR_ID_5MP)
	{
		return CtrlWrite(ARDUCAM_MEGA_POWER_MODE_REG, bEn ? ARDUCAM_MEGA_POWER_MODE_LOW : ARDUCAM_MEGA_POWER_MODE_NORMAL);
	}

	return RegWrite(ARDUCAM_MEGA_POWER_CTRL_REG, bEn ? ARDUCAM_MEGA_POWER_CTRL_LOW : ARDUCAM_MEGA_POWER_CTRL_NORMAL);
}

bool CamArducamMega::SensorReset()
{
	if (RegWrite(ARDUCAM_MEGA_RESET_REG, ARDUCAM_MEGA_RESET_SENSOR) == false)
	{
		return false;
	}

	return WaitStatus(ARDUCAM_MEGA_STATUS_SENSOR_MASK, ARDUCAM_MEGA_STATUS_SENSOR_IDLE);
}

bool CamArducamMega::StartFrame()
{
	vbFirstBurst = true;

	if (RegWrite(ARDUCAM_MEGA_FIFO_CTRL_REG, ARDUCAM_MEGA_FIFO_CTRL_CLEAR_DONE) == false ||
		RegWrite(ARDUCAM_MEGA_FIFO_CTRL_REG, ARDUCAM_MEGA_FIFO_CTRL_START) == false)
	{
		return false;
	}

	vbCapture = true;

	return true;
}
