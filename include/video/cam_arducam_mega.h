/**-------------------------------------------------------------------------
@file	cam_arducam_mega.h

@brief	Definitions for the ArduCAM Mega SPI camera modules

The ArduCAM Mega modules (2MP, 3MP, 5MP) are a CMOS image sensor behind a
controller holding the captured frame in its own frame memory (FIFO). The
MCU reaches the controller on SPI, mode 0, 8 MHz maximum. The controller
drives the sensor on its own I2C bus. The module has no interrupt line, the
capture done flag is polled.

SPI protocol, the register map and the sensor differences below follow the
ArduCAM Arducam_Mega library (MIT).
	Register write : address | 0x80, value
	Register read  : address, dummy byte, value
	Frame read     : 0x3C, then the frame data. A dummy byte precedes the data
					 on the first burst after a capture only. Following bursts
					 continue where the previous one stopped.

The sensor ID register tells the sensor:
	0x81 5MP, 0x82 3MP, 0x83 5MP, 0x84 3MP : older sensors. Other resolution
	codes, a subset of the resolutions, sharpness instead of the power mode
	register, no AE/AWB freeze.
	0x85 5MP, 0x86 3MP, 0x87 2MP : current sensors.
Largest resolution : 2592x1944 for 5MP, 2048x1536 for 3MP, 1600x1200 for 2MP.

The frame length is the FIFO length reported by the controller. For JPEG the
vendor examples stop at the end of image marker FF D9, the bytes after it are
not part of the image. The vendor library always uses JPEG in the video mode
(VIDEO_OPMODE_CONTINUOUS).

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
#ifndef __CAM_ARDUCAM_MEGA_H__
#define __CAM_ARDUCAM_MEGA_H__

#include <stdint.h>

#include "video/videodev.h"

/** @addtogroup Video
  * @{
  */

#define ARDUCAM_MEGA_SPI_RATE_MAX			8000000	//!< Hz, SPI mode 0
#define ARDUCAM_MEGA_REG_WR									(1<<7)	//!< Set in the register address for a write

#define ARDUCAM_MEGA_FRAMES_REG				0x01	//!< Number of frames per capture
#define ARDUCAM_MEGA_POWER_CTRL_REG			0x02	//!< Older sensors
#define ARDUCAM_MEGA_POWER_CTRL_LOW								0x07	//!< Low power
#define ARDUCAM_MEGA_POWER_CTRL_NORMAL							0x05
#define ARDUCAM_MEGA_FIFO_CTRL_REG			0x04
#define ARDUCAM_MEGA_FIFO_CTRL_CLEAR_DONE						(1<<0)	//!< Clear the capture done flag
#define ARDUCAM_MEGA_FIFO_CTRL_START							(1<<1)	//!< Start a capture
#define ARDUCAM_MEGA_RESET_REG				0x07
#define ARDUCAM_MEGA_RESET_SENSOR_READ							(1<<0)	//!< Read the sensor register selected by the debug registers
#define ARDUCAM_MEGA_RESET_SENSOR								(1<<6)	//!< Reset the sensor
#define ARDUCAM_MEGA_RESET_FIFO									(1<<7)	//!< Clear the frame memory
#define ARDUCAM_MEGA_DBG_DEVADDR_REG		0x0A	//!< Sensor I2C address used by the debug registers
#define ARDUCAM_MEGA_DBG_DEVADDR								0x78
#define ARDUCAM_MEGA_DBG_REGADDR_HI_REG		0x0B	//!< Sensor register address, high byte
#define ARDUCAM_MEGA_DBG_REGADDR_LO_REG		0x0C	//!< Sensor register address, low byte
#define ARDUCAM_MEGA_DBG_VAL_LO_REG			0x0D	//!< Sensor register value, low byte, written last
#define ARDUCAM_MEGA_DBG_VAL_HI_REG			0x0E	//!< Sensor register value, high byte
#define ARDUCAM_MEGA_FORMAT_REG				0x20
#define ARDUCAM_MEGA_FORMAT_JPEG								1
#define ARDUCAM_MEGA_FORMAT_RGB565								2
#define ARDUCAM_MEGA_FORMAT_YUV									3		//!< YUV 4:2:2
#define ARDUCAM_MEGA_RESOLUTION_REG			0x21	//!< Resolution code, see the table in the driver
#define ARDUCAM_MEGA_RESOLUTION_VIDEO							(1<<7)	//!< Video mode, else still picture
#define ARDUCAM_MEGA_BRIGHTNESS_REG			0x22	//!< Level code, -4 to +4
#define ARDUCAM_MEGA_CONTRAST_REG			0x23	//!< Level code, -3 to +3
#define ARDUCAM_MEGA_SATURATION_REG			0x24	//!< Level code, -3 to +3
#define ARDUCAM_MEGA_EV_REG					0x25	//!< Exposure compensation, level code, -3 to +3
#define ARDUCAM_MEGA_WHITE_BALANCE_REG		0x26	//!< ARDUCAM_MEGA_WB
#define ARDUCAM_MEGA_COLOR_EFFECT_REG		0x27	//!< ARDUCAM_MEGA_EFFECT
#define ARDUCAM_MEGA_SHARPNESS_REG			0x28	//!< Older sensors
#define ARDUCAM_MEGA_POWER_MODE_REG			0x28	//!< Current sensors
#define ARDUCAM_MEGA_POWER_MODE_NORMAL							0
#define ARDUCAM_MEGA_POWER_MODE_LOW								1
#define ARDUCAM_MEGA_AUTO_FOCUS_REG			0x29	//!< 5MP with auto focus lens
#define ARDUCAM_MEGA_QUALITY_REG			0x2A	//!< ARDUCAM_MEGA_QUALITY
#define ARDUCAM_MEGA_ROTATION_REG			0x2B
#define ARDUCAM_MEGA_ROTATION_180								1
#define ARDUCAM_MEGA_AE_FREEZE_REG			0x2C	//!< Current sensors
#define ARDUCAM_MEGA_AWB_FREEZE_REG			0x2D	//!< Current sensors
#define ARDUCAM_MEGA_AUTO_CTRL_REG			0x30
#define ARDUCAM_MEGA_AUTO_CTRL_GAIN								0		//!< Select the automatic gain
#define ARDUCAM_MEGA_AUTO_CTRL_EXPOSURE							1		//!< Select the automatic exposure
#define ARDUCAM_MEGA_AUTO_CTRL_WHITE_BALANCE					2		//!< Select the automatic white balance
#define ARDUCAM_MEGA_AUTO_CTRL_EN								(1<<7)	//!< Enable the selected automatic control
#define ARDUCAM_MEGA_GAIN_HI_REG			0x31	//!< Manual gain bits 9:8
#define ARDUCAM_MEGA_GAIN_LO_REG			0x32	//!< Manual gain bits 7:0
#define ARDUCAM_MEGA_EXPOSURE_HI_REG		0x33	//!< Manual exposure bits 19:16, older sensors
#define ARDUCAM_MEGA_EXPOSURE_MID_REG		0x34	//!< Manual exposure bits 15:8
#define ARDUCAM_MEGA_EXPOSURE_LO_REG		0x35	//!< Manual exposure bits 7:0
#define ARDUCAM_MEGA_BURST_FIFO_READ		0x3C	//!< Command, the frame data follow
#define ARDUCAM_MEGA_SINGLE_FIFO_READ		0x3D	//!< Command, one frame byte follows
#define ARDUCAM_MEGA_SENSOR_ID_REG			0x40
#define ARDUCAM_MEGA_SENSOR_ID_5MP_1							0x81
#define ARDUCAM_MEGA_SENSOR_ID_3MP_1							0x82
#define ARDUCAM_MEGA_SENSOR_ID_5MP_2							0x83
#define ARDUCAM_MEGA_SENSOR_ID_3MP_2							0x84
#define ARDUCAM_MEGA_SENSOR_ID_5MP								0x85	//!< First current sensor, older below
#define ARDUCAM_MEGA_SENSOR_ID_3MP								0x86
#define ARDUCAM_MEGA_SENSOR_ID_2MP								0x87
#define ARDUCAM_MEGA_YEAR_REG				0x41	//!< Controller firmware date, bits 5:0
#define ARDUCAM_MEGA_MONTH_REG				0x42	//!< Bits 3:0
#define ARDUCAM_MEGA_DAY_REG				0x43	//!< Bits 4:0
#define ARDUCAM_MEGA_STATUS_REG				0x44
#define ARDUCAM_MEGA_STATUS_SENSOR_MASK							(3<<0)	//!< Sensor I2C state
#define ARDUCAM_MEGA_STATUS_SENSOR_IDLE							(2<<0)	//!< Sensor I2C idle, register write done
#define ARDUCAM_MEGA_STATUS_CAPTURE_DONE						(1<<2)	//!< Frame captured in the FIFO
#define ARDUCAM_MEGA_STATUS_AUTO_BUSY							(1<<3)	//!< Automatic control change in progress, current sensors
#define ARDUCAM_MEGA_FIFO_SIZE_REG			0x45	//!< Frame length bits 7:0, then 0x46 bits 15:8, 0x47 bits 23:16
#define ARDUCAM_MEGA_SENSOR_DATA_REG		0x48	//!< Sensor register value of ARDUCAM_MEGA_RESET_SENSOR_READ
#define ARDUCAM_MEGA_FPGA_VERSION_REG		0x49

#define ARDUCAM_MEGA_LEVEL_MAX				3		//!< Contrast, saturation, exposure compensation
#define ARDUCAM_MEGA_BRIGHTNESS_LEVEL_MAX	4

/// White balance modes
typedef enum __ArduCam_Mega_White_Balance {
	ARDUCAM_MEGA_WB_AUTO,
	ARDUCAM_MEGA_WB_SUNNY,
	ARDUCAM_MEGA_WB_OFFICE,
	ARDUCAM_MEGA_WB_CLOUDY,
	ARDUCAM_MEGA_WB_HOME,
} ARDUCAM_MEGA_WB;

/// Color effects, the supported set depends on the sensor
typedef enum __ArduCam_Mega_Effect {
	ARDUCAM_MEGA_EFFECT_NONE,
	ARDUCAM_MEGA_EFFECT_BLUEISH,
	ARDUCAM_MEGA_EFFECT_REDISH,
	ARDUCAM_MEGA_EFFECT_BW,
	ARDUCAM_MEGA_EFFECT_SEPIA,
	ARDUCAM_MEGA_EFFECT_NEGATIVE,
	ARDUCAM_MEGA_EFFECT_GREENISH,
	ARDUCAM_MEGA_EFFECT_OVER_EXPOSURE,
	ARDUCAM_MEGA_EFFECT_SOLARIZE,
} ARDUCAM_MEGA_EFFECT;

/// JPEG quality
typedef enum __ArduCam_Mega_Quality {
	ARDUCAM_MEGA_QUALITY_HIGH,
	ARDUCAM_MEGA_QUALITY_DEFAULT,
	ARDUCAM_MEGA_QUALITY_LOW,
} ARDUCAM_MEGA_QUALITY;

#ifdef __cplusplus

/// ArduCAM Mega camera module, output stream only
class CamArducamMega : public VideoDevice {
public:
	CamArducamMega();

	/**
	 * @brief	Initialize the camera module.
	 *
	 * Resets the sensor and reads its ID. DeviceID returns the sensor ID in
	 * bits 7:0, the controller version in bits 15:8 and the controller date
	 * year, month, day in bits 23:16, 31:24, 39:32.
	 * Cfg.OutFmt.Fmt JPEG, RGB565 or YUV422, JPEG for another value. The
	 * frame rate is not adjustable, OutFmt.FrameRate is 0. Cfg.InFmt is not
	 * used.
	 *
	 * @param	Cfg		: Configuration data
	 * @param	pIntrf	: SPI interface, mode 0, ARDUCAM_MEGA_SPI_RATE_MAX maximum
	 *                    For interrupt mode, its configured callback must forward
	 *                    events and byte counts to IntrfEvent. Call camera
	 *                    operations from foreground context, with IRQs enabled.
	 * @param	pTimer	: Timer for the frame time stamp, nullptr if not used
	 *
	 * @return	true - Camera found and initialized
	 */
	virtual bool Init(const VideoDevCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = nullptr);

	/**
	 * @brief	Leave the low power mode.
	 */
	virtual bool Enable();

	/**
	 * @brief	Enter the low power mode. No capture until Enable.
	 */
	virtual void Disable();

	/**
	 * @brief	Reset the sensor.
	 *
	 * The format and resolution are written again on the next Start.
	 */
	virtual void Reset();

	virtual bool Start();
	virtual void Stop();
	virtual bool UpdateData();
	virtual int ReadFrame(uint8_t *pBuff, int Len);

	// Start a frame burst directly into caller storage. Returns -1 while an
	// accepted read is pending, or its completed byte count (0 on refusal).
	// Keep the buffer untouched until ReadFrameComplete returns >= 0. Only
	// one read may be outstanding. Polling SPI also works and returns inline.
	int ReadFrameAsync(uint8_t *pBuff, int Len);

	// Collect the outstanding burst in foreground context; -1 until its IRQ
	// completes. Advances the frame and starts the next capture when needed.
	// Stop does not cancel DMA: collect completion before reusing its buffer.
	int ReadFrameComplete();

	// Forward the configured SPI callback here. It only records completion.
	void IntrfEvent(DEVINTRF_EVT Evt, int Len);

	using VideoDevice::Format;
	virtual const VideoFmt_t &Format(VIDEO_STREAM Stream, const VideoFmt_t &Fmt);

	/**
	 * @brief	Read a register of the controller.
	 *
	 * Register address in pCmdAddr[0], bit 7 is cleared. The controller sends
	 * one dummy byte before the value.
	 */
	virtual int Read(uint8_t *pCmdAddr, int CmdAddrLen, uint8_t *pBuff, int BuffLen);

	/**
	 * @brief	Write a register of the controller.
	 *
	 * Register address in pCmdAddr[0], bit 7 is set.
	 */
	virtual int Write(uint8_t *pCmdAddr, int CmdAddrLen, const uint8_t *pData, int DataLen);

	/**
	 * @brief	Set the brightness.
	 *
	 * @param	Level : -4 to +4, 0 is the sensor default. Clamped to the range
	 *
	 * @return	true - Success
	 */
	bool Brightness(int Level) { return CtrlLevel(ARDUCAM_MEGA_BRIGHTNESS_REG, Level, ARDUCAM_MEGA_BRIGHTNESS_LEVEL_MAX); }

	/**
	 * @brief	Set the contrast.
	 *
	 * @param	Level : -3 to +3, 0 is the sensor default. Clamped to the range
	 *
	 * @return	true - Success
	 */
	bool Contrast(int Level) { return CtrlLevel(ARDUCAM_MEGA_CONTRAST_REG, Level, ARDUCAM_MEGA_LEVEL_MAX); }

	/**
	 * @brief	Set the color saturation.
	 *
	 * @param	Level : -3 to +3, 0 is the sensor default. Clamped to the range
	 *
	 * @return	true - Success
	 */
	bool Saturation(int Level) { return CtrlLevel(ARDUCAM_MEGA_SATURATION_REG, Level, ARDUCAM_MEGA_LEVEL_MAX); }

	/**
	 * @brief	Set the exposure compensation.
	 *
	 * @param	Level : -3 to +3, 0 is the sensor default. Clamped to the range
	 *
	 * @return	true - Success
	 */
	bool ExposureComp(int Level) { return CtrlLevel(ARDUCAM_MEGA_EV_REG, Level, ARDUCAM_MEGA_LEVEL_MAX); }

	bool WhiteBalance(ARDUCAM_MEGA_WB Mode) { return CtrlWrite(ARDUCAM_MEGA_WHITE_BALANCE_REG, Mode); }
	bool ColorEffect(ARDUCAM_MEGA_EFFECT Effect) { return CtrlWrite(ARDUCAM_MEGA_COLOR_EFFECT_REG, Effect); }
	bool Quality(ARDUCAM_MEGA_QUALITY Quality) { return CtrlWrite(ARDUCAM_MEGA_QUALITY_REG, Quality); }
	bool Rotate180(bool bEn) { return CtrlWrite(ARDUCAM_MEGA_ROTATION_REG, bEn ? ARDUCAM_MEGA_ROTATION_180 : 0); }

	bool AutoGain(bool bEn) { return AutoCtrl(ARDUCAM_MEGA_AUTO_CTRL_GAIN, bEn); }
	bool AutoExposure(bool bEn) { return AutoCtrl(ARDUCAM_MEGA_AUTO_CTRL_EXPOSURE, bEn); }
	bool AutoWhiteBalance(bool bEn) { return AutoCtrl(ARDUCAM_MEGA_AUTO_CTRL_WHITE_BALANCE, bEn); }

	/**
	 * @brief	Get the sensor ID.
	 *
	 * @return	ARDUCAM_MEGA_SENSOR_ID_..., 0 before Init
	 */
	uint8_t SensorId() { return vSensorId; }

private:
	bool TransferReady();
	int TransferWait(int Result);
	int ReadTransfer(const uint8_t *pCmd, int CmdLen, uint8_t *pBuff, int Len);
	uint8_t RegRead(uint8_t Reg);
	bool RegWrite(uint8_t Reg, uint8_t Val);
	bool CtrlWrite(uint8_t Reg, uint8_t Val);
	bool CtrlLevel(uint8_t Reg, int Level, int Max);
	bool AutoCtrl(uint8_t Sel, bool bEn);
	bool WaitStatus(uint8_t Mask, uint8_t Val);
	bool LowPower(bool bEn);
	bool SensorReset();
	bool StartFrame();

	alignas(4) uint8_t vTransferBuff[255];
	volatile bool vbTransferPending;
	volatile bool vbTransferDone;
	volatile int vTransferCount;
	volatile bool vbTransferFailed;
	bool vbFrameRead;
	int vFrameReadLen;

	uint8_t vSensorId;			//!< ARDUCAM_MEGA_SENSOR_ID_..., 0 none
	uint8_t vResMask;			//!< Sensor bit of the resolution table, 0 none
	uint8_t vResIdx;			//!< Selected entry of the resolution table
	uint8_t vRegFmt;			//!< Value written to the format register, 0 none
	uint8_t vRegRes;			//!< Value written to the resolution register, 0xFF none
	bool vbRun;					//!< Started, cleared by Stop
	bool vbCapture;				//!< Capture in progress
	bool vbFirstBurst;			//!< The next FIFO burst is the first of the frame
	uint32_t vRemain;			//!< Bytes of the ready frame not read yet
};

#endif // __cplusplus

/** @} End of group Video */

#endif // __CAM_ARDUCAM_MEGA_H__
