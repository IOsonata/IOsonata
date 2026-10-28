/**-------------------------------------------------------------------------
@file	videodev.h

@brief	Generic video device definitions

VideoDevice is the base of all video devices: camera, playback, recorder,
video codec, encoder, decoder. A device has up to two frame streams.

	VIDEO_STREAM_OUT : frames produced by the device, the application reads
					   them with ReadFrame. Camera, decoder or encoder output.
	VIDEO_STREAM_IN  : frames consumed by the device, the application writes
					   them with WriteFrame. Playback, recorder, decoder or
					   encoder input.

A camera has only the output stream, a playback device or a recorder only
the input stream, a codec both. Each stream has its own format. The frames
belong to the device, they are transferred over the DeviceIntrf given to
Init, SPI for a camera module for example.

Start begins processing frames, one frame (VIDEO_OPMODE_SINGLE) or frames
back to back (VIDEO_OPMODE_CONTINUOUS) until Stop. The application calls
UpdateData until it returns true, or receives DEV_EVT_DATA_RDY through the
device event handler, when an output frame is ready. The frame is then read
in parts of any size with ReadFrame until it returns 0. Input frames are
written in parts of any size with WriteFrame, the last part flagged.

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
#ifndef __VIDEODEV_H__
#define __VIDEODEV_H__

#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "device.h"

/** @addtogroup Video
  * @{
  */

/// @brief	Data format of the frames of a stream
typedef enum __Video_Format {
	VIDEO_FMT_NONE,				//!< Stream not used
	VIDEO_FMT_RGB565,			//!< Raw, 16 bits per pixel, red 5, green 6, blue 5
	VIDEO_FMT_YUV422,			//!< Raw, 16 bits per pixel, Y U Y V
	VIDEO_FMT_JPEG,				//!< Compressed, one JPEG image per frame, length varies
} VIDEO_FMT;

/// @brief	Frame stream of a video device
typedef enum __Video_Stream {
	VIDEO_STREAM_OUT,			//!< Frames produced by the device, read by the application
	VIDEO_STREAM_IN,			//!< Frames consumed by the device, written by the application
} VIDEO_STREAM;

#define VIDEO_STREAM_MAX		2

/// @brief	Operating mode
typedef enum __Video_OpMode {
	VIDEO_OPMODE_SINGLE,		//!< One frame per Start, still picture
	VIDEO_OPMODE_CONTINUOUS,	//!< Frames back to back until Stop
} VIDEO_OPMODE;

#pragma pack(push, 4)

/// @brief	Format of a frame stream
typedef struct __Video_Format_Desc {
	VIDEO_FMT Fmt;				//!< Data format, VIDEO_FMT_NONE for a stream not used
	uint16_t Width;				//!< Frame width in pixels
	uint16_t Height;			//!< Frame height in pixels
	uint32_t FrameRate;			//!< Frame rate in mHz (milliHertz), 0 for the device default
} VideoFmt_t;

#pragma pack(pop)

#ifdef __cplusplus

#pragma pack(push, 4)

/// @brief	Video device configuration
typedef struct __Video_Device_Config {
	uint32_t DevAddr;			//!< I2C 7 bits device address or SPI CS index
	VIDEO_OPMODE OpMode;		//!< Operating mode
	VideoFmt_t OutFmt;			//!< Output stream format, closest supported is selected
	VideoFmt_t InFmt;			//!< Input stream format, closest supported is selected
	DevEvtHandler_t EvtHandler;	//!< Receives DEV_EVT_DATA_RDY when an output frame is ready, nullptr if not used
} VideoDevCfg_t;

#pragma pack(pop)

/// @brief	Video device base class.
///
/// Require implementations :
///	- bool Init(const VideoDevCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer);
///	- bool Start();
///	- void Stop();
///	- bool UpdateData();
///	- const VideoFmt_t &Format(VIDEO_STREAM Stream, const VideoFmt_t &Fmt);
///
/// Implementation of the stream the device has
///	- int ReadFrame(uint8_t *pBuff, int Len);
///	- int WriteFrame(const uint8_t *pData, int Len, bool bEnd);
///
/// Require implementations from Device base class
///	- bool Enable();
///	- void Disable();
///	- void Reset();
///
class VideoDevice : virtual public Device {
public:
	VideoDevice() : vOpMode(VIDEO_OPMODE_SINGLE), vFmt{}, vFrameLen(0), vFrameTime(0) {}

	/**
	 * @brief	Initialize the device.
	 *
	 * @param	Cfg		: Configuration data
	 * @param	pIntrf	: Interface to the device. This pointer is kept
	 * 					  internally for all access to the device.
	 * 					  DONOT delete this object externally
	 * @param	pTimer	: Timer for the frame time stamp, nullptr if not used.
	 * 					  This pointer is kept internally.
	 * 					  DONOT delete this object externally
	 *
	 * @return	true - Success
	 */
	virtual bool Init(const VideoDevCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer = nullptr) = 0;

	/**
	 * @brief	Start processing frames.
	 *
	 * Capture, playback, recording, encoding or decoding, depending on the
	 * device. VIDEO_OPMODE_SINGLE processes one frame. VIDEO_OPMODE_CONTINUOUS
	 * processes frames until Stop. An output frame not read completely is
	 * dropped by the next Start.
	 *
	 * @return	true - Started
	 */
	virtual bool Start() = 0;

	/**
	 * @brief	Stop processing frames.
	 *
	 * No new frame is started. An output frame already ready can still be read.
	 */
	virtual void Stop() = 0;

	/**
	 * @brief	Check the device.
	 *
	 * Called by the application to poll the device. When an output frame is
	 * complete, FrameLength and FrameTimestamp are updated and the device event
	 * handler receives DEV_EVT_DATA_RDY.
	 *
	 * @return	true - A new output frame is ready
	 */
	virtual bool UpdateData() = 0;

	/**
	 * @brief	Read the next part of the ready output frame.
	 *
	 * Call repeatedly with a buffer of any size until it returns 0. In
	 * VIDEO_OPMODE_CONTINUOUS the next frame starts once this one is read
	 * completely.
	 *
	 * @param	pBuff	: Buffer receiving the frame data
	 * @param	Len		: Size of pBuff in bytes
	 *
	 * @return	Number of bytes copied. 0 when the frame has been read
	 * 			completely or no frame is ready. -1 when the device has no
	 * 			output stream
	 */
	virtual int ReadFrame(uint8_t *pBuff, int Len) { (void)pBuff; (void)Len; return -1; }

	/**
	 * @brief	Write the next part of an input frame.
	 *
	 * @param	pData	: Frame data
	 * @param	Len		: Length of pData in bytes
	 * @param	bEnd	: true on the last part of the frame
	 *
	 * @return	Number of bytes accepted, 0 when the device cannot take more
	 * 			now. -1 when the device has no input stream
	 */
	virtual int WriteFrame(const uint8_t *pData, int Len, bool bEnd) { (void)pData; (void)Len; (void)bEnd; return -1; }

	/**
	 * @brief	Set the format of a stream.
	 *
	 * The closest format, resolution and frame rate supported by the device
	 * are selected. Takes effect on the next Start.
	 *
	 * @param	Stream	: VIDEO_STREAM_OUT or VIDEO_STREAM_IN
	 * @param	Fmt		: Requested format
	 *
	 * @return	Format in effect. Fmt is VIDEO_FMT_NONE when the device does
	 * 			not have this stream
	 */
	virtual const VideoFmt_t &Format(VIDEO_STREAM Stream, const VideoFmt_t &Fmt) = 0;

	/**
	 * @brief	Get the format of a stream.
	 *
	 * @param	Stream	: VIDEO_STREAM_OUT or VIDEO_STREAM_IN
	 *
	 * @return	Format in effect
	 */
	const VideoFmt_t &Format(VIDEO_STREAM Stream) const { return vFmt[Stream]; }

	/**
	 * @brief	Set the operating mode.
	 *
	 * Takes effect on the next Start.
	 *
	 * @param	OpMode : VIDEO_OPMODE_SINGLE or VIDEO_OPMODE_CONTINUOUS
	 *
	 * @return	Operating mode in effect
	 */
	virtual VIDEO_OPMODE Mode(VIDEO_OPMODE OpMode) { vOpMode = OpMode; return vOpMode; }
	VIDEO_OPMODE Mode() const { return vOpMode; }

	/**
	 * @brief	Get the length of the ready output frame.
	 *
	 * A device may add padding after the JPEG end of image marker, up to
	 * this length.
	 *
	 * @return	Length in bytes, 0 when no frame is ready
	 */
	uint32_t FrameLength() const { return vFrameLen; }

	/**
	 * @brief	Get the time stamp of the ready output frame.
	 *
	 * @return	Time in usec when the frame was complete, 0 without timer
	 */
	uint64_t FrameTimestamp() const { return vFrameTime; }

protected:
	VIDEO_OPMODE vOpMode;				//!< Operating mode
	VideoFmt_t vFmt[VIDEO_STREAM_MAX];	//!< Format in effect, indexed by VIDEO_STREAM
	uint32_t vFrameLen;					//!< Length in bytes of the ready output frame, 0 none
	uint64_t vFrameTime;				//!< Time stamp in usec of the ready output frame
};

#endif // __cplusplus

/** @} End of group Video */

#endif // __VIDEODEV_H__
