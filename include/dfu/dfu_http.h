/**-------------------------------------------------------------------------
@file	dfu_http.h

@brief	Image download over HTTP/1.1 into slot 1, for an application
		updated over a network (LTE on nRF91).

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

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
#ifndef __DFU_HTTP_H__
#define __DFU_HTTP_H__

#include <stdint.h>

#include "device_intrf.h"
#include "dfu/dfu_mgr.h"

/** @addtogroup DFU
  * @{
  */

/// Longest path and host name taken.
#define DFU_HTTP_PATH_MAX		128
#define DFU_HTTP_HOST_MAX		64

/// Longest response header line taken; longer lines are skipped.
#define DFU_HTTP_LINE_MAX		160

typedef enum __Dfu_Http_State {
	DFU_HTTP_IDLE,				//!< Nothing asked
	DFU_HTTP_HEADER,			//!< Request sent, reading the response head
	DFU_HTTP_BODY,				//!< Writing the image
	DFU_HTTP_DONE,				//!< Image in slot 1, checked, marked pending
	DFU_HTTP_BROKEN,			//!< Stream ended early: Resume on a new one
	DFU_HTTP_FAILED,			//!< Refused, see Error; start again with Get
} DFU_HTTP_STATE;

// Why a download failed.
#define DFU_HTTP_ERR_NONE		0
#define DFU_HTTP_ERR_STATUS		-1		//!< Not 200 or 206
#define DFU_HTTP_ERR_HEADER		-2		//!< No usable length, or chunked
#define DFU_HTTP_ERR_RANGE		-3		//!< 206 not starting where asked
#define DFU_HTTP_ERR_SIZE		-4		//!< Larger than slot 1 takes
#define DFU_HTTP_ERR_WRITE		-5		//!< Memory refused the data
#define DFU_HTTP_ERR_IMAGE		-6		//!< Not an image for here, see ImgErr
#define DFU_HTTP_ERR_SEND		-7		//!< Request could not be sent

#ifdef __cplusplus

/**
 * @brief	Image download over HTTP/1.1 into slot 1.
 *
 * For an application whose link is a network rather than a host tool (LTE
 * on nRF91): it asks a server for a signed image and writes it into slot 1
 * with the same writer the SMP server uses, so the image is checked and
 * marked the same way and the stage 0 boot installs it.
 *
 * The stream is any connected DeviceIntrf: a socket wrapped the way a UART
 * is, TLS or not, which is the application's business. Only
 * Content-Length bodies are taken, with a Range request to resume an
 * interrupted download where it stopped.
 *
 * Nothing blocks: Poll reads what the stream has and returns. Call it from
 * the main loop or the application event queue, never from an interrupt:
 * it erases and writes memory.
 */
class DfuHttp {
public:
	DfuHttp();

	/**
	 * @brief	Set up.
	 *
	 * @param	pMgr : Manager in slot 1 mode, initialised, taking images
	 * 				   in order (not manifest only).
	 */
	bool Init(DfuMgr *pMgr);

	/**
	 * @brief	Ask for an image from the start.
	 *
	 * Sends the request on a stream that has just been connected to Host.
	 *
	 * @param	pStream : Connected stream.
	 * @param	pHost   : Host header value.
	 * @param	pPath   : Path of the image, starting with '/'.
	 */
	bool Get(DeviceIntrf *pStream, const char *pHost, const char *pPath);

	/**
	 * @brief	Go on with a download that broke, on a new stream.
	 *
	 * Asks for the bytes from where the image stopped. The server must answer
	 * 206 from that offset; a 200 starts the image over.
	 */
	bool Resume(DeviceIntrf *pStream);

	/**
	 * @brief	Read what the stream has.
	 *
	 * @return	The state after it.
	 */
	DFU_HTTP_STATE Poll(void);

	/**
	 * @brief	The stream will not bring more: closed by the server, or the
	 * 			link went. A body not complete becomes DFU_HTTP_BROKEN.
	 */
	void StreamEnded(void);

	DFU_HTTP_STATE State(void) const { return vState; }

	/// DFU_HTTP_ERR_ value when State is DFU_HTTP_FAILED.
	int Error(void) const { return vErr; }

	/// DFU_IMG_ERR_ value when Error is DFU_HTTP_ERR_IMAGE.
	int ImgErr(void) const { return vImgErr; }

	/// Image bytes in slot 1, and the image size once known.
	uint32_t Received(void) const { return vpWr != nullptr ? vpWr->Offset() : 0; }
	uint32_t Total(void) const { return vTotal; }

private:
	bool Request(uint32_t From);
	void Fail(int Err);
	void HeaderLine(void);
	bool HeaderEnd(void);
	void Body(const uint8_t *p, uint32_t Len);

	DfuMgr *vpWr;
	DeviceIntrf *vpStream;
	DFU_HTTP_STATE vState;
	int vErr;
	int vImgErr;

	char vHost[DFU_HTTP_HOST_MAX];
	char vPath[DFU_HTTP_PATH_MAX];

	// Response head
	char vLine[DFU_HTTP_LINE_MAX];
	uint32_t vLineLen;
	bool vbLineLong;
	bool vbStatusSeen;
	int vStatus;
	bool vbLen;
	uint32_t vLen;				//!< Content-Length
	bool vbRange;
	uint32_t vRangeFrom;		//!< Content-Range first byte
	uint32_t vRangeTotal;		//!< Content-Range total
	bool vbChunked;

	uint32_t vAsked;			//!< Offset the request asked from
	uint32_t vTotal;			//!< Image size, 0 until known
	uint32_t vBodyLeft;			//!< Bytes of this response still to come
};

#endif

/** @} */

#endif
