/**-------------------------------------------------------------------------
@file	dfu_serial.h

@brief	SMP serial transport, over any byte stream DeviceIntrf: UART, USB
		CDC.

The framing the SMP host tools use on a serial port (mcumgr, smpmgr,
nrfutil mcu-manager, AuTerm), the one MCUboot serial recovery speaks:

	packet		length (2 bytes, big endian, of what follows), the SMP
				packet, CRC-16 of the SMP packet (big endian)
	encoding	base64 of the whole of that
	frames		the base64 text split in lines of at most 127 bytes: the
				first starts 0x06 0x09, the others 0x04 0x14, each ends
				with a newline

Anything else on the line, console output for one, is skipped, so the port
can be shared with a console.

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
#ifndef __DFU_SERIAL_H__
#define __DFU_SERIAL_H__

#include <stdint.h>

#include "device_intrf.h"
#include "dfu/dfu_smp.h"

/** @addtogroup DFU
  * @{
  */

#define DFU_SERIAL_START1		0x06	//!< First frame of a packet
#define DFU_SERIAL_START2		0x09
#define DFU_SERIAL_CONT1		0x04	//!< Following frames
#define DFU_SERIAL_CONT2		0x14

/// Longest frame, marker and newline included.
#define DFU_SERIAL_FRAME_MAX	127

/// Base64 characters in a frame: what fits, in whole groups of 4.
#define DFU_SERIAL_B64_MAX		((DFU_SERIAL_FRAME_MAX - 3) / 4 * 4)

/// Receive buffer needed for requests of BufSize bytes: the length field,
/// the packet and its CRC.
#define DFU_SERIAL_RXBUF_SIZE(BufSize)	((BufSize) + 4)

/// Transmit buffer needed: the longest response DfuSmp writes plus the
/// length field and CRC.
#define DFU_SERIAL_TXBUF_SIZE	(512 + 4)

#ifdef __cplusplus

/// What to do once the response to a reset request is handed to the stream.
/// A byte stream with a transmit FIFO may still be sending it, so the reset
/// gives it the time the link needs.
typedef void (*DfuSerialReset_t)(void);

typedef struct __Dfu_Serial_Cfg {
	DfuSmp *pMgr;				//!< Initialised server
	DeviceIntrf *pIntrf;		//!< UART, USB CDC or any byte stream
	uint8_t *pRxBuf;			//!< DFU_SERIAL_RXBUF_SIZE(BufSize of pMgr)
	uint16_t RxBufSize;
	uint8_t *pTxBuf;			//!< DFU_SERIAL_TXBUF_SIZE
	uint16_t TxBufSize;
	DfuSerialReset_t Reset;		//!< Reset after the response, null when the
								//!< server has no reset callback of its own
	uint32_t TxRetry;			//!< Tries at a full transmit FIFO, 0 default
} DfuSerialCfg_t;

/// SMP over a byte stream, the serial transport of SMP.
class DfuSerial {
public:
	DfuSerial();

	/**
	 * @brief	Set up.
	 *
	 * Give pMgr DfuSerial::ResetCB and this object as its reset callback to
	 * have the reset done after the response is sent.
	 */
	bool Init(const DfuSerialCfg_t &Cfg);

	/**
	 * @brief	Take what the interface has received and serve a complete
	 * 			request.
	 *
	 * Reads the interface until it has nothing more, decodes frames, and
	 * when a packet is complete and its CRC good, hands it to the server
	 * and sends the response. Bytes that are not SMP frames, console output
	 * for one, are skipped.
	 *
	 * Call from the main loop or the application event queue, never from
	 * an interrupt: the server erases and writes memory.
	 *
	 * @return	true when a request was served.
	 */
	bool Poll(void);

	/**
	 * @brief	Feed received bytes instead of letting Poll read them.
	 *
	 * @return	true when a request was served.
	 */
	bool Feed(const uint8_t *pData, uint32_t Len);

	/// DfuSmpResetCb_t that defers the reset until the response is out.
	static void ResetCB(void *pCtx);

	/// Bad CRC or oversized packets seen, for a test to count.
	uint32_t Dropped(void) const { return vDropped; }

private:
	bool Byte(uint8_t b);
	bool Serve(void);
	bool Send(void);
	bool TxAll(const uint8_t *p, uint32_t Len);

	DfuSerialCfg_t vCfg;
	bool vbInit;
	bool vbReset;

	enum { SER_IDLE, SER_MARK1, SER_LINE, SER_SKIP } vState;
	uint8_t vMark;				//!< First marker byte seen
	bool vbInPkt;				//!< A start frame began a packet
	uint8_t vQuad[4];			//!< Base64 characters short of a group
	uint8_t vQuadLen;
	uint8_t vPad;				//!< Padding characters in the group
	uint32_t vLen;				//!< Decoded bytes
	uint32_t vDropped;
};

/// CRC-16 of SMP serial frames: polynomial 0x1021, initial 0, not reflected.
uint16_t DfuSerialCrc16(uint16_t Crc, const uint8_t *p, uint32_t Len);

#endif

/** @} */

#endif
