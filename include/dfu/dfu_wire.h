/**-------------------------------------------------------------------------
@file	dfu_wire.h

@brief	DFU wire protocol: fixed binary frames over any byte stream.

The protocol of the stage 0 boot, docs/architecture/dfu_wire.md. Frames are
SLIP (RFC 1055) delimited, each one:

	Op		1	request, response is Op | 0x80
	Seq		1	copied into the response
	Body	n	per operation, little endian
	Crc		4	CRC-32 IEEE over Op to the end of Body

A frame with a bad CRC, a bad escape, an unknown operation or too long is
dropped with no response, as is anything outside frames. Operations:

	INFO	state, sizes, versions
	BEGIN	the manifest (image header + TLV areas), in one frame or more;
			the signature is checked before anything is erased or written
	WRITE	payload bytes at an offset; a resend is harmless
	FINISH	check the image, make it startable
	RESET	restart, into the image or back into recovery
	ABORT	drop the upload
	ENTER	restart into recovery (an application asks stage 0 to serve)

DfuWire layers the IOsonata Slip interface (slip_intrf.h) over the
DeviceIntrf it is given, UART, UsbdCdc or BtIntrf, checks frames with the
library CRC-32 (crc.h) and leaves all image work to DfuMgr.

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
#ifndef __DFU_WIRE_H__
#define __DFU_WIRE_H__

#include <stdint.h>

#include "device_intrf.h"
#include "slip_intrf.h"
#include "dfu/dfu_mgr.h"

/** @addtogroup DFU
  * @{
  */

#define DFU_WIRE_PROTO_VER		1

// SLIP codes, as slip_intrf.cpp uses them; hosts and tests need them
#define DFU_WIRE_END			SLIP_END_CODE
#define DFU_WIRE_ESC			0xDB
#define DFU_WIRE_ESC_END		0xDC
#define DFU_WIRE_ESC_ESC		0xDD

// Operations
#define DFU_WIRE_OP_INFO		0x01
#define DFU_WIRE_OP_BEGIN		0x02
#define DFU_WIRE_OP_WRITE		0x03
#define DFU_WIRE_OP_FINISH		0x04
#define DFU_WIRE_OP_RESET		0x05
#define DFU_WIRE_OP_ABORT		0x06
#define DFU_WIRE_OP_ENTER		0x07
#define DFU_WIRE_OP_RSP			0x80	//!< Or'ed into a response Op

// Status, first byte of every response body
#define DFU_WIRE_OK				0
#define DFU_WIRE_ERR_STATE		1
#define DFU_WIRE_ERR_LEN		2
#define DFU_WIRE_ERR_HDR		3
#define DFU_WIRE_ERR_SIZE		4
#define DFU_WIRE_ERR_TLV		5
#define DFU_WIRE_ERR_KEY		6
#define DFU_WIRE_ERR_SIG		7
#define DFU_WIRE_ERR_VERSION	8
#define DFU_WIRE_ERR_OFFSET		9
#define DFU_WIRE_ERR_FLASH		10
#define DFU_WIRE_ERR_HASH		11
#define DFU_WIRE_ERR_ENTRY		12

// INFO state
#define DFU_WIRE_STATE_IDLE		0
#define DFU_WIRE_STATE_RECV		1
#define DFU_WIRE_STATE_DONE		2

// INFO flags
#define DFU_WIRE_F_DOWNGRADE	1		//!< Older images taken

/// Op, Seq and Crc around a body.
#define DFU_WIRE_FRAME_OVH		6

/// WRITE body before the data: the offset.
#define DFU_WIRE_WRITE_HDR		4

/// BEGIN body before the manifest bytes: total length, then offset, u16 each.
#define DFU_WIRE_BEGIN_HDR		4

/// Receive buffer for a MaxBody, decoded frame, plus the byte Slip leaves
/// after it (END, a pending escape).
#define DFU_WIRE_RXBUF_SIZE(MaxBody)	((MaxBody) + DFU_WIRE_WRITE_HDR + \
										 DFU_WIRE_FRAME_OVH + 1)

/// INFO response body length.
#define DFU_WIRE_INFO_LEN		40

#ifdef __cplusplus

/// Restart once the response is handed to the link. bRecovery: back into
/// stage 0 recovery. Given by the application: the link may need time to
/// send what is in its FIFO.
typedef void (*DfuWireReset_t)(bool bRecovery);

typedef struct __Dfu_Wire_Cfg {
	DfuMgr *pMgr;				//!< Initialised manager
	DeviceIntrf *pIntrf;		//!< UART, UsbdCdc, BtIntrf, any byte stream
	uint8_t *pRxBuf;			//!< DFU_WIRE_RXBUF_SIZE(MaxBody)
	uint16_t RxBufSize;
	uint16_t MaxBody;			//!< Largest WRITE data, power of 2
	uint16_t EraseMaxMs;		//!< Worst case one erase unit, for host timeouts
	uint32_t BootVer;			//!< Reported in INFO
	const uint8_t *pDevId;		//!< 8 bytes, or null for zeros
	DfuWireReset_t Reset;		//!< Null: DfuMgr::Reset right away
} DfuWireCfg_t;

class DfuWire {
public:
	DfuWire();

	bool Init(const DfuWireCfg_t &Cfg);

	/// Serve over another interface from now on. The upload state is the
	/// manager's and stays.
	bool SetIntrf(DeviceIntrf *pIntrf);

	/**
	 * @brief	Read what the interface has and serve complete frames.
	 *
	 * Call from the main loop or the application event queue, never from
	 * an interrupt: the manager erases and writes memory.
	 *
	 * @return	true when a request was served.
	 */
	bool Poll(void);

	/// Frames dropped: bad CRC, length or operation.
	uint32_t Dropped(void) const { return vDropped; }

private:
	bool Frame(void);
	void Serve(uint8_t Op, const uint8_t *pBody, uint32_t Len);
	void Rsp(uint8_t Op, const uint8_t *pBody, uint32_t Len);
	void RspStatus(uint8_t Op, int Status);
	void RspOffset(uint8_t Op, int Status);

	DfuWireCfg_t vCfg;
	bool vbInit;

	Slip vSlip;					//!< Framing over vCfg.pIntrf
	uint32_t vRxLen;			//!< Decoded bytes of the frame coming in
	bool vbSkip;				//!< Frame too long, dropped up to its end
	uint32_t vDropped;
	uint8_t vSeq;

	bool vbDone;				//!< FINISH succeeded, until BEGIN or ABORT
	uint16_t vManLen;			//!< BEGIN bytes gathered
	uint16_t vManTotal;			//!< BEGIN announced length, 0 when none

	uint8_t vTx[DFU_WIRE_INFO_LEN + DFU_WIRE_FRAME_OVH];	//!< Response, before
															//!< SLIP
};

/// Status for a DfuMgr or DFU_IMG_ERR_ result.
int DfuWireStatus(int Res);

#endif

/** @} */

#endif
