/**-------------------------------------------------------------------------
@file	rftag_proto_t2.h

@brief	NFC Forum Type 2 tag protocol engine

Target side Type 2 state machine over the RFTag facet. The engine serves
READ, WRITE, SECTOR SELECT and HALT from the tag memory image through the
facet memory access. It holds no hardware knowledge, the frame transport of
the concrete tag device feeds it through RFTag::ProcessFrame.

Anticollision and activation are transport work: the NFC-A peripheral handles
NFCID1, SENS_RES and SEL_RES. This engine starts at the first tag command
after activation.

Type 2 WRITE and SECTOR SELECT acknowledge with a 4 bit ACK or NAK per the
specification. The engine returns them as a single byte, a byte frame
transport sends 8 bits. Proper hardware behavior needs a bit frame response
path in the transport.

@author	Hoang Nguyen Hoan
@date	Jul. 7, 2026

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
#ifndef __RFTAG_PROTO_T2_H__
#define __RFTAG_PROTO_T2_H__

#include "rftag/rftag.h"

class RFTagProtoT2 : public RFTagProto {
public:
	RFTagProtoT2() : vSector(0), vbSecSelPending(false) {}

	/**
	 * @brief	Validate the tag for Type 2 and build the tag header.
	 *
	 * Requires a local memory image of at least header plus one data block.
	 * The id must be absent, meaning the engine default, or 7 bytes: Type 2
	 * uses the double size UID only. Any other length is a configuration
	 * error and is rejected rather than silently replaced by the default.
	 *
	 * @param	pTag	Tag the engine serves
	 *
	 * @return	true when the tag fits Type 2
	 */
	virtual bool Init(RFTag * const pTag);

	/**
	 * @brief	Handle one Type 2 command frame.
	 */
	virtual int OnFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);

	/**
	 * @brief	Engine default 7 byte UID, NXP manufacturer prefix.
	 */
	static const uint8_t *DefaultUid();

private:
	void BuildHeader();
	int CmdRead(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);
	int CmdWrite(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);

	uint8_t vSector;				//!< Current sector from SECTOR SELECT
	bool vbSecSelPending;			//!< First frame of a SECTOR SELECT seen
};

#endif // __RFTAG_PROTO_T2_H__
