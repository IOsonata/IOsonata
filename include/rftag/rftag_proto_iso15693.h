/**-------------------------------------------------------------------------
@file	rftag_proto_iso15693.h

@brief	ISO15693 NFC Forum Type 5 tag protocol engine

Target side ISO15693 VICC state machine over the RFTag facet. It serves
INVENTORY, STAY QUIET, SELECT, RESET TO READY, READ and WRITE SINGLE BLOCK,
READ MULTIPLE BLOCK and GET SYSTEM INFO from the tag memory image through the
facet. It holds no hardware knowledge, the frame transport feeds it through
RFTag::ProcessFrame.

Anticollision timing and framing are transport work. This engine implements
the addressed and selected mode rules: STAY QUIET and SELECT are addressed
only, a selected mode request is answered only when the tag is selected.

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
#ifndef __RFTAG_PROTO_ISO15693_H__
#define __RFTAG_PROTO_ISO15693_H__

#include "rftag/rftag.h"

#define ISO15693_UID_LEN		8

class RFTagProtoIso15693 : public RFTagProto {
public:
	RFTagProtoIso15693() : vbQuiet(false), vbSelected(false) {
		memset(vUid, 0, sizeof(vUid));
	}

	/**
	 * @brief	Validate the tag for ISO15693 and build the tag header.
	 *
	 * Requires a local memory image of at least header plus one block. The
	 * id is absent, meaning the engine default, or 8 bytes.
	 *
	 * @param	pTag	Tag the engine serves
	 *
	 * @return	true when the tag fits ISO15693
	 */
	virtual bool Init(RFTag * const pTag);

	/**
	 * @brief	Handle one ISO15693 request.
	 */
	virtual int OnFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);

	/**
	 * @brief	Engine default 8 byte UID, transmission order low octet first.
	 */
	static const uint8_t *DefaultUid();

private:
	void BuildHeader();
	int ReqStart(const uint8_t *pRx, int RxLen, bool *bForMe);
	int Inventory(uint8_t *pTx, int TxCap);
	int ReadSingle(const uint8_t *pRx, int RxLen, int Idx, bool bOption, uint8_t *pTx, int TxCap);
	int ReadMultiple(const uint8_t *pRx, int RxLen, int Idx, bool bOption, uint8_t *pTx, int TxCap);
	int WriteSingle(const uint8_t *pRx, int RxLen, int Idx, uint8_t *pTx, int TxCap);
	int GetSysInfo(uint8_t *pTx, int TxCap);

	bool vbQuiet;					//!< Tag is in the quiet state
	bool vbSelected;				//!< Tag is in the selected state
	uint8_t vUid[ISO15693_UID_LEN];	//!< UID, transmission order low octet first
};

#endif // __RFTAG_PROTO_ISO15693_H__
