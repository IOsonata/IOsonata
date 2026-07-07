/**-------------------------------------------------------------------------
@file	rftag_proto_t4.h

@brief	NFC Forum Type 4 tag protocol engine

Target side Type 4 state machine over the RFTag facet. It runs the ISO-DEP
(ISO 14443-4) block layer and the Type 4 file system: the NDEF application
select by AID, CC and NDEF file select, ReadBinary and UpdateBinary. Tag
memory is reached only through the facet, the engine holds no hardware
knowledge. The frame transport feeds it through RFTag::ProcessFrame.

RATS and ATS, activation and low level framing are transport work. This
engine starts at the first ISO-DEP block after activation.

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
#ifndef __RFTAG_PROTO_T4_H__
#define __RFTAG_PROTO_T4_H__

#include "rftag/rftag.h"

#define T4T_CC_LEN				15		//!< Capability Container file length

class RFTagProtoT4 : public RFTagProto {
public:
	RFTagProtoT4() : vbAppSelected(false), vSelectedFileId(0),
		vBlockNum(0), vLastTxLen(0), vbLastValid(false) {
		memset(vCcFile, 0, sizeof(vCcFile));
	}

	/**
	 * @brief	Validate the tag for Type 4 and build the CC file.
	 *
	 * Requires a local memory image of 2 to 65535 bytes serving as the NDEF
	 * file content, the two byte NLEN prefix included.
	 *
	 * @param	pTag	Tag the engine serves
	 *
	 * @return	true when the tag fits Type 4
	 */
	virtual bool Init(RFTag * const pTag);

	/**
	 * @brief	Handle one ISO-DEP block.
	 *
	 * @param	pRx		Received block, pTx must be a stable per session buffer
	 *					so a retransmission can replay the stored response.
	 */
	virtual int OnFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);

private:
	int Apdu(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);
	int Select(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);
	int ReadBinary(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);
	int UpdateBinary(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);
	void BuildCcFile();
	bool IsSelectedCc() const;
	bool IsSelectedNdef() const;

	bool vbAppSelected;				//!< NDEF application selected by AID
	uint16_t vSelectedFileId;		//!< Currently selected file id
	uint8_t vCcFile[T4T_CC_LEN];	//!< Capability Container file content
	uint8_t vBlockNum;				//!< PICC current block number, ISO 14443-4 rule C
	uint16_t vLastTxLen;			//!< Length of the last response for retransmission
	bool vbLastValid;				//!< Last response is valid for retransmission
};

#endif // __RFTAG_PROTO_T4_H__
