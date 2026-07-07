/**-------------------------------------------------------------------------
@file	rftag_proto_t4.cpp

@brief	NFC Forum Type 4 tag protocol engine implementation

ISO-DEP block layer and the Type 4 file access served from the tag memory
image through the facet. The NDEF application selects by AID, the CC and NDEF
files select by id, ReadBinary and UpdateBinary move file content. The
ISO-DEP layer follows the ISO 14443-4 PICC block number rules with I-block
and R-block retransmission replay. Response chaining is not implemented.

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
#include <string.h>
#include <algorithm>
using namespace std;

#include "rftag/rftag_proto_t4.h"

#define T4T_AID_LEN			7

#define T4T_CC_FILE_ID		0xE103
#define T4T_NDEF_FILE_ID	0xE104
#define T4T_MLE				0x00F6
#define T4T_MLC				0x00F6

#define ISO7816_SW_OK			0x9000
#define ISO7816_SW_WRONG_LEN	0x6700
#define ISO7816_SW_NOT_ALLOWED	0x6982
#define ISO7816_SW_FILE_NOT_FOUND	0x6A82
#define ISO7816_SW_NO_SPACE		0x6A84
#define ISO7816_SW_WRONG_P1P2	0x6A86
#define ISO7816_SW_INS_NOT_SUPP	0x6D00
#define ISO7816_SW_CLA_NOT_SUPP	0x6E00

// ISO-DEP protocol control byte fields
#define PCB_IBLOCK_MASK			0xE2
#define PCB_IBLOCK_VAL			0x02
#define PCB_BLOCKNUM			0x01
#define PCB_CID_PRESENT			0x08
#define PCB_NAD_PRESENT			0x04
#define PCB_CHAINING			0x10
#define PCB_SBLOCK_DESELECT		0xC2
#define PCB_SBLOCK_MASK			0xF7
#define PCB_RBLOCK_MASK			0xE0
#define PCB_RBLOCK_VAL			0xA0
#define PCB_RBLOCK_NAK			0x10
#define PCB_RATS				0xE0

static const uint8_t s_T4tAid[T4T_AID_LEN] = {
	0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01
};

static int T4tRespSw(uint8_t *pTx, int TxCap, uint16_t Sw)
{
	if (pTx == nullptr || TxCap < 2)
	{
		return 0;
	}

	pTx[0] = (uint8_t)(Sw >> 8);
	pTx[1] = (uint8_t)Sw;

	return 2;
}

static int T4tRespData(uint8_t *pTx, int TxCap, const uint8_t *pData, int DataLen, uint16_t Sw)
{
	if (pTx == nullptr || TxCap < 2)
	{
		return 0;
	}

	int l = min(DataLen, TxCap - 2);

	if (l > 0 && pData)
	{
		memcpy(pTx, pData, l);
	}

	pTx[l] = (uint8_t)(Sw >> 8);
	pTx[l + 1] = (uint8_t)Sw;

	return l + 2;
}

void RFTagProtoT4::BuildCcFile()
{
	uint8_t *p = vCcFile;
	uint16_t ndefSize = (uint16_t)vpTag->MemSize();

	p[0] = 0x00;
	p[1] = T4T_CC_LEN;
	p[2] = 0x20;
	p[3] = (uint8_t)(T4T_MLE >> 8);
	p[4] = (uint8_t)T4T_MLE;
	p[5] = (uint8_t)(T4T_MLC >> 8);
	p[6] = (uint8_t)T4T_MLC;
	p[7] = 0x04;
	p[8] = 0x06;
	p[9] = (uint8_t)(T4T_NDEF_FILE_ID >> 8);
	p[10] = (uint8_t)T4T_NDEF_FILE_ID;
	p[11] = (uint8_t)(ndefSize >> 8);
	p[12] = (uint8_t)ndefSize;
	p[13] = 0x00;
	// Write access byte, 0x00 open, 0xFF no write access
	p[14] = vpTag->ReadOnly() ? 0xFF : 0x00;
}

bool RFTagProtoT4::IsSelectedCc() const
{
	return vbAppSelected && vSelectedFileId == T4T_CC_FILE_ID;
}

bool RFTagProtoT4::IsSelectedNdef() const
{
	return vbAppSelected && vSelectedFileId == T4T_NDEF_FILE_ID;
}

int RFTagProtoT4::Select(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (RxLen < 5)
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_LEN);
	}

	uint8_t p1 = pRx[2];
	uint8_t lc = pRx[4];

	if (RxLen < (5 + lc))
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_LEN);
	}

	if (p1 == 0x04)
	{
		if (lc != T4T_AID_LEN || memcmp(&pRx[5], s_T4tAid, T4T_AID_LEN) != 0)
		{
			return T4tRespSw(pTx, TxCap, ISO7816_SW_FILE_NOT_FOUND);
		}

		vbAppSelected = true;
		vSelectedFileId = 0;
		vpTag->EvtHandler(RFTAG_EVT_SELECTED, 0, 0);

		return T4tRespSw(pTx, TxCap, ISO7816_SW_OK);
	}

	if (p1 == 0x00)
	{
		if (lc != 2)
		{
			return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_LEN);
		}

		uint16_t fid = ((uint16_t)pRx[5] << 8) | pRx[6];

		if (vbAppSelected && (fid == T4T_CC_FILE_ID || fid == T4T_NDEF_FILE_ID))
		{
			vSelectedFileId = fid;
			return T4tRespSw(pTx, TxCap, ISO7816_SW_OK);
		}

		return T4tRespSw(pTx, TxCap, ISO7816_SW_FILE_NOT_FOUND);
	}

	return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_P1P2);
}

int RFTagProtoT4::ReadBinary(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (RxLen < 5)
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_LEN);
	}

	uint32_t off = ((uint32_t)pRx[2] << 8) | pRx[3];
	int le = pRx[4];

	if (le == 0)
	{
		le = 256;
	}

	le = min(le, (int)T4T_MLE);

	if (IsSelectedCc())
	{
		if (off >= sizeof(vCcFile))
		{
			return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_P1P2);
		}

		int l = min(le, (int)(sizeof(vCcFile) - off));
		return T4tRespData(pTx, TxCap, &vCcFile[off], l, ISO7816_SW_OK);
	}

	if (IsSelectedNdef())
	{
		if (off >= vpTag->MemSize())
		{
			return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_P1P2);
		}

		int l = min(le, (int)(vpTag->MemSize() - off));
		// Serve from the image through the facet.
		uint8_t buf[T4T_MLE];
		l = min(l, (int)sizeof(buf));
		int got = vpTag->MemRead(off, buf, l);
		return T4tRespData(pTx, TxCap, buf, got, ISO7816_SW_OK);
	}

	return T4tRespSw(pTx, TxCap, ISO7816_SW_FILE_NOT_FOUND);
}

int RFTagProtoT4::UpdateBinary(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (RxLen < 5)
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_LEN);
	}

	if (IsSelectedNdef() == false || vpTag->ReadOnly())
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_NOT_ALLOWED);
	}

	uint32_t off = ((uint32_t)pRx[2] << 8) | pRx[3];
	int lc = pRx[4];

	if (RxLen < (5 + lc) || lc > T4T_MLC)
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_LEN);
	}

	if (off + lc > vpTag->MemSize())
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_NO_SPACE);
	}

	if (lc > 0)
	{
		vpTag->MemWrite(off, &pRx[5], lc);
		vpTag->EvtHandler(RFTAG_EVT_MEM_CHANGED, off, lc);
	}

	return T4tRespSw(pTx, TxCap, ISO7816_SW_OK);
}

int RFTagProtoT4::Apdu(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (RxLen < 4)
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_LEN);
	}

	if (pRx[0] != 0x00)
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_CLA_NOT_SUPP);
	}

	switch (pRx[1])
	{
		case 0xA4:
			return Select(pRx, RxLen, pTx, TxCap);

		case 0xB0:
			return ReadBinary(pRx, RxLen, pTx, TxCap);

		case 0xD6:
			return UpdateBinary(pRx, RxLen, pTx, TxCap);

		default:
			return T4tRespSw(pTx, TxCap, ISO7816_SW_INS_NOT_SUPP);
	}
}

bool RFTagProtoT4::Init(RFTag * const pTag)
{
	if (pTag == nullptr || pTag->MemSize() < 2 || pTag->MemSize() > 0xFFFF)
	{
		return false;
	}

	vpTag = pTag;
	vbAppSelected = false;
	vSelectedFileId = 0;
	vBlockNum = 0;
	vLastTxLen = 0;
	vbLastValid = false;

	BuildCcFile();

	return true;
}

// ISO 14443-4 PICC side block handling.
//
// Block number rules: rule C inits the PICC block number to 1 at activation.
// Rule D toggles it on each received I-block. A received I-block whose block
// number equals the current one is a PCD retransmission, the stored response
// is replayed without executing the command again.
//
// R-block rules: rule 11 retransmits the last block when the received block
// number equals the current one. Rule 12 answers R(NAK) with R(ACK) when the
// numbers differ. Rule 13 continues chaining on R(ACK) with a different
// number, response chaining is not implemented so no reply is sent.
//
// Retransmission precondition: pTx must be the same buffer on every call of
// a session. RFTag::ProcessFrame passes the tag response buffer, which
// satisfies this. The stored response is replayed from that buffer without
// rebuilding it.
int RFTagProtoT4::OnFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (vpTag == nullptr || pRx == nullptr || RxLen < 1)
	{
		return 0;
	}

	uint8_t pcb = pRx[0];

	// RATS starts the ISO-DEP layer. Answer with an ATS.
	if (pcb == PCB_RATS)
	{
		vbAppSelected = false;
		vSelectedFileId = 0;
		vBlockNum = 1;
		vbLastValid = false;
		vLastTxLen = 0;

		if (TxCap < 3)
		{
			return 0;
		}

		// TL, T0 with TB present and FSCI 8 for 256 byte frames, TB FWI 8.
		pTx[0] = 0x03;
		pTx[1] = 0x28;
		pTx[2] = 0x80;

		return 3;
	}

	// S-block DESELECT ends the exchange. Echo it and clear selection. The
	// CID bit is preserved in the echo when present.
	if ((pcb & PCB_SBLOCK_MASK) == PCB_SBLOCK_DESELECT)
	{
		bool bCid = (pcb & PCB_CID_PRESENT) != 0;
		int flen = bCid ? 2 : 1;

		if (RxLen < flen || TxCap < flen)
		{
			return 0;
		}

		vbAppSelected = false;
		vSelectedFileId = 0;
		vbLastValid = false;

		pTx[0] = pcb;
		if (bCid)
		{
			pTx[1] = pRx[1];
		}

		vpTag->EvtHandler(RFTAG_EVT_DESELECTED, 0, 0);

		return flen;
	}

	// R-block. Rules 11 and 12.
	if ((pcb & PCB_RBLOCK_MASK) == PCB_RBLOCK_VAL)
	{
		uint8_t rxbn = pcb & PCB_BLOCKNUM;

		if (rxbn == (vBlockNum & 1))
		{
			// Rule 11, retransmit the last block. The response bytes are
			// still in pTx from the previous call.
			if (vbLastValid && vLastTxLen > 0 && (int)vLastTxLen <= TxCap)
			{
				return vLastTxLen;
			}

			return 0;
		}

		if (pcb & PCB_RBLOCK_NAK)
		{
			// Rule 12, answer R(NAK) with R(ACK) using the current number.
			bool bCid = (pcb & PCB_CID_PRESENT) != 0;
			int flen = bCid ? 2 : 1;

			if (RxLen < flen || TxCap < flen)
			{
				return 0;
			}

			pTx[0] = (uint8_t)(PCB_RBLOCK_VAL | 0x02 | (vBlockNum & 1));
			if (bCid)
			{
				pTx[0] |= PCB_CID_PRESENT;
				pTx[1] = pRx[1];
			}

			vbLastValid = true;
			vLastTxLen = flen;

			return flen;
		}

		// Rule 13, R(ACK) with a different number continues response
		// chaining. Not chaining, no reply.
		return 0;
	}

	// I-block holds an APDU. Strip the header, run the command, wrap the reply.
	if ((pcb & PCB_IBLOCK_MASK) == PCB_IBLOCK_VAL)
	{
		int hdr = 1;
		uint8_t cid = 0;
		bool bCid = (pcb & PCB_CID_PRESENT) != 0;
		uint8_t rxbn = pcb & PCB_BLOCKNUM;

		if (pcb & PCB_CHAINING)
		{
			// APDU chaining is not implemented yet. Do not process partial APDUs.
			return 0;
		}

		if (rxbn == (vBlockNum & 1))
		{
			// PCD retransmission of the last I-block. Replay the stored
			// response without executing the command again.
			if (vbLastValid && vLastTxLen > 0 && (int)vLastTxLen <= TxCap)
			{
				return vLastTxLen;
			}

			return 0;
		}

		if (bCid)
		{
			if (RxLen < 2)
			{
				return 0;
			}
			cid = pRx[1];
			hdr++;
		}

		if (pcb & PCB_NAD_PRESENT)
		{
			hdr++;
		}

		if (RxLen < hdr)
		{
			return 0;
		}

		// Rule D, toggle on I-block reception. After the toggle the current
		// number equals the received one and the response uses it.
		vBlockNum = rxbn;

		// The response uses one I-block.
		int rhdr = 1;
		uint8_t rpcb = (uint8_t)(PCB_IBLOCK_VAL | (vBlockNum & 1));

		if (bCid)
		{
			rpcb |= PCB_CID_PRESENT;
			rhdr++;
		}

		if (rhdr >= TxCap)
		{
			return 0;
		}

		pTx[0] = rpcb;
		if (bCid)
		{
			pTx[1] = cid;
		}

		int l = Apdu(&pRx[hdr], RxLen - hdr, &pTx[rhdr], TxCap - rhdr);

		if (l <= 0)
		{
			vbLastValid = false;
			return 0;
		}

		vbLastValid = true;
		vLastTxLen = (uint16_t)(rhdr + l);

		return rhdr + l;
	}

	// Other frames are not answered.
	return 0;
}
