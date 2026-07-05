/*--------------------------------------------------------------------------
File   : rftag_proto_t4t.cpp

Author : Hoang Nguyen Hoan          Jul. 5, 2026

Desc   : NFC Forum Type 4 Tag target protocol for RFTag

This module is the RFTag protocol behavior selected when RFTagCfg_t Proto is
RFTAG_PROTO_NFC_T4. It runs on top of a frame transport. RFTag hands each reader
frame to OnFrame, which handles the ISO-DEP layer (RATS and I-block framing)
and the ISO 7816-4 command set (SELECT, READ BINARY, UPDATE BINARY) against the
Capability Container and the NDEF file. The NDEF file is the tag local memory
pointed to by RFTagDev_t pMem and starts with a 2 byte NLEN field.

The ISO-DEP interface bytes and block handling are a first pass and are meant to
be checked and tuned on hardware.

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#include <string.h>
#include <algorithm>
using namespace std;

#include "rftag/rftag.h"

#define T4T_AID_LEN			7

#define T4T_CC_FILE_ID		0xE103
#define T4T_NDEF_FILE_ID	0xE104
#define T4T_MLE				0x00F6
#define T4T_MLC				0x00F6
#define T4T_CC_LEN			15

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
#define PCB_RATS				0xE0

typedef struct {
	bool bAppSelected;
	uint16_t SelectedFileId;
	uint8_t CcFile[T4T_CC_LEN];
} T4tState_t;

static_assert(sizeof(T4tState_t) <= RFTAG_PROTO_STATE_SIZE, "T4T state too large for RFTag ProtoState");

static const uint8_t s_T4tAid[T4T_AID_LEN] = {
	0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01
};

static inline T4tState_t *T4tGetState(RFTagDev_t * const pDev)
{
	return (T4tState_t *)pDev->ProtoState;
}

static void T4tBuildCcFile(RFTagDev_t * const pDev)
{
	T4tState_t *st = T4tGetState(pDev);
	uint8_t *p = st->CcFile;
	uint16_t ndefSize = (uint16_t)pDev->MemSize;

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
	p[14] = 0x00;
}

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

static bool T4tIsSelectedCc(RFTagDev_t * const pDev)
{
	T4tState_t *st = T4tGetState(pDev);
	return st->bAppSelected && st->SelectedFileId == T4T_CC_FILE_ID;
}

static bool T4tIsSelectedNdef(RFTagDev_t * const pDev)
{
	T4tState_t *st = T4tGetState(pDev);
	return st->bAppSelected && st->SelectedFileId == T4T_NDEF_FILE_ID;
}

static int T4tSelect(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	T4tState_t *st = T4tGetState(pDev);

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

		st->bAppSelected = true;
		st->SelectedFileId = 0;
		RFTagEvtDispatch(pDev, RFTAG_EVT_SELECTED, 0, 0, 0);

		return T4tRespSw(pTx, TxCap, ISO7816_SW_OK);
	}

	if (p1 == 0x00)
	{
		if (lc != 2)
		{
			return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_LEN);
		}

		uint16_t fid = ((uint16_t)pRx[5] << 8) | pRx[6];

		if (st->bAppSelected && (fid == T4T_CC_FILE_ID || fid == T4T_NDEF_FILE_ID))
		{
			st->SelectedFileId = fid;
			return T4tRespSw(pTx, TxCap, ISO7816_SW_OK);
		}

		return T4tRespSw(pTx, TxCap, ISO7816_SW_FILE_NOT_FOUND);
	}

	return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_P1P2);
}

static int T4tReadBinary(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	T4tState_t *st = T4tGetState(pDev);

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

	if (T4tIsSelectedCc(pDev))
	{
		if (off >= sizeof(st->CcFile))
		{
			return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_P1P2);
		}

		int l = min(le, (int)(sizeof(st->CcFile) - off));
		return T4tRespData(pTx, TxCap, &st->CcFile[off], l, ISO7816_SW_OK);
	}

	if (T4tIsSelectedNdef(pDev))
	{
		if (off >= pDev->MemSize)
		{
			return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_P1P2);
		}

		int l = min(le, (int)(pDev->MemSize - off));
		return T4tRespData(pTx, TxCap, &pDev->pMem[off], l, ISO7816_SW_OK);
	}

	return T4tRespSw(pTx, TxCap, ISO7816_SW_FILE_NOT_FOUND);
}

static int T4tUpdateBinary(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (RxLen < 5)
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_LEN);
	}

	if (T4tIsSelectedNdef(pDev) == false)
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_NOT_ALLOWED);
	}

	uint32_t off = ((uint32_t)pRx[2] << 8) | pRx[3];
	int lc = pRx[4];

	if (RxLen < (5 + lc) || lc > T4T_MLC)
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_WRONG_LEN);
	}

	if (off + lc > pDev->MemSize)
	{
		return T4tRespSw(pTx, TxCap, ISO7816_SW_NO_SPACE);
	}

	if (lc > 0)
	{
		memcpy(&pDev->pMem[off], &pRx[5], lc);
		RFTagEvtDispatch(pDev, RFTAG_EVT_MEM_CHANGED, off, lc, 0);
	}

	return T4tRespSw(pTx, TxCap, ISO7816_SW_OK);
}

static int T4tApdu(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
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
			return T4tSelect(pDev, pRx, RxLen, pTx, TxCap);

		case 0xB0:
			return T4tReadBinary(pDev, pRx, RxLen, pTx, TxCap);

		case 0xD6:
			return T4tUpdateBinary(pDev, pRx, RxLen, pTx, TxCap);

		default:
			return T4tRespSw(pTx, TxCap, ISO7816_SW_INS_NOT_SUPP);
	}
}

static bool T4tProtoInit(RFTagDev_t * const pDev)
{
	if (pDev->pMem == nullptr || pDev->MemSize < 2 || pDev->MemSize > 0xFFFF)
	{
		return false;
	}

	T4tState_t *st = T4tGetState(pDev);

	memset(st, 0, sizeof(T4tState_t));
	T4tBuildCcFile(pDev);

	return true;
}

static int T4tOnFrame(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (RxLen < 1)
	{
		return 0;
	}

	uint8_t pcb = pRx[0];

	// RATS starts the ISO-DEP layer. Answer with an ATS.
	if (pcb == PCB_RATS)
	{
		T4tState_t *st = T4tGetState(pDev);
		st->bAppSelected = false;
		st->SelectedFileId = 0;

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

	// S-block DESELECT ends the exchange. Echo it and clear selection.
	if (pcb == PCB_SBLOCK_DESELECT)
	{
		T4tState_t *st = T4tGetState(pDev);
		st->bAppSelected = false;
		st->SelectedFileId = 0;

		if (TxCap < 1)
		{
			return 0;
		}

		pTx[0] = pcb;
		RFTagEvtDispatch(pDev, RFTAG_EVT_DESELECTED, 0, 0, 0);

		return 1;
	}

	// I-block holds an APDU. Strip the header, run the command, wrap the reply.
	if ((pcb & PCB_IBLOCK_MASK) == PCB_IBLOCK_VAL)
	{
		int hdr = 1;
		uint8_t cid = 0;
		bool bCid = (pcb & PCB_CID_PRESENT) != 0;

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

		// Chaining is not handled in this pass. The response uses one I-block.
		int rhdr = 1;
		uint8_t rpcb = (uint8_t)(PCB_IBLOCK_VAL | (pcb & PCB_BLOCKNUM));

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

		int l = T4tApdu(pDev, &pRx[hdr], RxLen - hdr, &pTx[rhdr], TxCap - rhdr);

		if (l <= 0)
		{
			return 0;
		}

		return rhdr + l;
	}

	// R-block and other frames are not answered in this pass.
	return 0;
}

static const RFTagProto_t s_T4tProto = {
	.Init = T4tProtoInit,
	.OnFrame = T4tOnFrame,
	.OnApdu = T4tApdu,
};

bool RFTagProtoT4tBind(RFTagDev_t * const pDev)
{
	if (pDev == nullptr)
	{
		return false;
	}

	pDev->pProto = &s_T4tProto;

	return true;
}
