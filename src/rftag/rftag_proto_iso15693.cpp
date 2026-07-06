/**-------------------------------------------------------------------------
@file	rftag_proto_iso15693.cpp

@brief	ISO15693 and NFC Forum Type 5 Tag target protocol for RFTag

RFTag protocol behavior selected when RFTagCfg_t Proto is RFTAG_PROTO_ISO15693.
ISO15693 is a 13.56 MHz vicinity protocol. This is the tag, VICC, side. After
the transport finishes activation and anticollision the reader, VCD, sends
requests and the tag answers. This module answers the mandatory NFC Type 5
command set against the tag local memory pointed to by RFTagDev_t pMem.

The vicinity air interface is a different transport from the proximity NFC
used by the Type 2 and Type 4 tags, so this protocol needs an HF vicinity
frame transport that runs activation and anticollision. The transport moves
whole ISO15693 requests and replies, the SOF, EOF and CRC are handled below
the protocol, so RFTAG_XCAP_CRC applies.

Memory image, 4 byte blocks, the whole image is pMem:
	block 0		Capability Container
	block 1..	NDEF data area as a TLV, 0x03 LEN DATA 0xFE

Init builds block 0. The NDEF area is written by RFTagSetNdef with NdefFmt
RFTAG_NDEF_FMT_TLV and NdefAddr 4, which is block 1. The 8 byte UID is not in
the block memory, it is returned by Inventory and Get System Information. It
must equal the UID the transport uses for anticollision.

First pass notes, checked on hardware:
	Inventory answers a single tag in every slot, slot and mask handling for
	multi tag anticollision is left to the transport or a later pass.
	The Capability Container MLEN and access bits and the Get System
	Information geometry are set to common values and tuned on hardware.
	Only the 1 byte Capability Container is built, memory up to 2040 bytes.

@author	Hoang Nguyen Hoan
@date	Jul. 5, 2026

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

#include "rftag/rftag.h"

// Request flag bits, common
#define REQ_FLAG_INVENTORY		0x04
#define REQ_FLAG_PROTEXT		0x08
// Request flag bits, when the inventory flag is 0
#define REQ_FLAG_SELECT			0x10
#define REQ_FLAG_ADDRESS		0x20
#define REQ_FLAG_OPTION			0x40

// Response error flag
#define RESP_FLAG_ERROR			0x01

// Mandatory and common Type 5 commands
#define CMD_INVENTORY			0x01
#define CMD_STAY_QUIET			0x02
#define CMD_READ_SINGLE			0x20
#define CMD_WRITE_SINGLE		0x21
#define CMD_READ_MULTIPLE		0x23
#define CMD_SELECT				0x25
#define CMD_RESET_TO_READY		0x26
#define CMD_GET_SYS_INFO		0x2B

// Error codes
#define ERR_NOT_SUPPORTED		0x01
#define ERR_NOT_RECOGNIZED		0x02
#define ERR_OPTION				0x03
#define ERR_BLOCK_NA			0x10
#define ERR_BLOCK_LOCKED		0x12
#define ERR_WRITE_FAILED		0x13

#define ISO15693_UID_LEN		8
#define ISO15693_BLOCK_SIZE		4
#define ISO15693_HEADER_LEN		4		//!< Block 0, Capability Container
#define ISO15693_CC_MAGIC		0xE1
#define ISO15693_DSFID			0x00
#define ISO15693_AFI			0x00
#define ISO15693_IC_REF			0x01

typedef struct {
	bool bQuiet;
	bool bSelected;
	uint8_t Uid[ISO15693_UID_LEN];		//!< Transmission order, low octet first
} Iso15693State_t;

static_assert(sizeof(Iso15693State_t) <= RFTAG_PROTO_STATE_SIZE, "ISO15693 state too large for RFTag ProtoState");

// Module default UID in transmission order, low octet first. The last octet
// 0xE0 is the fixed ISO15693 indicator, the next is the manufacturer code.
static const uint8_t s_Iso15693DefaultUid[ISO15693_UID_LEN] = {
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x04, 0xE0
};

static bool Iso15693ProtoInit(RFTagDev_t * const pDev);
static int Iso15693OnFrame(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);

static const RFTagProto_t s_Iso15693Proto = {
	.Init = Iso15693ProtoInit,
	.OnFrame = Iso15693OnFrame,
	.OnApdu = nullptr,
};

static inline Iso15693State_t *Iso15693GetState(RFTagDev_t * const pDev)
{
	return (Iso15693State_t *)pDev->ProtoState;
}

const uint8_t *RFTagProtoIso15693DefaultUid(void)
{
	return s_Iso15693DefaultUid;
}

static void Iso15693BuildHeader(RFTagDev_t * const pDev)
{
	uint8_t *p = pDev->pMem;
	uint32_t units = pDev->MemSize / 8;

	if (units > 0xFF)
	{
		units = 0xFF;
	}

	// Block 0, Capability Container, 1 byte form
	p[0] = ISO15693_CC_MAGIC;
	// Version 1.0 in the top bits. Write access denied when read only.
	p[1] = pDev->bReadOnly ? (0x40 | 0x0C) : 0x40;
	p[2] = (uint8_t)units;			// memory size in 8 byte units
	p[3] = 0x01;					// feature, read multiple block supported

	// Block 1, empty NDEF message TLV so the tag reads as a valid empty tag.
	if (pDev->MemSize >= ISO15693_HEADER_LEN + 3)
	{
		p[4] = 0x03;
		p[5] = 0x00;
		p[6] = 0xFE;
	}
}

// Returns the request payload start after flags, command and the optional UID,
// and reports whether the request is addressed to this tag. When addressed and
// the UID does not match, bForMe is false and the caller stays silent.
static int Iso15693ReqStart(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, bool *bForMe)
{
	Iso15693State_t *st = Iso15693GetState(pDev);
	uint8_t flags = pRx[0];
	int idx = 2;		// past flags and command

	*bForMe = true;

	if (flags & REQ_FLAG_ADDRESS)
	{
		if (RxLen < idx + ISO15693_UID_LEN)
		{
			*bForMe = false;
			return -1;
		}

		if (memcmp(&pRx[idx], st->Uid, ISO15693_UID_LEN) != 0)
		{
			*bForMe = false;
		}

		idx += ISO15693_UID_LEN;
	}

	return idx;
}

static int Iso15693Error(uint8_t *pTx, int TxCap, uint8_t Err)
{
	if (TxCap < 2)
	{
		return 0;
	}

	pTx[0] = RESP_FLAG_ERROR;
	pTx[1] = Err;

	return 2;
}

static int Iso15693Inventory(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	(void)pRx;
	(void)RxLen;

	Iso15693State_t *st = Iso15693GetState(pDev);

	// A quiet tag does not answer inventory.
	if (st->bQuiet)
	{
		return 0;
	}

	if (TxCap < 2 + ISO15693_UID_LEN)
	{
		return 0;
	}

	// Flags 0, DSFID, then the UID low octet first.
	pTx[0] = 0x00;
	pTx[1] = ISO15693_DSFID;
	memcpy(&pTx[2], st->Uid, ISO15693_UID_LEN);

	return 2 + ISO15693_UID_LEN;
}

static int Iso15693ReadSingle(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, int Idx,
                              bool bOption, uint8_t *pTx, int TxCap)
{
	if (RxLen < Idx + 1)
	{
		return Iso15693Error(pTx, TxCap, ERR_NOT_RECOGNIZED);
	}

	uint32_t blk = pRx[Idx];
	uint32_t addr = blk * ISO15693_BLOCK_SIZE;

	if (addr + ISO15693_BLOCK_SIZE > pDev->MemSize)
	{
		return Iso15693Error(pTx, TxCap, ERR_BLOCK_NA);
	}

	int n = 0;

	pTx[n++] = 0x00;					// response flags, no error
	if (bOption)
	{
		pTx[n++] = 0x00;				// block security status
	}

	if (n + ISO15693_BLOCK_SIZE > TxCap)
	{
		return 0;
	}

	memcpy(&pTx[n], &pDev->pMem[addr], ISO15693_BLOCK_SIZE);
	n += ISO15693_BLOCK_SIZE;

	return n;
}

static int Iso15693ReadMultiple(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, int Idx,
                                bool bOption, uint8_t *pTx, int TxCap)
{
	if (RxLen < Idx + 2)
	{
		return Iso15693Error(pTx, TxCap, ERR_NOT_RECOGNIZED);
	}

	uint32_t first = pRx[Idx];
	uint32_t count = (uint32_t)pRx[Idx + 1] + 1;		// field is count minus 1
	uint32_t addr = first * ISO15693_BLOCK_SIZE;
	uint32_t total = count * ISO15693_BLOCK_SIZE;

	if (addr + total > pDev->MemSize)
	{
		return Iso15693Error(pTx, TxCap, ERR_BLOCK_NA);
	}

	int n = 0;

	pTx[n++] = 0x00;

	for (uint32_t b = 0; b < count; b++)
	{
		if (bOption)
		{
			if (n + 1 > TxCap)
			{
				return 0;
			}
			pTx[n++] = 0x00;			// block security status
		}

		if (n + ISO15693_BLOCK_SIZE > TxCap)
		{
			return 0;
		}

		memcpy(&pTx[n], &pDev->pMem[addr + b * ISO15693_BLOCK_SIZE], ISO15693_BLOCK_SIZE);
		n += ISO15693_BLOCK_SIZE;
	}

	return n;
}

static int Iso15693WriteSingle(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, int Idx,
                               uint8_t *pTx, int TxCap)
{
	if (RxLen < Idx + 1 + ISO15693_BLOCK_SIZE)
	{
		return Iso15693Error(pTx, TxCap, ERR_NOT_RECOGNIZED);
	}

	uint32_t blk = pRx[Idx];

	// Block 0 holds the Capability Container. A read only tag rejects writes.
	if (blk == 0 || pDev->bReadOnly)
	{
		return Iso15693Error(pTx, TxCap, ERR_BLOCK_LOCKED);
	}

	uint32_t addr = blk * ISO15693_BLOCK_SIZE;

	if (addr + ISO15693_BLOCK_SIZE > pDev->MemSize)
	{
		return Iso15693Error(pTx, TxCap, ERR_BLOCK_NA);
	}

	memcpy(&pDev->pMem[addr], &pRx[Idx + 1], ISO15693_BLOCK_SIZE);
	RFTagEvtDispatch(pDev, RFTAG_EVT_MEM_CHANGED, addr, ISO15693_BLOCK_SIZE, 0);

	if (TxCap < 1)
	{
		return 0;
	}

	pTx[0] = 0x00;		// response flags, no error
	return 1;
}

static int Iso15693GetSysInfo(RFTagDev_t * const pDev, uint8_t *pTx, int TxCap)
{
	Iso15693State_t *st = Iso15693GetState(pDev);
	uint32_t blocks = pDev->MemSize / ISO15693_BLOCK_SIZE;

	if (blocks > 256)
	{
		blocks = 256;
	}

	// Flags, info flags, UID, DSFID, AFI, block count and size, IC reference
	if (TxCap < 2 + ISO15693_UID_LEN + 4)
	{
		return 0;
	}

	int n = 0;

	pTx[n++] = 0x00;					// response flags, no error
	pTx[n++] = 0x0F;					// DSFID, AFI, memory size, IC ref present
	memcpy(&pTx[n], st->Uid, ISO15693_UID_LEN);
	n += ISO15693_UID_LEN;
	pTx[n++] = ISO15693_DSFID;
	pTx[n++] = ISO15693_AFI;
	pTx[n++] = (uint8_t)(blocks - 1);				// block count minus 1
	pTx[n++] = (uint8_t)(ISO15693_BLOCK_SIZE - 1);	// block size minus 1
	pTx[n++] = ISO15693_IC_REF;

	return n;
}

static bool Iso15693ProtoInit(RFTagDev_t * const pDev)
{
	if (pDev->pMem == nullptr || pDev->MemSize < ISO15693_HEADER_LEN + ISO15693_BLOCK_SIZE)
	{
		return false;
	}

	// Accept 0, meaning use the module default, or the 8 byte UID from config.
	if (pDev->IdLen != 0 && pDev->IdLen != ISO15693_UID_LEN)
	{
		return false;
	}

	Iso15693State_t *st = Iso15693GetState(pDev);

	memset(st, 0, sizeof(Iso15693State_t));

	if (pDev->IdLen == ISO15693_UID_LEN)
	{
		memcpy(st->Uid, pDev->NfcId, ISO15693_UID_LEN);
	}
	else
	{
		memcpy(st->Uid, s_Iso15693DefaultUid, ISO15693_UID_LEN);
	}

	Iso15693BuildHeader(pDev);

	return true;
}

static int Iso15693OnFrame(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (RxLen < 2)
	{
		return 0;
	}

	Iso15693State_t *st = Iso15693GetState(pDev);
	uint8_t flags = pRx[0];
	uint8_t cmd = pRx[1];

	if (flags & REQ_FLAG_INVENTORY)
	{
		if (cmd == CMD_INVENTORY)
		{
			return Iso15693Inventory(pDev, pRx, RxLen, pTx, TxCap);
		}

		return 0;
	}

	bool bForMe = true;
	int idx = Iso15693ReqStart(pDev, pRx, RxLen, &bForMe);

	if (idx < 0 || bForMe == false)
	{
		// Not addressed to this tag, stay silent.
		return 0;
	}

	bool bOption = (flags & REQ_FLAG_OPTION) != 0;
	bool bAddressed = (flags & REQ_FLAG_ADDRESS) != 0;
	bool bSelectedReq = (flags & REQ_FLAG_SELECT) != 0;

	// A quiet tag answers only addressed or selected requests.
	if (st->bQuiet && bAddressed == false && bSelectedReq == false)
	{
		return 0;
	}

	// A selected mode request is answered only by the tag in selected state.
	if (bSelectedReq && st->bSelected == false)
	{
		return 0;
	}

	switch (cmd)
	{
		case CMD_STAY_QUIET:
			// Addressed only. Do not go quiet on a non-addressed request.
			if (bAddressed == false)
			{
				return 0;
			}
			st->bQuiet = true;
			return 0;

		case CMD_SELECT:
			// Addressed only. Reject a non-addressed SELECT.
			if (bAddressed == false)
			{
				return Iso15693Error(pTx, TxCap, ERR_NOT_RECOGNIZED);
			}
			st->bSelected = true;
			st->bQuiet = false;
			pTx[0] = 0x00;
			return 1;

		case CMD_RESET_TO_READY:
			st->bSelected = false;
			st->bQuiet = false;
			pTx[0] = 0x00;
			return 1;

		case CMD_READ_SINGLE:
			return Iso15693ReadSingle(pDev, pRx, RxLen, idx, bOption, pTx, TxCap);

		case CMD_READ_MULTIPLE:
			return Iso15693ReadMultiple(pDev, pRx, RxLen, idx, bOption, pTx, TxCap);

		case CMD_WRITE_SINGLE:
			return Iso15693WriteSingle(pDev, pRx, RxLen, idx, pTx, TxCap);

		case CMD_GET_SYS_INFO:
			return Iso15693GetSysInfo(pDev, pTx, TxCap);

		default:
			return Iso15693Error(pTx, TxCap, ERR_NOT_SUPPORTED);
	}
}

bool RFTagProtoIso15693Bind(RFTagDev_t * const pDev)
{
	if (pDev == nullptr)
	{
		return false;
	}

	pDev->pProto = &s_Iso15693Proto;

	return true;
}
