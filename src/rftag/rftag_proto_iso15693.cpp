/**-------------------------------------------------------------------------
@file	rftag_proto_iso15693.cpp

@brief	ISO15693 NFC Forum Type 5 tag protocol engine implementation

ISO15693 VICC command set served from the tag memory image through the facet:
block 0 Capability Container, blocks from 1 hold NDEF. INVENTORY answers the
UID, STAY QUIET and SELECT switch the tag state under the addressed mode
rules, READ and WRITE SINGLE BLOCK and READ MULTIPLE BLOCK move block data,
GET SYSTEM INFO reports the geometry.

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

#include "rftag/rftag_proto_iso15693.h"

// Request flags
#define REQ_FLAG_INVENTORY		0x04
#define REQ_FLAG_PROTEXT		0x08
#define REQ_FLAG_SELECT			0x10
#define REQ_FLAG_ADDRESS		0x20
#define REQ_FLAG_OPTION			0x40

#define RESP_FLAG_ERROR			0x01

// Commands
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

#define ISO15693_BLOCK_SIZE		4
#define ISO15693_HEADER_LEN		4		//!< Block 0, Capability Container
#define ISO15693_CC_MAGIC		0xE1
#define ISO15693_DSFID			0x00
#define ISO15693_AFI			0x00
#define ISO15693_IC_REF			0x01

// Module default UID in transmission order, low octet first. The last octet
// 0xE0 is the fixed ISO15693 indicator, the next is the manufacturer code.
static const uint8_t s_Iso15693DefaultUid[ISO15693_UID_LEN] = {
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x04, 0xE0
};

const uint8_t *RFTagProtoIso15693::DefaultUid()
{
	return s_Iso15693DefaultUid;
}

void RFTagProtoIso15693::BuildHeader()
{
	uint8_t p[ISO15693_HEADER_LEN + 3];
	uint32_t units = vpTag->MemSize() / 8;
	int n = ISO15693_HEADER_LEN;

	if (units > 0xFF)
	{
		units = 0xFF;
	}

	// Block 0, Capability Container, 1 byte form
	p[0] = ISO15693_CC_MAGIC;
	// Version 1.0 in the top bits. Write access denied when read only.
	p[1] = vpTag->ReadOnly() ? (0x40 | 0x0C) : 0x40;
	p[2] = (uint8_t)units;			// memory size in 8 byte units
	p[3] = 0x01;					// feature, read multiple block supported

	// Block 1, empty NDEF message TLV so the tag reads as a valid empty tag.
	if (vpTag->MemSize() >= ISO15693_HEADER_LEN + 3)
	{
		p[4] = 0x03;
		p[5] = 0x00;
		p[6] = 0xFE;
		n += 3;
	}

	vpTag->MemWrite(0, p, n);
}

// Returns the request payload start after flags, command and the optional UID,
// and reports whether the request is addressed to this tag. When addressed and
// the UID does not match, bForMe is false and the caller stays silent.
int RFTagProtoIso15693::ReqStart(const uint8_t *pRx, int RxLen, bool *bForMe)
{
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

		if (memcmp(&pRx[idx], vUid, ISO15693_UID_LEN) != 0)
		{
			*bForMe = false;
		}

		idx += ISO15693_UID_LEN;
	}

	return idx;
}

static int Iso15693Error(uint8_t *pTx, int TxCap, uint8_t Err)
{
	if (pTx == nullptr || TxCap < 2)
	{
		return 0;
	}

	pTx[0] = RESP_FLAG_ERROR;
	pTx[1] = Err;

	return 2;
}

int RFTagProtoIso15693::Inventory(uint8_t *pTx, int TxCap)
{
	// A quiet tag does not answer inventory.
	if (vbQuiet)
	{
		return 0;
	}

	if (pTx == nullptr || TxCap < 2 + ISO15693_UID_LEN)
	{
		return 0;
	}

	// Flags 0, DSFID, then the UID low octet first.
	pTx[0] = 0x00;
	pTx[1] = ISO15693_DSFID;
	memcpy(&pTx[2], vUid, ISO15693_UID_LEN);

	return 2 + ISO15693_UID_LEN;
}

int RFTagProtoIso15693::ReadSingle(const uint8_t *pRx, int RxLen, int Idx,
								   bool bOption, uint8_t *pTx, int TxCap)
{
	if (RxLen < Idx + 1)
	{
		return Iso15693Error(pTx, TxCap, ERR_NOT_RECOGNIZED);
	}

	uint32_t blk = pRx[Idx];
	uint32_t addr = blk * ISO15693_BLOCK_SIZE;

	if (addr + ISO15693_BLOCK_SIZE > vpTag->MemSize())
	{
		return Iso15693Error(pTx, TxCap, ERR_BLOCK_NA);
	}

	int n = bOption ? 2 : 1;

	if (pTx == nullptr || n + ISO15693_BLOCK_SIZE > TxCap)
	{
		return 0;
	}

	pTx[0] = 0x00;					// response flags, no error
	if (bOption)
	{
		pTx[1] = 0x00;				// block security status
	}

	if (vpTag->MemRead(addr, &pTx[n], ISO15693_BLOCK_SIZE) != ISO15693_BLOCK_SIZE)
	{
		return 0;
	}

	return n + ISO15693_BLOCK_SIZE;
}

int RFTagProtoIso15693::ReadMultiple(const uint8_t *pRx, int RxLen, int Idx,
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

	if (addr + total > vpTag->MemSize())
	{
		return Iso15693Error(pTx, TxCap, ERR_BLOCK_NA);
	}

	uint32_t need = 1U + total + (bOption ? count : 0U);
	if (pTx == nullptr || TxCap < 0 || need > (uint32_t)TxCap)
	{
		return 0;
	}

	int n = 0;
	pTx[n++] = 0x00;

	for (uint32_t b = 0; b < count; b++)
	{
		if (bOption)
		{
			pTx[n++] = 0x00;			// block security status
		}

		if (vpTag->MemRead(addr + b * ISO15693_BLOCK_SIZE, &pTx[n],
						   ISO15693_BLOCK_SIZE) != ISO15693_BLOCK_SIZE)
		{
			return 0;
		}
		n += ISO15693_BLOCK_SIZE;
	}

	return n;
}

int RFTagProtoIso15693::WriteSingle(const uint8_t *pRx, int RxLen, int Idx,
									uint8_t *pTx, int TxCap)
{
	if (RxLen < Idx + 1 + ISO15693_BLOCK_SIZE)
	{
		return Iso15693Error(pTx, TxCap, ERR_NOT_RECOGNIZED);
	}

	uint32_t blk = pRx[Idx];

	// Block 0 holds the Capability Container. A read only tag rejects writes.
	if (blk == 0 || vpTag->ReadOnly())
	{
		return Iso15693Error(pTx, TxCap, ERR_BLOCK_LOCKED);
	}

	uint32_t addr = blk * ISO15693_BLOCK_SIZE;

	if (addr + ISO15693_BLOCK_SIZE > vpTag->MemSize())
	{
		return Iso15693Error(pTx, TxCap, ERR_BLOCK_NA);
	}

	if (vpTag->MemWrite(addr, &pRx[Idx + 1], ISO15693_BLOCK_SIZE) !=
		ISO15693_BLOCK_SIZE)
	{
		return Iso15693Error(pTx, TxCap, ERR_WRITE_FAILED);
	}

	if (pTx == nullptr || TxCap < 1)
	{
		return 0;
	}

	vpTag->EvtHandler(RFTAG_EVT_MEM_CHANGED, addr, ISO15693_BLOCK_SIZE);
	pTx[0] = 0x00;		// response flags, no error
	return 1;
}

int RFTagProtoIso15693::GetSysInfo(uint8_t *pTx, int TxCap)
{
	uint32_t blocks = vpTag->MemSize() / ISO15693_BLOCK_SIZE;

	if (blocks > 256)
	{
		blocks = 256;
	}

	// Flags, info flags, UID, DSFID, AFI, block count and size, IC reference
	if (pTx == nullptr || TxCap < 2 + ISO15693_UID_LEN + 4)
	{
		return 0;
	}

	int n = 0;

	pTx[n++] = 0x00;					// response flags, no error
	pTx[n++] = 0x0F;					// DSFID, AFI, memory size, IC ref present
	memcpy(&pTx[n], vUid, ISO15693_UID_LEN);
	n += ISO15693_UID_LEN;
	pTx[n++] = ISO15693_DSFID;
	pTx[n++] = ISO15693_AFI;
	pTx[n++] = (uint8_t)(blocks - 1);				// block count minus 1
	pTx[n++] = (uint8_t)(ISO15693_BLOCK_SIZE - 1);	// block size minus 1
	pTx[n++] = ISO15693_IC_REF;

	return n;
}

bool RFTagProtoIso15693::Init(RFTag * const pTag)
{
	if (pTag == nullptr || pTag->MemSize() < ISO15693_HEADER_LEN + ISO15693_BLOCK_SIZE)
	{
		return false;
	}

	// Accept 0, meaning use the engine default, or the 8 byte UID from config.
	if (pTag->IdLen() != 0 && pTag->IdLen() != ISO15693_UID_LEN)
	{
		return false;
	}

	vpTag = pTag;
	vbQuiet = false;
	vbSelected = false;

	if (pTag->IdLen() == ISO15693_UID_LEN)
	{
		memcpy(vUid, pTag->NfcId(), ISO15693_UID_LEN);
	}
	else
	{
		memcpy(vUid, s_Iso15693DefaultUid, ISO15693_UID_LEN);
	}

	BuildHeader();

	return true;
}

int RFTagProtoIso15693::OnFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (vpTag == nullptr || pRx == nullptr || RxLen < 2)
	{
		return 0;
	}

	uint8_t flags = pRx[0];
	uint8_t cmd = pRx[1];

	if (flags & REQ_FLAG_INVENTORY)
	{
		if (cmd == CMD_INVENTORY)
		{
			return Inventory(pTx, TxCap);
		}

		return 0;
	}

	bool bForMe = true;
	int idx = ReqStart(pRx, RxLen, &bForMe);

	if (idx < 0 || bForMe == false)
	{
		// Not addressed to this tag, stay silent.
		return 0;
	}

	bool bOption = (flags & REQ_FLAG_OPTION) != 0;
	bool bAddressed = (flags & REQ_FLAG_ADDRESS) != 0;
	bool bSelectedReq = (flags & REQ_FLAG_SELECT) != 0;

	// A quiet tag answers only addressed or selected requests.
	if (vbQuiet && bAddressed == false && bSelectedReq == false)
	{
		return 0;
	}

	// A selected mode request is answered only by the tag in selected state.
	if (bSelectedReq && vbSelected == false)
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
			vbQuiet = true;
			return 0;

		case CMD_SELECT:
			// Addressed only. Reject a non-addressed SELECT.
			if (bAddressed == false)
			{
				return Iso15693Error(pTx, TxCap, ERR_NOT_RECOGNIZED);
			}
			if (pTx == nullptr || TxCap < 1)
			{
				return 0;
			}
			vbSelected = true;
			vbQuiet = false;
			pTx[0] = 0x00;
			return 1;

		case CMD_RESET_TO_READY:
			if (pTx == nullptr || TxCap < 1)
			{
				return 0;
			}
			vbSelected = false;
			vbQuiet = false;
			pTx[0] = 0x00;
			return 1;

		case CMD_READ_SINGLE:
			return ReadSingle(pRx, RxLen, idx, bOption, pTx, TxCap);

		case CMD_READ_MULTIPLE:
			return ReadMultiple(pRx, RxLen, idx, bOption, pTx, TxCap);

		case CMD_WRITE_SINGLE:
			return WriteSingle(pRx, RxLen, idx, pTx, TxCap);

		case CMD_GET_SYS_INFO:
			return GetSysInfo(pTx, TxCap);

		default:
			return Iso15693Error(pTx, TxCap, ERR_NOT_SUPPORTED);
	}
}
