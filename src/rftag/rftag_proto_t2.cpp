/**-------------------------------------------------------------------------
@file	rftag_proto_t2.cpp

@brief	NFC Forum Type 2 tag protocol engine implementation

Type 2 static and dynamic layout served from the tag memory image: blocks
0..2 UID with check bytes and lock bytes, block 3 Capability Container, data
from block 4. READ returns 16 bytes zero filled past memory, WRITE applies
the selected sector and protects the static region and read only tags,
SECTOR SELECT is the two frame sequence, HALT ends with no reply.

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

#include "rftag/rftag_proto_t2.h"

#define T2T_CMD_READ			0x30
#define T2T_CMD_WRITE			0xA2
#define T2T_CMD_SECTOR_SELECT	0xC2
#define T2T_CMD_HALT			0x50

#define T2T_ACK					0x0A	//!< 4 bit ACK
#define T2T_NAK					0x00	//!< 4 bit NAK

#define T2T_BLOCK_SIZE			4
#define T2T_READ_BLOCKS			4		//!< READ returns 4 blocks
#define T2T_HEADER_LEN			16		//!< Blocks 0..3
#define T2T_CC_MAGIC			0xE1
#define T2T_CC_VERSION			0x10
#define T2T_CT					0x88	//!< Cascade tag for UID BCC0

// Engine default UID, NXP manufacturer prefix 0x04. Replace by matching the
// transport NFCID1 in a deployment that needs a unique or specific id.
static const uint8_t s_T2DefaultUid[7] = {
	0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

const uint8_t *RFTagProtoT2::DefaultUid()
{
	return s_T2DefaultUid;
}

void RFTagProtoT2::BuildHeader()
{
	uint8_t hdr[T2T_HEADER_LEN + 3];

	// Type 2 uses a 7 byte double size UID. Take it from the tag when a
	// 7 byte id is set, otherwise fall back to the engine default.
	const uint8_t *uid = vpTag->IdLen() == 7 ? vpTag->NfcId() : s_T2DefaultUid;

	// Blocks 0..2, UID with the two block check bytes and lock bytes
	hdr[0] = uid[0];
	hdr[1] = uid[1];
	hdr[2] = uid[2];
	hdr[3] = (uint8_t)(T2T_CT ^ uid[0] ^ uid[1] ^ uid[2]);
	hdr[4] = uid[3];
	hdr[5] = uid[4];
	hdr[6] = uid[5];
	hdr[7] = uid[6];
	hdr[8] = (uint8_t)(uid[3] ^ uid[4] ^ uid[5] ^ uid[6]);
	hdr[9] = 0x00;	// internal
	hdr[10] = 0x00;	// lock 0
	hdr[11] = 0x00;	// lock 1

	// Block 3, Capability Container
	uint32_t dataLen = vpTag->MemSize() - T2T_HEADER_LEN;
	uint32_t units = dataLen / 8;

	if (units > 0xFF)
	{
		units = 0xFF;
	}

	hdr[12] = T2T_CC_MAGIC;
	hdr[13] = T2T_CC_VERSION;
	hdr[14] = (uint8_t)units;
	// Access: high nibble read, low nibble write. 0x00 open, 0x0F no write.
	hdr[15] = vpTag->ReadOnly() ? 0x0F : 0x00;

	int hlen = T2T_HEADER_LEN;

	// Block 4, empty NDEF message TLV so the tag reads as a valid empty tag
	// until SetNdef writes real content.
	if (vpTag->MemSize() >= T2T_HEADER_LEN + 3)
	{
		hdr[16] = 0x03;	// NDEF message TLV
		hdr[17] = 0x00;	// length 0
		hdr[18] = 0xFE;	// terminator TLV
		hlen += 3;
	}

	vpTag->MemWrite(0, hdr, hlen);
}

int RFTagProtoT2::CmdRead(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	vLastTxBits = 0;
	if (RxLen < 2 || pTx == nullptr || TxCap < 1)
	{
		return 0;
	}

	uint32_t blk = (uint32_t)vSector * 256u + pRx[1];
	uint32_t addr = blk * T2T_BLOCK_SIZE;

	if (addr >= vpTag->MemSize())
	{
		pTx[0] = T2T_NAK;
		vLastTxBits = 4;
		return 1;
	}

	// 16 bytes from the image, zero filled past the end of memory.
	int n = T2T_READ_BLOCKS * T2T_BLOCK_SIZE;
	if (TxCap < n)
	{
		return 0;
	}

	memset(pTx, 0, n);

	int avail = vpTag->MemRead(addr, pTx, n);
	if (avail <= 0)
	{
		pTx[0] = T2T_NAK;
		vLastTxBits = 4;
		return 1;
	}

	vpTag->EvtHandler(RFTAG_EVT_READ, addr, n);
	vLastTxBits = n * 8;

	return n;
}

int RFTagProtoT2::CmdWrite(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	vLastTxBits = 0;
	if (RxLen < 6 || pTx == nullptr || TxCap < 1)
	{
		return 0;
	}

	// Apply the selected sector, the same way READ does. The static UID,
	// lock and CC region is absolute blocks 0..3, that is sector 0 only.
	uint32_t blk = (uint32_t)vSector * 256u + pRx[1];

	// A read only tag rejects every write.
	if (blk < 4 || vpTag->ReadOnly())
	{
		pTx[0] = T2T_NAK;
		vLastTxBits = 4;
		return 1;
	}

	uint32_t addr = blk * T2T_BLOCK_SIZE;

	if (addr + T2T_BLOCK_SIZE > vpTag->MemSize())
	{
		pTx[0] = T2T_NAK;
		vLastTxBits = 4;
		return 1;
	}

	if (vpTag->MemWrite(addr, &pRx[2], T2T_BLOCK_SIZE) != T2T_BLOCK_SIZE)
	{
		pTx[0] = T2T_NAK;
		vLastTxBits = 4;
		return 1;
	}

	vpTag->EvtHandler(RFTAG_EVT_MEM_CHANGED, addr, T2T_BLOCK_SIZE);

	pTx[0] = T2T_ACK;
	vLastTxBits = 4;
	return 1;
}

bool RFTagProtoT2::Init(RFTag * const pTag)
{
	if (pTag == nullptr || pTag->MemSize() < T2T_HEADER_LEN + T2T_BLOCK_SIZE)
	{
		return false;
	}

	if (pTag->IdLen() != 0 && pTag->IdLen() != 7)
	{
		return false;
	}

	vpTag = pTag;
	vSector = 0;
	vbSecSelPending = false;
	vLastTxBits = 0;

	BuildHeader();

	return true;
}

int RFTagProtoT2::OnFrame(const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	vLastTxBits = 0;
	if (vpTag == nullptr || pRx == nullptr || RxLen < 1)
	{
		return 0;
	}

	// Second frame of a SECTOR SELECT, the sector number and 3 RFU bytes.
	// Require the full 4 byte frame. A sector whose base is past the memory
	// is not available, answer NAK. A valid selection switches with no reply.
	if (vbSecSelPending)
	{
		vbSecSelPending = false;

		if (RxLen < 4)
		{
			return 0;
		}

		if ((uint32_t)pRx[0] * 256u * T2T_BLOCK_SIZE >= vpTag->MemSize())
		{
			if (pTx != nullptr && TxCap >= 1)
			{
				pTx[0] = T2T_NAK;
				vLastTxBits = 4;
				return 1;
			}
			return 0;
		}

		vSector = pRx[0];

		return 0;
	}

	switch (pRx[0])
	{
		case T2T_CMD_READ:
			return CmdRead(pRx, RxLen, pTx, TxCap);

		case T2T_CMD_WRITE:
			return CmdWrite(pRx, RxLen, pTx, TxCap);

		case T2T_CMD_SECTOR_SELECT:
			// First frame, 0xC2 0xFF. Acknowledge and wait for the number.
			if (RxLen >= 2 && pRx[1] == 0xFF && pTx != nullptr && TxCap >= 1)
			{
				vbSecSelPending = true;
				pTx[0] = T2T_ACK;
				vLastTxBits = 4;
				return 1;
			}
			if (pTx != nullptr && TxCap >= 1)
			{
				pTx[0] = T2T_NAK;
				vLastTxBits = 4;
				return 1;
			}
			return 0;

		case T2T_CMD_HALT:
			// Go to halt, no reply.
			return 0;

		default:
			if (pTx != nullptr && TxCap >= 1)
			{
				pTx[0] = T2T_NAK;
				vLastTxBits = 4;
				return 1;
			}
			return 0;
	}
}
