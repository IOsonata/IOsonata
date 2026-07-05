/**-------------------------------------------------------------------------
@file	rftag_proto_t2t.cpp

@brief	NFC Forum Type 2 Tag target protocol for RFTag

RFTag protocol behavior selected when RFTagCfg_t Proto is RFTAG_PROTO_NFC_T2.
Type 2 is NFC-A with no ISO-DEP layer. After the hardware finishes activation
and anticollision the reader talks directly to the tag with block commands.
This module answers READ, WRITE and SECTOR SELECT against the tag local
memory pointed to by RFTagDev_t pMem.

Memory image, 4 byte blocks, the whole image is pMem:
	block 0..2	UID, BCC and lock bytes
	block 3		Capability Container
	block 4..	NDEF data area as a TLV, 0x03 LEN DATA 0xFE

Init builds blocks 0..3. The NDEF area is written by RFTagSetNdef with
NdefFmt RFTAG_NDEF_FMT_TLV and NdefAddr 16, which is block 4.

The UID in block 0..2 must equal the NFCID1 the transport uses for
anticollision, otherwise a reader that cross checks the two rejects the tag.
Type 2 uses the 7 byte double size UID only. Configure RFTagCfg_t IdLen as 0
for the module default or 7 for a supplied id, any other length is rejected
at init. RFTagProtoT2tDefaultUid returns the module default so the transport
can be configured with the same value.

READ replies with 16 bytes and the hardware appends the CRC. WRITE and
SECTOR SELECT reply with a 4 bit ACK or NAK per the Type 2 specification.
This module returns them as a single byte, a byte frame transport sends 8
bits. A bit accurate reply needs the bit level transmit path of the
transport and is the hardware tuning point for Type 2, in the same way the
ATS bytes are for Type 4.

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

typedef struct {
	uint8_t Sector;				//!< Current sector from SECTOR SELECT
	bool bSecSelPending;		//!< First frame of a SECTOR SELECT seen
} T2tState_t;

static_assert(sizeof(T2tState_t) <= RFTAG_PROTO_STATE_SIZE, "T2T state too large for RFTag ProtoState");

// Module default UID, NXP manufacturer prefix 0x04. Replace by matching the
// transport NFCID1 in a deployment that needs a unique or specific id.
static const uint8_t s_T2tDefaultUid[7] = {
	0x04, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66
};

static bool T2tProtoInit(RFTagDev_t * const pDev);
static int T2tOnFrame(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap);

static const RFTagProto_t s_T2tProto = {
	.Init = T2tProtoInit,
	.OnFrame = T2tOnFrame,
	.OnApdu = nullptr,
};

static inline T2tState_t *T2tGetState(RFTagDev_t * const pDev)
{
	return (T2tState_t *)pDev->ProtoState;
}

const uint8_t *RFTagProtoT2tDefaultUid(void)
{
	return s_T2tDefaultUid;
}

static void T2tBuildHeader(RFTagDev_t * const pDev)
{
	uint8_t *p = pDev->pMem;

	// Type 2 uses a 7 byte double size UID. Take it from the config when a
	// 7 byte id is set, otherwise fall back to the module default.
	const uint8_t *uid = pDev->IdLen == 7 ? pDev->NfcId : s_T2tDefaultUid;

	// Blocks 0..2, UID with the two block check bytes and lock bytes
	p[0] = uid[0];
	p[1] = uid[1];
	p[2] = uid[2];
	p[3] = (uint8_t)(T2T_CT ^ uid[0] ^ uid[1] ^ uid[2]);
	p[4] = uid[3];
	p[5] = uid[4];
	p[6] = uid[5];
	p[7] = uid[6];
	p[8] = (uint8_t)(uid[3] ^ uid[4] ^ uid[5] ^ uid[6]);
	p[9] = 0x00;	// internal
	p[10] = 0x00;	// lock 0
	p[11] = 0x00;	// lock 1

	// Block 3, Capability Container
	uint32_t dataLen = pDev->MemSize - T2T_HEADER_LEN;
	uint32_t units = dataLen / 8;

	if (units > 0xFF)
	{
		units = 0xFF;
	}

	p[12] = T2T_CC_MAGIC;
	p[13] = T2T_CC_VERSION;
	p[14] = (uint8_t)units;
	// Access: high nibble read, low nibble write. 0x00 open, 0x0F no write.
	p[15] = pDev->bReadOnly ? 0x0F : 0x00;

	// Block 4, empty NDEF message TLV so the tag reads as a valid empty tag
	// until RFTagSetNdef writes real content.
	if (pDev->MemSize >= T2T_HEADER_LEN + 3)
	{
		p[16] = 0x03;	// NDEF message TLV
		p[17] = 0x00;	// length 0
		p[18] = 0xFE;	// terminator TLV
	}
}

static int T2tRead(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (RxLen < 2 || TxCap < T2T_READ_BLOCKS * T2T_BLOCK_SIZE)
	{
		return 0;
	}

	T2tState_t *st = T2tGetState(pDev);
	uint32_t blk = (uint32_t)st->Sector * 256 + pRx[1];
	uint32_t addr = blk * T2T_BLOCK_SIZE;

	if (addr >= pDev->MemSize)
	{
		pTx[0] = T2T_NAK;
		return 1;
	}

	// 16 bytes from the image, zero filled past the end of memory.
	int n = T2T_READ_BLOCKS * T2T_BLOCK_SIZE;

	for (int i = 0; i < n; i++)
	{
		uint32_t a = addr + i;
		pTx[i] = a < pDev->MemSize ? pDev->pMem[a] : 0x00;
	}

	return n;
}

static int T2tWrite(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (RxLen < 6 || TxCap < 1)
	{
		return 0;
	}

	// Apply the selected sector, the same way READ does. The static UID,
	// lock and CC region is absolute blocks 0..3, that is sector 0 only.
	uint32_t blk = (uint32_t)T2tGetState(pDev)->Sector * 256u + pRx[1];

	// A read only tag rejects every write.
	if (blk < 4 || pDev->bReadOnly)
	{
		pTx[0] = T2T_NAK;
		return 1;
	}

	uint32_t addr = blk * T2T_BLOCK_SIZE;

	if (addr + T2T_BLOCK_SIZE > pDev->MemSize)
	{
		pTx[0] = T2T_NAK;
		return 1;
	}

	memcpy(&pDev->pMem[addr], &pRx[2], T2T_BLOCK_SIZE);
	RFTagEvtDispatch(pDev, RFTAG_EVT_MEM_CHANGED, addr, T2T_BLOCK_SIZE, 0);

	pTx[0] = T2T_ACK;
	return 1;
}

static bool T2tProtoInit(RFTagDev_t * const pDev)
{
	if (pDev->pMem == nullptr || pDev->MemSize < T2T_HEADER_LEN + T2T_BLOCK_SIZE)
	{
		return false;
	}

	// Type 2 uses the 7 byte double size UID only. Accept 0, meaning use the
	// module default, or 7. Any other length is a configuration error and is
	// rejected rather than silently replaced by the default.
	if (pDev->IdLen != 0 && pDev->IdLen != 7)
	{
		return false;
	}

	T2tState_t *st = T2tGetState(pDev);

	memset(st, 0, sizeof(T2tState_t));
	T2tBuildHeader(pDev);

	return true;
}

static int T2tOnFrame(RFTagDev_t * const pDev, const uint8_t *pRx, int RxLen, uint8_t *pTx, int TxCap)
{
	if (RxLen < 1)
	{
		return 0;
	}

	T2tState_t *st = T2tGetState(pDev);

	// Second frame of a SECTOR SELECT, the sector number and 3 RFU bytes.
	// Require the full 4 byte frame. A sector whose base is past the memory
	// is not available, answer NAK. A valid selection switches with no reply.
	if (st->bSecSelPending)
	{
		st->bSecSelPending = false;

		if (RxLen < 4)
		{
			return 0;
		}

		if ((uint32_t)pRx[0] * 256u * T2T_BLOCK_SIZE >= pDev->MemSize)
		{
			if (TxCap >= 1)
			{
				pTx[0] = T2T_NAK;
				return 1;
			}
			return 0;
		}

		st->Sector = pRx[0];

		return 0;
	}

	switch (pRx[0])
	{
		case T2T_CMD_READ:
			return T2tRead(pDev, pRx, RxLen, pTx, TxCap);

		case T2T_CMD_WRITE:
			return T2tWrite(pDev, pRx, RxLen, pTx, TxCap);

		case T2T_CMD_SECTOR_SELECT:
			// First frame, 0xC2 0xFF. Acknowledge and wait for the number.
			if (RxLen >= 2 && pRx[1] == 0xFF && TxCap >= 1)
			{
				st->bSecSelPending = true;
				pTx[0] = T2T_ACK;
				return 1;
			}
			if (TxCap >= 1)
			{
				pTx[0] = T2T_NAK;
				return 1;
			}
			return 0;

		case T2T_CMD_HALT:
			// Go to halt, no reply.
			return 0;

		default:
			if (TxCap >= 1)
			{
				pTx[0] = T2T_NAK;
				return 1;
			}
			return 0;
	}
}

bool RFTagProtoT2tBind(RFTagDev_t * const pDev)
{
	if (pDev == nullptr)
	{
		return false;
	}

	pDev->pProto = &s_T2tProto;

	return true;
}
