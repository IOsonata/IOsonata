/**-------------------------------------------------------------------------
@file	rftag_t2t_exerciser.cpp

@brief	Host side exerciser for the RFTag Type 2 protocol module

Drives rftag_proto_t2t through the Type 2 command set: READ across the header
and data blocks, WRITE to the data area, write protection of the static
blocks and of a read only tag, SECTOR SELECT and unknown command handling.
Build and run on the host, no hardware.

Link rule: build this file with rftag_proto_t2t.cpp only. Do not link
rftag.cpp, it defines RFTagEvtDispatch and this file provides its own as the
event observation hook, linking both is a duplicate symbol.

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
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "rftag/rftag.h"

static int s_MemChanged = 0;
static int s_Pass = 0;
static int s_Fail = 0;

void RFTagEvtDispatch(RFTagDev_t * const pDev, RFTAG_EVT Evt, uint32_t Addr, uint32_t Len, uint32_t Flags)
{
	(void)pDev;
	(void)Addr;
	(void)Len;
	(void)Flags;

	if (Evt == RFTAG_EVT_MEM_CHANGED)
	{
		s_MemChanged++;
	}
}

static void Check(const char *pName, bool bOk)
{
	printf("  [%s] %s\n", bOk ? "PASS" : "FAIL", pName);
	if (bOk)
	{
		s_Pass++;
	}
	else
	{
		s_Fail++;
	}
}

static void SetupTag(RFTagDev_t *pDev, uint8_t *pMem, uint32_t Size, bool bRo)
{
	memset(pDev, 0, sizeof(*pDev));
	pDev->pMem = pMem;
	pDev->MemSize = Size;
	pDev->bReadOnly = bRo;
	RFTagProtoT2tBind(pDev);
	pDev->pProto->Init(pDev);
}

int main()
{
	static uint8_t mem[64];		// 16 blocks, 12 data blocks after the header
	RFTagDev_t dev;
	uint8_t tx[64];
	int l;

	SetupTag(&dev, mem, sizeof(mem), false);

	printf("== 1. Header and CC ==\n");

	{
		const uint8_t *uid = RFTagProtoT2tDefaultUid();
		// READ block 0 returns UID with BCC0 and part of block 1
		uint8_t f[] = { 0x30, 0x00 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("READ returns 16 bytes", l == 16);
		Check("UID0..2 in block 0", tx[0] == uid[0] && tx[1] == uid[1] && tx[2] == uid[2]);
		Check("BCC0 valid", tx[3] == (uint8_t)(0x88 ^ uid[0] ^ uid[1] ^ uid[2]));
		Check("BCC1 valid", tx[8] == (uint8_t)(uid[3] ^ uid[4] ^ uid[5] ^ uid[6]));
	}

	{
		// READ block 3 returns CC then the NDEF area start
		uint8_t f[] = { 0x30, 0x03 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		uint32_t units = (sizeof(mem) - 16) / 8;
		Check("CC magic and version", tx[0] == 0xE1 && tx[1] == 0x10);
		Check("CC size units", tx[2] == units);
		Check("CC access read write", tx[3] == 0x00);
		Check("empty NDEF TLV present", tx[4] == 0x03 && tx[5] == 0x00 && tx[6] == 0xFE);
	}

	printf("== 2. WRITE to data area ==\n");

	{
		int mem0 = s_MemChanged;
		uint8_t f[] = { 0xA2, 0x04, 0xDE, 0xAD, 0xBE, 0xEF };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("WRITE block 4 ACK", l == 1 && tx[0] == 0x0A);
		Check("MEM_CHANGED once", s_MemChanged == mem0 + 1);
		uint8_t r[] = { 0x30, 0x04 };
		dev.pProto->OnFrame(&dev, r, sizeof(r), tx, sizeof(tx));
		Check("data read back", tx[0] == 0xDE && tx[1] == 0xAD && tx[2] == 0xBE && tx[3] == 0xEF);
	}

	printf("== 3. Static block and range protection ==\n");

	{
		int mem0 = s_MemChanged;
		uint8_t f[] = { 0xA2, 0x00, 0x11, 0x22, 0x33, 0x44 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("WRITE block 0 NAK", l == 1 && tx[0] == 0x00);
		Check("no write to UID", s_MemChanged == mem0);
	}

	{
		// block 16 is past a 64 byte image, out of range
		uint8_t f[] = { 0xA2, 0x10, 0x01, 0x02, 0x03, 0x04 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("WRITE out of range NAK", l == 1 && tx[0] == 0x00);
	}

	{
		uint8_t f[] = { 0x30, 0x10 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("READ out of range NAK", l == 1 && tx[0] == 0x00);
	}

	printf("== 4. SECTOR SELECT ==\n");

	{
		uint8_t f1[] = { 0xC2, 0xFF };
		l = dev.pProto->OnFrame(&dev, f1, sizeof(f1), tx, sizeof(tx));
		Check("sector select frame 1 ACK", l == 1 && tx[0] == 0x0A);
		// Sector 1 base is 1024, past this 64 byte image, so it is not
		// available and the second frame must NAK.
		uint8_t f2[] = { 0x01, 0x00, 0x00, 0x00 };
		l = dev.pProto->OnFrame(&dev, f2, sizeof(f2), tx, sizeof(tx));
		Check("unavailable sector NAK", l == 1 && tx[0] == 0x00);
		// Short second frame is ignored
		uint8_t f1b[] = { 0xC2, 0xFF };
		dev.pProto->OnFrame(&dev, f1b, sizeof(f1b), tx, sizeof(tx));
		uint8_t f2b[] = { 0x00, 0x00 };
		l = dev.pProto->OnFrame(&dev, f2b, sizeof(f2b), tx, sizeof(tx));
		Check("short second frame ignored", l == 0);
	}

	printf("== 4b. Multi sector write applies sector ==\n");

	{
		// A 2 sector image, one full sector plus a second sector block 4.
		static uint8_t big[1024 + 32];
		RFTagDev_t ms;
		SetupTag(&ms, big, sizeof(big), false);

		// Select sector 1
		uint8_t s1[] = { 0xC2, 0xFF };
		ms.pProto->OnFrame(&ms, s1, sizeof(s1), tx, sizeof(tx));
		uint8_t s2[] = { 0x01, 0x00, 0x00, 0x00 };
		l = ms.pProto->OnFrame(&ms, s2, sizeof(s2), tx, sizeof(tx));
		Check("sector 1 selected, no reply", l == 0);

		// Write block 4 in sector 1, absolute block 260, byte 1040
		uint8_t w[] = { 0xA2, 0x04, 0xCA, 0xFE, 0xF0, 0x0D };
		l = ms.pProto->OnFrame(&ms, w, sizeof(w), tx, sizeof(tx));
		Check("sector 1 write ACK", l == 1 && tx[0] == 0x0A);
		Check("landed in sector 1 not sector 0", big[1040] == 0xCA && big[16] != 0xCA);

		// Read block 4 in sector 1 returns the written data
		uint8_t r[] = { 0x30, 0x04 };
		ms.pProto->OnFrame(&ms, r, sizeof(r), tx, sizeof(tx));
		Check("sector 1 read back", tx[0] == 0xCA && tx[1] == 0xFE);
	}

	printf("== 5. Unknown command ==\n");

	{
		uint8_t f[] = { 0x1B, 0x00 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("unknown command NAK", l == 1 && tx[0] == 0x00);
	}

	printf("== 6. Read only tag ==\n");

	{
		static uint8_t romem[64];
		RFTagDev_t ro;
		SetupTag(&ro, romem, sizeof(romem), true);

		uint8_t c[] = { 0x30, 0x03 };
		ro.pProto->OnFrame(&ro, c, sizeof(c), tx, sizeof(tx));
		Check("CC access read only 0x0F", tx[3] == 0x0F);

		int mem0 = s_MemChanged;
		uint8_t w[] = { 0xA2, 0x04, 0x01, 0x02, 0x03, 0x04 };
		l = ro.pProto->OnFrame(&ro, w, sizeof(w), tx, sizeof(tx));
		Check("WRITE NAK on read only", l == 1 && tx[0] == 0x00);
		Check("no memory write", s_MemChanged == mem0);
	}

	printf("== 7. UID length handling ==\n");

	{
		static uint8_t um[64];
		RFTagDev_t u;

		// A 7 byte configured UID is used in the header
		uint8_t myuid[7] = { 0x04, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
		memset(&u, 0, sizeof(u));
		u.pMem = um;
		u.MemSize = sizeof(um);
		u.IdLen = 7;
		memcpy(u.NfcId, myuid, 7);
		RFTagProtoT2tBind(&u);
		Check("init with 7 byte UID", u.pProto->Init(&u));

		uint8_t r[] = { 0x30, 0x00 };
		u.pProto->OnFrame(&u, r, sizeof(r), tx, sizeof(tx));
		Check("configured UID in block 0", tx[0] == 0x04 && tx[1] == 0xAA && tx[2] == 0xBB);
		Check("BCC0 of configured UID", tx[3] == (uint8_t)(0x88 ^ 0x04 ^ 0xAA ^ 0xBB));

		// A 4 byte length is not supported and must fail init
		RFTagDev_t u4;
		memset(&u4, 0, sizeof(u4));
		u4.pMem = um;
		u4.MemSize = sizeof(um);
		u4.IdLen = 4;
		RFTagProtoT2tBind(&u4);
		Check("init rejects IdLen 4", u4.pProto->Init(&u4) == false);
	}

	printf("\nresult: pass=%d fail=%d\n", s_Pass, s_Fail);

	return s_Fail == 0 ? 0 : 1;
}
