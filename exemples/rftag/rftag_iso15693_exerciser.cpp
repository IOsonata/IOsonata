/**-------------------------------------------------------------------------
@file	rftag_iso15693_exerciser.cpp

@brief	Host side exerciser for the RFTag ISO15693 Type 5 protocol module

Drives rftag_proto_iso15693 through the Type 5 command set: Inventory, Get
System Information, Read Single Block, Read Multiple Blocks, Write Single
Block, Capability Container protection, read only tag, addressed mode, Select,
Stay Quiet and Reset to Ready. Build and run on the host, no hardware.

Link rule: build this file with rftag_proto_iso15693.cpp only. Do not link
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
	RFTagProtoIso15693Bind(pDev);
	pDev->pProto->Init(pDev);
}

int main()
{
	static uint8_t mem[64];		// 16 blocks, block 0 CC, 15 data blocks
	RFTagDev_t dev;
	uint8_t tx[128];
	int l;

	SetupTag(&dev, mem, sizeof(mem), false);

	const uint8_t *uid = RFTagProtoIso15693DefaultUid();

	printf("== 1. Inventory ==\n");

	{
		// Inventory flag set, mask length 0
		uint8_t f[] = { 0x26, 0x01, 0x00 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("inventory answered", l == 10 && tx[0] == 0x00);
		Check("DSFID then UID low octet first", tx[1] == 0x00 && tx[2] == uid[0] && tx[9] == uid[7]);
	}

	printf("== 2. Get System Information ==\n");

	{
		uint8_t f[] = { 0x02, 0x2B };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("sys info no error", l >= 14 && tx[0] == 0x00 && tx[1] == 0x0F);
		Check("UID echoed", memcmp(&tx[2], uid, 8) == 0);
		Check("block count minus 1", tx[12] == (sizeof(mem) / 4) - 1);
		Check("block size minus 1", tx[13] == 3);
	}

	printf("== 3. Capability Container and empty NDEF ==\n");

	{
		uint8_t f[] = { 0x02, 0x20, 0x00 };		// read single, block 0
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("read block 0 no error", l == 5 && tx[0] == 0x00);
		Check("CC magic and size", tx[1] == 0xE1 && tx[3] == (sizeof(mem) / 8));
		uint8_t r1[] = { 0x02, 0x20, 0x01 };	// block 1, NDEF area
		dev.pProto->OnFrame(&dev, r1, sizeof(r1), tx, sizeof(tx));
		Check("empty NDEF TLV", tx[1] == 0x03 && tx[2] == 0x00 && tx[3] == 0xFE);
	}

	printf("== 4. Write and read back ==\n");

	{
		int mem0 = s_MemChanged;
		uint8_t w[] = { 0x02, 0x21, 0x02, 0xDE, 0xAD, 0xBE, 0xEF };	// write block 2
		l = dev.pProto->OnFrame(&dev, w, sizeof(w), tx, sizeof(tx));
		Check("write single no error", l == 1 && tx[0] == 0x00);
		Check("MEM_CHANGED once", s_MemChanged == mem0 + 1);
		uint8_t r[] = { 0x02, 0x20, 0x02 };
		dev.pProto->OnFrame(&dev, r, sizeof(r), tx, sizeof(tx));
		Check("data read back", tx[1] == 0xDE && tx[2] == 0xAD && tx[3] == 0xBE && tx[4] == 0xEF);
	}

	printf("== 5. Read multiple blocks ==\n");

	{
		uint8_t f[] = { 0x02, 0x23, 0x01, 0x02 };		// from block 1, 3 blocks
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("read multiple 3 blocks", l == 1 + 3 * 4 && tx[0] == 0x00);
	}

	printf("== 6. Protection and range ==\n");

	{
		uint8_t w0[] = { 0x02, 0x21, 0x00, 0x11, 0x22, 0x33, 0x44 };		// write CC
		l = dev.pProto->OnFrame(&dev, w0, sizeof(w0), tx, sizeof(tx));
		Check("write CC block locked", l == 2 && tx[0] == 0x01 && tx[1] == 0x12);

		uint8_t ro[] = { 0x02, 0x20, 0x40 };		// block 64, out of range
		l = dev.pProto->OnFrame(&dev, ro, sizeof(ro), tx, sizeof(tx));
		Check("read out of range block NA", l == 2 && tx[0] == 0x01 && tx[1] == 0x10);
	}

	printf("== 7. Addressed mode ==\n");

	{
		// Addressed read of block 1 with the matching UID
		uint8_t f[16];
		int n = 0;
		f[n++] = 0x22;			// address flag
		f[n++] = 0x20;			// read single
		memcpy(&f[n], uid, 8);
		n += 8;
		f[n++] = 0x01;			// block 1
		l = dev.pProto->OnFrame(&dev, f, n, tx, sizeof(tx));
		Check("addressed read matches UID", l == 5 && tx[0] == 0x00);

		// Addressed read with a wrong UID, no reply
		uint8_t f2[16];
		n = 0;
		f2[n++] = 0x22;
		f2[n++] = 0x20;
		memset(&f2[n], 0xAA, 8);
		n += 8;
		f2[n++] = 0x01;
		l = dev.pProto->OnFrame(&dev, f2, n, tx, sizeof(tx));
		Check("addressed read wrong UID silent", l == 0);
	}

	printf("== 8. State, Stay Quiet and Reset ==\n");

	{
		// Stay Quiet addressed
		uint8_t sq[16];
		int n = 0;
		sq[n++] = 0x22;
		sq[n++] = 0x02;
		memcpy(&sq[n], uid, 8);
		n += 8;
		l = dev.pProto->OnFrame(&dev, sq, n, tx, sizeof(tx));
		Check("stay quiet no reply", l == 0);

		// Non addressed inventory now ignored
		uint8_t inv[] = { 0x26, 0x01, 0x00 };
		l = dev.pProto->OnFrame(&dev, inv, sizeof(inv), tx, sizeof(tx));
		Check("quiet tag ignores inventory", l == 0);

		// Select addressed brings it back
		uint8_t sel[16];
		n = 0;
		sel[n++] = 0x22;
		sel[n++] = 0x25;
		memcpy(&sel[n], uid, 8);
		n += 8;
		l = dev.pProto->OnFrame(&dev, sel, n, tx, sizeof(tx));
		Check("select no error", l == 1 && tx[0] == 0x00);

		uint8_t inv2[] = { 0x26, 0x01, 0x00 };
		l = dev.pProto->OnFrame(&dev, inv2, sizeof(inv2), tx, sizeof(tx));
		Check("inventory answered after select", l == 10);
	}

	printf("== 8b. Addressed and selected mode gating ==\n");

	{
		static uint8_t selmem[64];
		RFTagDev_t sd;
		SetupTag(&sd, selmem, sizeof(selmem), false);
		const uint8_t *suid = RFTagProtoIso15693DefaultUid();

		// Stay Quiet without address must not silence the tag.
		uint8_t sqn[] = { 0x02, 0x02 };
		sd.pProto->OnFrame(&sd, sqn, sizeof(sqn), tx, sizeof(tx));
		uint8_t invq[] = { 0x26, 0x01, 0x00 };
		l = sd.pProto->OnFrame(&sd, invq, sizeof(invq), tx, sizeof(tx));
		Check("unaddressed stay quiet ignored", l == 10);

		// Selected flag read before Select must be silent.
		uint8_t rsel[] = { 0x12, 0x20, 0x00 };
		l = sd.pProto->OnFrame(&sd, rsel, sizeof(rsel), tx, sizeof(tx));
		Check("selected read before select silent", l == 0);

		// Select without address must not select the tag.
		uint8_t seln[] = { 0x02, 0x25 };
		sd.pProto->OnFrame(&sd, seln, sizeof(seln), tx, sizeof(tx));
		l = sd.pProto->OnFrame(&sd, rsel, sizeof(rsel), tx, sizeof(tx));
		Check("unaddressed select does not select", l == 0);

		// Select addressed, then selected flag read must answer.
		uint8_t sela[16];
		int n = 0;
		sela[n++] = 0x22;
		sela[n++] = 0x25;
		memcpy(&sela[n], suid, 8);
		n += 8;
		sd.pProto->OnFrame(&sd, sela, n, tx, sizeof(tx));
		l = sd.pProto->OnFrame(&sd, rsel, sizeof(rsel), tx, sizeof(tx));
		Check("selected read after select answers", l > 0);

		// Reset to Ready clears selected state.
		uint8_t rtr[] = { 0x02, 0x26 };
		sd.pProto->OnFrame(&sd, rtr, sizeof(rtr), tx, sizeof(tx));
		l = sd.pProto->OnFrame(&sd, rsel, sizeof(rsel), tx, sizeof(tx));
		Check("selected read after reset silent", l == 0);
	}

	printf("== 9. Read only tag ==\n");

	{
		static uint8_t romem[64];
		RFTagDev_t ro;
		SetupTag(&ro, romem, sizeof(romem), true);

		uint8_t rc[] = { 0x02, 0x20, 0x00 };
		ro.pProto->OnFrame(&ro, rc, sizeof(rc), tx, sizeof(tx));
		Check("CC write access denied bits", (tx[2] & 0x0C) == 0x0C);

		int mem0 = s_MemChanged;
		uint8_t w[] = { 0x02, 0x21, 0x02, 0x01, 0x02, 0x03, 0x04 };
		l = ro.pProto->OnFrame(&ro, w, sizeof(w), tx, sizeof(tx));
		Check("write locked on read only", l == 2 && tx[0] == 0x01 && tx[1] == 0x12);
		Check("no memory write", s_MemChanged == mem0);
	}

	printf("== 10. UID length handling ==\n");

	{
		static uint8_t um[64];
		RFTagDev_t u;
		uint8_t myuid[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x04, 0xE0 };
		memset(&u, 0, sizeof(u));
		u.pMem = um;
		u.MemSize = sizeof(um);
		u.IdLen = 8;
		memcpy(u.NfcId, myuid, 8);
		RFTagProtoIso15693Bind(&u);
		Check("init with 8 byte UID", u.pProto->Init(&u));

		uint8_t inv[] = { 0x26, 0x01, 0x00 };
		u.pProto->OnFrame(&u, inv, sizeof(inv), tx, sizeof(tx));
		Check("configured UID in inventory", memcmp(&tx[2], myuid, 8) == 0);

		RFTagDev_t u5;
		memset(&u5, 0, sizeof(u5));
		u5.pMem = um;
		u5.MemSize = sizeof(um);
		u5.IdLen = 5;
		RFTagProtoIso15693Bind(&u5);
		Check("init rejects IdLen 5", u5.pProto->Init(&u5) == false);
	}

	printf("\nresult: pass=%d fail=%d\n", s_Pass, s_Fail);

	return s_Fail == 0 ? 0 : 1;
}
