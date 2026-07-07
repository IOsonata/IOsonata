/**-------------------------------------------------------------------------
@file	rftag_iso15693_exerciser.cpp

@brief	Host exerciser for the RFTag facet with the ISO15693 engine

Drives a local memory RFTag with an attached RFTagProtoIso15693 engine
through ProcessFrame. Checks inventory, system information, the CC and empty
NDEF layout, write and read back, read multiple, block protection and range,
addressed mode, the addressed and selected mode gating, a read only tag, and
UID length handling. Events are captured per instance through an EvtHandler
override, no global hook.

Link rule: build with rftag.cpp, rftag_proto_iso15693.cpp and device.cpp.

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
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "rftag/rftag.h"
#include "rftag/rftag_proto_iso15693.h"

static int s_Pass = 0;
static int s_Fail = 0;

static void Check(const char *pName, bool bOk)
{
	if (bOk) { s_Pass++; printf("  [PASS] %s\n", pName); }
	else { s_Fail++; printf("  [FAIL] %s\n", pName); }
}

class TestTag : public RFTag {
public:
	TestTag() : MemChanged(0) {}

	virtual void EvtHandler(RFTAG_EVT Evt, uint32_t P0, uint32_t P1)
	{
		(void)P0; (void)P1;
		if (Evt == RFTAG_EVT_MEM_CHANGED) { MemChanged++; }
	}

	int MemChanged;
};

static bool SetupTag(TestTag &Tag, RFTagProtoIso15693 &Proto, uint8_t *pMem,
					 uint32_t Size, bool bRo, const uint8_t *pUid = nullptr, int IdLen = 0)
{
	RFTagCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));

	cfg.pMem = pMem;
	cfg.MemSize = Size;
	cfg.bReadOnly = bRo;
	cfg.NdefAddr = 0;
	cfg.NdefMaxLen = Size;
	cfg.NdefFmt = RFTAG_NDEF_FMT_NONE;

	if (pUid != nullptr && IdLen > 0)
	{
		memcpy(cfg.NfcId, pUid, IdLen);
		cfg.IdLen = IdLen;
	}

	if (Tag.Init(cfg) == false)
	{
		return false;
	}

	return Tag.Attach(&Proto);
}

int main(void)
{
	static uint8_t mem[64];		// 16 blocks, block 0 CC, 15 data blocks
	uint8_t tx[128];
	int l;

	TestTag dev;
	RFTagProtoIso15693 iso;

	if (SetupTag(dev, iso, mem, sizeof(mem), false) == false)
	{
		printf("setup failed\n");
		return 1;
	}

	const uint8_t *uid = RFTagProtoIso15693::DefaultUid();

	printf("== 1. Inventory ==\n");
	{
		// Inventory flag set, mask length 0
		uint8_t f[] = { 0x26, 0x01, 0x00 };
		l = dev.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("inventory answered", l == 10 && tx[0] == 0x00);
		Check("DSFID then UID low octet first", tx[1] == 0x00 && tx[2] == uid[0] && tx[9] == uid[7]);
	}

	printf("== 2. Get System Information ==\n");
	{
		uint8_t f[] = { 0x02, 0x2B };
		l = dev.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("sys info no error", l >= 14 && tx[0] == 0x00 && tx[1] == 0x0F);
		Check("UID echoed", memcmp(&tx[2], uid, 8) == 0);
		Check("block count minus 1", tx[12] == (sizeof(mem) / 4) - 1);
		Check("block size minus 1", tx[13] == 3);
	}

	printf("== 3. Capability Container and empty NDEF ==\n");
	{
		uint8_t f[] = { 0x02, 0x20, 0x00 };		// read single, block 0
		l = dev.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("read block 0 no error", l == 5 && tx[0] == 0x00);
		Check("CC magic and size", tx[1] == 0xE1 && tx[3] == (sizeof(mem) / 8));
		uint8_t r1[] = { 0x02, 0x20, 0x01 };	// block 1, NDEF area
		dev.ProcessFrame(r1, sizeof(r1), tx, sizeof(tx));
		Check("empty NDEF TLV", tx[1] == 0x03 && tx[2] == 0x00 && tx[3] == 0xFE);
	}

	printf("== 4. Write and read back ==\n");
	{
		int mem0 = dev.MemChanged;
		uint8_t w[] = { 0x02, 0x21, 0x02, 0xDE, 0xAD, 0xBE, 0xEF };	// write block 2
		l = dev.ProcessFrame(w, sizeof(w), tx, sizeof(tx));
		Check("write single no error", l == 1 && tx[0] == 0x00);
		Check("MEM_CHANGED once", dev.MemChanged == mem0 + 1);
		uint8_t r[] = { 0x02, 0x20, 0x02 };
		dev.ProcessFrame(r, sizeof(r), tx, sizeof(tx));
		Check("data read back", tx[1] == 0xDE && tx[2] == 0xAD && tx[3] == 0xBE && tx[4] == 0xEF);
	}

	printf("== 5. Read multiple blocks ==\n");
	{
		uint8_t f[] = { 0x02, 0x23, 0x01, 0x02 };		// from block 1, 3 blocks
		l = dev.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("read multiple 3 blocks", l == 1 + 3 * 4 && tx[0] == 0x00);
	}

	printf("== 6. Protection and range ==\n");
	{
		uint8_t w0[] = { 0x02, 0x21, 0x00, 0x11, 0x22, 0x33, 0x44 };		// write CC
		l = dev.ProcessFrame(w0, sizeof(w0), tx, sizeof(tx));
		Check("write CC block locked", l == 2 && tx[0] == 0x01 && tx[1] == 0x12);

		uint8_t ro[] = { 0x02, 0x20, 0x40 };		// block 64, out of range
		l = dev.ProcessFrame(ro, sizeof(ro), tx, sizeof(tx));
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
		l = dev.ProcessFrame(f, n, tx, sizeof(tx));
		Check("addressed read matches UID", l == 5 && tx[0] == 0x00);

		// Addressed read with a wrong UID, no reply
		uint8_t f2[16];
		n = 0;
		f2[n++] = 0x22;
		f2[n++] = 0x20;
		memset(&f2[n], 0xAA, 8);
		n += 8;
		f2[n++] = 0x01;
		l = dev.ProcessFrame(f2, n, tx, sizeof(tx));
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
		l = dev.ProcessFrame(sq, n, tx, sizeof(tx));
		Check("stay quiet no reply", l == 0);

		// Non addressed inventory now ignored
		uint8_t inv[] = { 0x26, 0x01, 0x00 };
		l = dev.ProcessFrame(inv, sizeof(inv), tx, sizeof(tx));
		Check("quiet tag ignores inventory", l == 0);

		// Select addressed brings it back
		uint8_t sel[16];
		n = 0;
		sel[n++] = 0x22;
		sel[n++] = 0x25;
		memcpy(&sel[n], uid, 8);
		n += 8;
		l = dev.ProcessFrame(sel, n, tx, sizeof(tx));
		Check("select no error", l == 1 && tx[0] == 0x00);

		uint8_t inv2[] = { 0x26, 0x01, 0x00 };
		l = dev.ProcessFrame(inv2, sizeof(inv2), tx, sizeof(tx));
		Check("inventory answered after select", l == 10);
	}

	printf("== 8b. Addressed and selected mode gating ==\n");
	{
		static uint8_t selmem[64];
		TestTag sd;
		RFTagProtoIso15693 sdiso;
		SetupTag(sd, sdiso, selmem, sizeof(selmem), false);
		const uint8_t *suid = RFTagProtoIso15693::DefaultUid();

		// Stay Quiet without address must not silence the tag.
		uint8_t sqn[] = { 0x02, 0x02 };
		sd.ProcessFrame(sqn, sizeof(sqn), tx, sizeof(tx));
		uint8_t invq[] = { 0x26, 0x01, 0x00 };
		l = sd.ProcessFrame(invq, sizeof(invq), tx, sizeof(tx));
		Check("unaddressed stay quiet ignored", l == 10);

		// Selected flag read before Select must be silent.
		uint8_t rsel[] = { 0x12, 0x20, 0x00 };
		l = sd.ProcessFrame(rsel, sizeof(rsel), tx, sizeof(tx));
		Check("selected read before select silent", l == 0);

		// Select without address must not select the tag.
		uint8_t seln[] = { 0x02, 0x25 };
		sd.ProcessFrame(seln, sizeof(seln), tx, sizeof(tx));
		l = sd.ProcessFrame(rsel, sizeof(rsel), tx, sizeof(tx));
		Check("unaddressed select does not select", l == 0);

		// Select addressed, then selected flag read must answer.
		uint8_t sela[16];
		int n = 0;
		sela[n++] = 0x22;
		sela[n++] = 0x25;
		memcpy(&sela[n], suid, 8);
		n += 8;
		sd.ProcessFrame(sela, n, tx, sizeof(tx));
		l = sd.ProcessFrame(rsel, sizeof(rsel), tx, sizeof(tx));
		Check("selected read after select answers", l > 0);

		// Reset to Ready clears selected state.
		uint8_t rtr[] = { 0x02, 0x26 };
		sd.ProcessFrame(rtr, sizeof(rtr), tx, sizeof(tx));
		l = sd.ProcessFrame(rsel, sizeof(rsel), tx, sizeof(tx));
		Check("selected read after reset silent", l == 0);
	}

	printf("== 9. Read only tag ==\n");
	{
		static uint8_t romem[64];
		TestTag ro;
		RFTagProtoIso15693 roiso;
		SetupTag(ro, roiso, romem, sizeof(romem), true);

		uint8_t rc[] = { 0x02, 0x20, 0x00 };
		ro.ProcessFrame(rc, sizeof(rc), tx, sizeof(tx));
		Check("CC write access denied bits", (tx[2] & 0x0C) == 0x0C);

		int mem0 = ro.MemChanged;
		uint8_t w[] = { 0x02, 0x21, 0x02, 0x01, 0x02, 0x03, 0x04 };
		l = ro.ProcessFrame(w, sizeof(w), tx, sizeof(tx));
		Check("write locked on read only", l == 2 && tx[0] == 0x01 && tx[1] == 0x12);
		Check("no memory write", ro.MemChanged == mem0);
	}

	printf("== 10. UID length handling ==\n");
	{
		static uint8_t um[64];
		uint8_t myuid[8] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x04, 0xE0 };
		TestTag u;
		RFTagProtoIso15693 uiso;
		Check("init with 8 byte UID", SetupTag(u, uiso, um, sizeof(um), false, myuid, 8));

		uint8_t inv[] = { 0x26, 0x01, 0x00 };
		u.ProcessFrame(inv, sizeof(inv), tx, sizeof(tx));
		Check("configured UID in inventory", memcmp(&tx[2], myuid, 8) == 0);

		static uint8_t um5[64];
		uint8_t uid5[5] = { 0x11, 0x22, 0x33, 0x44, 0x55 };
		TestTag u5;
		RFTagProtoIso15693 u5iso;
		Check("init rejects IdLen 5", SetupTag(u5, u5iso, um5, sizeof(um5), false, uid5, 5) == false);
	}

	printf("\nresult: pass=%d fail=%d\n", s_Pass, s_Fail);
	return s_Fail == 0 ? 0 : 1;
}
