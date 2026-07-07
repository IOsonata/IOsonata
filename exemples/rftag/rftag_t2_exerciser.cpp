/**-------------------------------------------------------------------------
@file	rftag_t2_exerciser.cpp

@brief	Host exerciser for the RFTag facet with the Type 2 engine

Drives a local memory RFTag with an attached RFTagProtoT2 engine through
ProcessFrame, the same entry a frame transport uses. Checks the Type 2
header, READ, WRITE with protection and sector rules, the SECTOR SELECT
sequence, UID validation, the NDEF round trip through the facet, the write
protect callback, and per instance event capture through an EvtHandler
override.

Link rule: build this file with rftag.cpp, rftag_proto_t2.cpp and
device.cpp. Events are observed by subclassing the tag, each exerciser owns
its instances, nothing is shared between test executables.

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
#include "rftag/rftag_proto_t2.h"

static int s_Pass = 0;
static int s_Fail = 0;

static void Check(const char *pName, bool bOk)
{
	if (bOk)
	{
		s_Pass++;
		printf("  [PASS] %s\n", pName);
	}
	else
	{
		s_Fail++;
		printf("  [FAIL] %s\n", pName);
	}
}

// Tag subclass capturing events per instance. This replaces the former
// global dispatch hook: each test observes its own tag object.
class TestTag : public RFTag {
public:
	TestTag() : MemChanged(0), Reads(0), LastAddr(0), LastLen(0) {}

	virtual void EvtHandler(RFTAG_EVT Evt, uint32_t P0, uint32_t P1)
	{
		if (Evt == RFTAG_EVT_MEM_CHANGED)
		{
			MemChanged++;
			LastAddr = P0;
			LastLen = P1;
		}
		else if (Evt == RFTAG_EVT_READ)
		{
			Reads++;
		}
	}

	int MemChanged;
	int Reads;
	uint32_t LastAddr;
	uint32_t LastLen;
};

// Board style write protect hook capture.
static int s_WpCalls = 0;
static bool s_WpLast = false;

static bool WpCtrl(void *pCtx, bool bVal)
{
	(void)pCtx;
	s_WpCalls++;
	s_WpLast = bVal;
	return true;
}

static bool SetupTag(TestTag &Tag, RFTagProtoT2 &Proto, uint8_t *pMem,
					 uint32_t MemSize, bool bReadOnly)
{
	RFTagCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	memset(pMem, 0, MemSize);

	cfg.pMem = pMem;
	cfg.MemSize = MemSize;
	cfg.bReadOnly = bReadOnly;
	cfg.NdefAddr = 16 + 2;			// data area, past the empty TLV header
	cfg.NdefMaxLen = MemSize - 16 - 2;
	cfg.NdefFmt = RFTAG_NDEF_FMT_TLV;

	if (Tag.Init(cfg) == false)
	{
		return false;
	}

	return Tag.Attach(&Proto);
}

int main(void)
{
	uint8_t mem[144];
	uint8_t tx[32];
	int l;

	printf("== 1. Init and header ==\n");
	{
		TestTag tag;
		RFTagProtoT2 t2;
		Check("init and attach", SetupTag(tag, t2, mem, sizeof(mem), false));
		Check("proto bound", tag.Proto() == &t2);

		uint8_t hdr[19];
		tag.MemRead(0, hdr, sizeof(hdr));

		const uint8_t *uid = RFTagProtoT2::DefaultUid();
		Check("uid block 0", hdr[0] == uid[0] && hdr[1] == uid[1] && hdr[2] == uid[2]);
		Check("bcc0", hdr[3] == (uint8_t)(0x88 ^ uid[0] ^ uid[1] ^ uid[2]));
		Check("bcc1", hdr[8] == (uint8_t)(uid[3] ^ uid[4] ^ uid[5] ^ uid[6]));
		Check("cc magic version", hdr[12] == 0xE1 && hdr[13] == 0x10);
		Check("cc size units", hdr[14] == (sizeof(mem) - 16) / 8);
		Check("cc open access", hdr[15] == 0x00);
		Check("empty ndef tlv", hdr[16] == 0x03 && hdr[17] == 0x00 && hdr[18] == 0xFE);
	}

	printf("== 2. READ ==\n");
	{
		TestTag tag;
		RFTagProtoT2 t2;
		SetupTag(tag, t2, mem, sizeof(mem), false);

		uint8_t rd0[] = { 0x30, 0x00 };
		l = tag.ProcessFrame(rd0, sizeof(rd0), tx, sizeof(tx));
		Check("read block 0 returns 16", l == 16);
		Check("read content matches image", memcmp(tx, mem, 16) == 0);
		Check("read event seen", tag.Reads == 1);

		// Read near the end zero fills past memory: block 34 is bytes
		// 136..151, the image ends at 144.
		uint8_t rde[] = { 0x30, 34 };
		mem[143] = 0x5A;
		l = tag.ProcessFrame(rde, sizeof(rde), tx, sizeof(tx));
		Check("read at end returns 16", l == 16);
		Check("in range byte served", tx[7] == 0x5A);
		Check("past end zero filled", tx[8] == 0x00 && tx[15] == 0x00);

		// Out of range block NAKs.
		uint8_t rdo[] = { 0x30, 0xFF };
		l = tag.ProcessFrame(rdo, sizeof(rdo), tx, sizeof(tx));
		Check("read out of range NAK", l == 1 && tx[0] == 0x00);
	}

	printf("== 3. WRITE ==\n");
	{
		TestTag tag;
		RFTagProtoT2 t2;
		SetupTag(tag, t2, mem, sizeof(mem), false);

		uint8_t wr[] = { 0xA2, 0x05, 0xDE, 0xAD, 0xBE, 0xEF };
		l = tag.ProcessFrame(wr, sizeof(wr), tx, sizeof(tx));
		Check("write block 5 ACK", l == 1 && tx[0] == 0x0A);
		Check("memory updated", mem[20] == 0xDE && mem[23] == 0xEF);
		Check("mem changed event", tag.MemChanged == 1 && tag.LastAddr == 20 && tag.LastLen == 4);

		// Static region blocks 0..3 rejected.
		uint8_t wrs[] = { 0xA2, 0x02, 0x11, 0x22, 0x33, 0x44 };
		l = tag.ProcessFrame(wrs, sizeof(wrs), tx, sizeof(tx));
		Check("write static region NAK", l == 1 && tx[0] == 0x00);

		// Out of range NAK.
		uint8_t wro[] = { 0xA2, 0xFE, 0x11, 0x22, 0x33, 0x44 };
		l = tag.ProcessFrame(wro, sizeof(wro), tx, sizeof(tx));
		Check("write out of range NAK", l == 1 && tx[0] == 0x00);
	}

	printf("== 4. Read only tag ==\n");
	{
		TestTag tag;
		RFTagProtoT2 t2;
		SetupTag(tag, t2, mem, sizeof(mem), true);

		uint8_t cc[4];
		tag.MemRead(12, cc, sizeof(cc));
		Check("cc no write access", cc[3] == 0x0F);

		uint8_t wr[] = { 0xA2, 0x05, 0xDE, 0xAD, 0xBE, 0xEF };
		l = tag.ProcessFrame(wr, sizeof(wr), tx, sizeof(tx));
		Check("reader write rejected", l == 1 && tx[0] == 0x00);
		Check("no memory change", tag.MemChanged == 0);

		// Host side SetNdef still works on a RF read only tag.
		const uint8_t ndef[] = { 0xD1, 0x01, 0x01, 0x54, 0x41 };
		Check("host SetNdef on read only tag", tag.SetNdef(ndef, sizeof(ndef)));
	}

	printf("== 5. SECTOR SELECT ==\n");
	{
		// 2048 bytes allows sector 1: block 256 is byte 1024.
		static uint8_t bigmem[2048];
		TestTag tag;
		RFTagProtoT2 t2;
		SetupTag(tag, t2, bigmem, sizeof(bigmem), false);

		uint8_t ss1[] = { 0xC2, 0xFF };
		l = tag.ProcessFrame(ss1, sizeof(ss1), tx, sizeof(tx));
		Check("sector select frame 1 ACK", l == 1 && tx[0] == 0x0A);

		uint8_t ss2[] = { 0x01, 0x00, 0x00, 0x00 };
		l = tag.ProcessFrame(ss2, sizeof(ss2), tx, sizeof(tx));
		Check("sector select frame 2 silent", l == 0);

		// Write block 0 of sector 1, absolute block 256, byte 1024.
		uint8_t wr[] = { 0xA2, 0x00, 0xAA, 0xBB, 0xCC, 0xDD };
		l = tag.ProcessFrame(wr, sizeof(wr), tx, sizeof(tx));
		Check("write in sector 1 ACK", l == 1 && tx[0] == 0x0A);
		Check("sector 1 memory updated", bigmem[1024] == 0xAA && bigmem[1027] == 0xDD);

		// Unavailable sector NAKs on the second frame.
		tag.ProcessFrame(ss1, sizeof(ss1), tx, sizeof(tx));
		uint8_t ssbad[] = { 0x07, 0x00, 0x00, 0x00 };
		l = tag.ProcessFrame(ssbad, sizeof(ssbad), tx, sizeof(tx));
		Check("unavailable sector NAK", l == 1 && tx[0] == 0x00);
	}

	printf("== 6. UID handling ==\n");
	{
		TestTag tag;
		RFTagProtoT2 t2;
		RFTagCfg_t cfg;

		memset(&cfg, 0, sizeof(cfg));
		memset(mem, 0, sizeof(mem));
		cfg.pMem = mem;
		cfg.MemSize = sizeof(mem);
		cfg.NdefAddr = 18;
		cfg.NdefMaxLen = sizeof(mem) - 18;
		cfg.NdefFmt = RFTAG_NDEF_FMT_TLV;
		static const uint8_t uid7[7] = { 0x04, 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6 };
		memcpy(cfg.NfcId, uid7, 7);
		cfg.IdLen = 7;

		Check("init with 7 byte uid", tag.Init(cfg) && tag.Attach(&t2));
		Check("configured uid in header", mem[0] == 0x04 && mem[7] == 0xF6);

		// Invalid id length rejected at attach.
		TestTag tag2;
		RFTagProtoT2 t22;
		cfg.IdLen = 5;
		Check("init ok with 5 byte id", tag2.Init(cfg));
		Check("attach rejects 5 byte id", tag2.Attach(&t22) == false);
	}

	printf("== 7. NDEF round trip through the facet ==\n");
	{
		TestTag tag;
		RFTagProtoT2 t2;
		SetupTag(tag, t2, mem, sizeof(mem), false);

		const uint8_t ndef[] = { 0xD1, 0x01, 0x05, 0x54, 0x02, 0x65, 0x6E, 0x48, 0x69 };
		Check("SetNdef", tag.SetNdef(ndef, sizeof(ndef)));
		Check("SetNdef raises mem changed", tag.MemChanged == 1);

		uint8_t back[32];
		l = tag.GetNdef(back, sizeof(back));
		Check("GetNdef length", l == (int)sizeof(ndef));
		Check("GetNdef content", memcmp(back, ndef, sizeof(ndef)) == 0);

		// The reader sees the same bytes through READ frames: NdefAddr 18 is
		// in block 4, TLV type at byte 18.
		uint8_t rd[] = { 0x30, 0x04 };
		l = tag.ProcessFrame(rd, sizeof(rd), tx, sizeof(tx));
		Check("reader sees TLV", l == 16 && tx[2] == 0x03 && tx[3] == sizeof(ndef));
	}

	printf("== 8. Write protect hook ==\n");
	{
		TestTag tag;
		RFTagProtoT2 t2;
		SetupTag(tag, t2, mem, sizeof(mem), false);

		// No hook configured: unsupported, honest false.
		Check("no hook reports unsupported", tag.SetWriteProt(true) == false);

		// With the board hook: driven with the requested value.
		TestTag tag2;
		RFTagProtoT2 t22;
		RFTagCfg_t cfg;
		memset(&cfg, 0, sizeof(cfg));
		memset(mem, 0, sizeof(mem));
		cfg.pMem = mem;
		cfg.MemSize = sizeof(mem);
		cfg.NdefAddr = 18;
		cfg.NdefMaxLen = sizeof(mem) - 18;
		cfg.NdefFmt = RFTAG_NDEF_FMT_TLV;
		cfg.WrProtCB = WpCtrl;

		s_WpCalls = 0;
		tag2.Init(cfg);
		tag2.Attach(&t22);
		Check("hook applied on protect", tag2.SetWriteProt(true) && s_WpCalls == 1 && s_WpLast == true);
		Check("hook applied on release", tag2.SetWriteProt(false) && s_WpCalls == 2 && s_WpLast == false);
	}

	printf("== 9. Two instances stay independent ==\n");
	{
		static uint8_t memA[144], memB[144];
		TestTag tagA, tagB;
		RFTagProtoT2 t2A, t2B;

		SetupTag(tagA, t2A, memA, sizeof(memA), false);
		SetupTag(tagB, t2B, memB, sizeof(memB), false);

		uint8_t wr[] = { 0xA2, 0x05, 0x11, 0x22, 0x33, 0x44 };
		tagA.ProcessFrame(wr, sizeof(wr), tx, sizeof(tx));

		Check("tag A saw its event", tagA.MemChanged == 1);
		Check("tag B saw nothing", tagB.MemChanged == 0);
		Check("tag B memory untouched", memB[20] == 0x00);
	}

	printf("\nresult: pass=%d fail=%d\n", s_Pass, s_Fail);
	return s_Fail == 0 ? 0 : 1;
}



