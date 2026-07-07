/**-------------------------------------------------------------------------
@file	rftag_t4_exerciser.cpp

@brief	Host exerciser for the RFTag facet with the Type 4 engine

Drives a local memory RFTag with an attached RFTagProtoT4 engine through
ProcessFrame. Checks activation and the NDEF read path, the ISO 14443-4
R-block rules 11 and 12, S(DESELECT), chained I-block handling, rule D
duplicate replay, DESELECT with CID, and a read only tag. Events are captured
per instance through an EvtHandler override, no global hook.

Link rule: build this file with rftag.cpp, rftag_proto_t4.cpp and device.cpp.

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
#include "rftag/rftag_proto_t4.h"

static int s_Pass = 0;
static int s_Fail = 0;
static int s_Gap = 0;

static void Check(const char *pName, bool bOk)
{
	if (bOk) { s_Pass++; printf("  [PASS] %s\n", pName); }
	else { s_Fail++; printf("  [FAIL] %s\n", pName); }
}

// A known limitation, reported but not counted as a failure.
static void Gap(const char *pName, bool bOk)
{
	if (bOk) { s_Gap++; printf("  [GAP ] %s\n", pName); }
	else { s_Fail++; printf("  [FAIL] %s\n", pName); }
}

class TestTag : public RFTag {
public:
	TestTag() : Selected(0), Deselected(0), MemChanged(0) {}

	virtual void EvtHandler(RFTAG_EVT Evt, uint32_t P0, uint32_t P1)
	{
		(void)P0; (void)P1;
		if (Evt == RFTAG_EVT_SELECTED) { Selected++; }
		else if (Evt == RFTAG_EVT_DESELECTED) { Deselected++; }
		else if (Evt == RFTAG_EVT_MEM_CHANGED) { MemChanged++; }
	}

	int Selected;
	int Deselected;
	int MemChanged;
};

static bool SetupTag(TestTag &Tag, RFTagProtoT4 &Proto, uint8_t *pMem,
					 uint32_t MemSize, bool bReadOnly)
{
	RFTagCfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));

	cfg.pMem = pMem;
	cfg.MemSize = MemSize;
	cfg.bReadOnly = bReadOnly;
	cfg.NdefAddr = 0;
	cfg.NdefMaxLen = MemSize;
	cfg.NdefFmt = RFTAG_NDEF_FMT_NLEN16;

	if (Tag.Init(cfg) == false)
	{
		return false;
	}

	return Tag.Attach(&Proto);
}

int main(void)
{
	static uint8_t file[512];
	uint8_t tx[512];
	int l;

	TestTag tag;
	RFTagProtoT4 t4;

	if (SetupTag(tag, t4, file, sizeof(file), false) == false)
	{
		printf("setup failed\n");
		return 1;
	}

	// NDEF file: NLEN 4, payload ABCD
	file[0] = 0x00;
	file[1] = 0x04;
	memcpy(&file[2], "ABCD", 4);

	printf("== 1. Activation and NDEF read ==\n");
	{
		uint8_t f[] = { 0xE0, 0x80 };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("ATS returned", l >= 3 && tx[0] == (uint8_t)l);
	}
	{
		uint8_t f[] = { 0x02, 0x00, 0xA4, 0x04, 0x00, 0x07,
						0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01 };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("app select 9000, bn echo", l == 3 && tx[0] == 0x02 && tx[1] == 0x90 && tx[2] == 0x00);
		Check("SELECTED event", tag.Selected == 1);
	}
	{
		uint8_t f[] = { 0x03, 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03 };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("CC select 9000", l == 3 && tx[1] == 0x90);
	}
	{
		uint8_t f[] = { 0x02, 0x00, 0xB0, 0x00, 0x00, 0x0F };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("CC 15 bytes, maps NDEF fid", l == 18 && tx[10] == 0xE1 && tx[11] == 0x04);
	}
	{
		uint8_t f[] = { 0x03, 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04 };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("NDEF select 9000", l == 3 && tx[1] == 0x90);
	}
	{
		uint8_t f[] = { 0x02, 0x00, 0xB0, 0x00, 0x00, 0x02 };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("NLEN 0004 read", l == 5 && tx[1] == 0x00 && tx[2] == 0x04);
	}

	printf("== 2. R-blocks per ISO 14443-4 rules 11 and 12 ==\n");
	{
		// Rule 11: R-block with the current block number replays the last block.
		uint8_t f[] = { 0xA2 };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("rule 11 NLEN response replayed", l == 5 && tx[0] == 0x02 && tx[1] == 0x00 && tx[2] == 0x04);
	}
	{
		// Rule 12: R(NAK) with a different block number answered R(ACK).
		uint8_t f[] = { 0xB3 };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("rule 12 R(ACK,0) answered", l == 1 && tx[0] == 0xA2);
	}
	{
		uint8_t f[] = { 0xA2 };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("rule 11 replays last block, the R(ACK)", l == 1 && tx[0] == 0xA2);
	}
	{
		// Rule 13: R(ACK) with a different number, no chaining, no reply.
		uint8_t f[] = { 0xA3 };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("rule 13 no reply outside chaining", l == 0);
	}

	printf("== 3. S(DESELECT) ==\n");
	{
		uint8_t f[] = { 0xC2 };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("deselect echoed", l == 1 && tx[0] == 0xC2);
		Check("DESELECTED event", tag.Deselected == 1);
	}

	printf("== 4. Chained I-block ==\n");
	{
		uint8_t r[] = { 0xE0, 0x80 };
		tag.ProcessFrame(r, sizeof(r), tx, sizeof(tx));
		uint8_t a[] = { 0x02, 0x00, 0xA4, 0x04, 0x00, 0x07,
						0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01 };
		tag.ProcessFrame(a, sizeof(a), tx, sizeof(tx));
		uint8_t s[] = { 0x03, 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04 };
		tag.ProcessFrame(s, sizeof(s), tx, sizeof(tx));
		int mem0 = tag.MemChanged;
		// Chaining bit set: not supported, dropped.
		uint8_t c[] = { 0x12, 0x00, 0xD6, 0x00, 0x02, 0x02, 0x11, 0x22 };
		l = tag.ProcessFrame(c, sizeof(c), tx, sizeof(tx));
		Gap("chained UPDATE dropped, large writes stall", l == 0);
		Check("chained frame caused no memory write", tag.MemChanged == mem0);
	}

	printf("== 5. Duplicate block number, rule D retransmission ==\n");
	{
		int mem0 = tag.MemChanged;
		// The last accepted I-block was block 1 (the E1 04 select). A new
		// I-block toggles to 0. Send an UPDATE at block 0, then repeat it.
		uint8_t u[] = { 0x02, 0x00, 0xD6, 0x00, 0x02, 0x02, 0x55, 0x66 };
		int l1 = tag.ProcessFrame(u, sizeof(u), tx, sizeof(tx));
		Check("UPDATE executed once", l1 == 3 && tx[1] == 0x90 && tag.MemChanged == mem0 + 1);
		int l2 = tag.ProcessFrame(u, sizeof(u), tx, sizeof(tx));
		Check("duplicate replayed, no second write", l2 == l1 && tx[1] == 0x90 && tag.MemChanged == mem0 + 1);
	}

	printf("== 6. DESELECT with CID ==\n");
	{
		uint8_t f[] = { 0xCA, 0x00 };
		l = tag.ProcessFrame(f, sizeof(f), tx, sizeof(tx));
		Check("S(DESELECT) CID echoed", l == 2 && tx[0] == 0xCA && tx[1] == 0x00);
	}

	printf("== 7. Read only tag ==\n");
	{
		static uint8_t rofile[512];
		TestTag ro;
		RFTagProtoT4 t4ro;

		memcpy(rofile, file, sizeof(rofile));
		SetupTag(ro, t4ro, rofile, sizeof(rofile), true);

		uint8_t r0[] = { 0xE0, 0x80 };
		ro.ProcessFrame(r0, sizeof(r0), tx, sizeof(tx));
		uint8_t a0[] = { 0x02, 0x00, 0xA4, 0x04, 0x00, 0x07,
						 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01 };
		ro.ProcessFrame(a0, sizeof(a0), tx, sizeof(tx));

		uint8_t sc[] = { 0x03, 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03 };
		ro.ProcessFrame(sc, sizeof(sc), tx, sizeof(tx));
		uint8_t rc[] = { 0x02, 0x00, 0xB0, 0x00, 0x00, 0x0F };
		l = ro.ProcessFrame(rc, sizeof(rc), tx, sizeof(tx));
		Check("CC write access 0xFF", l == 18 && tx[15] == 0xFF);

		uint8_t sn[] = { 0x03, 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04 };
		ro.ProcessFrame(sn, sizeof(sn), tx, sizeof(tx));
		int mem0 = ro.MemChanged;
		uint8_t up[] = { 0x02, 0x00, 0xD6, 0x00, 0x02, 0x02, 0x55, 0x66 };
		l = ro.ProcessFrame(up, sizeof(up), tx, sizeof(tx));
		Check("UPDATE rejected 6982", l == 3 && tx[1] == 0x69 && tx[2] == 0x82);
		Check("no memory write on read only tag", ro.MemChanged == mem0);

		uint8_t rd[] = { 0x03, 0x00, 0xB0, 0x00, 0x00, 0x02 };
		l = ro.ProcessFrame(rd, sizeof(rd), tx, sizeof(tx));
		Check("READ still allowed", l == 5 && tx[l-2] == 0x90);
	}

	printf("\nresult: pass=%d fail=%d gap=%d\n", s_Pass, s_Fail, s_Gap);
	return s_Fail == 0 ? 0 : 1;
}
