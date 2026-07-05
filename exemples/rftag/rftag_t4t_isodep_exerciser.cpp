/*--------------------------------------------------------------------------
File   : rftag_t4t_isodep_exerciser.cpp

Author : Hoang Nguyen Hoan          Jul. 5, 2026

Desc   : Host side ISO-DEP exerciser for the RFTag Type 4 protocol module

Drives rftag_proto_t4t through the frame sequences a phone reader sends
during a tag session. Serves two purposes: a behavior matrix of the current
module against each sequence, and a regression once R-block and S-block
handling and chaining are added. Build and run on the host, no hardware.

Sequence groups:
	1. Activation and NDEF read, the mandatory happy path
	2. Presence check R(ACK) and R(NAK), phones poll with these mid session
	3. S(DESELECT) end of exchange
	4. Chained I-block, sent when a payload exceeds one frame
	5. Duplicate block number, reader retransmission after a missed reply

Link rule: build this file with rftag_proto_t4t.cpp only. Do not link
rftag.cpp, it defines RFTagEvtDispatch and this file provides its own as
the event observation hook, linking both is a duplicate symbol.

Expected results are marked PASS for behavior that must hold and GAP for
sequences the module does not answer yet. A GAP turning into a response is
a change to review, not an automatic pass.

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
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "rftag/rftag.h"

static int s_Selected = 0;
static int s_Deselected = 0;
static int s_MemChanged = 0;
static int s_Pass = 0;
static int s_Fail = 0;
static int s_Gap = 0;

void RFTagEvtDispatch(RFTagDev_t * const pDev, RFTAG_EVT Evt, uint32_t Addr, uint32_t Len, uint32_t Flags)
{
	(void)pDev;
	(void)Addr;
	(void)Len;
	(void)Flags;

	switch (Evt)
	{
		case RFTAG_EVT_SELECTED:
			s_Selected++;
			break;
		case RFTAG_EVT_DESELECTED:
			s_Deselected++;
			break;
		case RFTAG_EVT_MEM_CHANGED:
			s_MemChanged++;
			break;
		default:
			break;
	}
}

static void Dump(const char *pTag, const uint8_t *p, int n)
{
	printf("  %-24s ->", pTag);
	for (int i = 0; i < n && i < 20; i++)
	{
		printf(" %02X", p[i]);
	}
	if (n > 20)
	{
		printf(" ...");
	}
	printf("  (%d)\n", n);
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

// A GAP records behavior known to be missing. bStillGap true means the module
// still does not answer, which is the recorded state. When it turns false the
// new behavior must be reviewed and the check converted to a PASS rule.
static void Gap(const char *pName, bool bStillGap)
{
	printf("  [%s] %s\n", bStillGap ? "GAP " : "CHANGED", pName);
	s_Gap++;
}

int main()
{
	static uint8_t file[512];
	RFTagDev_t dev;
	uint8_t tx[RFTAG_TX_FRAME_MAX];
	int l;

	memset(&dev, 0, sizeof(dev));
	dev.pMem = file;
	dev.MemSize = sizeof(file);

	if (RFTagProtoT4tBind(&dev) == false || dev.pProto->Init(&dev) == false)
	{
		printf("module init failed\n");
		return 1;
	}

	// NDEF file: NLEN 4, payload ABCD
	file[0] = 0x00;
	file[1] = 0x04;
	memcpy(&file[2], "ABCD", 4);

	printf("== 1. Activation and NDEF read ==\n");

	{
		uint8_t f[] = { 0xE0, 0x80 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Dump("RATS", tx, l);
		Check("ATS returned", l >= 3 && tx[0] == (uint8_t)l);
	}

	{
		uint8_t f[] = { 0x02, 0x00, 0xA4, 0x04, 0x00, 0x07,
						0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Dump("I(0) SELECT app", tx, l);
		Check("app select 9000, bn echo", l == 3 && tx[0] == 0x02 && tx[1] == 0x90 && tx[2] == 0x00);
		Check("SELECTED event", s_Selected == 1);
	}

	{
		uint8_t f[] = { 0x03, 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("CC select 9000", l == 3 && tx[1] == 0x90);
	}

	{
		uint8_t f[] = { 0x02, 0x00, 0xB0, 0x00, 0x00, 0x0F };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Dump("I(0) READ CC", tx, l);
		Check("CC 15 bytes, maps NDEF fid", l == 18 && tx[10] == 0xE1 && tx[11] == 0x04);
	}

	{
		uint8_t f[] = { 0x03, 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("NDEF select 9000", l == 3 && tx[1] == 0x90);
	}

	{
		uint8_t f[] = { 0x02, 0x00, 0xB0, 0x00, 0x00, 0x02 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Dump("I(0) READ NLEN", tx, l);
		Check("NLEN 0004 read", l == 5 && tx[1] == 0x00 && tx[2] == 0x04);
	}

	printf("== 2. R-blocks per ISO 14443-4 rules 11 and 12 ==\n");

	// Session state here: last exchange was I(0) READ NLEN, PICC block
	// number is 0 and the stored response is the NLEN read. Rule 11 uses
	// the last block sent, whatever kind it was, so the retransmission
	// flow is exercised before the presence check changes the last block.
	{
		// Rule 11: the PCD missed the response and asks again with the
		// same number. The NLEN I-block is replayed, not re executed.
		uint8_t f[] = { 0xB2 };	// R(NAK) bn 0, PICC bn 0
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Dump("R(NAK,0) retransmit", tx, l);
		Check("rule 11 NLEN response replayed", l == 5 && tx[0] == 0x02 && tx[1] == 0x00 && tx[2] == 0x04);
	}

	{
		// Rule 12: R(NAK) with a different number is answered by R(ACK)
		// with the PICC current number. This is the phone presence check.
		uint8_t f[] = { 0xB3 };	// R(NAK) bn 1, PICC bn 0
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Dump("R(NAK,1) presence", tx, l);
		Check("rule 12 R(ACK,0) answered", l == 1 && tx[0] == 0xA2);
	}

	{
		// Rule 11 after the presence check: the last block sent is now the
		// rule 12 R(ACK), so that is what a same number R(ACK) replays.
		uint8_t f[] = { 0xA2 };	// R(ACK) bn 0, PICC bn 0
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("rule 11 replays last block, the R(ACK)", l == 1 && tx[0] == 0xA2);
	}

	{
		// Rule 13: R(ACK) with a different number continues chaining only.
		// Not chaining, no reply expected.
		uint8_t f[] = { 0xA3 };	// R(ACK) bn 1, PICC bn 0
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("rule 13 no reply outside chaining", l == 0);
	}

	printf("== 3. S(DESELECT) ==\n");

	{
		uint8_t f[] = { 0xC2 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Dump("S(DESELECT)", tx, l);
		Check("deselect echoed", l == 1 && tx[0] == 0xC2);
		Check("DESELECTED event", s_Deselected == 1);
	}

	printf("== 4. Chained I-block ==\n");

	{
		// Reactivate for a fresh session
		uint8_t r[] = { 0xE0, 0x80 };
		dev.pProto->OnFrame(&dev, r, sizeof(r), tx, sizeof(tx));
		uint8_t a[] = { 0x02, 0x00, 0xA4, 0x04, 0x00, 0x07,
						0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01 };
		dev.pProto->OnFrame(&dev, a, sizeof(a), tx, sizeof(tx));
		uint8_t s[] = { 0x03, 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x04 };
		dev.pProto->OnFrame(&dev, s, sizeof(s), tx, sizeof(tx));

		int mem0 = s_MemChanged;
		uint8_t c[] = { 0x12, 0x00, 0xD6, 0x00, 0x00, 0x02, 0xAA, 0xBB };
		l = dev.pProto->OnFrame(&dev, c, sizeof(c), tx, sizeof(tx));
		Gap("chained UPDATE dropped, large writes stall", l == 0);
		Check("chained frame caused no memory write", s_MemChanged == mem0);
	}

	printf("== 5. Duplicate block number, rule D retransmission ==\n");

	{
		// An I-block received with the PICC current number is a PCD
		// retransmission. The stored response is replayed and the command
		// is not executed again. Verified with UPDATE, where a second
		// execution would be a double write.
		int mem0 = s_MemChanged;
		uint8_t u[] = { 0x02, 0x00, 0xD6, 0x00, 0x02, 0x02, 0x41, 0x42 };
		int l1 = dev.pProto->OnFrame(&dev, u, sizeof(u), tx, sizeof(tx));
		Check("UPDATE executed once", l1 == 3 && tx[1] == 0x90 && s_MemChanged == mem0 + 1);
		int l2 = dev.pProto->OnFrame(&dev, u, sizeof(u), tx, sizeof(tx));
		Check("duplicate replayed, no second write", l2 == l1 && tx[1] == 0x90 && s_MemChanged == mem0 + 1);
	}

	printf("== 6. DESELECT with CID ==\n");

	{
		uint8_t f[] = { 0xCA, 0x00 };
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Check("S(DESELECT) CID echoed", l == 2 && tx[0] == 0xCA && tx[1] == 0x00);
	}

	printf("\nresult: pass=%d fail=%d gap=%d\n", s_Pass, s_Fail, s_Gap);

	return s_Fail == 0 ? 0 : 1;
}
