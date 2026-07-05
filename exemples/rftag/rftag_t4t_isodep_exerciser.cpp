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

	printf("== 2. Presence check R-blocks ==\n");

	{
		uint8_t f[] = { 0xA2 };	// R(ACK) bn 0
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Gap("R(ACK) unanswered, phone presence check will fail", l == 0);
	}

	{
		uint8_t f[] = { 0xB3 };	// R(NAK) bn 1
		l = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		Gap("R(NAK) unanswered, no retransmission path", l == 0);
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

	printf("== 5. Duplicate block number ==\n");

	{
		// Same READ sent twice with the same block number simulates a reader
		// retransmission. The spec expects the previous response again without
		// re executing the command. The module re executes, which is harmless
		// for READ but wrong for UPDATE. Recorded as a gap.
		uint8_t f[] = { 0x02, 0x00, 0xB0, 0x00, 0x00, 0x02 };
		int l1 = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		uint8_t first[8];
		memcpy(first, tx, l1 < 8 ? l1 : 8);
		int l2 = dev.pProto->OnFrame(&dev, f, sizeof(f), tx, sizeof(tx));
		bool bSame = (l1 == l2) && memcmp(first, tx, l1 < 8 ? l1 : 8) == 0;
		Gap("duplicate bn re executes instead of replaying stored response", bSame);
	}

	printf("\nresult: pass=%d fail=%d gap=%d\n", s_Pass, s_Fail, s_Gap);

	return s_Fail == 0 ? 0 : 1;
}
