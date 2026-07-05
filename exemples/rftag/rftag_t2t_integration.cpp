/**-------------------------------------------------------------------------
@file	rftag_t2t_integration.cpp

@brief	Public path integration test for the RFTag Type 2 tag

Unlike the protocol exerciser, this test goes through the public RFTagInit
path with RFTAG_PROTO_NFC_T2, writes NDEF with RFTagSetNdef in the TLV format
at block 4, has a reader READ the data area, and checks the RFTagGetNdef round
trip. It validates the build gating, the config consistency of NdefAddr,
NdefMaxLen and Size, and the TLV writer against the module.

Build with RFTAG_PROTO_T2T_ENABLE and link rftag.cpp, rftag_ndef.cpp,
rftag_proto_t2t.cpp and device_intrf.cpp. This test uses the real
RFTagEvtDispatch from rftag.cpp, it does not define its own.

rftag.cpp includes the target headers idelay.h and iopinctrl.h, which are per
part. To build this on a host, put a host include path ahead that provides
small stubs for usDelay, msDelay and the IOPin inline helpers, and link the
IOPinConfig stub below. The IOsonata host test include set already provides
these stubs.

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

#include "device_intrf.h"
#include "rftag/rftag.h"
#include "rftag/rftag_ndef.h"

// Host stub for the target pin driver, not used on this path.
extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
}

static int s_Pass = 0;
static int s_Fail = 0;

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

int main()
{
	static uint8_t mem[128];
	static DevIntrf_t dummy;		// RFTagInit requires a non null transport

	memset(&dummy, 0, sizeof(dummy));
	atomic_flag_clear(&dummy.bBusy);

	RFTagDev_t tag;
	RFTagCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.Proto = RFTAG_PROTO_NFC_T2;
	cfg.XCap = RFTAG_XCAP_ANTICOL | RFTAG_XCAP_CRC | RFTAG_XCAP_FDT;
	cfg.pMem = mem;
	cfg.MemSize = sizeof(mem);
	cfg.AddrLen = 0;
	cfg.Size = sizeof(mem);
	cfg.NdefFmt = RFTAG_NDEF_FMT_TLV;
	cfg.NdefAddr = 16;					// block 4
	cfg.NdefMaxLen = sizeof(mem) - 16;

	Check("RFTagInit with RFTAG_PROTO_NFC_T2", RFTagInit(&tag, &cfg, &dummy));

	uint8_t msgbuf[64];
	RFNdefMsg_t msg;

	RFNdefInit(&msg, msgbuf, sizeof(msgbuf));
	RFNdefAddText(&msg, "en", "T2");

	Check("RFTagSetNdef TLV", RFTagSetNdef(&tag, msg.pBuf, msg.Len));

	// A reader reads block 4, the data area start, the TLV must be there.
	uint8_t tx[32];
	uint8_t rd[] = { 0x30, 0x04 };
	int l = tag.pProto->OnFrame(&tag, rd, sizeof(rd), tx, sizeof(tx));

	Check("READ block 4 returns 16 bytes", l == 16);
	Check("NDEF message TLV at block 4", tx[0] == 0x03 && tx[1] == (uint8_t)msg.Len);

	// GetNdef round trips the message through the core.
	uint8_t back[64];
	int g = RFTagGetNdef(&tag, back, sizeof(back));

	Check("GetNdef round trip", g == (int)msg.Len && memcmp(back, msg.pBuf, g) == 0);

	printf("\nresult: pass=%d fail=%d\n", s_Pass, s_Fail);

	return s_Fail == 0 ? 0 : 1;
}
