/**-------------------------------------------------------------------------
@file	bt_smp_link_test_stubs.h

@brief	Capture buffer shared with the SMP link test stubs

The stubs live in their own translation unit so their strong definitions beat
the weak ones inside bt_smp.cpp, which the test includes. This header is the
only thing the two sides share.

@author	Hoang Nguyen Hoan
@date	Aug. 14, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#ifndef __BT_SMP_LINK_TEST_STUBS_H__
#define __BT_SMP_LINK_TEST_STUBS_H__

#include <stddef.h>
#include <stdint.h>

#include "bluetooth/bt_gap.h"

#define BT_SMP_TEST_MAX_PDU			16
#define BT_SMP_TEST_MAX_PDU_LEN		96

/// Every SMP PDU the host handed to the transport, in order, stripped of the
/// ACL and L2CAP headers so index 0 of each entry is the SMP Code.
typedef struct __Bt_Smp_Test_Capture {
	int Count;
	uint16_t ConnHdl[BT_SMP_TEST_MAX_PDU];
	size_t Len[BT_SMP_TEST_MAX_PDU];
	uint8_t Pdu[BT_SMP_TEST_MAX_PDU][BT_SMP_TEST_MAX_PDU_LEN];
} BtSmpTestCapture_t;

extern BtSmpTestCapture_t g_BtSmpTestCapture;

/// Discard everything captured so far.
void BtSmpTestCaptureReset(void);

/// Number of captured PDUs whose SMP Code is this one.
int BtSmpTestCaptureCount(uint8_t Code);

/// Times BtSmpBondAdd was reached since the last reset.
int BtSmpTestBondAddCount(void);

/// What the next BtSmpBondAdd calls answer. Reset restores true.
void BtSmpTestBondAddResultSet(bool Result);

/// Times BtSmpBondStoreFailed was reached since the last reset.
int BtSmpTestBondStoreFailedCount(void);

/// Present a peer record to BtPeerFindByHdl for this handle, or withdraw it.
/// Off by default and after a reset: most cases want the null peer, which
/// skips the security state reporting and leaves key distribution to run on
/// the link record alone. A case about what the link reports turns it on.
/// pHciDev is the device the user interaction paths read back off the peer;
/// pass null when the case does not send anything.
void BtSmpTestPeerSet(bool Present, uint16_t ConnHdl, bool Secure,
					  void *pHciDev);

/// The last BtConnSec_t handed to BtGapConnSecSet, and how many times it was
/// called since the last reset.
void BtSmpTestConnSecGet(BtConnSec_t *pSec);
int BtSmpTestConnSecCount(void);

#endif // __BT_SMP_LINK_TEST_STUBS_H__
