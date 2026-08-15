/**-------------------------------------------------------------------------
@file	bt_smp_link_test_stubs.cpp

@brief	Transport capture and platform stubs for the SMP link host tests

The transport and platform hooks in bt_smp.cpp are weak definitions so a port
can supply its own. The test includes bt_smp.cpp to reach the per-link state,
which puts those weak definitions in the test translation unit, so the
replacements have to live here: a strong definition in a separate object beats
the weak one at link time.

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
#include <string.h>

#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_smp.h"

#include "bt_smp_link_test_stubs.h"

BtSmpTestCapture_t g_BtSmpTestCapture;

static int s_BondAddCount;
static bool s_BondAddResult = true;
static int s_BondStoreFailedCount;
static BtDevice_t s_Peer;
static bool s_PeerPresent;
static BtConnSec_t s_ConnSec;
static int s_ConnSecCount;

void BtSmpTestCaptureReset(void)
{
	memset(&g_BtSmpTestCapture, 0, sizeof(g_BtSmpTestCapture));
	s_BondAddCount = 0;
	s_BondStoreFailedCount = 0;
	s_BondAddResult = true;
	memset(&s_Peer, 0, sizeof(s_Peer));
	s_PeerPresent = false;
	memset(&s_ConnSec, 0, sizeof(s_ConnSec));
	s_ConnSecCount = 0;
}

void BtSmpTestPeerSet(bool Present, uint16_t ConnHdl, bool Secure)
{
	memset(&s_Peer, 0, sizeof(s_Peer));
	s_Peer.Conn.Hdl = ConnHdl;
	s_Peer.bSecure = Secure;
	s_PeerPresent = Present;
}

void BtSmpTestConnSecGet(BtConnSec_t *pSec)
{
	if (pSec != nullptr)
	{
		*pSec = s_ConnSec;
	}
}

int BtSmpTestConnSecCount(void)
{
	return s_ConnSecCount;
}

int BtSmpTestCaptureCount(uint8_t Code)
{
	int n = 0;

	for (int i = 0; i < g_BtSmpTestCapture.Count; i++)
	{
		if (g_BtSmpTestCapture.Len[i] > 0 && g_BtSmpTestCapture.Pdu[i][0] == Code)
		{
			n++;
		}
	}

	return n;
}

int BtSmpTestBondAddCount(void)
{
	return s_BondAddCount;
}

// The SMP core hands whole ACL packets to the transport. Keep the SMP payload
// only, so a case reads the PDU codes it expects without walking headers.
uint32_t BtHciSendAcl(BtHciDevice_t * const pDev, BtHciACLDataPacket_t * const pAcl)
{
	(void)pDev;

	if (pAcl == nullptr || g_BtSmpTestCapture.Count >= BT_SMP_TEST_MAX_PDU)
	{
		return 0;
	}

	const BtL2CapPdu_t *pPdu = (const BtL2CapPdu_t*)pAcl->Data;
	size_t len = pPdu->Hdr.Len;

	if (pPdu->Hdr.Cid != BT_L2CAP_CID_SEC_MNGR || len == 0 ||
		len > BT_SMP_TEST_MAX_PDU_LEN)
	{
		return 0;
	}

	int idx = g_BtSmpTestCapture.Count;

	g_BtSmpTestCapture.ConnHdl[idx] = (uint16_t)pAcl->Hdr.ConnHdl;
	g_BtSmpTestCapture.Len[idx] = len;
	memcpy(g_BtSmpTestCapture.Pdu[idx], &pPdu->Smp, len);
	g_BtSmpTestCapture.Count++;

	return (uint32_t)(pAcl->Hdr.Len + sizeof(BtHciACLDataPacketHdr_t));
}

void BtSmpTestBondAddResultSet(bool Result)
{
	s_BondAddResult = Result;
}

int BtSmpTestBondStoreFailedCount(void)
{
	return s_BondStoreFailedCount;
}

bool BtSmpBondAdd(uint16_t ConnHdl, const BtSmpKeys_t *pKeys)
{
	(void)ConnHdl;
	(void)pKeys;

	s_BondAddCount++;

	return s_BondAddResult;
}

void BtSmpBondStoreFailed(uint16_t ConnHdl)
{
	(void)ConnHdl;

	s_BondStoreFailedCount++;
}

bool BtSmpBondLtkLookup(uint16_t ConnHdl, uint64_t Rand, uint16_t Ediv,
						uint8_t Ltk[16])
{
	(void)ConnHdl;
	(void)Rand;
	(void)Ediv;
	(void)Ltk;

	return false;
}

bool BtSmpBondKeysLookup(uint16_t ConnHdl, uint64_t Rand, uint16_t Ediv,
						 BtSmpKeys_t *pKeys)
{
	(void)ConnHdl;
	(void)Rand;
	(void)Ediv;
	(void)pKeys;

	return false;
}

bool BtSmpBonded(uint16_t ConnHdl)
{
	(void)ConnHdl;

	return false;
}

void BtSmpBondLoad(void)
{
}

void BtGapConnSecSet(uint16_t ConnHdl, const BtConnSec_t *pSec)
{
	(void)ConnHdl;

	if (pSec != nullptr)
	{
		s_ConnSec = *pSec;
	}
	s_ConnSecCount++;
}

void BtGattCccdRestoreBonded(uint16_t ConnHdl)
{
	(void)ConnHdl;
}

// No peer pool in this test. A null peer skips the security-state reporting
// block, which is not what most of these cases are about, and leaves the key
// distribution phase to run on the link record alone. A case about what the
// link reports calls BtSmpTestPeerSet to present one.
BtDevice_t *BtPeerFindByHdl(uint16_t ConnHdl)
{
	if (s_PeerPresent && s_Peer.Conn.Hdl == ConnHdl)
	{
		return &s_Peer;
	}

	return nullptr;
}

uint8_t BtPeerRole(uint16_t ConnHdl)
{
	(void)ConnHdl;

	return BT_CONN_ROLE_UNKNOWN;
}
