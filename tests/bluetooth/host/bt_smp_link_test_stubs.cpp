// Transport capture for the SMP link tests.
//
// bt_smp.cpp carries a weak BtHciSendAcl of its own, so a test that includes
// that source cannot define a strong one in the same translation unit. This
// file supplies it from outside, where the strong symbol wins, and records
// what SMP put on the link. BtSmpBondAdd is weak in the same source for the
// same reason and is counted here too.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_smp.h"
#include "bt_smp_link_test_stubs.h"

SmpTxCapture g_SmpTx = {};

int g_SmpBondAddCount = 0;

void SmpBondAddReset(void)
{
	g_SmpBondAddCount = 0;
}

extern "C" void BtSmpBondAdd(uint16_t ConnHdl, const BtSmpKeys_t *pKeys)
{
	(void)ConnHdl;
	(void)pKeys;
	g_SmpBondAddCount++;
}

void SmpTxReset(void)
{
	std::memset(&g_SmpTx, 0, sizeof(g_SmpTx));
}

uint32_t BtHciSendAcl(BtHciDevice_t * const pDev, BtHciACLDataPacket_t * const pAcl)
{
	(void)pDev;

	if (pAcl == nullptr)
	{
		return 0;
	}

	const BtL2CapPdu_t *l2 = (const BtL2CapPdu_t*)pAcl->Data;
	if (l2->Hdr.Cid != BT_L2CAP_CID_SEC_MNGR || l2->Hdr.Len == 0)
	{
		return 0;
	}

	if (g_SmpTx.Count < SMP_TX_CAPTURE_MAX)
	{
		SmpTxPdu &pdu = g_SmpTx.Pdu[g_SmpTx.Count];
		size_t len = l2->Hdr.Len;
		if (len > sizeof(pdu.Data))
		{
			len = sizeof(pdu.Data);
		}
		pdu.ConnHdl = pAcl->Hdr.ConnHdl;
		pdu.Len = len;
		std::memcpy(pdu.Data, &l2->Smp, len);
	}

	g_SmpTx.Count++;
	return (uint32_t)pAcl->Hdr.Len + sizeof(pAcl->Hdr);
}
