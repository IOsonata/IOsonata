// ATT client-builder harness support.
//
// The dual-host test exercises server ATT processing through its pipe-based
// transport, but its client-builder checks only capture the generated ACL
// packet. Provide the production request path with one synthetic client peer,
// ordered transmit reservations and immediate harness completion after capture.

#include <cstddef>
#include <cstdint>

#include "bluetooth/bt_att.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_peer.h"

namespace {

constexpr uint16_t kBuilderConnHdl = 0x0123;
uint32_t s_Tick = 1;

void CompleteCapturedRequest(uint16_t ConnHdl)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr)
	{
		return;
	}

	if (pPeer->TxPendCount > 0)
	{
		pPeer->TxPendHead =
				(uint8_t)((pPeer->TxPendHead + 1U) % BT_DEV_TXPEND_MAX);
		pPeer->TxPendCount--;
	}

	// The packet-builder checks are independent requests. Treat the captured
	// packet as having received its matching response before the next check.
	pPeer->bAttReqPending = false;
	pPeer->AttReqOpcode = 0;
	pPeer->AttRspOpcode = 0;
	pPeer->AttReqTime = 0;
}

} // namespace

extern "C" {

uint32_t BtGattMsTick(void)
{
	return s_Tick++;
}

bool BtGattTxPendReserve(uint16_t ConnHdl, BtGattChar_t *pChar,
						uint16_t NbPkt)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || NbPkt == 0 ||
		pPeer->TxPendCount >= BT_DEV_TXPEND_MAX)
	{
		return false;
	}

	uint8_t idx = (uint8_t)((pPeer->TxPendHead + pPeer->TxPendCount) %
								 BT_DEV_TXPEND_MAX);
	pPeer->TxPend[idx].pChar = pChar;
	pPeer->TxPend[idx].Remain = NbPkt;
	pPeer->TxPendCount++;
	return true;
}

void BtGattTxPendRelease(uint16_t ConnHdl)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pPeer->TxPendCount == 0)
	{
		return;
	}

	uint8_t idx = (uint8_t)((pPeer->TxPendHead +
							pPeer->TxPendCount - 1U) % BT_DEV_TXPEND_MAX);
	pPeer->TxPend[idx].pChar = nullptr;
	pPeer->TxPend[idx].Remain = 0;
	pPeer->TxPendCount--;
}

uint32_t BtHciSendAcl(BtHciDevice_t * const pDev,
					  BtHciACLDataPacket_t * const pPacket)
{
	if (pDev == nullptr || pPacket == nullptr || pDev->SendData == nullptr)
	{
		return 0;
	}

	uint32_t len = (uint32_t)sizeof(pPacket->Hdr) + pPacket->Hdr.Len;
	uint32_t sent = pDev->SendData(pPacket, len);
	if (sent != 0)
	{
		CompleteCapturedRequest(pPacket->Hdr.ConnHdl);
	}
	return sent;
}

} // extern "C"

// bt_att_dual_host_test.cpp owns BtPeerFindByHdl and its server peer pool.
// Before running its original main, reuse one zero-initialized pool slot as the
// synthetic client connection used by the packet-builder checks. Each forked
// server calls ServerInit and replaces the pool with its real connection state.
extern int BtAttDualHostMain();

#ifdef main
#undef main
#endif

int main()
{
	BtDevice_t *pBuilderPeer = BtPeerFindByHdl(0);
	if (pBuilderPeer == nullptr)
	{
		return 2;
	}

	pBuilderPeer->Conn.Hdl = kBuilderConnHdl;
	pBuilderPeer->Conn.MaxMtu = BT_ATT_MTU_MIN;
	pBuilderPeer->pHciDev = nullptr;
	pBuilderPeer->TxPendHead = 0;
	pBuilderPeer->TxPendCount = 0;
	pBuilderPeer->bAttReqPending = false;
	pBuilderPeer->bAttTimedOut = false;

	return BtAttDualHostMain();
}
