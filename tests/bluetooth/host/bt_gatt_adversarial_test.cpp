// Adversarial GATT tests.
//
// These tests cross the GATT/ATT boundary and verify that initialization,
// subscription teardown and transmit completion bookkeeping preserve state
// under failure and mixed traffic.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bt_test_harness.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_peer.h"

namespace {

bttest::Context s_Test("GATT adversarial host tests");

constexpr uint16_t kConnHdl = 0x0044;

BtDevice_t s_Peer;
BtHciDevice_t s_Hci;
int s_HciPacketCount = 0;
int s_TxCompleteCount = 0;
int s_NotifyTransitions = 0;
int s_IndicateTransitions = 0;
bool s_LastNotifyState = false;
bool s_LastIndicateState = false;

BtGattChar_t s_InitialChar;
BtGattSrvc_t s_InitialService;
BtGattChar_t s_SubChar;
BtGattSrvc_t s_SubService;
uint8_t s_InitialData[4] = { 0x11, 0x22, 0x33, 0x44 };

uint32_t DummySendData(void *, uint32_t Len)
{
	return Len;
}

void TxComplete(BtGattChar_t *, int)
{
	s_TxCompleteCount++;
}

void NotifyChanged(BtGattChar_t *, bool Enabled, uint16_t)
{
	s_NotifyTransitions++;
	s_LastNotifyState = Enabled;
}

void IndicateChanged(BtGattChar_t *, bool Enabled, uint16_t)
{
	s_IndicateTransitions++;
	s_LastIndicateState = Enabled;
}

void ResetPeer()
{
	std::memset(&s_Peer, 0, sizeof(s_Peer));
	std::memset(&s_Hci, 0, sizeof(s_Hci));
	s_Peer.Conn.Hdl = kConnHdl;
	s_Peer.pHciDev = &s_Hci;
	s_Hci.SendData = DummySendData;
	s_HciPacketCount = 0;
	s_TxCompleteCount = 0;
	s_NotifyTransitions = 0;
	s_IndicateTransitions = 0;
	s_LastNotifyState = false;
	s_LastIndicateState = false;
}

void BuildInitialService()
{
	std::memset(&s_InitialChar, 0, sizeof(s_InitialChar));
	std::memset(&s_InitialService, 0, sizeof(s_InitialService));

	s_InitialChar.Uuid = 0xFFF1;
	s_InitialChar.MaxDataLen = sizeof(s_InitialData);
	s_InitialChar.Property = BT_GATT_CHAR_PROP_READ;
	s_InitialChar.pValue = s_InitialData;
	s_InitialChar.ValueLen = sizeof(s_InitialData);

	s_InitialService.UuidSrvc = 0xFFE0;
	s_InitialService.NbChar = 1;
	s_InitialService.pCharArray = &s_InitialChar;
}

void BuildSubscriptionService()
{
	std::memset(&s_SubChar, 0, sizeof(s_SubChar));
	std::memset(&s_SubService, 0, sizeof(s_SubService));

	s_SubChar.Uuid = 0xFFF2;
	s_SubChar.MaxDataLen = 8;
	s_SubChar.Property =
			BT_GATT_CHAR_PROP_READ |
			BT_GATT_CHAR_PROP_NOTIFY |
			BT_GATT_CHAR_PROP_INDICATE;
	s_SubChar.SetNotifCB = NotifyChanged;
	s_SubChar.SetIndCB = IndicateChanged;
	s_SubChar.TxCompleteCB = TxComplete;

	s_SubService.UuidSrvc = 0xFFE1;
	s_SubService.NbChar = 1;
	s_SubService.pCharArray = &s_SubChar;
}

void TestInitialValueIsCopied()
{
	BuildInitialService();
	BT_CHECK(s_Test, BtGattSrvcAdd(&s_InitialService));
	BT_CHECK(s_Test, s_InitialChar.pValue != nullptr);
	BT_CHECK(s_Test, s_InitialChar.pValue != s_InitialData);
	BT_CHECK(s_Test, s_InitialChar.ValueLen == sizeof(s_InitialData));
	BT_CHECK(s_Test,
			std::memcmp(s_InitialChar.pValue,
						s_InitialData, sizeof(s_InitialData)) == 0);
}

void TestDisconnectReportsSubscriptionDisable()
{
	BuildSubscriptionService();
	BT_CHECK(s_Test, BtGattSrvcAdd(&s_SubService));
	BT_CHECK(s_Test, s_SubChar.CccdHdl != BT_ATT_HANDLE_INVALID);

	s_NotifyTransitions = 0;
	s_IndicateTransitions = 0;
	BT_CHECK(s_Test, BtGattCccdSet(kConnHdl, s_SubChar.CccdHdl,
			BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION |
			BT_DESC_CLIENT_CHAR_CONFIG_INDICATION));
	BT_CHECK(s_Test, s_NotifyTransitions == 1);
	BT_CHECK(s_Test, s_IndicateTransitions == 1);
	BT_CHECK(s_Test, s_LastNotifyState);
	BT_CHECK(s_Test, s_LastIndicateState);

	s_NotifyTransitions = 0;
	s_IndicateTransitions = 0;
	BtGattCccdClear(kConnHdl);
	BT_CHECK(s_Test, s_Peer.Conn.NbCccd == 0);
	BT_CHECK(s_Test, s_NotifyTransitions == 1);
	BT_CHECK(s_Test, s_IndicateTransitions == 1);
	BT_CHECK(s_Test, !s_LastNotifyState);
	BT_CHECK(s_Test, !s_LastIndicateState);
}

void TestMixedPacketCompletionOrdering()
{
	s_Peer.TxPendHead = 0;
	s_Peer.TxPendCount = 0;
	s_TxCompleteCount = 0;

	// An ATT response goes out on this link. It owns no GATT completion, so
	// the layer that sent it reports the packet; bt_hci_host.cpp does this
	// for real, and the stub below only counts.
	BtHciACLDataPacket_t untracked = {};
	BT_CHECK(s_Test, BtHciSendAcl(&s_Hci, &untracked) != 0);
	BtGattTxPendUntracked(kConnHdl, 1);

	BtGattTxPendingAdd(kConnHdl, &s_SubChar);
	BT_CHECK(s_Test, s_Peer.TxPendCount == 2);

	BtGattSendCompleted(kConnHdl, 1);
	BT_CHECK(s_Test, s_TxCompleteCount == 0);
	BT_CHECK(s_Test, s_Peer.TxPendCount == 1);

	BtGattSendCompleted(kConnHdl, 1);
	BT_CHECK(s_Test, s_TxCompleteCount == 1);
	BT_CHECK(s_Test, s_Peer.TxPendCount == 0);
}

void TestFragmentCompletionWaitsForFinalPacket()
{
	s_Peer.TxPendHead = 0;
	s_Peer.TxPendCount = 0;
	s_TxCompleteCount = 0;

	// One notification that the transport split into three ACL packets. The
	// send path derives the count from the PDU size and the controller's ACL
	// length; here it is stated directly.
	BT_CHECK(s_Test, BtGattTxPendReserve(kConnHdl, &s_SubChar, 3));
	BT_CHECK(s_Test, s_Peer.TxPendCount == 1);

	BtGattSendCompleted(kConnHdl, 1);
	BT_CHECK(s_Test, s_TxCompleteCount == 0);
	BtGattSendCompleted(kConnHdl, 1);
	BT_CHECK(s_Test, s_TxCompleteCount == 0);
	BtGattSendCompleted(kConnHdl, 1);
	BT_CHECK(s_Test, s_TxCompleteCount == 1);
}

void TestFullCompletionQueueRejectsSend()
{
	s_Peer.Conn.NbCccd = 1;
	s_Peer.Conn.Cccd[0].Hdl = s_SubChar.CccdHdl;
	s_Peer.Conn.Cccd[0].Value =
			BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION;
	s_Peer.TxPendHead = 0;
	s_Peer.TxPendCount = BT_DEV_TXPEND_MAX;
	s_HciPacketCount = 0;

	uint8_t value[2] = { 0xAA, 0x55 };
	bool ok = BtGattCharNotify(kConnHdl, &s_SubChar,
			value, sizeof(value));

	BT_CHECK(s_Test, !ok);
	BT_CHECK(s_Test, s_HciPacketCount == 0);
	BT_CHECK(s_Test, s_Peer.TxPendCount == BT_DEV_TXPEND_MAX);
}

} // namespace

extern "C" {

BtDevice_t *BtPeerFindByHdl(uint16_t Hdl)
{
	return Hdl == kConnHdl ? &s_Peer : nullptr;
}

uint16_t BtPeerCount(void)
{
	return 1;
}

BtDevice_t *BtPeerSlot(uint16_t Idx)
{
	return Idx == 0 ? &s_Peer : nullptr;
}

size_t BtPeerGetConnectedHandles(uint16_t *pHdl, size_t MaxCount)
{
	if (pHdl != nullptr && MaxCount > 0)
	{
		pHdl[0] = kConnHdl;
		return 1;
	}
	return 0;
}

void BtSmpBondCccdSave(uint16_t, uint16_t, uint16_t)
{
}

uint8_t BtSmpBondCccdGet(uint16_t, uint16_t *, uint16_t *, uint8_t)
{
	return 0;
}

bool BtSmpSignVerify(uint16_t, const uint8_t *, size_t, const uint8_t *)
{
	return false;
}

bool BtGapConnSecGet(uint16_t, BtConnSec_t *)
{
	return false;
}

void BtGattSrvcEvtHandler(BtGattSrvc_t * const, uint32_t, void * const)
{
}

uint32_t BtHciSendAcl(BtHciDevice_t * const,
					  BtHciACLDataPacket_t * const pPacket)
{
	s_HciPacketCount++;
	return pPacket != nullptr ?
			(uint32_t)pPacket->Hdr.Len + sizeof(pPacket->Hdr) : 1;
}

void BtGattClientNotified(uint16_t, uint16_t, uint8_t *, uint16_t)
{
}

} // extern "C"

int main()
{
	ResetPeer();
	BtAttDBInit(4096);

	s_Test.Run("initial value copy", TestInitialValueIsCopied);
	s_Test.Run("subscription teardown callbacks",
			   TestDisconnectReportsSubscriptionDisable);
	s_Test.Run("mixed packet completion ordering",
			   TestMixedPacketCompletionOrdering);
	s_Test.Run("fragment completion ordering",
			   TestFragmentCompletionWaitsForFinalPacket);
	s_Test.Run("completion queue exhaustion",
			   TestFullCompletionQueueRejectsSend);
	return s_Test.Finish();
}
