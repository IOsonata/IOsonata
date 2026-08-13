// Coverage for TX-complete attribution in src/bluetooth/bt_gatt.cpp.
//
// The controller reports completed HCI ACL packets, not GATT operations. The
// per-link ring holds every transmitted packet group in controller acceptance
// order. A null owner represents ATT, SMP, L2CAP, or indication traffic with no
// ACL-completion callback. Notification owners fire only after their entire
// packet group has completed; indications fire only after ATT confirmation.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_app.h"

// BtGattSrvcAdd resolves the device wide default security type from here
// when a service and its characteristics name none. The real definition
// lives in bt_app.cpp, which this test does not link.
BtAppData_t g_BtAppData;

namespace {

int s_Failures = 0;
int s_Checks = 0;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

constexpr uint16_t kConnHdl = 0x0021;

alignas(8) uint8_t s_PeerMem[BT_PEER_POOL_MEMSIZE(2)];
BtHciDevice_t s_HciDev;

int      s_TxCompleteCount = 0;
int      s_HciPacketCount = 0;
uint32_t s_SendResult = 1;

BtGattSrvc_t s_Srvc;
BtGattChar_t s_Char[2];
uint8_t s_Value[64];

uint32_t SendDataStub(void *, uint32_t Len) { return Len; }

void TxCompleteCB(BtGattChar_t *, int)
{
	++s_TxCompleteCount;
}

BtDevice_t *Peer()
{
	return BtPeerFindByHdl(kConnHdl);
}

// Bring up one link with an MTU large enough to send a value that has to be
// fragmented, and both characteristics subscribed for notification.
void Setup(uint16_t AclMaxLen)
{
	std::memset(&s_HciDev, 0, sizeof(s_HciDev));
	s_HciDev.AclMaxLen = AclMaxLen;
	s_HciDev.SendData = SendDataStub;

	BtPeerInit(s_PeerMem, sizeof(s_PeerMem));

	std::memset(s_Char, 0, sizeof(s_Char));

	// The service is registered once: BtGattCccdSet resolves a CCCD handle by
	// walking the service list, so the list has to hold this service, and the
	// list has no public reset.
	static bool s_SrvcListed = false;
	if (s_SrvcListed == false)
	{
		std::memset(&s_Srvc, 0, sizeof(s_Srvc));
		s_Srvc.NbChar = 2;
		s_Srvc.pCharArray = s_Char;
		BtGattInsertSrvcList(&s_Srvc);
		s_SrvcListed = true;
	}

	for (int i = 0; i < 2; i++)
	{
		s_Char[i].Property = BT_GATT_CHAR_PROP_NOTIFY |
			BT_GATT_CHAR_PROP_INDICATE;
		s_Char[i].ValHdl = (uint16_t)(0x0010 + i);
		s_Char[i].CccdHdl = (uint16_t)(0x0011 + i);
		s_Char[i].MaxDataLen = sizeof(s_Value);
		s_Char[i].pValue = s_Value;
		s_Char[i].pSrvc = &s_Srvc;
		s_Char[i].TxCompleteCB = TxCompleteCB;
	}

	BtDevice_t *pPeer = BtPeerAlloc(kConnHdl);
	if (pPeer != nullptr)
	{
		pPeer->pHciDev = &s_HciDev;
		pPeer->Conn.MaxMtu = 247;
	}

	BtGattCccdSet(kConnHdl, s_Char[0].CccdHdl, BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);
	BtGattCccdSet(kConnHdl, s_Char[1].CccdHdl, BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);

	s_TxCompleteCount = 0;
	s_HciPacketCount = 0;
	s_SendResult = 1;
	std::memset(s_Value, 0xA5, sizeof(s_Value));
}

// ---- a packet nobody owns does not fire a callback -------------------------

void TestUntrackedPacketDoesNotFire()
{
	Setup(64);

	// An ATT response goes out first: one packet, no owner. It occupies its
	// actual position in the ordered ring.
	CHECK(BtGattTxPendUntracked(kConnHdl, 1));
	CHECK(Peer()->TxPendCount == 1);
	CHECK(Peer()->TxPend[Peer()->TxPendHead].pChar == nullptr);

	CHECK(BtGattCharNotify(kConnHdl, &s_Char[0], s_Value, 8));
	CHECK(Peer()->TxPendCount == 2);

	// The controller reports the response. The notification is still in
	// flight, so nothing fires and its entry stays.
	BtGattSendCompleted(kConnHdl, 1);
	CHECK(s_TxCompleteCount == 0);
	CHECK(Peer()->TxPendCount == 1);

	// Now the notification completes.
	BtGattSendCompleted(kConnHdl, 1);
	CHECK(s_TxCompleteCount == 1);
	CHECK(Peer()->TxPendCount == 0);
}

// Separate unowned operations retain separate positions in send order.
void TestUntrackedGroupsRemainOrdered()
{
	Setup(64);

	CHECK(BtGattTxPendUntracked(kConnHdl, 1));
	CHECK(BtGattTxPendUntracked(kConnHdl, 1));
	CHECK(BtGattTxPendUntracked(kConnHdl, 1));
	CHECK(Peer()->TxPendCount == 3);

	CHECK(BtGattCharNotify(kConnHdl, &s_Char[0], s_Value, 8));
	CHECK(Peer()->TxPendCount == 4);

	BtGattSendCompleted(kConnHdl, 3);
	CHECK(s_TxCompleteCount == 0);
	CHECK(Peer()->TxPendCount == 1);
}

// ---- a fragmented notification fires once, at the end ----------------------

void TestFragmentedNotifyFiresOnce()
{
	// 20-octet ACL payload, so a 60-byte value plus headers spans 4 packets.
	Setup(20);

	CHECK(BtGattCharNotify(kConnHdl, &s_Char[0], s_Value, 60));
	CHECK(Peer()->TxPendCount == 1);

	// Fragments trickle in. None of them is the whole operation.
	BtGattSendCompleted(kConnHdl, 1);
	CHECK(s_TxCompleteCount == 0);
	BtGattSendCompleted(kConnHdl, 1);
	CHECK(s_TxCompleteCount == 0);
	BtGattSendCompleted(kConnHdl, 1);
	CHECK(s_TxCompleteCount == 0);

	// The last one finishes it.
	BtGattSendCompleted(kConnHdl, 1);
	CHECK(s_TxCompleteCount == 1);
	CHECK(Peer()->TxPendCount == 0);
}

// A single report covering every fragment finishes the operation exactly once.
void TestFragmentsReportedTogether()
{
	Setup(20);

	CHECK(BtGattCharNotify(kConnHdl, &s_Char[0], s_Value, 60));
	BtGattSendCompleted(kConnHdl, 4);
	CHECK(s_TxCompleteCount == 1);
	CHECK(Peer()->TxPendCount == 0);
}

// ---- an indication completes only after ATT confirmation -------------------

void TestIndicationWaitsForConfirmation()
{
	Setup(64);

	CHECK(BtGattCccdSet(kConnHdl, s_Char[0].CccdHdl,
		BT_DESC_CLIENT_CHAR_CONFIG_INDICATION));
	CHECK(BtGattCharIndicate(kConnHdl, &s_Char[0], s_Value, 8));
	CHECK(Peer()->Conn.bIndCfmPending);
	CHECK(Peer()->pIndChar == &s_Char[0]);
	CHECK(Peer()->TxPendCount == 1);
	CHECK(Peer()->TxPend[Peer()->TxPendHead].pChar == nullptr);
	CHECK(s_HciPacketCount == 1);

	// Only one indication may be outstanding on the ATT bearer.
	CHECK(BtGattCharIndicate(kConnHdl, &s_Char[1], s_Value, 8) == false);
	CHECK(Peer()->TxPendCount == 1);
	CHECK(s_HciPacketCount == 1);

	// Controller completion only releases the ACL credit. It must not report
	// the indication complete to the application.
	BtGattSendCompleted(kConnHdl, 1);
	CHECK(s_TxCompleteCount == 0);
	CHECK(Peer()->TxPendCount == 0);
	CHECK(Peer()->Conn.bIndCfmPending);
	CHECK(Peer()->pIndChar == &s_Char[0]);

	BtGattHandleValueConfirm(kConnHdl);
	CHECK(Peer()->Conn.bIndCfmPending == false);
	CHECK(Peer()->pIndChar == nullptr);
	CHECK(s_TxCompleteCount == 1);

	// A duplicate confirmation has no owner and does not fire twice.
	BtGattHandleValueConfirm(kConnHdl);
	CHECK(s_TxCompleteCount == 1);
}

// ---- the ring is reserved before the packet goes out -----------------------

void TestReservationRefusesWhenFull()
{
	Setup(64);

	// Fill the ring with entries nobody will complete.
	for (int i = 0; i < BT_DEV_TXPEND_MAX; i++)
	{
		CHECK(BtGattTxPendReserve(kConnHdl, &s_Char[0], 1));
	}
	CHECK(Peer()->TxPendCount == BT_DEV_TXPEND_MAX);
	CHECK(BtGattTxPendReserve(kConnHdl, &s_Char[0], 1) == false);
	CHECK(BtGattTxPendUntracked(kConnHdl, 1) == false);

	// A notification now has nowhere to record itself, so it is refused
	// rather than transmitted untracked.
	s_HciPacketCount = 0;
	CHECK(BtGattCharNotify(kConnHdl, &s_Char[1], s_Value, 8) == false);
	CHECK(s_HciPacketCount == 0);
	CHECK(Peer()->TxPendCount == BT_DEV_TXPEND_MAX);
}

// A transport that refuses the packet gives the reservation back, so a full
// ring is not left behind by failed sends.
void TestFailedSendReleasesReservation()
{
	Setup(64);

	s_SendResult = 0;					// transport refuses
	CHECK(BtGattCharNotify(kConnHdl, &s_Char[0], s_Value, 8) == false);
	CHECK(Peer()->TxPendCount == 0);

	s_SendResult = 1;
	CHECK(BtGattCharNotify(kConnHdl, &s_Char[0], s_Value, 8));
	CHECK(Peer()->TxPendCount == 1);
}

// ---- a characteristic with no callback still occupies the ring -------------

// It consumes a controller completion like any other traffic. Leaving it out
// walks the ring out of step with the link on its own.
void TestNoCallbackCharStillTracked()
{
	Setup(64);
	s_Char[0].TxCompleteCB = nullptr;

	CHECK(BtGattCharNotify(kConnHdl, &s_Char[0], s_Value, 8));
	CHECK(Peer()->TxPendCount == 1);

	CHECK(BtGattCharNotify(kConnHdl, &s_Char[1], s_Value, 8));
	CHECK(Peer()->TxPendCount == 2);

	// The first completion belongs to the callback-less char and fires
	// nothing; the second belongs to s_Char[1] and fires once.
	BtGattSendCompleted(kConnHdl, 1);
	CHECK(s_TxCompleteCount == 0);
	BtGattSendCompleted(kConnHdl, 1);
	CHECK(s_TxCompleteCount == 1);
}

// ---- ordering across mixed traffic -----------------------------------------

void TestSendOrderPreserved()
{
	Setup(64);

	CHECK(BtGattCharNotify(kConnHdl, &s_Char[0], s_Value, 8));
	CHECK(BtGattTxPendUntracked(kConnHdl, 2));
	CHECK(BtGattCharNotify(kConnHdl, &s_Char[1], s_Value, 8));
	CHECK(Peer()->TxPendCount == 3);

	// One report covering everything drains the groups in send order and
	// fires exactly the two notifications.
	BtGattSendCompleted(kConnHdl, 4);
	CHECK(s_TxCompleteCount == 2);
	CHECK(Peer()->TxPendCount == 0);

	// A report larger than what is outstanding does not underflow the ring.
	BtGattSendCompleted(kConnHdl, 5);
	CHECK(s_TxCompleteCount == 2);
	CHECK(Peer()->TxPendCount == 0);
}

} // namespace

// Stubs for the layers below bt_gatt.cpp.
extern "C" {

uint32_t BtHciSendAcl(BtHciDevice_t * const, BtHciACLDataPacket_t * const pAcl)
{
	if (s_SendResult == 0)
	{
		return 0;
	}

	++s_HciPacketCount;

	return (uint32_t)pAcl->Hdr.Len + sizeof(BtHciACLDataPacketHdr_t);
}

void BtGattSrvcEvtHandler(BtGattSrvc_t * const, uint32_t, void * const) {}
uint8_t BtSmpBondCccdGet(uint16_t, uint16_t *, uint16_t *, uint8_t) { return 0; }
void BtSmpBondCccdSave(uint16_t, uint16_t, uint16_t) {}

} // extern "C"

int main()
{
	TestUntrackedPacketDoesNotFire();
	TestUntrackedGroupsRemainOrdered();
	TestFragmentedNotifyFiresOnce();
	TestFragmentsReportedTogether();
	TestIndicationWaitsForConfirmation();
	TestReservationRefusesWhenFull();
	TestFailedSendReleasesReservation();
	TestNoCallbackCharStillTracked();
	TestSendOrderPreserved();

	if (s_Failures != 0)
	{
		std::printf("GATT TX-complete host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("GATT TX-complete host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
