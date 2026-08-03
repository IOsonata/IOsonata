// Client-side ATT transaction coverage.
//
// ATT permits one outstanding request per bearer. A normal response must match
// the expected response opcode and its complete structure; an Error Response
// must name the outstanding request opcode and contain all five bytes. The
// bearer remains unusable after the 30-second transaction timeout. Read By Type
// and Read By Group Type carry only 16- or 128-bit Attribute Types, so IOsonata
// 32-bit UUIDs are expanded to the Bluetooth base.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_att.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_uuid.h"

namespace {

constexpr uint16_t kConnHdl = 0x0042;

BtDevice_t s_Peer;
BtHciDevice_t s_Hci;
uint8_t s_LastAcl[BT_HCI_BUFFER_MAX_SIZE];
uint32_t s_Now;
int s_SendCount;
int s_ReserveCount;
int s_ReleaseCount;
bool s_ReserveOk;
bool s_SendOk;
int s_Checks;
int s_Failures;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

uint32_t DummySend(void *, uint32_t Len)
{
	return Len;
}

void Setup()
{
	std::memset(&s_Peer, 0, sizeof(s_Peer));
	std::memset(&s_Hci, 0, sizeof(s_Hci));
	std::memset(s_LastAcl, 0, sizeof(s_LastAcl));

	s_Peer.Conn.Hdl = kConnHdl;
	s_Peer.Conn.MaxMtu = 247;
	s_Peer.pHciDev = &s_Hci;
	s_Hci.SendData = DummySend;
	s_Hci.AclMaxLen = 251;

	s_Now = 100;
	s_SendCount = 0;
	s_ReserveCount = 0;
	s_ReleaseCount = 0;
	s_ReserveOk = true;
	s_SendOk = true;
}

BtL2CapPdu_t *LastPdu()
{
	return (BtL2CapPdu_t *)((BtHciACLDataPacket_t *)s_LastAcl)->Data;
}

void TestMatchingResponseOwnsBearer()
{
	Setup();
	uint8_t value = 0x5A;

	CHECK(BtAttWriteRequest(&s_Hci, kConnHdl, 0x0020, &value, 1));
	CHECK(s_Peer.bAttReqPending);
	CHECK(s_Peer.AttReqOpcode == BT_ATT_OPCODE_ATT_WRITE_REQ);
	CHECK(s_Peer.AttRspOpcode == BT_ATT_OPCODE_ATT_WRITE_RSP);
	CHECK(s_SendCount == 1);

	// A second request cannot pass the first one.
	CHECK(BtAttReadRequest(&s_Hci, kConnHdl, 0x0021) == false);
	CHECK(s_SendCount == 1);

	BtAttReqRsp_t rsp = {};
	rsp.OpCode = BT_ATT_OPCODE_ATT_READ_RSP;
	BtAttProcessRsp(kConnHdl, &rsp, 1);
	CHECK(s_Peer.bAttReqPending);

	// An Error Response for a different request also does not release it.
	std::memset(&rsp, 0, sizeof(rsp));
	rsp.OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
	rsp.ErrorRsp.ReqOpCode = BT_ATT_OPCODE_ATT_READ_REQ;
	rsp.ErrorRsp.Error = BT_ATT_ERROR_INVALID_HANDLE;
	BtAttProcessRsp(kConnHdl, &rsp, 5);
	CHECK(s_Peer.bAttReqPending);

	// The matching response releases the bearer before dispatch, allowing the
	// next request immediately.
	std::memset(&rsp, 0, sizeof(rsp));
	rsp.OpCode = BT_ATT_OPCODE_ATT_WRITE_RSP;
	BtAttProcessRsp(kConnHdl, &rsp, 1);
	CHECK(s_Peer.bAttReqPending == false);
	CHECK(s_Peer.AttReqOpcode == 0);
	CHECK(s_Peer.AttRspOpcode == 0);
	CHECK(BtAttReadRequest(&s_Hci, kConnHdl, 0x0021));
	CHECK(s_SendCount == 2);
}

void TestMatchingErrorResponseReleasesBearer()
{
	Setup();
	uint8_t value = 0xA5;
	CHECK(BtAttWriteRequest(&s_Hci, kConnHdl, 0x0020, &value, 1));

	BtAttReqRsp_t rsp = {};
	rsp.OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
	rsp.ErrorRsp.ReqOpCode = BT_ATT_OPCODE_ATT_WRITE_REQ;
	rsp.ErrorRsp.Hdl = 0x0020;
	rsp.ErrorRsp.Error = BT_ATT_ERROR_WRITE_NOT_PERMITTED;
	BtAttProcessRsp(kConnHdl, &rsp, 5);

	CHECK(s_Peer.bAttReqPending == false);
	CHECK(BtAttReadRequest(&s_Hci, kConnHdl, 0x0021));
}

void TestMalformedMatchedResponseKeepsBearer()
{
	Setup();
	CHECK(BtAttExchangeMtuRequest(&s_Hci, kConnHdl, 247));

	BtAttReqRsp_t rsp = {};
	rsp.OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP;

	// Matching opcode but no MTU field: do not release the transaction and do
	// not read outside the declared PDU.
	BtAttProcessRsp(kConnHdl, &rsp, 1);
	CHECK(s_Peer.bAttReqPending);
	CHECK(s_Peer.Conn.MaxMtu == 247);

	// The structure is present but the peer's Rx MTU is below the mandatory
	// ATT minimum. It is still not a completed transaction.
	rsp.ExchgMtuReqRsp.RxMtu = BT_ATT_MTU_MIN - 1;
	BtAttProcessRsp(kConnHdl, &rsp, 3);
	CHECK(s_Peer.bAttReqPending);
	CHECK(s_Peer.Conn.MaxMtu == 247);

	rsp.ExchgMtuReqRsp.RxMtu = BT_ATT_MTU_MIN;
	BtAttProcessRsp(kConnHdl, &rsp, 3);
	CHECK(s_Peer.bAttReqPending == false);
	CHECK(s_Peer.Conn.MaxMtu == BT_ATT_MTU_MIN);

	Setup();
	uint8_t value = 0x31;
	CHECK(BtAttWriteRequest(&s_Hci, kConnHdl, 0x0020, &value, 1));

	// A one-byte Error Response exposes only the opcode. Reading ReqOpCode
	// would be outside the PDU, and it cannot complete the outstanding write.
	std::memset(&rsp, 0, sizeof(rsp));
	rsp.OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
	BtAttProcessRsp(kConnHdl, &rsp, 1);
	CHECK(s_Peer.bAttReqPending);

	rsp.ErrorRsp.ReqOpCode = BT_ATT_OPCODE_ATT_WRITE_REQ;
	rsp.ErrorRsp.Hdl = 0x0020;
	rsp.ErrorRsp.Error = BT_ATT_ERROR_WRITE_NOT_PERMITTED;
	BtAttProcessRsp(kConnHdl, &rsp, 5);
	CHECK(s_Peer.bAttReqPending == false);

	Setup();
	BtUuid_t charDecl = {};
	charDecl.BaseIdx = 0;
	charDecl.Type = BT_UUID_TYPE_16;
	charDecl.Uuid16 = BT_UUID_DECLARATIONS_CHARACTERISTIC;
	s_Peer.Discovery.UuidType = charDecl;
	CHECK(BtAttReadByTypeRequest(&s_Hci, kConnHdl, 1, 0xFFFF, &charDecl));

	// Characteristic declarations are exactly seven or twenty-one bytes per
	// tuple. An eight-byte tuple previously entered the 128-bit path and read
	// beyond the element.
	std::memset(&rsp, 0, sizeof(rsp));
	rsp.OpCode = BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP;
	rsp.ReadByTypeRsp.Len = 8;
	BtAttProcessRsp(kConnHdl, &rsp, 10);
	CHECK(s_Peer.bAttReqPending);
	CHECK(s_Peer.NbSrvc == 0);

	// A full matching Error Response can terminate this request.
	std::memset(&rsp, 0, sizeof(rsp));
	rsp.OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
	rsp.ErrorRsp.ReqOpCode = BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ;
	rsp.ErrorRsp.Hdl = 1;
	rsp.ErrorRsp.Error = BT_ATT_ERROR_ATT_NOT_FOUND;
	BtAttProcessRsp(kConnHdl, &rsp, 5);
	CHECK(s_Peer.bAttReqPending == false);
}

void TestTransactionTimeoutLocksBearer()
{
	Setup();
	uint8_t value = 0x11;
	CHECK(BtAttWriteRequest(&s_Hci, kConnHdl, 0x0020, &value, 1));

	s_Now += 30000;
	CHECK(BtAttReadRequest(&s_Hci, kConnHdl, 0x0021) == false);
	CHECK(s_Peer.bAttReqPending == false);
	CHECK(s_Peer.bAttTimedOut);
	CHECK(BtAttReadRequest(&s_Hci, kConnHdl, 0x0021) == false);
	CHECK(s_SendCount == 1);
}

void TestReservationAndSendRollback()
{
	Setup();
	s_ReserveOk = false;
	CHECK(BtAttReadRequest(&s_Hci, kConnHdl, 0x0021) == false);
	CHECK(s_SendCount == 0);
	CHECK(s_Peer.bAttReqPending == false);

	Setup();
	s_SendOk = false;
	CHECK(BtAttReadRequest(&s_Hci, kConnHdl, 0x0021) == false);
	CHECK(s_ReserveCount == 1);
	CHECK(s_ReleaseCount == 1);
	CHECK(s_Peer.bAttReqPending == false);
}

void CheckUuid32Pdu(bool Group)
{
	Setup();
	BtUuid_t uuid = {};
	uuid.BaseIdx = 0;
	uuid.Type = BT_UUID_TYPE_32;
	uuid.Uuid32 = 0x11223344;

	bool ok = Group ?
		BtAttReadByGroupTypeRequest(&s_Hci, kConnHdl, 1, 0xFFFF, &uuid) :
		BtAttReadByTypeRequest(&s_Hci, kConnHdl, 1, 0xFFFF, &uuid);
	CHECK(ok);

	BtL2CapPdu_t *pdu = LastPdu();
	CHECK(pdu->Hdr.Len == 21);
	CHECK(pdu->Att.OpCode == (Group ? BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ :
		BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ));

	uint8_t expected[16];
	CHECK(BtUuidTo128(&uuid, expected));
	const uint8_t *actual = Group ?
		pdu->Att.ReadByGroupTypeReq.Uuid.Uuid128 :
		pdu->Att.ReadByTypeReq.Uuid.Uuid128;
	CHECK(std::memcmp(actual, expected, sizeof(expected)) == 0);
}

void TestUuid32ExpandsTo128()
{
	CheckUuid32Pdu(false);
	CheckUuid32Pdu(true);
}

} // namespace

extern "C" {

BtDevice_t *BtPeerFindByHdl(uint16_t Hdl)
{
	return Hdl == kConnHdl ? &s_Peer : nullptr;
}

bool BtGattTxPendReserve(uint16_t, BtGattChar_t *, uint16_t)
{
	++s_ReserveCount;
	return s_ReserveOk;
}

void BtGattTxPendRelease(uint16_t)
{
	++s_ReleaseCount;
}

uint32_t BtGattMsTick(void)
{
	return s_Now;
}

uint32_t BtHciSendAcl(BtHciDevice_t *, BtHciACLDataPacket_t *pAcl)
{
	if (!s_SendOk)
	{
		return 0;
	}

	uint32_t len = (uint32_t)sizeof(pAcl->Hdr) + pAcl->Hdr.Len;
	std::memcpy(s_LastAcl, pAcl, len);
	++s_SendCount;
	return len;
}

uint16_t BtAttGetMtu(void)
{
	return 247;
}

void BtDeviceDiscovered(BtDevice_t *) {}

} // extern "C"

int main()
{
	TestMatchingResponseOwnsBearer();
	TestMatchingErrorResponseReleasesBearer();
	TestMalformedMatchedResponseKeepsBearer();
	TestTransactionTimeoutLocksBearer();
	TestReservationAndSendRollback();
	TestUuid32ExpandsTo128();

	if (s_Failures != 0)
	{
		std::printf("ATT client transaction tests: %d failure(s), %d checks\n",
			s_Failures, s_Checks);
		return 1;
	}

	std::printf("ATT client transaction tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
