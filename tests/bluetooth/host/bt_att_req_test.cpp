// Request-level coverage for BtAttProcessReq: the Read By Type response
// packing and sizing, invalid starting handle, the Write Request handle and
// length error codes. The database-layer test in bt_att_db_test.cpp does not
// drive BtAttProcessReq, so these paths were previously unverified.
//
// PDUs are built in plain byte buffers in on-air little-endian order and cast
// to BtAttReqRsp_t for the call, so the variable-length Data fields are never
// indexed past their declared [1] size.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_peer.h"

// Records CCCD writes reaching the GATT layer, so a test can assert that a
// malformed CCCD command never gets there.
int g_CccdSetCount = 0;
bool g_CccdCanStore = true;
uint16_t g_CccdSetVal = 0;
// Error the stubbed GATT value check reports. The real check lives in
// bt_gatt.cpp and has its own suite; here it is forced so the ATT layer's
// handling of a rejected value can be driven directly.
uint8_t g_CccdWriteErr = 0;

// Minimal externals BtAttProcessReq reaches for. This test drives the ATT
// server logic directly, so peers, CCCD state, security and signing are stubbed
// to a plain unencrypted single link with no subscriptions.
extern "C" {

// A single stub peer so the Prepare/Execute Write paths (which resolve the
// link through BtPeerFindByHdl) can queue and apply prepared writes. Enabled
// per test by g_PeerEnabled; the long-write buffer is a fixed scratch area.
static BtDevice_t s_StubPeer;
static uint8_t    s_StubLongWrBuff[512];
bool              g_PeerEnabled = false;

BtDevice_t *BtPeerFindByHdl(uint16_t Hdl)
{
	if (!g_PeerEnabled)
	{
		return nullptr;
	}
	s_StubPeer.Conn.Hdl            = Hdl;
	s_StubPeer.Conn.pLongWrBuff    = s_StubLongWrBuff;
	s_StubPeer.Conn.LongWrBuffSize = (uint16_t)sizeof(s_StubLongWrBuff);
	return &s_StubPeer;
}

bool BtGapConnSecGet(uint16_t, BtConnSec_t *) { return false; }

uint16_t BtGattCccdGet(uint16_t, uint16_t) { return 0; }
uint8_t BtGattCccdValueError(BtGattChar_t *, uint16_t) { return g_CccdWriteErr; }
bool BtGattCccdCanStore(uint16_t, uint16_t, uint16_t)
{
	return g_CccdCanStore;
}
bool BtGattCccdSet(uint16_t, uint16_t, uint16_t Value)
{
	++g_CccdSetCount;
	g_CccdSetVal = Value;
	return true;
}
void BtGattClientNotified(uint16_t, uint16_t, uint8_t *, uint16_t) {}
void BtGattHandleValueConfirm(uint16_t) {}

bool BtSmpSignVerify(uint16_t, const uint8_t *, size_t, const uint8_t *)
{
	return false;
}

} // extern "C"

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

constexpr uint16_t kConnHdl = 0x0001;
constexpr uint16_t kCharUuid = 0xFFF1;

// Response opcode + first scalar are at fixed offsets; the Read By Type data
// list begins after opcode(1) + length(1).
constexpr size_t kRbtDataOff = 2;
// Error Response layout: opcode(1) + reqopcode(1) + handle(2) + error(1).
constexpr size_t kErrReqOp = 1;
constexpr size_t kErrCode  = 4;

void PutLe16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)(v >> 8);
}

uint16_t GetLe16(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

// Add one characteristic-value attribute (type UUID kCharUuid) to the DB and
// wire it to a caller-owned BtChar_t so reads and writes see a real value.
BtAttDBEntry_t *AddCharValue(BtChar_t *pChar, uint8_t *pValue, uint16_t ValueLen,
							 uint16_t MaxLen, uint32_t Property)
{
	std::memset(pChar, 0, sizeof(*pChar));
	pChar->Uuid = kCharUuid;
	pChar->MaxDataLen = MaxLen;
	pChar->Property = Property;
	pChar->pValue = pValue;
	pChar->ValueLen = ValueLen;

	BtUuid16_t uuid = { 0, BT_UUID_TYPE_16, kCharUuid };
	BtAttDBEntry_t *entry =
		BtAttDBAddEntry(&uuid, (int)(sizeof(BtAttCharValue_t) + MaxLen));
	if (entry == nullptr)
	{
		return nullptr;
	}

	BtAttCharValue_t *cv = (BtAttCharValue_t *)entry->Data;
	cv->pChar = pChar;
	BtAttDBEntrySetPermission(entry,
							  BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE);
	return entry;
}

// opcode(1) + start(2) + end(2) + uuid16(2)
int BuildReadByType(uint8_t *pReq, uint16_t Start, uint16_t End, uint16_t Uuid16)
{
	pReq[0] = BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ;
	PutLe16(pReq + 1, Start);
	PutLe16(pReq + 3, End);
	PutLe16(pReq + 5, Uuid16);
	return 7;
}

// opcode(1) + handle(2) + value(Len)
int BuildWrite(uint8_t *pReq, uint16_t Hdl, const uint8_t *pVal, int Len)
{
	pReq[0] = BT_ATT_OPCODE_ATT_WRITE_REQ;
	PutLe16(pReq + 1, Hdl);
	if (Len > 0)
	{
		std::memcpy(pReq + 3, pVal, (size_t)Len);
	}
	return 3 + Len;
}

// ---- ATT_MTU negotiation --------------------------------------------------

// Core Vol 3 Part F 3.4.2.2: ATT_MTU is the minimum of the Client Rx MTU and
// the Server Rx MTU, and each of those has to be at least the default. Every
// port needs the same answer, and both Nordic ports read it out of the peer
// slot before sizing a notification, so the rule lives in one place.
void TestMtuNegotiationRule()
{
	CHECK(BtAttMtuNegotiated(100, BT_ATT_MTU_MAX) == 100);
	CHECK(BtAttMtuNegotiated(BT_ATT_MTU_MAX, 100) == 100);
	CHECK(BtAttMtuNegotiated(BT_ATT_MTU_MAX, BT_ATT_MTU_MAX) ==
		BT_ATT_MTU_MAX);
	CHECK(BtAttMtuNegotiated(BT_ATT_MTU_MIN, BT_ATT_MTU_MAX) ==
		BT_ATT_MTU_MIN);

	// A peer below the floor broke the same section. Its offer must not
	// shrink the link under what every ATT PDU may assume.
	CHECK(BtAttMtuNegotiated(BT_ATT_MTU_MIN - 1, BT_ATT_MTU_MAX) ==
		BT_ATT_MTU_MIN);
	CHECK(BtAttMtuNegotiated(0, BT_ATT_MTU_MAX) == BT_ATT_MTU_MIN);
	CHECK(BtAttMtuNegotiated(1, 1) == BT_ATT_MTU_MIN);
}

// The server side of the same exchange: the response names what this server
// can receive, and the link keeps the minimum.
void TestExchangeMtuRequestRecordsTheLink()
{
	BtAttDBInit(1024);
	BtAttSetMtu(BT_ATT_MTU_MAX);
	g_PeerEnabled = true;
	s_StubPeer.Conn.MaxMtu = 0;

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};
	BtAttReqRsp_t *req = (BtAttReqRsp_t *)reqbuf;
	BtAttReqRsp_t *rsp = (BtAttReqRsp_t *)rspbuf;

	req->OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ;
	req->ExchgMtuReqRsp.RxMtu = 100;
	uint32_t n = BtAttProcessReq(kConnHdl, req, 3, rsp);
	CHECK(n == 3);
	CHECK(rsp->OpCode == BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP);
	CHECK(rsp->ExchgMtuReqRsp.RxMtu == BT_ATT_MTU_MAX);
	CHECK(s_StubPeer.Conn.MaxMtu == 100);

	// An offer above what this server can receive is capped by it.
	s_StubPeer.Conn.MaxMtu = 0;
	req->ExchgMtuReqRsp.RxMtu = 512;
	CHECK(BtAttProcessReq(kConnHdl, req, 3, rsp) == 3);
	CHECK(s_StubPeer.Conn.MaxMtu == BT_ATT_MTU_MAX);

	// An offer below the default breaks 3.4.2.2 and this server answers it
	// with an error rather than negotiating, so the link keeps whatever it
	// had. The floor in the rule above is for the ports whose stack hands
	// over the peer's value unchecked.
	s_StubPeer.Conn.MaxMtu = 100;
	req->ExchgMtuReqRsp.RxMtu = 10;
	CHECK(BtAttProcessReq(kConnHdl, req, 3, rsp) == 5);
	CHECK(rsp->OpCode == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(s_StubPeer.Conn.MaxMtu == 100);

	s_StubPeer.Conn.MaxMtu = 0;
	g_PeerEnabled = false;
}

// ---- invalid handles ------------------------------------------------------

void TestInvalidStartHandleAndWriteHandle()
{
	BtAttDBInit(1024);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	BtChar_t chr;
	uint8_t val[8] = { 0xAA, 0xBB, 0xCC, 0xDD };
	CHECK(AddCharValue(&chr, val, 4, sizeof(val),
					   BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE) != nullptr);

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};
	BtAttReqRsp_t *req = (BtAttReqRsp_t *)reqbuf;
	BtAttReqRsp_t *rsp = (BtAttReqRsp_t *)rspbuf;

	// Read By Type with Starting Handle 0x0000 -> Invalid Handle (0x01).
	int len = BuildReadByType(reqbuf, 0x0000, 0xFFFF, kCharUuid);
	uint32_t n = BtAttProcessReq(kConnHdl, req, len, rsp);
	CHECK(n == 5);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(rspbuf[kErrReqOp] == BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ);
	CHECK(rspbuf[kErrCode] == BT_ATT_ERROR_INVALID_HANDLE);

	// Write Request to a handle that is not in the database -> Invalid Handle,
	// not Attribute Not Found.
	std::memset(rspbuf, 0, sizeof(rspbuf));
	uint8_t one = 0x01;
	len = BuildWrite(reqbuf, 0x7000, &one, 1);		// no such attribute
	n = BtAttProcessReq(kConnHdl, req, len, rsp);
	CHECK(n == 5);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(rspbuf[kErrReqOp] == BT_ATT_OPCODE_ATT_WRITE_REQ);
	CHECK(rspbuf[kErrCode] == BT_ATT_ERROR_INVALID_HANDLE);
}

// ---- Read By Type packs every pair that fits ------------------------------

void TestReadByTypePacksMultiplePairs()
{
	BtAttDBInit(2048);
	BtAttSetMtu(BT_ATT_MTU_MIN);		// 23

	// Three attributes of the same type, each a 4-byte value. A pair is
	// handle(2) + value(4) = 6 bytes; opcode + length + 3 pairs = 20 <= 23, so
	// all three must appear in one response.
	BtChar_t chr[3];
	uint8_t val[3][4] = { {1,1,1,1}, {2,2,2,2}, {3,3,3,3} };
	BtAttDBEntry_t *e[3];
	for (int i = 0; i < 3; ++i)
	{
		e[i] = AddCharValue(&chr[i], val[i], 4, 4, BT_GATT_CHAR_PROP_READ);
		CHECK(e[i] != nullptr);
	}

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};
	int len = BuildReadByType(reqbuf, 0x0001, 0xFFFF, kCharUuid);
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, len,
								 (BtAttReqRsp_t *)rspbuf);

	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP);
	CHECK(rspbuf[1] == 6);				// pair length = 2 handle + 4 value
	CHECK(n == 2 + 3 * 6);				// opcode + length + three pairs = 20

	// Verify the three handle/value pairs are present and in order.
	const uint8_t *p = rspbuf + kRbtDataOff;
	for (int i = 0; i < 3; ++i)
	{
		CHECK(GetLe16(p) == e[i]->Hdl);
		CHECK(std::memcmp(p + 2, val[i], 4) == 0);
		p += 6;
	}
}

// ---- Read By Type never truncates a longer value to the common length -----

void TestReadByTypeRejectsTruncatedPair()
{
	BtAttDBInit(2048);
	BtAttSetMtu(38);

	// Two 10-byte values then a 30-byte value, same type, at MTU 38. The first
	// two pairs (12 bytes each) are packed. For the third, a length-blind
	// implementation would cap the read at 38 - 24 - 4 = 10 bytes, so the
	// truncated 10-byte value would match the common pair length and be emitted
	// as if complete. The response must instead stop at the two full pairs and
	// leave the longer attribute for the client to fetch separately.
	BtChar_t chr0, chr1, chr2;
	uint8_t v0[10], v1[10], v2[30];
	for (int i = 0; i < 10; ++i) { v0[i] = (uint8_t)(0xA0 + i); v1[i] = (uint8_t)(0xC0 + i); }
	for (int i = 0; i < 30; ++i) v2[i] = (uint8_t)(0xB0 + i);

	BtAttDBEntry_t *e0 = AddCharValue(&chr0, v0, 10, 10, BT_GATT_CHAR_PROP_READ);
	BtAttDBEntry_t *e1 = AddCharValue(&chr1, v1, 10, 10, BT_GATT_CHAR_PROP_READ);
	BtAttDBEntry_t *e2 = AddCharValue(&chr2, v2, 30, 30, BT_GATT_CHAR_PROP_READ);
	CHECK(e0 != nullptr);
	CHECK(e1 != nullptr);
	CHECK(e2 != nullptr);

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};
	int len = BuildReadByType(reqbuf, 0x0001, 0xFFFF, kCharUuid);
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, len,
								 (BtAttReqRsp_t *)rspbuf);

	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP);
	CHECK(rspbuf[1] == 12);				// pair length = 2 handle + 10 value
	CHECK(n == 2 + 2 * 12);				// exactly the two full-length pairs
	CHECK(GetLe16(rspbuf + kRbtDataOff) == e0->Hdl);
	CHECK(std::memcmp(rspbuf + kRbtDataOff + 2, v0, 10) == 0);
	CHECK(GetLe16(rspbuf + kRbtDataOff + 12) == e1->Hdl);
	CHECK(std::memcmp(rspbuf + kRbtDataOff + 14, v1, 10) == 0);

	// The third attribute's handle must not appear anywhere in the response.
	bool thirdPresent = false;
	for (size_t off = kRbtDataOff; off + 1 < n; ++off)
	{
		if (GetLe16(rspbuf + off) == e2->Hdl)
		{
			thirdPresent = true;
		}
	}
	CHECK(!thirdPresent);
}

// ---- Read By Type with a 128-bit attribute type ---------------------------

void TestReadByType128Bit()
{
	BtAttDBInit(2048);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	// A characteristic value whose type is a vendor 128-bit UUID. Register its
	// base and build the packed DB type via the real UUID table. Bytes 12-13
	// (little-endian) are the 16-bit short id.
	uint8_t vendor128[16] = {
		0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
		0x93, 0xF3, 0xA3, 0xB5, 0x34, 0x12, 0x40, 0x6E };
	BtUuid16_t vtype;
	int idx = BtUuid128To16(&vtype, vendor128);
	CHECK(idx > 0);					// a custom base, not the SIG base

	BtChar_t chr;
	std::memset(&chr, 0, sizeof(chr));
	chr.Uuid = vtype.Uuid; chr.MaxDataLen = 4; chr.Property = BT_GATT_CHAR_PROP_READ;
	uint8_t val[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
	chr.pValue = val; chr.ValueLen = 4;

	BtAttDBEntry_t *e = BtAttDBAddEntry(&vtype, (int)(sizeof(BtAttCharValue_t) + 4));
	CHECK(e != nullptr);
	((BtAttCharValue_t *)e->Data)->pChar = &chr;
	BtAttDBEntrySetPermission(e, BT_ATT_PERMISSION_READ);

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};

	// Read By Type carrying the 128-bit type: op + start(2) + end(2) + uuid128(16).
	reqbuf[0] = BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ;
	PutLe16(reqbuf + 1, 0x0001);
	PutLe16(reqbuf + 3, 0xFFFF);
	std::memcpy(reqbuf + 5, vendor128, 16);
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 5 + 16,
								 (BtAttReqRsp_t *)rspbuf);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP);
	CHECK(rspbuf[1] == 6);						// pair = handle(2) + value(4)
	CHECK(n == 2 + 6);
	CHECK(GetLe16(rspbuf + 2) == e->Hdl);
	CHECK(std::memcmp(rspbuf + 4, val, 4) == 0);

	// A well-formed 128-bit type whose base is not registered matches nothing:
	// Attribute Not Found, no crash.
	uint8_t unknown128[16] = {
		0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
		0x09, 0x0A, 0x0B, 0x0C, 0x99, 0x88, 0x0D, 0x0E };
	std::memset(rspbuf, 0, sizeof(rspbuf));
	std::memcpy(reqbuf + 5, unknown128, 16);
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 5 + 16,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(rspbuf[kErrCode] == BT_ATT_ERROR_ATT_NOT_FOUND);

	// A malformed type length (neither 2 nor 16) is Invalid PDU.
	std::memset(rspbuf, 0, sizeof(rspbuf));
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 5 + 8,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(rspbuf[kErrCode] == BT_ATT_ERROR_INVALID_PDU);
}

// ---- Write Request length handling ----------------------------------------

void TestWriteLengthHandling()
{
	BtAttDBInit(1024);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	BtChar_t chr;
	uint8_t stored[8] = { 0x10, 0x11, 0x12, 0x13, 0, 0, 0, 0 };
	BtAttDBEntry_t *e =
		AddCharValue(&chr, stored, 4, 8, BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE);
	CHECK(e != nullptr);

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};

	// Oversized write (12 bytes into an 8-byte characteristic) must be rejected
	// with Invalid Attribute Value Length and must not modify the stored value.
	uint8_t big[12];
	for (int i = 0; i < 12; ++i) big[i] = (uint8_t)(0x50 + i);
	int len = BuildWrite(reqbuf, e->Hdl, big, 12);
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, len,
								 (BtAttReqRsp_t *)rspbuf);
	CHECK(n == 5);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(rspbuf[kErrCode] == BT_ATT_ERROR_INVALID_ATT_VALUE);
	CHECK(chr.ValueLen == 4);					// unchanged
	CHECK(stored[0] == 0x10 && stored[3] == 0x13);	// untouched

	// A write that fits (6 bytes) succeeds and grows the value to 6.
	std::memset(rspbuf, 0, sizeof(rspbuf));
	uint8_t six[6];
	for (int i = 0; i < 6; ++i) six[i] = (uint8_t)(0x60 + i);
	len = BuildWrite(reqbuf, e->Hdl, six, 6);
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, len,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 1);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_WRITE_RSP);
	CHECK(chr.ValueLen == 6);
	CHECK(stored[0] == 0x60 && stored[5] == 0x65);

	// A shorter write (4 bytes) replaces the value: the length must shrink to
	// 4, not stay at 6 with stale trailing bytes.
	std::memset(rspbuf, 0, sizeof(rspbuf));
	uint8_t four[4] = { 0x70, 0x71, 0x72, 0x73 };
	len = BuildWrite(reqbuf, e->Hdl, four, 4);
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, len,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 1);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_WRITE_RSP);
	CHECK(chr.ValueLen == 4);
	CHECK(stored[0] == 0x70 && stored[3] == 0x73);

	// A zero-length write is valid, acknowledged, and clears the value.
	std::memset(rspbuf, 0, sizeof(rspbuf));
	len = BuildWrite(reqbuf, e->Hdl, nullptr, 0);
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, len,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 1);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_WRITE_RSP);
	CHECK(chr.ValueLen == 0);
}

// ---- Write Command length handling ----------------------------------------

void TestWriteCommandLengthHandling()
{
	BtAttDBInit(1024);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	BtChar_t chr;
	uint8_t stored[8] = { 0x10, 0x11, 0x12, 0x13, 0, 0, 0, 0 };
	// Write Command requires the Write-Without-Response property.
	BtAttDBEntry_t *e =
		AddCharValue(&chr, stored, 4, 8, BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE_WORESP);
	CHECK(e != nullptr);

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};

	// An over-long Write Command (12 bytes into an 8-byte characteristic) must
	// be ignored entirely: no response, and the stored value and its length are
	// unchanged (Vol 3 Part F 3.4.5.3).
	reqbuf[0] = BT_ATT_OPCODE_ATT_CMD;
	PutLe16(reqbuf + 1, e->Hdl);
	for (int i = 0; i < 12; ++i) reqbuf[3 + i] = (uint8_t)(0x80 + i);
	std::memset(rspbuf, 0xEE, sizeof(rspbuf));		// sentinel: must stay untouched
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 3 + 12,
								 (BtAttReqRsp_t *)rspbuf);
	CHECK(n == 0);							// command produces no response
	CHECK(rspbuf[0] == 0xEE);				// response buffer not written
	CHECK(chr.ValueLen == 4);				// stored length unchanged
	CHECK(stored[0] == 0x10 && stored[3] == 0x13);	// stored data unchanged

	// A valid Write Command (6 bytes) replaces the value and length, no response.
	reqbuf[0] = BT_ATT_OPCODE_ATT_CMD;
	PutLe16(reqbuf + 1, e->Hdl);
	for (int i = 0; i < 6; ++i) reqbuf[3 + i] = (uint8_t)(0x90 + i);
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 3 + 6,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 0);
	CHECK(chr.ValueLen == 6);
	CHECK(stored[0] == 0x90 && stored[5] == 0x95);
}

// ---- CCCD Write Command: only an exactly-2-octet write reaches GATT --------

void TestWriteCommandCccdLength()
{
	BtAttDBInit(1024);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	g_CccdWriteErr = 0;

	// A notifiable characteristic and its CCCD descriptor.
	BtChar_t chr;
	std::memset(&chr, 0, sizeof(chr));
	chr.Uuid = 0xFFF7;
	chr.Property = BT_GATT_CHAR_PROP_NOTIFY;

	BtUuid16_t cccdUuid = { 0, BT_UUID_TYPE_16,
						   BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION };
	BtAttDBEntry_t *e =
		BtAttDBAddEntry(&cccdUuid, (int)sizeof(BtDescClientCharConfig_t));
	CHECK(e != nullptr);
	((BtDescClientCharConfig_t *)e->Data)->pChar = &chr;
	((BtDescClientCharConfig_t *)e->Data)->CccVal = 0;
	BtAttDBEntrySetPermission(e, BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE);

	uint8_t reqbuf[64] = {};

	uint8_t rspbuf[8] = {};

	// A zero-length CCCD command is malformed: it must not reach the GATT layer.
	g_CccdSetCount = 0;
	reqbuf[0] = BT_ATT_OPCODE_ATT_CMD;
	PutLe16(reqbuf + 1, e->Hdl);
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 3 + 0,
								 (BtAttReqRsp_t *)rspbuf);
	CHECK(n == 0);
	CHECK(g_CccdSetCount == 0);

	// A 1-byte CCCD command is malformed: it must not reach the GATT layer.
	g_CccdSetCount = 0;
	reqbuf[3] = 0x01;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 3 + 1,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 0);
	CHECK(g_CccdSetCount == 0);

	// A 3-byte CCCD command is malformed too.
	g_CccdSetCount = 0;
	reqbuf[3] = 0x01; reqbuf[4] = 0x00; reqbuf[5] = 0x00;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 3 + 3,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 0);
	CHECK(g_CccdSetCount == 0);

	// A valid 2-byte CCCD command (enable notifications) does reach GATT.
	g_CccdSetCount = 0;
	g_CccdSetVal = 0xFFFF;
	reqbuf[3] = 0x01; reqbuf[4] = 0x00;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 3 + 2,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 0);
	CHECK(g_CccdSetCount == 1);
	CHECK(g_CccdSetVal == BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);
}

// ---- CCCD value rejection is reported, not stored --------------------------

// A CCCD value the characteristic cannot support (a reserved bit, or a bit
// whose property is absent) must come back as an Error Response carrying the
// code the GATT layer chose, and must not reach BtGattCccdSet. The same value
// arriving as a Write Command is dropped, since a command has no error path.
void TestWriteRequestCccdValueRejected()
{
	BtAttDBInit(1024);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	BtChar_t chr;
	std::memset(&chr, 0, sizeof(chr));
	chr.Uuid = 0xFFF8;
	chr.Property = BT_GATT_CHAR_PROP_READ;

	BtUuid16_t cccdUuid = { 0, BT_UUID_TYPE_16,
						   BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION };
	BtAttDBEntry_t *e =
		BtAttDBAddEntry(&cccdUuid, (int)sizeof(BtDescClientCharConfig_t));
	CHECK(e != nullptr);
	if (e == nullptr)
	{
		return;
	}
	((BtDescClientCharConfig_t *)e->Data)->pChar = &chr;
	((BtDescClientCharConfig_t *)e->Data)->CccVal = 0;
	BtAttDBEntrySetPermission(e, BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE);

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[16] = {};

	// Write Request with a value the GATT layer rejects.
	g_CccdWriteErr = BT_ATT_ERROR_CCCD_IMPROPER_CFG;
	g_CccdSetCount = 0;
	reqbuf[0] = BT_ATT_OPCODE_ATT_WRITE_REQ;
	PutLe16(reqbuf + 1, e->Hdl);
	PutLe16(reqbuf + 3, BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);

	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 3 + 2,
								 (BtAttReqRsp_t *)rspbuf);
	CHECK(n == sizeof(BtAttErrorRsp_t) + 1);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(rspbuf[kErrReqOp] == BT_ATT_OPCODE_ATT_WRITE_REQ);
	CHECK(GetLe16(rspbuf + 2) == e->Hdl);
	CHECK(rspbuf[kErrCode] == BT_ATT_ERROR_CCCD_IMPROPER_CFG);
	CHECK(g_CccdSetCount == 0);

	// The same value as a Write Command is ignored without a response.
	g_CccdSetCount = 0;
	reqbuf[0] = BT_ATT_OPCODE_ATT_CMD;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 3 + 2,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 0);
	CHECK(g_CccdSetCount == 0);

	// With the value accepted, the same Write Request succeeds.
	g_CccdWriteErr = 0;
	g_CccdSetCount = 0;
	reqbuf[0] = BT_ATT_OPCODE_ATT_WRITE_REQ;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 3 + 2,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 1);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_WRITE_RSP);
	CHECK(g_CccdSetCount == 1);
	CHECK(g_CccdSetVal == BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);
}

// A CCCD reached through Prepare Write / Execute Write must still end up
// exactly two octets at offset 0. A single-octet prepared write slips past the
// permission check (which only inspects a value of two octets or more), so the
// execute step is the only place left to reject it. Before this check the
// execute step reported success while the write was quietly dropped.
void TestExecuteWriteCccdValueRejected()
{
	BtAttDBInit(1024);
	BtAttSetMtu(BT_ATT_MTU_MIN);
	g_PeerEnabled = true;
	g_CccdWriteErr = 0;

	BtChar_t chr;
	std::memset(&chr, 0, sizeof(chr));
	chr.Uuid = 0xFFF9;
	chr.Property = BT_GATT_CHAR_PROP_NOTIFY;

	BtUuid16_t cccdUuid = { 0, BT_UUID_TYPE_16,
						   BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION };
	BtAttDBEntry_t *e =
		BtAttDBAddEntry(&cccdUuid, (int)sizeof(BtDescClientCharConfig_t));
	CHECK(e != nullptr);
	if (e == nullptr)
	{
		g_PeerEnabled = false;
		return;
	}
	((BtDescClientCharConfig_t *)e->Data)->pChar = &chr;
	((BtDescClientCharConfig_t *)e->Data)->CccVal = 0;
	BtAttDBEntrySetPermission(e, BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE);

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[16] = {};

	// Prepare Write: opcode(1) handle(2) offset(2) value(1). One octet, so the
	// permission check leaves the value alone and the record is queued.
	g_CccdSetCount = 0;
	reqbuf[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	PutLe16(reqbuf + 1, e->Hdl);
	PutLe16(reqbuf + 3, 0);
	reqbuf[5] = BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION;

	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 6,
								 (BtAttReqRsp_t *)rspbuf);
	CHECK(n == 6);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);

	// Execute Write: opcode(1) flags(1), flags 1 = write.
	reqbuf[0] = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	reqbuf[1] = 1;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 2,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == sizeof(BtAttErrorRsp_t) + 1);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(rspbuf[kErrReqOp] == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ);
	CHECK(rspbuf[kErrCode] == BT_ATT_ERROR_INVALID_ATT_VALUE);
	CHECK(g_CccdSetCount == 0);

	// A well formed two-octet prepared write still goes through.
	g_CccdSetCount = 0;
	reqbuf[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	PutLe16(reqbuf + 1, e->Hdl);
	PutLe16(reqbuf + 3, 0);
	PutLe16(reqbuf + 5, BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 7,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 7);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);

	reqbuf[0] = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	reqbuf[1] = 1;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 2,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 1);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP);
	CHECK(g_CccdSetCount == 1);
	CHECK(g_CccdSetVal == BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);

	g_PeerEnabled = false;
}

// ---- Find By Type Value: permission gate and non-grouping end handle ------

void TestFindByTypeValueOracleAndEndHandle()
{
	BtAttDBInit(2048);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	// Two attributes of the same (non-grouping) type carrying the same value:
	// the first readable, the second read-encrypt-protected. On an unencrypted
	// link the protected one must not be matched (else its value is confirmed
	// by comparison), and a matched non-grouping attribute reports End Group
	// Handle equal to its own handle.
	const uint16_t type = 0xFFF5;
	uint8_t vOpen[2] = { 0xAA, 0xBB };
	uint8_t vProt[2] = { 0xAA, 0xBB };

	BtChar_t chrOpen, chrProt;
	// Reuse AddCharValue then override the type UUID and permission below.
	std::memset(&chrOpen, 0, sizeof(chrOpen));
	std::memset(&chrProt, 0, sizeof(chrProt));
	chrOpen.Uuid = type; chrOpen.MaxDataLen = 2; chrOpen.Property = BT_GATT_CHAR_PROP_READ;
	chrOpen.pValue = vOpen; chrOpen.ValueLen = 2;
	chrProt.Uuid = type; chrProt.MaxDataLen = 2; chrProt.Property = BT_GATT_CHAR_PROP_READ;
	chrProt.pValue = vProt; chrProt.ValueLen = 2;

	// Layout: eOpen (type), filler (other type), eProt (type). The filler
	// between the two matches means the grouping heuristic would give eOpen a
	// group end past its own handle, so the End-Group == found-handle assertion
	// below also isolates the non-grouping fix.
	BtChar_t chrFill;
	uint8_t vFill[2] = { 0, 0 };
	std::memset(&chrFill, 0, sizeof(chrFill));
	chrFill.Uuid = 0xFFF6; chrFill.MaxDataLen = 2; chrFill.Property = BT_GATT_CHAR_PROP_READ;
	chrFill.pValue = vFill; chrFill.ValueLen = 2;

	BtUuid16_t u = { 0, BT_UUID_TYPE_16, type };
	BtUuid16_t uFill = { 0, BT_UUID_TYPE_16, 0xFFF6 };
	BtAttDBEntry_t *eOpen = BtAttDBAddEntry(&u, (int)(sizeof(BtAttCharValue_t) + 2));
	BtAttDBEntry_t *eFill = BtAttDBAddEntry(&uFill, (int)(sizeof(BtAttCharValue_t) + 2));
	BtAttDBEntry_t *eProt = BtAttDBAddEntry(&u, (int)(sizeof(BtAttCharValue_t) + 2));
	CHECK(eOpen != nullptr);
	CHECK(eFill != nullptr);
	CHECK(eProt != nullptr);
	((BtAttCharValue_t *)eOpen->Data)->pChar = &chrOpen;
	((BtAttCharValue_t *)eFill->Data)->pChar = &chrFill;
	((BtAttCharValue_t *)eProt->Data)->pChar = &chrProt;
	BtAttDBEntrySetPermission(eOpen, BT_ATT_PERMISSION_READ);
	BtAttDBEntrySetPermission(eFill, BT_ATT_PERMISSION_READ);
	BtAttDBEntrySetPermission(eProt, BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_READ_ENCRYPT);

	// opcode(1) + start(2) + end(2) + type(2) + value(2)
	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};
	reqbuf[0] = BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ;
	PutLe16(reqbuf + 1, 0x0001);
	PutLe16(reqbuf + 3, 0xFFFF);
	PutLe16(reqbuf + 5, type);
	reqbuf[7] = 0xAA;
	reqbuf[8] = 0xBB;
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 9,
								 (BtAttReqRsp_t *)rspbuf);

	// Only the open attribute matches: one 4-byte handle range.
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP);
	CHECK(n == 1 + 4);
	uint16_t foundStart = GetLe16(rspbuf + 1);
	uint16_t foundEnd = GetLe16(rspbuf + 3);
	CHECK(foundStart == eOpen->Hdl);
	CHECK(foundEnd == eOpen->Hdl);			// non-grouping: end == found handle

	// The protected attribute's handle must not appear in the response.
	bool protPresent = false;
	for (size_t off = 1; off + 1 < n; off += 2)
	{
		if (GetLe16(rspbuf + off) == eProt->Hdl)
		{
			protPresent = true;
		}
	}
	CHECK(!protPresent);
}

// ---- Find By Type Value: chunked value comparison (no full-MTU scratch) ----

void TestFindByTypeValueChunkedCompare()
{
	BtAttDBInit(2048);
	BtAttSetMtu(BT_ATT_MTU_MAX);		// room for a 100-byte value in the request

	uint8_t val[100];
	for (int i = 0; i < 100; ++i) val[i] = (uint8_t)(i * 7 + 3);

	BtChar_t chr;
	BtAttDBEntry_t *e = AddCharValue(&chr, val, 100, 100, BT_GATT_CHAR_PROP_READ);
	CHECK(e != nullptr);

	uint8_t reqbuf[128] = {};
	uint8_t rspbuf[64] = {};

	// Exact 100-byte value matches - exercises multi-chunk comparison (64 + 36).
	reqbuf[0] = BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ;
	PutLe16(reqbuf + 1, 0x0001);
	PutLe16(reqbuf + 3, 0xFFFF);
	PutLe16(reqbuf + 5, kCharUuid);
	std::memcpy(reqbuf + 7, val, 100);
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 7 + 100,
								 (BtAttReqRsp_t *)rspbuf);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP);
	CHECK(n == 1 + 4);
	CHECK(GetLe16(rspbuf + 1) == e->Hdl);

	// A value differing only in the last chunk (byte 90) must not match.
	uint8_t bad[100];
	std::memcpy(bad, val, 100);
	bad[90] = (uint8_t)(bad[90] ^ 0xFF);
	std::memcpy(reqbuf + 7, bad, 100);
	std::memset(rspbuf, 0, sizeof(rspbuf));
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 7 + 100,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(rspbuf[kErrCode] == BT_ATT_ERROR_ATT_NOT_FOUND);

	// A shorter prefix (different complete length) must not match either.
	std::memcpy(reqbuf + 7, val, 50);
	std::memset(rspbuf, 0, sizeof(rspbuf));
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 7 + 50,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
}

// ---- Read Multiple concatenates values and sets the response opcode -------

void TestReadMultiple()
{
	BtAttDBInit(2048);
	BtAttSetMtu(BT_ATT_MTU_MIN);		// 23: the case the old guard broke

	BtChar_t chr[3];
	uint8_t val[3][4] = { {1,1,1,1}, {2,2,2,2}, {3,3,3,3} };
	BtAttDBEntry_t *e[3];
	for (int i = 0; i < 3; ++i)
	{
		e[i] = AddCharValue(&chr[i], val[i], 4, 4, BT_GATT_CHAR_PROP_READ);
		CHECK(e[i] != nullptr);
	}

	// opcode(1) + three handles(2 each)
	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};
	reqbuf[0] = BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ;
	PutLe16(reqbuf + 1, e[0]->Hdl);
	PutLe16(reqbuf + 3, e[1]->Hdl);
	PutLe16(reqbuf + 5, e[2]->Hdl);
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 1 + 3 * 2,
								 (BtAttReqRsp_t *)rspbuf);

	// Response is opcode + the three 4-byte values concatenated.
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_READ_MULTIPLE_RSP);
	CHECK(n == 1 + 3 * 4);
	CHECK(std::memcmp(rspbuf + 1, val[0], 4) == 0);
	CHECK(std::memcmp(rspbuf + 1 + 4, val[1], 4) == 0);
	CHECK(std::memcmp(rspbuf + 1 + 8, val[2], 4) == 0);
}

// ---- Read By Group Type rejects starting handle 0x0000 --------------------

void TestReadByGroupTypeInvalidStartHandle()
{
	BtAttDBInit(1024);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};

	// opcode(1) + start(2) + end(2) + uuid16(2), start = 0x0000 -> Invalid Handle.
	reqbuf[0] = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ;
	PutLe16(reqbuf + 1, 0x0000);
	PutLe16(reqbuf + 3, 0xFFFF);
	PutLe16(reqbuf + 5, 0x2800);		// Primary Service group type
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 7,
								 (BtAttReqRsp_t *)rspbuf);
	CHECK(n == 5);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(rspbuf[kErrReqOp] == BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ);
	CHECK(rspbuf[kErrCode] == BT_ATT_ERROR_INVALID_HANDLE);
}

// ---- Execute Write applies non-contiguous prepared writes -----------------

// A Reliable Write may patch disjoint regions of one attribute. The queued
// chunks are applied in order at their own offsets with no contiguity
// requirement (Vol 3 Part F 3.4.6); the old code rejected a gap with Invalid
// Offset.
void TestExecuteWriteNonContiguous()
{
	BtAttDBInit(2048);
	BtAttSetMtu(247);

	BtChar_t chr;
	uint8_t val[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	BtAttDBEntry_t *e = AddCharValue(&chr, val, sizeof(val), sizeof(val),
									 BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE);
	CHECK(e != nullptr);

	g_PeerEnabled = true;
	s_StubPeer.Conn.LongWrLen = 0;

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};

	// Prepare Write at offset 0 = {0xAA, 0xBB}.
	reqbuf[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	PutLe16(reqbuf + 1, e->Hdl);
	PutLe16(reqbuf + 3, 0);
	reqbuf[5] = 0xAA;
	reqbuf[6] = 0xBB;
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 5 + 2,
								 (BtAttReqRsp_t *)rspbuf);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);
	(void)n;

	// Prepare Write at offset 6 = {0xCC, 0xDD}: a gap at bytes 2..5.
	std::memset(rspbuf, 0, sizeof(rspbuf));
	reqbuf[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	PutLe16(reqbuf + 1, e->Hdl);
	PutLe16(reqbuf + 3, 6);
	reqbuf[5] = 0xCC;
	reqbuf[6] = 0xDD;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 5 + 2,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);

	// Execute Write (flag 0x01): both chunks apply, no Invalid Offset error.
	std::memset(rspbuf, 0, sizeof(rspbuf));
	reqbuf[0] = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	reqbuf[1] = 0x01;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 2,
						(BtAttReqRsp_t *)rspbuf);
	CHECK(n == 1);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP);

	// Both disjoint regions were written; the gap is untouched.
	CHECK(val[0] == 0xAA);
	CHECK(val[1] == 0xBB);
	CHECK(val[2] == 0x00);
	CHECK(val[5] == 0x00);
	CHECK(val[6] == 0xCC);
	CHECK(val[7] == 0xDD);

	g_PeerEnabled = false;
}

// ---- Read Multiple Variable reports the full (untruncated) value length ----

// The Length field of each tuple is the complete attribute value length, even
// when the last value is truncated to fit the MTU (Vol 3 Part F 3.4.4.13). The
// old code reported the emitted (truncated) count, hiding the truncation.
void TestReadMultipleVariableFullLength()
{
	BtAttDBInit(2048);
	BtAttSetMtu(BT_ATT_MTU_MIN);		// 23

	BtChar_t chrA;
	uint8_t valA[4] = { 0x11, 0x22, 0x33, 0x44 };
	BtAttDBEntry_t *eA = AddCharValue(&chrA, valA, 4, 4, BT_GATT_CHAR_PROP_READ);
	CHECK(eA != nullptr);

	BtChar_t chrB;
	uint8_t valB[20];
	for (int i = 0; i < 20; ++i) valB[i] = (uint8_t)(0xA0 + i);
	BtAttDBEntry_t *eB = AddCharValue(&chrB, valB, 20, 20, BT_GATT_CHAR_PROP_READ);
	CHECK(eB != nullptr);

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};
	reqbuf[0] = BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ;
	PutLe16(reqbuf + 1, eA->Hdl);
	PutLe16(reqbuf + 3, eB->Hdl);
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 1 + 2 * 2,
								 (BtAttReqRsp_t *)rspbuf);

	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP);
	// Tuple A: full length 4, value present in full.
	CHECK(GetLe16(rspbuf + 1) == 4);
	CHECK(std::memcmp(rspbuf + 3, valA, 4) == 0);
	// Tuple B: Length reports the full 20 even though only part of the value
	// fits in the 23-octet MTU.
	CHECK(GetLe16(rspbuf + 7) == 20);
	// The response fills the MTU: opcode + (2+4) + (2 + 14) = 23.
	CHECK(n == 23);
}

// Execute Write is atomic (Vol 3 Part F 3.4.6): if any queued record fails
// validation, no attribute may have been modified. Queue a valid write to a
// good handle, then an invalid one, and check the first value is untouched.
void TestExecuteWriteAtomicFailure()
{
	BtAttDBInit(2048);
	BtAttSetMtu(247);

	BtChar_t chr;
	uint8_t val[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	BtAttDBEntry_t *e = AddCharValue(&chr, val, sizeof(val), sizeof(val),
									 BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE);
	CHECK(e != nullptr);

	g_PeerEnabled = true;
	s_StubPeer.Conn.LongWrLen = 0;

	uint8_t reqbuf[64] = {};
	uint8_t rspbuf[64] = {};

	// Record 1, valid: write {0xAA, 0xBB} at offset 0 of the real handle.
	reqbuf[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	PutLe16(reqbuf + 1, e->Hdl);
	PutLe16(reqbuf + 3, 0);
	reqbuf[5] = 0xAA;
	reqbuf[6] = 0xBB;
	BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 5 + 2,
					(BtAttReqRsp_t *)rspbuf);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);

	// Record 2, invalid: offset past the end of the attribute. Queue it
	// directly, since Prepare Write validates the handle but not the offset.
	uint8_t *q = s_StubPeer.Conn.pLongWrBuff + s_StubPeer.Conn.LongWrLen;
	uint16_t badOff = (uint16_t)(sizeof(val) + 4);
	uint16_t vlen = 1;
	PutLe16(q, e->Hdl);
	PutLe16(q + 2, badOff);
	PutLe16(q + 4, vlen);
	q[6] = 0xCC;
	s_StubPeer.Conn.LongWrLen = (uint16_t)(s_StubPeer.Conn.LongWrLen + 6 + vlen);

	// Execute: must fail with Invalid Offset and leave the value untouched.
	std::memset(rspbuf, 0, sizeof(rspbuf));
	reqbuf[0] = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	reqbuf[1] = 0x01;
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)reqbuf, 2,
								 (BtAttReqRsp_t *)rspbuf);

	CHECK(n == 5);
	CHECK(rspbuf[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(rspbuf[kErrReqOp] == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ);
	CHECK(rspbuf[kErrCode] == BT_ATT_ERROR_INVALID_OFFSET);

	// The valid record must NOT have been applied.
	CHECK(val[0] == 0x00);
	CHECK(val[1] == 0x00);

	g_PeerEnabled = false;
}

} // namespace

int main()
{
	TestMtuNegotiationRule();
	TestExchangeMtuRequestRecordsTheLink();
	TestInvalidStartHandleAndWriteHandle();
	TestReadByTypePacksMultiplePairs();
	TestReadByTypeRejectsTruncatedPair();
	TestReadByType128Bit();
	TestWriteLengthHandling();
	TestWriteCommandLengthHandling();
	TestWriteCommandCccdLength();
	TestWriteRequestCccdValueRejected();
	TestExecuteWriteCccdValueRejected();
	TestFindByTypeValueOracleAndEndHandle();
	TestFindByTypeValueChunkedCompare();
	TestReadMultiple();
	TestReadByGroupTypeInvalidStartHandle();
	TestExecuteWriteNonContiguous();
	TestExecuteWriteAtomicFailure();
	TestReadMultipleVariableFullLength();

	if (s_Failures != 0)
	{
		std::printf("ATT request host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("ATT request host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
