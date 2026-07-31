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

// Minimal externals BtAttProcessReq reaches for. This test drives the ATT
// server logic directly, so peers, CCCD state, security and signing are stubbed
// to a plain unencrypted single link with no subscriptions.
extern "C" {

BtDevice_t *BtPeerFindByHdl(uint16_t) { return nullptr; }

bool BtGapConnSecGet(uint16_t, BtConnSec_t *) { return false; }

uint16_t BtGattCccdGet(uint16_t, uint16_t) { return 0; }
bool BtGattCccdSet(uint16_t, uint16_t, uint16_t) { return true; }
void BtGattClientNotified(uint16_t, uint16_t, uint8_t *, uint16_t) {}
void BtGattHandleValueConfirm(uint16_t) {}

bool BtUuidGetBase(int, uint8_t *) { return false; }

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

} // namespace

int main()
{
	TestInvalidStartHandleAndWriteHandle();
	TestReadByTypePacksMultiplePairs();
	TestReadByTypeRejectsTruncatedPair();
	TestWriteLengthHandling();
	TestWriteCommandLengthHandling();
	TestFindByTypeValueOracleAndEndHandle();
	TestReadByGroupTypeInvalidStartHandle();

	if (s_Failures != 0)
	{
		std::printf("ATT request host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("ATT request host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
