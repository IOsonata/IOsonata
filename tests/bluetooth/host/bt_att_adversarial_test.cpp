// Adversarial ATT tests.
//
// The functional ATT suite proves valid requests work. This suite drives
// malformed PDUs, resource failure, offset handling and command replacement
// rules. Every check describes behavior required at the ATT boundary, not the
// implementation path used to reach it.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bt_test_harness.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_peer.h"

namespace {

bttest::Context s_Test("ATT adversarial host tests");

constexpr uint16_t kConnHdl = 0x0042;
constexpr uint16_t kCharUuid = 0xFFF1;
constexpr uint8_t kInsufficientResources = 0x11;
constexpr uint8_t kCccdImproperCfg = 0xFD;

BtDevice_t s_Peer;
uint8_t s_LongWrite[128];
bool s_PeerEnabled = false;
bool s_CccdSetResult = true;
int s_CccdSetCount = 0;
// The real BtGattCccdValueError rejects reserved bits and a notify or indicate
// bit the characteristic does not grant. A stub fixed at 0 would leave the
// improper configuration error unreachable from the ATT server.
uint8_t s_CccdValueErr = 0;
bool s_SignatureValid = false;
int s_ConfirmCount = 0;

void PutLe16(uint8_t *pData, uint16_t Value)
{
	pData[0] = (uint8_t)Value;
	pData[1] = (uint8_t)(Value >> 8);
}

uint16_t GetLe16(const uint8_t *pData)
{
	return (uint16_t)(pData[0] | ((uint16_t)pData[1] << 8));
}

void ResetPeer()
{
	std::memset(&s_Peer, 0, sizeof(s_Peer));
	std::memset(s_LongWrite, 0, sizeof(s_LongWrite));
	s_Peer.Conn.Hdl = kConnHdl;
	s_Peer.Conn.pLongWrBuff = s_LongWrite;
	s_Peer.Conn.LongWrBuffSize = sizeof(s_LongWrite);
	s_Peer.Conn.LongWrLen = 0;
	s_PeerEnabled = true;
}

BtAttDBEntry_t *AddChar(BtGattChar_t *pChar, uint8_t *pValue,
						uint16_t ValueLen, uint16_t MaxLen,
						uint32_t Property)
{
	std::memset(pChar, 0, sizeof(*pChar));
	pChar->Uuid = kCharUuid;
	pChar->MaxDataLen = MaxLen;
	pChar->Property = Property;
	pChar->pValue = pValue;
	pChar->ValueLen = ValueLen;

	BtUuid16_t uuid = { 0, BT_UUID_TYPE_16, kCharUuid };
	BtAttDBEntry_t *pEntry = BtAttDBAddEntry(
			&uuid, (int)(sizeof(BtAttCharValue_t) + MaxLen));
	if (pEntry == nullptr)
	{
		return nullptr;
	}

	((BtAttCharValue_t *)pEntry->Data)->pChar = pChar;
	BtAttDBEntrySetPermission(pEntry,
			BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE);
	return pEntry;
}

BtAttDBEntry_t *AddCccd(BtGattChar_t *pChar)
{
	BtUuid16_t uuid = {
		0,
		BT_UUID_TYPE_16,
		BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION
	};
	BtAttDBEntry_t *pEntry = BtAttDBAddEntry(
			&uuid, (int)sizeof(BtDescClientCharConfig_t));
	if (pEntry == nullptr)
	{
		return nullptr;
	}

	BtDescClientCharConfig_t *pCccd =
			(BtDescClientCharConfig_t *)pEntry->Data;
	pCccd->pChar = pChar;
	pCccd->CccVal = 0;
	BtAttDBEntrySetPermission(pEntry,
			BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE);
	return pEntry;
}

void CheckInvalidPdu(const uint8_t *pRsp, uint32_t Len, uint8_t ReqOp)
{
	BT_CHECK(s_Test, Len == sizeof(BtAttErrorRsp_t) + 1);
	BT_CHECK(s_Test, pRsp[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	BT_CHECK(s_Test, pRsp[1] == ReqOp);
	BT_CHECK(s_Test, pRsp[4] == BT_ATT_ERROR_INVALID_PDU);
}

void TestFixedPduLengths()
{
	BtAttDBInit(2048);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	BtGattChar_t chr;
	uint8_t value[4] = { 1, 2, 3, 4 };
	BtAttDBEntry_t *pEntry = AddChar(&chr, value, 4, 4,
			BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE);
	BT_CHECK(s_Test, pEntry != nullptr);
	if (pEntry == nullptr)
	{
		return;
	}

	uint8_t req[32] = {};
	uint8_t rsp[32] = {};

	req[0] = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ;
	PutLe16(req + 1, BT_ATT_MTU_MIN);
	req[3] = 0xA5;
	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)req, 4,
			(BtAttReqRsp_t *)rsp);
	CheckInvalidPdu(rsp, n, BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ);

	std::memset(rsp, 0, sizeof(rsp));
	req[0] = BT_ATT_OPCODE_ATT_READ_REQ;
	PutLe16(req + 1, pEntry->Hdl);
	req[3] = 0x5A;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)req, 4,
			(BtAttReqRsp_t *)rsp);
	CheckInvalidPdu(rsp, n, BT_ATT_OPCODE_ATT_READ_REQ);

	std::memset(rsp, 0, sizeof(rsp));
	req[0] = BT_ATT_OPCODE_ATT_READ_BLOB_REQ;
	PutLe16(req + 1, pEntry->Hdl);
	PutLe16(req + 3, 0);
	req[5] = 0x5A;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)req, 6,
			(BtAttReqRsp_t *)rsp);
	CheckInvalidPdu(rsp, n, BT_ATT_OPCODE_ATT_READ_BLOB_REQ);

	ResetPeer();
	std::memset(rsp, 0, sizeof(rsp));
	req[0] = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	req[1] = 0;
	req[2] = 0x5A;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)req, 3,
			(BtAttReqRsp_t *)rsp);
	CheckInvalidPdu(rsp, n, BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ);
	s_PeerEnabled = false;

	s_ConfirmCount = 0;
	std::memset(rsp, 0xCC, sizeof(rsp));
	req[0] = BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM;
	req[1] = 0x5A;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)req, 2,
			(BtAttReqRsp_t *)rsp);
	BT_CHECK(s_Test, n == 0);
	BT_CHECK(s_Test, s_ConfirmCount == 0);
	BT_CHECK(s_Test, rsp[0] == 0xCC);
}

void TestMalformedCommandsAreSilent()
{
	uint8_t req[32] = {};
	uint8_t rsp[32] = {};

	for (int len = 1; len < 3; len++)
	{
		std::memset(rsp, 0xA5, sizeof(rsp));
		req[0] = BT_ATT_OPCODE_ATT_CMD;
		uint32_t n = BtAttProcessReq(kConnHdl,
				(BtAttReqRsp_t *)req, len, (BtAttReqRsp_t *)rsp);
		BT_CHECK(s_Test, n == 0);
		BT_CHECK(s_Test, rsp[0] == 0xA5);
	}

	for (int len = 1; len < 15; len++)
	{
		std::memset(rsp, 0x5A, sizeof(rsp));
		req[0] = BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD;
		uint32_t n = BtAttProcessReq(kConnHdl,
				(BtAttReqRsp_t *)req, len, (BtAttReqRsp_t *)rsp);
		BT_CHECK(s_Test, n == 0);
		BT_CHECK(s_Test, rsp[0] == 0x5A);
	}
}

void TestCccdFailurePropagation()
{
	BtAttDBInit(2048);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	BtGattChar_t chr = {};
	chr.Uuid = 0xFFF2;
	chr.Property = BT_GATT_CHAR_PROP_NOTIFY;
	BtAttDBEntry_t *pCccd = AddCccd(&chr);
	BT_CHECK(s_Test, pCccd != nullptr);
	if (pCccd == nullptr)
	{
		return;
	}

	uint8_t req[32] = {};
	uint8_t rsp[32] = {};
	req[0] = BT_ATT_OPCODE_ATT_WRITE_REQ;
	PutLe16(req + 1, pCccd->Hdl);
	PutLe16(req + 3, BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);

	s_CccdSetCount = 0;
	s_CccdSetResult = false;
	uint32_t n = BtAttProcessReq(kConnHdl,
			(BtAttReqRsp_t *)req, 5, (BtAttReqRsp_t *)rsp);
	BT_CHECK(s_Test, n == sizeof(BtAttErrorRsp_t) + 1);
	BT_CHECK(s_Test, rsp[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	BT_CHECK(s_Test, rsp[1] == BT_ATT_OPCODE_ATT_WRITE_REQ);
	BT_CHECK(s_Test, GetLe16(rsp + 2) == pCccd->Hdl);
	BT_CHECK(s_Test, rsp[4] == kInsufficientResources);
	BT_CHECK(s_Test, s_CccdSetCount == 1);

	ResetPeer();
	s_CccdSetCount = 0;
	std::memset(rsp, 0, sizeof(rsp));
	req[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	PutLe16(req + 1, pCccd->Hdl);
	PutLe16(req + 3, 0);
	PutLe16(req + 5, BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)req, 7,
			(BtAttReqRsp_t *)rsp);
	BT_CHECK(s_Test, n == 7);
	BT_CHECK(s_Test, rsp[0] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);

	std::memset(rsp, 0, sizeof(rsp));
	req[0] = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	req[1] = 1;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)req, 2,
			(BtAttReqRsp_t *)rsp);
	BT_CHECK(s_Test, n == sizeof(BtAttErrorRsp_t) + 1);
	BT_CHECK(s_Test, rsp[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	BT_CHECK(s_Test, rsp[1] == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ);
	BT_CHECK(s_Test, rsp[4] == kInsufficientResources);
	BT_CHECK(s_Test, s_CccdSetCount == 1);

	s_PeerEnabled = false;
	s_CccdSetResult = true;
}

// A CCCD value the characteristic does not grant is answered Client
// Characteristic Configuration Descriptor Improperly Configured, and the write
// never reaches the store. Both the direct write and the prepared write have
// to validate, since a prepared one is committed by an Execute Write that is
// not allowed to fail.
void TestCccdImproperConfigurationIsRejected()
{
	BtAttDBInit(2048);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	BtGattChar_t chr = {};
	chr.Uuid = 0xFFF3;
	chr.Property = BT_GATT_CHAR_PROP_NOTIFY;
	BtAttDBEntry_t *pCccd = AddCccd(&chr);
	BT_CHECK(s_Test, pCccd != nullptr);
	if (pCccd == nullptr)
	{
		return;
	}

	ResetPeer();
	s_CccdValueErr = kCccdImproperCfg;
	s_CccdSetCount = 0;

	uint8_t req[32] = {};
	uint8_t rsp[32] = {};
	req[0] = BT_ATT_OPCODE_ATT_WRITE_REQ;
	PutLe16(req + 1, pCccd->Hdl);
	PutLe16(req + 3, BT_DESC_CLIENT_CHAR_CONFIG_INDICATION);

	uint32_t n = BtAttProcessReq(kConnHdl,
			(BtAttReqRsp_t *)req, 5, (BtAttReqRsp_t *)rsp);
	BT_CHECK(s_Test, n == sizeof(BtAttErrorRsp_t) + 1);
	BT_CHECK(s_Test, rsp[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	BT_CHECK(s_Test, rsp[1] == BT_ATT_OPCODE_ATT_WRITE_REQ);
	BT_CHECK(s_Test, GetLe16(rsp + 2) == pCccd->Hdl);
	BT_CHECK(s_Test, rsp[4] == kCccdImproperCfg);
	// Rejected before the store, so nothing was written.
	BT_CHECK(s_Test, s_CccdSetCount == 0);

	// The prepared form refuses the value when it is queued, not when the queue
	// is executed. 3.4.6.3 has Execute Write commit what was already accepted,
	// so a value error the Prepare could see belongs to the Prepare. Capacity
	// is the opposite case and is checked at execute time, which is what
	// TestCccdFailurePropagation covers.
	ResetPeer();
	s_CccdSetCount = 0;
	std::memset(rsp, 0, sizeof(rsp));
	req[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	PutLe16(req + 1, pCccd->Hdl);
	PutLe16(req + 3, 0);
	PutLe16(req + 5, BT_DESC_CLIENT_CHAR_CONFIG_INDICATION);
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)req, 7,
			(BtAttReqRsp_t *)rsp);
	BT_CHECK(s_Test, n == sizeof(BtAttErrorRsp_t) + 1);
	BT_CHECK(s_Test, rsp[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	BT_CHECK(s_Test, rsp[1] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ);
	BT_CHECK(s_Test, GetLe16(rsp + 2) == pCccd->Hdl);
	BT_CHECK(s_Test, rsp[4] == kCccdImproperCfg);
	BT_CHECK(s_Test, s_CccdSetCount == 0);

	// Nothing was queued, so an Execute Write has nothing to commit and
	// succeeds rather than reporting the rejected value a second time.
	std::memset(rsp, 0, sizeof(rsp));
	req[0] = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	req[1] = 1;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)req, 2,
			(BtAttReqRsp_t *)rsp);
	BT_CHECK(s_Test, rsp[0] == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP);
	BT_CHECK(s_Test, s_CccdSetCount == 0);

	s_PeerEnabled = false;
	s_CccdValueErr = 0;
}

void TestReadBlobSlicesDeclaration()
{
	BtAttDBInit(1024);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	BtUuid16_t type = {
		0,
		BT_UUID_TYPE_16,
		BT_UUID_DECLARATIONS_PRIMARY_SERVICE
	};
	BtAttDBEntry_t *pEntry =
			BtAttDBAddEntry(&type, (int)sizeof(BtAttSrvcDeclar_t));
	BT_CHECK(s_Test, pEntry != nullptr);
	if (pEntry == nullptr)
	{
		return;
	}

	BtAttSrvcDeclar_t *pDecl = (BtAttSrvcDeclar_t *)pEntry->Data;
	std::memset(pDecl, 0, sizeof(*pDecl));
	pDecl->Uuid = { 0, BT_UUID_TYPE_16, { 0x1800 } };
	BtAttDBEntrySetPermission(pEntry, BT_ATT_PERMISSION_READ);

	uint8_t req[16] = {};
	uint8_t rsp[16] = {};
	req[0] = BT_ATT_OPCODE_ATT_READ_BLOB_REQ;
	PutLe16(req + 1, pEntry->Hdl);
	PutLe16(req + 3, 1);

	uint32_t n = BtAttProcessReq(kConnHdl,
			(BtAttReqRsp_t *)req, 5, (BtAttReqRsp_t *)rsp);
	BT_CHECK(s_Test, n == 2);
	BT_CHECK(s_Test, rsp[0] == BT_ATT_OPCODE_ATT_READ_BLOB_RSP);
	BT_CHECK(s_Test, rsp[1] == 0x18);

	PutLe16(req + 3, 3);
	n = BtAttProcessReq(kConnHdl,
			(BtAttReqRsp_t *)req, 5, (BtAttReqRsp_t *)rsp);
	BT_CHECK(s_Test, n == sizeof(BtAttErrorRsp_t) + 1);
	BT_CHECK(s_Test, rsp[0] == BT_ATT_OPCODE_ATT_ERROR_RSP);
	BT_CHECK(s_Test, rsp[4] == BT_ATT_ERROR_INVALID_OFFSET);
}

void TestReadMultipleVariableNeedsTwoHandles()
{
	BtAttDBInit(1024);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	BtGattChar_t chr;
	uint8_t value[2] = { 0xAA, 0xBB };
	BtAttDBEntry_t *pEntry = AddChar(&chr, value, 2, 2,
			BT_GATT_CHAR_PROP_READ);
	BT_CHECK(s_Test, pEntry != nullptr);
	if (pEntry == nullptr)
	{
		return;
	}

	uint8_t req[16] = {};
	uint8_t rsp[16] = {};
	req[0] = BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ;
	PutLe16(req + 1, pEntry->Hdl);

	uint32_t n = BtAttProcessReq(kConnHdl,
			(BtAttReqRsp_t *)req, 3, (BtAttReqRsp_t *)rsp);
	CheckInvalidPdu(rsp, n,
			BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ);
}

void BuildSignedWrite(uint8_t *pReq, uint16_t Hdl,
						  const uint8_t *pValue, uint16_t ValueLen)
{
	pReq[0] = BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD;
	PutLe16(pReq + 1, Hdl);
	if (ValueLen > 0)
	{
		std::memcpy(pReq + 3, pValue, ValueLen);
	}
	std::memset(pReq + 3 + ValueLen, 0xC3, 12);
}

void TestSignedWriteReplacementRules()
{
	BtAttDBInit(1024);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	BtGattChar_t chr;
	uint8_t stored[8] = { 1, 2, 3, 4, 0, 0, 0, 0 };
	BtAttDBEntry_t *pEntry = AddChar(&chr, stored, 4, 4,
			BT_GATT_CHAR_PROP_AUTH_SIGNED);
	BT_CHECK(s_Test, pEntry != nullptr);
	if (pEntry == nullptr)
	{
		return;
	}

	s_SignatureValid = true;
	uint8_t req[64] = {};
	uint8_t rsp[16] = {};

	uint8_t large[6] = { 9, 8, 7, 6, 5, 4 };
	BuildSignedWrite(req, pEntry->Hdl, large, sizeof(large));
	uint32_t n = BtAttProcessReq(kConnHdl,
			(BtAttReqRsp_t *)req, 3 + sizeof(large) + 12,
			(BtAttReqRsp_t *)rsp);
	BT_CHECK(s_Test, n == 0);
	BT_CHECK(s_Test, chr.ValueLen == 4);
	BT_CHECK(s_Test, stored[0] == 1 && stored[3] == 4);

	stored[0] = 1;
	stored[1] = 2;
	stored[2] = 3;
	stored[3] = 4;
	chr.ValueLen = 4;

	uint8_t shortValue[2] = { 0xA1, 0xA2 };
	BuildSignedWrite(req, pEntry->Hdl, shortValue, sizeof(shortValue));
	n = BtAttProcessReq(kConnHdl,
			(BtAttReqRsp_t *)req, 3 + sizeof(shortValue) + 12,
			(BtAttReqRsp_t *)rsp);
	BT_CHECK(s_Test, n == 0);
	BT_CHECK(s_Test, chr.ValueLen == 2);
	BT_CHECK(s_Test, stored[0] == 0xA1);
	BT_CHECK(s_Test, stored[1] == 0xA2);

	s_SignatureValid = false;
}

// A request longer than the link ATT_MTU must be refused before any of its
// parameters reach the response. Core Vol 3 Part F section 3.2.8 makes ATT_MTU
// the maximum size of any packet sent between a client and a server, and
// section 3.3 requires an invalid request - the example given is a PDU of the
// wrong length - to be answered with Invalid PDU (0x04) and Attribute Handle
// In Error 0x0000.
//
// Prepare Write is the request that makes an unchecked length reachable: it
// echoes the whole value back. The buffers here have the same geometry as
// BtHciProcessData, where the response is built in a BT_HCI_BUFFER_MAX_SIZE
// stack buffer behind an ACL header and an L2CAP header. Without the gate the
// echo runs off the end of that buffer and the sanitizer build reports a stack
// buffer overflow.
void TestOversizeRequestRejected()
{
	constexpr size_t kAttOffset =
			sizeof(BtHciACLDataPacketHdr_t) + sizeof(BtL2CapHdr_t);
	// Largest ATT PDU the reassembly path in BtHciProcessData will deliver.
	constexpr int kMaxReasmAtt =
			BT_HCI_BUFFER_MAX_SIZE - (int)sizeof(BtL2CapHdr_t);

	BtAttDBInit(2048);
	BtAttSetMtu(BT_ATT_MTU_MIN);

	BtGattChar_t chr;
	uint8_t value[8] = {};
	BtAttDBEntry_t *pEntry = AddChar(&chr, value, 0, sizeof(value),
			BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE);
	BT_CHECK(s_Test, pEntry != nullptr);
	if (pEntry == nullptr)
	{
		return;
	}

	// The shipped peripheral examples give one link a 512 byte long write
	// pool, so the queue has room for the oversize value and the request is
	// not turned away as Prepare Queue Full before the echo.
	uint8_t longWrite[512];
	ResetPeer();
	s_Peer.Conn.pLongWrBuff = longWrite;
	s_Peer.Conn.LongWrBuffSize = sizeof(longWrite);

	uint8_t reqBuf[BT_HCI_BUFFER_MAX_SIZE] = {};
	uint8_t rspBuf[BT_HCI_BUFFER_MAX_SIZE] = {};
	uint8_t *pReq = reqBuf + sizeof(BtL2CapHdr_t);
	BtAttReqRsp_t *pRsp = (BtAttReqRsp_t *)(rspBuf + kAttOffset);
	const uint8_t *pRspBytes = (const uint8_t *)pRsp;

	pReq[0] = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	PutLe16(pReq + 1, pEntry->Hdl);
	PutLe16(pReq + 3, 0);
	std::memset(pReq + 5, 0xA5, (size_t)kMaxReasmAtt - 5);

	uint32_t n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)pReq,
			kMaxReasmAtt, pRsp);
	CheckInvalidPdu(pRspBytes, n, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ);
	BT_CHECK(s_Test, s_Peer.Conn.LongWrLen == 0);

	// One octet over the MTU is refused.
	std::memset(rspBuf, 0, sizeof(rspBuf));
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)pReq,
			BT_ATT_MTU_MIN + 1, pRsp);
	CheckInvalidPdu(pRspBytes, n, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ);
	BT_CHECK(s_Test, s_Peer.Conn.LongWrLen == 0);

	// Exactly at the MTU is still accepted, so the gate does not cost the
	// peer the largest legal request.
	std::memset(rspBuf, 0, sizeof(rspBuf));
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)pReq,
			BT_ATT_MTU_MIN, pRsp);
	BT_CHECK(s_Test, n == (uint32_t)BT_ATT_MTU_MIN);
	BT_CHECK(s_Test, pRspBytes[0] == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);
	BT_CHECK(s_Test, s_Peer.Conn.LongWrLen == 6 + BT_ATT_MTU_MIN - 5);

	// A command has no response, so an oversize one is dropped in silence
	// rather than answered.
	std::memset(rspBuf, 0xC3, sizeof(rspBuf));
	pReq[0] = BT_ATT_OPCODE_ATT_CMD;
	n = BtAttProcessReq(kConnHdl, (BtAttReqRsp_t *)pReq, kMaxReasmAtt, pRsp);
	BT_CHECK(s_Test, n == 0);
	BT_CHECK(s_Test, pRspBytes[0] == 0xC3);

	s_PeerEnabled = false;
}

} // namespace

extern "C" {

BtDevice_t *BtPeerFindByHdl(uint16_t Hdl)
{
	if (!s_PeerEnabled || Hdl != kConnHdl)
	{
		return nullptr;
	}
	return &s_Peer;
}

bool BtGapConnSecGet(uint16_t, BtConnSec_t *)
{
	return false;
}

uint16_t BtGattCccdGet(uint16_t, uint16_t)
{
	return 0;
}

uint8_t BtGattCccdValueError(BtGattChar_t *, uint16_t)
{
	return s_CccdValueErr;
}

bool BtGattCccdSet(uint16_t, uint16_t, uint16_t)
{
	s_CccdSetCount++;
	return s_CccdSetResult;
}

void BtGattClientNotified(uint16_t, uint16_t, uint8_t *, uint16_t)
{
}

void BtGattHandleValueConfirm(uint16_t)
{
	s_ConfirmCount++;
}

bool BtSmpSignVerify(uint16_t, const uint8_t *, size_t, const uint8_t *)
{
	return s_SignatureValid;
}

} // extern "C"

int main()
{
	s_Test.Run("fixed PDU lengths", TestFixedPduLengths);
	s_Test.Run("malformed commands are silent", TestMalformedCommandsAreSilent);
	s_Test.Run("CCCD failure propagation", TestCccdFailurePropagation);
	s_Test.Run("CCCD improper configuration rejected",
			   TestCccdImproperConfigurationIsRejected);
	s_Test.Run("Read Blob declaration slicing", TestReadBlobSlicesDeclaration);
	s_Test.Run("Read Multiple Variable handle count",
			   TestReadMultipleVariableNeedsTwoHandles);
	s_Test.Run("Signed Write replacement rules",
			   TestSignedWriteReplacementRules);
	s_Test.Run("oversize request rejected", TestOversizeRequestRejected);
	return s_Test.Finish();
}
