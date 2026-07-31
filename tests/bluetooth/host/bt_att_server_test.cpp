#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_att.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_smp.h"

namespace {

constexpr uint16_t kConnHdl = 0x0042;
constexpr size_t kPacketSize = BT_ATT_MTU_MAX + 32U;

int s_Failures = 0;
int s_Checks = 0;

#define CHECK(expr) do { \
	++s_Checks; \
	if (!(expr)) { \
		std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
		++s_Failures; \
	} \
} while (0)

BtDevice_t s_Peer = {};
BtConnSec_t s_Security = {};
uint8_t s_LongWrite[96] = {};

uint16_t s_CccdHdl = 0;
uint16_t s_CccdValue = 0;
int s_CccdSetCount = 0;

int s_WriteCount = 0;
int s_WriteOffset = -1;
int s_WriteLen = -1;
uint8_t s_WriteData[32] = {};

int s_NotifyCount = 0;
uint16_t s_NotifyConnHdl = 0;
uint16_t s_NotifyValHdl = 0;
uint16_t s_NotifyLen = 0;
uint8_t s_NotifyData[32] = {};

int s_ConfirmCount = 0;
uint16_t s_ConfirmConnHdl = 0;

alignas(4) uint8_t s_ReqMem[kPacketSize] = {};
alignas(4) uint8_t s_RspMem[kPacketSize] = {};

BtAttReqRsp_t *Req()
{
	return reinterpret_cast<BtAttReqRsp_t *>(s_ReqMem);
}

BtAttReqRsp_t *Rsp()
{
	return reinterpret_cast<BtAttReqRsp_t *>(s_RspMem);
}

void ResetPackets()
{
	std::memset(s_ReqMem, 0, sizeof(s_ReqMem));
	std::memset(s_RspMem, 0, sizeof(s_RspMem));
}

void ResetPeer()
{
	std::memset(&s_Peer, 0, sizeof(s_Peer));
	std::memset(&s_Security, 0, sizeof(s_Security));
	std::memset(s_LongWrite, 0, sizeof(s_LongWrite));

	s_Peer.Conn.Hdl = kConnHdl;
	s_Peer.Conn.MaxMtu = BT_ATT_MTU_MIN;
	s_Peer.Conn.pLongWrBuff = s_LongWrite;
	s_Peer.Conn.LongWrBuffSize = sizeof(s_LongWrite);
	s_Peer.Conn.LongWrLen = 0;

	s_CccdHdl = 0;
	s_CccdValue = 0;
	s_CccdSetCount = 0;

	s_WriteCount = 0;
	s_WriteOffset = -1;
	s_WriteLen = -1;
	std::memset(s_WriteData, 0, sizeof(s_WriteData));

	s_NotifyCount = 0;
	s_NotifyConnHdl = 0;
	s_NotifyValHdl = 0;
	s_NotifyLen = 0;
	std::memset(s_NotifyData, 0, sizeof(s_NotifyData));

	s_ConfirmCount = 0;
	s_ConfirmConnHdl = 0;
}

uint32_t Process(int ReqLen)
{
	std::memset(s_RspMem, 0, sizeof(s_RspMem));
	return BtAttProcessReq(kConnHdl, Req(), ReqLen, Rsp());
}

void CheckError(uint32_t RspLen, uint8_t ReqOpCode, uint16_t Hdl, uint8_t Error)
{
	CHECK(RspLen == sizeof(BtAttErrorRsp_t) + 1U);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_ERROR_RSP);
	CHECK(Rsp()->ErrorRsp.ReqOpCode == ReqOpCode);
	CHECK(Rsp()->ErrorRsp.Hdl == Hdl);
	CHECK(Rsp()->ErrorRsp.Error == Error);
}

void WriteCallback(BtChar_t *, uint8_t *pData, int Offset, int Len)
{
	++s_WriteCount;
	s_WriteOffset = Offset;
	s_WriteLen = Len;

	size_t copyLen = Len > 0 ? static_cast<size_t>(Len) : 0U;
	if (copyLen > sizeof(s_WriteData))
	{
		copyLen = sizeof(s_WriteData);
	}
	if (copyLen > 0 && pData != nullptr)
	{
		std::memcpy(s_WriteData, pData, copyLen);
	}
}

BtAttDBEntry_t *AddCharacteristic(BtGattChar_t *pChar, uint16_t Uuid,
								  uint32_t Property, uint16_t MaxDataLen,
								  const uint8_t *pInitial, uint16_t InitialLen)
{
	std::memset(pChar, 0, sizeof(*pChar));
	pChar->Uuid = Uuid;
	pChar->MaxDataLen = MaxDataLen;
	pChar->Property = Property;
	pChar->WrCB = WriteCallback;

	BtUuid16_t typeUuid = { 0, BT_UUID_TYPE_16, Uuid };
	BtAttDBEntry_t *pEntry =
		BtAttDBAddEntry(&typeUuid,
						static_cast<int>(sizeof(BtAttCharValue_t) + MaxDataLen));
	if (pEntry == nullptr)
	{
		return nullptr;
	}

	BtAttCharValue_t *pValue =
		reinterpret_cast<BtAttCharValue_t *>(pEntry->Data);
	pValue->pChar = pChar;
	pChar->Hdl = pEntry->Hdl;
	pChar->ValHdl = pEntry->Hdl;
	pChar->pValue = pValue->Data;

	if (InitialLen > MaxDataLen)
	{
		InitialLen = MaxDataLen;
	}
	if (InitialLen > 0 && pInitial != nullptr)
	{
		std::memcpy(pChar->pValue, pInitial, InitialLen);
	}
	pChar->ValueLen = InitialLen;

	return pEntry;
}

BtAttDBEntry_t *AddCccd(BtGattChar_t *pChar)
{
	BtUuid16_t typeUuid = {
		0,
		BT_UUID_TYPE_16,
		BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION
	};
	BtAttDBEntry_t *pEntry =
		BtAttDBAddEntry(&typeUuid, sizeof(BtDescClientCharConfig_t));
	if (pEntry == nullptr)
	{
		return nullptr;
	}

	BtDescClientCharConfig_t *pCccd =
		reinterpret_cast<BtDescClientCharConfig_t *>(pEntry->Data);
	pCccd->pChar = pChar;
	pCccd->CccVal = 0;
	pChar->CccdHdl = pEntry->Hdl;
	s_CccdHdl = pEntry->Hdl;

	return pEntry;
}

void TestDatabaseAllocator()
{
	BtAttDBInit(256);

	BtUuid16_t uuid = { 0, BT_UUID_TYPE_16, 0xFFF1 };
	CHECK(BtAttDBAddEntry(nullptr, 4) == nullptr);
	CHECK(BtAttDBAddEntry(&uuid, -1) == nullptr);
	CHECK(BtAttDBAddEntry(&uuid, 0x10000) == nullptr);

	BtAttDBEntry_t *pFirst = BtAttDBAddEntry(&uuid, 8);
	BtAttDBEntry_t *pSecond = BtAttDBAddEntry(&uuid, 12);

	CHECK(pFirst != nullptr);
	CHECK(pSecond != nullptr);
	if (pFirst != nullptr && pSecond != nullptr)
	{
		CHECK(pFirst->Hdl == 1);
		CHECK(pSecond->Hdl == 2);
		CHECK(pFirst->pPrev == nullptr);
		CHECK(pFirst->pNext == pSecond);
		CHECK(pSecond->pPrev == pFirst);
		CHECK(BtAttDBFindHandle(1) == pFirst);
		CHECK(BtAttDBFindHandle(2) == pSecond);
		CHECK(BtAttDBFindHandle(3) == nullptr);
	}

	int count = 2;
	while (BtAttDBAddEntry(&uuid, 12) != nullptr)
	{
		++count;
		CHECK(count < 32);
	}
	CHECK(count >= 2);
	CHECK(BtAttDBAddEntry(&uuid, 12) == nullptr);

	BtAttDBInit(256);
	BtAttDBEntry_t *pReset = BtAttDBAddEntry(&uuid, 8);
	CHECK(pReset != nullptr);
	if (pReset != nullptr)
	{
		CHECK(pReset->Hdl == 1);
	}
}

void TestMtuAndMalformedPdu()
{
	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ;
	Req()->ExchgMtuReqRsp.RxMtu = 64;

	uint32_t len = Process(3);
	CHECK(len == sizeof(BtAttExchgMtuReqRsp_t) + 1U);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP);
	CHECK(Rsp()->ExchgMtuReqRsp.RxMtu == BT_ATT_MTU_MAX);
	CHECK(s_Peer.Conn.MaxMtu == 64);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_READ_REQ;
	Req()->ReadReq.Hdl = 1;
	CheckError(Process(2), BT_ATT_OPCODE_ATT_READ_REQ, 0,
			   BT_ATT_ERROR_INVALID_PDU);

	ResetPackets();
	Req()->OpCode = 0x14;
	CheckError(Process(1), 0x14, 0, BT_ATT_ERROR_REQUEST_NOT_SUPP);
}

void TestReadAndSecurity(BtGattChar_t *pChar, BtAttDBEntry_t *pEntry)
{
	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_READ_REQ;
	Req()->ReadReq.Hdl = pEntry->Hdl;

	uint32_t len = Process(3);
	CHECK(len == pChar->ValueLen + 1U);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_READ_RSP);
	CHECK(std::memcmp(Rsp()->ReadRsp.Data, pChar->pValue,
					  pChar->ValueLen) == 0);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_READ_BLOB_REQ;
	Req()->ReadBlobReq.Hdl = pEntry->Hdl;
	Req()->ReadBlobReq.Offset = 6;
	len = Process(5);
	CHECK(len == 6);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_READ_BLOB_RSP);
	CHECK(std::memcmp(Rsp()->ReadBlobRsp.Data, "world", 5) == 0);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_READ_BLOB_REQ;
	Req()->ReadBlobReq.Hdl = pEntry->Hdl;
	Req()->ReadBlobReq.Offset = static_cast<uint16_t>(pChar->ValueLen + 1U);
	CheckError(Process(5), BT_ATT_OPCODE_ATT_READ_BLOB_REQ, pEntry->Hdl,
			   BT_ATT_ERROR_INVALID_OFFSET);

	uint32_t property = pChar->Property;
	pChar->Property &= ~BT_GATT_CHAR_PROP_READ;

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_READ_REQ;
	Req()->ReadReq.Hdl = pEntry->Hdl;
	CheckError(Process(3), BT_ATT_OPCODE_ATT_READ_REQ, pEntry->Hdl,
			   BT_ATT_ERROR_READ_NOT_PERMITTED);
	pChar->Property = property;

	pEntry->Permission = BT_ATT_PERMISSION_READ_ENCRYPT;
	s_Security = {};

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_READ_REQ;
	Req()->ReadReq.Hdl = pEntry->Hdl;
	CheckError(Process(3), BT_ATT_OPCODE_ATT_READ_REQ, pEntry->Hdl,
			   BT_ATT_ERROR_INSUF_AUTHEN);

	s_Security.Level = BT_GAP_SEC_LEVEL_ENC_UNAUTH;
	s_Security.KeySize = 16;
	len = Process(3);
	CHECK(len == pChar->ValueLen + 1U);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_READ_RSP);

	pEntry->Permission = 0;
	s_Security = {};

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_READ_REQ;
	Req()->ReadReq.Hdl = 0x7777;
	CheckError(Process(3), BT_ATT_OPCODE_ATT_READ_REQ, 0x7777,
			   BT_ATT_ERROR_INVALID_HANDLE);
}

void TestWrite(BtGattChar_t *pChar, BtAttDBEntry_t *pEntry)
{
	pChar->ValueLen = 0;
	s_WriteCount = 0;

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_WRITE_REQ;
	Req()->WriteReq.Hdl = pEntry->Hdl;
	std::memcpy(Req()->WriteReq.Data, "abc", 3);

	uint32_t len = Process(6);
	CHECK(len == 1);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_WRITE_RSP);
	CHECK(pChar->ValueLen == 3);
	CHECK(std::memcmp(pChar->pValue, "abc", 3) == 0);
	CHECK(s_WriteCount == 1);
	CHECK(s_WriteOffset == 0);
	CHECK(s_WriteLen == 3);
	CHECK(std::memcmp(s_WriteData, "abc", 3) == 0);

	pChar->ValueLen = 0;
	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_CMD;
	Req()->WriteCmd.Hdl = pEntry->Hdl;
	std::memcpy(Req()->WriteCmd.Data, "xy", 2);

	len = Process(5);
	CHECK(len == 0);
	CHECK(pChar->ValueLen == 2);
	CHECK(std::memcmp(pChar->pValue, "xy", 2) == 0);
	CHECK(s_WriteCount == 2);

	uint32_t property = pChar->Property;
	pChar->Property &= ~BT_GATT_CHAR_PROP_WRITE;

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_WRITE_REQ;
	Req()->WriteReq.Hdl = pEntry->Hdl;
	Req()->WriteReq.Data[0] = 0x5A;
	CheckError(Process(4), BT_ATT_OPCODE_ATT_WRITE_REQ, pEntry->Hdl,
			   BT_ATT_ERROR_WRITE_NOT_PERMITTED);
	pChar->Property = property;
}

void TestPreparedWrite(BtGattChar_t *pChar, BtAttDBEntry_t *pEntry)
{
	pChar->ValueLen = 0;
	s_Peer.Conn.LongWrLen = 0;
	s_Peer.Conn.LongWrBuffSize = sizeof(s_LongWrite);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	Req()->PrepareWriteReq.Hdl = pEntry->Hdl;
	Req()->PrepareWriteReq.Offset = 0;
	std::memcpy(Req()->PrepareWriteReq.Data, "abc", 3);

	uint32_t len = Process(8);
	CHECK(len == 8);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP);
	CHECK(Rsp()->PrepareWriteRsp.Hdl == pEntry->Hdl);
	CHECK(Rsp()->PrepareWriteRsp.Offset == 0);
	CHECK(std::memcmp(Rsp()->PrepareWriteRsp.Data, "abc", 3) == 0);
	CHECK(s_Peer.Conn.LongWrLen == 9);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	Req()->ExecuteWriteReq.Flag = 1;
	len = Process(2);
	CHECK(len == 1);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP);
	CHECK(s_Peer.Conn.LongWrLen == 0);
	CHECK(pChar->ValueLen == 3);
	CHECK(std::memcmp(pChar->pValue, "abc", 3) == 0);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	Req()->PrepareWriteReq.Hdl = pEntry->Hdl;
	Req()->PrepareWriteReq.Offset = 3;
	std::memcpy(Req()->PrepareWriteReq.Data, "de", 2);
	CHECK(Process(7) == 7);
	CHECK(s_Peer.Conn.LongWrLen == 8);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	Req()->ExecuteWriteReq.Flag = 0;
	CHECK(Process(2) == 1);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP);
	CHECK(s_Peer.Conn.LongWrLen == 0);
	CHECK(pChar->ValueLen == 3);
	CHECK(std::memcmp(pChar->pValue, "abc", 3) == 0);

	s_Peer.Conn.LongWrBuffSize = 6;
	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	Req()->PrepareWriteReq.Hdl = pEntry->Hdl;
	Req()->PrepareWriteReq.Offset = 3;
	Req()->PrepareWriteReq.Data[0] = 0xAA;
	CheckError(Process(6), BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ, pEntry->Hdl,
			   BT_ATT_ERROR_PREPARE_QUE_FULL);
	CHECK(s_Peer.Conn.LongWrLen == 0);
	s_Peer.Conn.LongWrBuffSize = sizeof(s_LongWrite);
}

void TestCccd(BtGattChar_t *pNotifyChar, BtAttDBEntry_t *pCccd)
{
	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_WRITE_REQ;
	Req()->WriteReq.Hdl = pCccd->Hdl;
	Req()->WriteReq.Data[0] = BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION;
	Req()->WriteReq.Data[1] = 0;

	uint32_t len = Process(5);
	CHECK(len == 1);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_WRITE_RSP);
	CHECK(s_CccdSetCount == 1);
	CHECK(s_CccdHdl == pCccd->Hdl);
	CHECK(s_CccdValue == BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_READ_REQ;
	Req()->ReadReq.Hdl = pCccd->Hdl;
	len = Process(3);
	CHECK(len == 3);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_READ_RSP);
	CHECK(Rsp()->ReadRsp.Data[0] == BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);
	CHECK(Rsp()->ReadRsp.Data[1] == 0);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_WRITE_REQ;
	Req()->WriteReq.Hdl = pCccd->Hdl;
	Req()->WriteReq.Data[0] = BT_DESC_CLIENT_CHAR_CONFIG_INDICATION;
	Req()->WriteReq.Data[1] = 0;
	CheckError(Process(5), BT_ATT_OPCODE_ATT_WRITE_REQ, pCccd->Hdl,
			   BT_ATT_ERROR_VALUE_NOT_ALLOWED);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_WRITE_REQ;
	Req()->WriteReq.Hdl = pCccd->Hdl;
	Req()->WriteReq.Data[0] = 1;
	CheckError(Process(4), BT_ATT_OPCODE_ATT_WRITE_REQ, pCccd->Hdl,
			   BT_ATT_ERROR_INVALID_ATT_VALUE);

	CHECK((pNotifyChar->Property & BT_GATT_CHAR_PROP_NOTIFY) != 0);
	CHECK((pNotifyChar->Property & BT_GATT_CHAR_PROP_INDICATE) == 0);
}

void TestClientValueEvents()
{
	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF;
	Req()->HandleValueNtf.ValHdl = 0x1234;
	std::memcpy(Req()->HandleValueNtf.Data, "ntf", 3);

	CHECK(Process(6) == 0);
	CHECK(s_NotifyCount == 1);
	CHECK(s_NotifyConnHdl == kConnHdl);
	CHECK(s_NotifyValHdl == 0x1234);
	CHECK(s_NotifyLen == 3);
	CHECK(std::memcmp(s_NotifyData, "ntf", 3) == 0);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND;
	Req()->HandleValueInd.Hdl = 0x2345;
	std::memcpy(Req()->HandleValueInd.Data, "ind", 3);

	CHECK(Process(6) == 1);
	CHECK(Rsp()->OpCode == BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM);
	CHECK(s_NotifyCount == 2);
	CHECK(s_NotifyValHdl == 0x2345);

	ResetPackets();
	Req()->OpCode = BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM;
	CHECK(Process(1) == 0);
	CHECK(s_ConfirmCount == 1);
	CHECK(s_ConfirmConnHdl == kConnHdl);
}

} // namespace

extern "C" BtDevice_t *BtPeerFindByHdl(uint16_t ConnHdl)
{
	return ConnHdl == kConnHdl ? &s_Peer : nullptr;
}

extern "C" bool BtGapConnSecGet(uint16_t ConnHdl, BtConnSec_t *pSec)
{
	if (ConnHdl != kConnHdl || pSec == nullptr)
	{
		return false;
	}

	*pSec = s_Security;
	return true;
}

extern "C" uint16_t BtGattCccdGet(uint16_t ConnHdl, uint16_t CccdHdl)
{
	if (ConnHdl != kConnHdl || CccdHdl != s_CccdHdl)
	{
		return 0;
	}

	return s_CccdValue;
}

extern "C" bool BtGattCccdSet(uint16_t ConnHdl, uint16_t CccdHdl,
							 uint16_t Value)
{
	if (ConnHdl != kConnHdl || CccdHdl != s_CccdHdl)
	{
		return false;
	}

	s_CccdValue = Value;
	++s_CccdSetCount;
	return true;
}

extern "C" bool BtSmpSignVerify(uint16_t, const uint8_t *, size_t,
							   const uint8_t *)
{
	return false;
}

extern "C" void BtGattClientNotified(uint16_t ConnHdl, uint16_t ValHdl,
									uint8_t *pData, uint16_t Len)
{
	++s_NotifyCount;
	s_NotifyConnHdl = ConnHdl;
	s_NotifyValHdl = ValHdl;
	s_NotifyLen = Len;

	size_t copyLen = Len;
	if (copyLen > sizeof(s_NotifyData))
	{
		copyLen = sizeof(s_NotifyData);
	}
	if (copyLen > 0 && pData != nullptr)
	{
		std::memcpy(s_NotifyData, pData, copyLen);
	}
}

extern "C" void BtGattHandleValueConfirm(uint16_t ConnHdl)
{
	++s_ConfirmCount;
	s_ConfirmConnHdl = ConnHdl;
}

int main()
{
	ResetPeer();
	TestDatabaseAllocator();

	BtAttDBInit(2048);

	const uint8_t initial[] = "hello-world";
	BtGattChar_t readWriteChar = {};
	BtAttDBEntry_t *pReadWrite =
		AddCharacteristic(&readWriteChar, 0xFFF1,
						  BT_GATT_CHAR_PROP_READ |
						  BT_GATT_CHAR_PROP_WRITE |
						  BT_GATT_CHAR_PROP_WRITE_WORESP,
						  32, initial, sizeof(initial) - 1U);

	BtGattChar_t notifyChar = {};
	BtAttDBEntry_t *pNotify =
		AddCharacteristic(&notifyChar, 0xFFF2,
						  BT_GATT_CHAR_PROP_READ |
						  BT_GATT_CHAR_PROP_NOTIFY,
						  8, nullptr, 0);
	BtAttDBEntry_t *pCccd = pNotify != nullptr ? AddCccd(&notifyChar) : nullptr;

	CHECK(pReadWrite != nullptr);
	CHECK(pNotify != nullptr);
	CHECK(pCccd != nullptr);

	if (pReadWrite != nullptr && pCccd != nullptr)
	{
		TestMtuAndMalformedPdu();
		TestReadAndSecurity(&readWriteChar, pReadWrite);
		TestWrite(&readWriteChar, pReadWrite);
		TestPreparedWrite(&readWriteChar, pReadWrite);
		TestCccd(&notifyChar, pCccd);
		TestClientValueEvents();
	}

	if (s_Failures != 0)
	{
		std::printf("ATT host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("ATT host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
