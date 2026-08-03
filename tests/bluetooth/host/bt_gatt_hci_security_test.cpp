#include <cstddef>
#include <cstdint>
#include <cstring>

#include "bt_test_harness.h"
#include "crypto/icrypto.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_smp.h"

namespace {

constexpr uint16_t kConnHdl = 0x0042;

bttest::Context s_Test("Native HCI GATT security tests");
BtDevice_t s_Peer;
BtHciDevice_t s_Hci;
BtConnSec_t s_Sec;
BtGattSrvc_t s_Service;
BtGattChar_t s_Char;
uint8_t s_Value[32];
uint8_t s_EntryStorage[sizeof(BtAttDBEntry_t) + sizeof(BtAttCharValue_t)] = {};
BtAttDBEntry_t *s_Entry = reinterpret_cast<BtAttDBEntry_t *>(s_EntryStorage);
bool s_NotifyEnabled = true;
bool s_IndicateEnabled = true;
bool s_TransportAccept = true;
int s_SendCount = 0;
int s_ReserveCount = 0;
int s_ReleaseCount = 0;

void Reset()
{
	std::memset(&s_Peer, 0, sizeof(s_Peer));
	std::memset(&s_Hci, 0, sizeof(s_Hci));
	std::memset(&s_Sec, 0, sizeof(s_Sec));
	std::memset(&s_Service, 0, sizeof(s_Service));
	std::memset(&s_Char, 0, sizeof(s_Char));
	std::memset(s_Value, 0xA5, sizeof(s_Value));
	std::memset(s_EntryStorage, 0, sizeof(s_EntryStorage));

	s_Peer.Conn.Hdl = kConnHdl;
	s_Peer.Conn.MaxMtu = BT_ATT_MTU_MIN;
	s_Peer.pHciDev = &s_Hci;
	s_Hci.AclMaxLen = 27;

	s_Char.Uuid = 0xFFF1;
	s_Char.MaxDataLen = sizeof(s_Value);
	s_Char.Property = BT_GATT_CHAR_PROP_READ |
			BT_GATT_CHAR_PROP_WRITE |
			BT_GATT_CHAR_PROP_NOTIFY |
			BT_GATT_CHAR_PROP_INDICATE;
	s_Char.pValue = s_Value;
	s_Char.ValueLen = 4;
	s_Char.ValHdl = 0x0025;
	s_Char.CccdHdl = 0x0026;
	s_Char.pSrvc = &s_Service;

	s_Service.SecType = BT_GAP_SECTYPE_STATICKEY_NO_MITM;
	s_Service.NbChar = 1;
	s_Service.pCharArray = &s_Char;

	s_Entry->TypeUuid.BaseIdx = 1;
	s_Entry->TypeUuid.Type = BT_UUID_TYPE_16;
	s_Entry->TypeUuid.Uuid = s_Char.Uuid;
	auto *pValue = reinterpret_cast<BtAttCharValue_t *>(s_Entry->Data);
	pValue->pChar = &s_Char;

	s_NotifyEnabled = true;
	s_IndicateEnabled = true;
	s_TransportAccept = true;
	s_SendCount = 0;
	s_ReserveCount = 0;
	s_ReleaseCount = 0;
}

void TestSecurityErrors()
{
	Reset();

	BT_CHECK(s_Test,
			BtAttAccessSecurityError(kConnHdl, s_Entry, true) ==
			BT_ATT_ERROR_INSUF_ENCRYPT);

	s_Sec.Level = BT_GAP_SEC_LEVEL_ENC_UNAUTH;
	s_Sec.KeySize = 16;
	BT_CHECK(s_Test,
			BtAttAccessSecurityError(kConnHdl, s_Entry, true) == 0);

	s_Service.SecType = BT_GAP_SECTYPE_STATICKEY_MITM;
	BT_CHECK(s_Test,
			BtAttAccessSecurityError(kConnHdl, s_Entry, true) ==
			BT_ATT_ERROR_INSUF_AUTHEN);

	s_Sec.Level = BT_GAP_SEC_LEVEL_ENC_AUTH;
	s_Sec.KeySize = 6;
	BT_CHECK(s_Test,
			BtAttAccessSecurityError(kConnHdl, s_Entry, true) ==
			BT_ATT_ERROR_ENCRYPT_KEY_TOO_SHORT);

	s_Service.SecType = BT_GAP_SECTYPE_LESC_MITM;
	s_Sec.Level = BT_GAP_SEC_LEVEL_LESC_AUTH;
	s_Sec.KeySize = 16;
	s_Sec.Flags = 0;
	BT_CHECK(s_Test,
			BtAttAccessSecurityError(kConnHdl, s_Entry, true) ==
			BT_ATT_ERROR_INSUF_AUTHEN);

	s_Sec.Flags = BT_GAP_SEC_FLAG_SC;
	BT_CHECK(s_Test,
			BtAttAccessSecurityError(kConnHdl, s_Entry, true) == 0);
}

void TestMtuBoundDoesNotTruncate()
{
	Reset();
	uint8_t payload[BT_ATT_MTU_MIN - 2];
	std::memset(payload, 0x3C, sizeof(payload));
	uint8_t before[sizeof(s_Value)];
	std::memcpy(before, s_Value, sizeof(before));
	uint16_t beforeLen = s_Char.ValueLen;

	BT_CHECK(s_Test,
			!BtGattCharNotify(kConnHdl, &s_Char,
							 payload, sizeof(payload)));
	BT_CHECK(s_Test, s_SendCount == 0);
	BT_CHECK(s_Test, s_ReserveCount == 0);
	BT_CHECK(s_Test, s_Char.ValueLen == beforeLen);
	BT_CHECK(s_Test,
			std::memcmp(s_Value, before, sizeof(before)) == 0);
}

void TestTransportFailureDoesNotMutateValue()
{
	Reset();
	s_TransportAccept = false;
	uint8_t payload[4] = { 1, 2, 3, 4 };
	uint8_t before[sizeof(s_Value)];
	std::memcpy(before, s_Value, sizeof(before));
	uint16_t beforeLen = s_Char.ValueLen;

	BT_CHECK(s_Test,
			!BtGattCharNotify(kConnHdl, &s_Char,
							 payload, sizeof(payload)));
	BT_CHECK(s_Test, s_SendCount == 1);
	BT_CHECK(s_Test, s_ReserveCount == 1);
	BT_CHECK(s_Test, s_ReleaseCount == 1);
	BT_CHECK(s_Test, s_Char.ValueLen == beforeLen);
	BT_CHECK(s_Test,
			std::memcmp(s_Value, before, sizeof(before)) == 0);
}

void TestSuccessfulSendUpdatesValue()
{
	Reset();
	uint8_t payload[4] = { 1, 2, 3, 4 };

	BT_CHECK(s_Test,
			BtGattCharNotify(kConnHdl, &s_Char,
							payload, sizeof(payload)));
	BT_CHECK(s_Test, s_SendCount == 1);
	BT_CHECK(s_Test, s_ReserveCount == 1);
	BT_CHECK(s_Test, s_ReleaseCount == 0);
	BT_CHECK(s_Test, s_Char.ValueLen == sizeof(payload));
	BT_CHECK(s_Test,
			std::memcmp(s_Value, payload, sizeof(payload)) == 0);
}

} // namespace

extern "C" {

void CryptoSecureWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = static_cast<volatile uint8_t *>(pData);
	while (Len-- > 0)
	{
		*p++ = 0;
	}
}

bool BtSmpBondKeysLookup(uint16_t, uint64_t, uint16_t, BtSmpKeys_t *pKeys)
{
	if (pKeys != nullptr)
	{
		std::memset(pKeys, 0, sizeof(*pKeys));
	}
	return false;
}

bool BtGapConnSecGet(uint16_t ConnHdl, BtConnSec_t *pSec)
{
	if (ConnHdl != kConnHdl || pSec == nullptr)
	{
		return false;
	}
	*pSec = s_Sec;
	return true;
}

BtDevice_t *BtPeerFindByHdl(uint16_t ConnHdl)
{
	return ConnHdl == kConnHdl ? &s_Peer : nullptr;
}

bool BtGattCharNotifyEnabled(uint16_t, BtGattChar_t *)
{
	return s_NotifyEnabled;
}

bool BtGattCharIndicateEnabled(uint16_t, BtGattChar_t *)
{
	return s_IndicateEnabled;
}

bool BtGattTxPendReserve(uint16_t, BtGattChar_t *, uint16_t)
{
	s_ReserveCount++;
	return true;
}

void BtGattTxPendRelease(uint16_t)
{
	s_ReleaseCount++;
}

uint32_t BtHciSendAcl(BtHciDevice_t * const,
					  BtHciACLDataPacket_t * const pPacket)
{
	s_SendCount++;
	if (!s_TransportAccept || pPacket == nullptr)
	{
		return 0;
	}
	return (uint32_t)pPacket->Hdr.Len + sizeof(BtHciACLDataPacketHdr_t);
}

uint32_t BtGattMsTick(void)
{
	return 100;
}

} // extern "C"

int main()
{
	s_Test.Run("security error mapping", TestSecurityErrors);
	s_Test.Run("MTU bound rejects truncation", TestMtuBoundDoesNotTruncate);
	s_Test.Run("transport failure preserves value",
			   TestTransportFailureDoesNotMutateValue);
	s_Test.Run("successful send updates value", TestSuccessfulSendUpdatesValue);
	return s_Test.Finish();
}
