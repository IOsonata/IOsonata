#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softrng.h"

#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gap.h"

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

const BtAdvData_t *FindAd(const uint8_t *pData, size_t Len, uint8_t Type)
{
	size_t off = 0;

	while (off + sizeof(BtAdvDataHdr_t) <= Len)
	{
		const BtAdvData_t *p = reinterpret_cast<const BtAdvData_t *>(&pData[off]);
		if (p->Hdr.Len == 0)
		{
			break;
		}

		size_t recLen = static_cast<size_t>(p->Hdr.Len) + 1U;
		if (recLen < sizeof(BtAdvDataHdr_t) || off + recLen > Len)
		{
			break;
		}

		if (p->Hdr.Type == Type)
		{
			return p;
		}
		off += recLen;
	}

	return nullptr;
}


// --- Encrypted Advertising Data in the shared encoder ---------------------

// The integration lives here rather than in a port because every port reaches
// its advertising data through BtAdvEncode with the same BtAdvPacket_t. These
// cases work on a packet directly, which is what BtAdvEncode hands over.

// BtEadRandGen refuses an engine reporting IsSecure() false, so these cases
// present one that claims otherwise. Nothing outside a test does that.
class TestRng : public CryptoSoftRng {
public:
	bool IsSecure() const override { return true; }
};

CryptoSoftAes s_EadAes;
TestRng s_EadRng;

const BtEadKey_t kEadKey = {
	{ 0x57,0xA9,0xDA,0x12,0xD1,0x2E,0x6E,0x13,
	  0x1E,0x20,0x61,0x2A,0xD1,0x0A,0x6A,0x19 },
	{ 0x9E,0x7A,0x00,0xEF,0xB1,0x7A,0xE7,0x46 }
};

// A packet holding a couple of ordinary AD structures.
int BuildPlain(uint8_t *pBuf, int Max, BtAdvPacket_t *pPkt)
{
	const uint8_t name[6] = { 'S','e','n','s','o','r' };
	const uint8_t flags = 0x06;

	pPkt->Len = 0;
	CHECK(BtAdvDataAdd(pPkt, BT_GAP_DATA_TYPE_FLAGS, (uint8_t*)&flags, 1));
	CHECK(BtAdvDataAdd(pPkt, BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME,
					   (uint8_t*)name, sizeof(name)));
	(void)pBuf;
	(void)Max;

	return pPkt->Len;
}

// With no key installed nothing happens at all, so a build that never uses
// the feature takes no new path.
void TestNotArmedLeavesThePacketAlone()
{
	uint8_t buf[64];
	BtAdvPacket_t pkt = { static_cast<int>(sizeof(buf)), 0, buf };

	CHECK(BtAdvEadKeySet(nullptr));
	CHECK(BtAdvEadIsArmed() == false);

	int len = BuildPlain(buf, sizeof(buf), &pkt);
	uint8_t before[64];

	std::memcpy(before, buf, len);

	CHECK(BtAdvEncrypt(&pkt) == false);
	CHECK(pkt.Len == len);
	CHECK(std::memcmp(buf, before, len) == 0);
}

// Armed, the packet becomes one Encrypted Data AD structure, and decrypting
// its AD data gives back exactly what was there before.
void TestArmedWrapsThePacket()
{
	uint8_t buf[64];
	BtAdvPacket_t pkt = { static_cast<int>(sizeof(buf)), 0, buf };

	int len = BuildPlain(buf, sizeof(buf), &pkt);
	uint8_t plain[64];

	std::memcpy(plain, buf, len);

	CHECK(BtAdvEadKeySet(&kEadKey));
	CHECK(BtAdvEadIsArmed());
	CHECK(BtAdvEncrypt(&pkt));

	// One AD structure: Length, Type, then randomizer, ciphertext and MIC.
	CHECK(pkt.Len == len + 2 + BTEAD_OVERHEAD);
	CHECK(buf[0] == 1 + BTEAD_OVERHEAD + len);
	CHECK(buf[1] == BTEAD_AD_TYPE);

	// The plaintext is gone from the packet.
	CHECK(std::memcmp(&buf[2], plain, len) != 0);

	uint8_t back[64];
	size_t n = BtEadDecrypt(&kEadKey, &buf[2], (size_t)(pkt.Len - 2),
							back, sizeof(back));

	CHECK(n == (size_t)len);
	CHECK(std::memcmp(back, plain, len) == 0);

	CHECK(BtAdvEadKeySet(nullptr));
}

// The scanner half. A scan report handler has the whole advertising payload,
// not the AD data of one structure, so what it needs is to be handed the
// report and get the plaintext back. Nothing did that before: the cipher was
// reachable and the report was not.
void TestAReportIsDecrypted()
{
	uint8_t buf[64];
	BtAdvPacket_t pkt = { static_cast<int>(sizeof(buf)), 0, buf };

	int len = BuildPlain(buf, sizeof(buf), &pkt);
	uint8_t plain[64];

	std::memcpy(plain, buf, len);

	CHECK(BtAdvEadKeySet(&kEadKey));
	CHECK(BtAdvEncrypt(&pkt));

	uint8_t back[64];
	size_t n = BtAdvDecrypt(&kEadKey, buf, (size_t)pkt.Len, back, sizeof(back));

	CHECK(n == (size_t)len);
	CHECK(std::memcmp(back, plain, len) == 0);

	// The structure is found wherever it sits, not only first. A report that
	// leads with Flags is the ordinary case on air.
	uint8_t rep[80];
	int off = 0;

	rep[off++] = 2;
	rep[off++] = 0x01;					// Flags
	rep[off++] = 0x06;
	std::memcpy(&rep[off], buf, pkt.Len);
	off += pkt.Len;

	std::memset(back, 0, sizeof(back));
	n = BtAdvDecrypt(&kEadKey, rep, (size_t)off, back, sizeof(back));
	CHECK(n == (size_t)len);
	CHECK(std::memcmp(back, plain, len) == 0);

	// A report with no Encrypted Data structure yields nothing rather than
	// treating some other structure as ciphertext.
	CHECK(BtAdvDecrypt(&kEadKey, rep, 3, back, sizeof(back)) == 0);

	// A structure claiming more than the report holds is malformed, and a
	// length octet of zero ends the sequence. Neither may be read past.
	uint8_t bad[8] = { 0x40, BTEAD_AD_TYPE, 1, 2, 3, 4, 5, 6 };
	CHECK(BtAdvDecrypt(&kEadKey, bad, sizeof(bad), back, sizeof(back)) == 0);

	uint8_t pad[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	CHECK(BtAdvDecrypt(&kEadKey, pad, sizeof(pad), back, sizeof(back)) == 0);

	// A wrong key does not authenticate, so nothing comes back. This is the
	// check that says the MIC is being verified and not merely stripped.
	BtEadKey_t wrong = kEadKey;

	wrong.SessionKey[0] ^= 0xFF;
	CHECK(BtAdvDecrypt(&wrong, buf, (size_t)pkt.Len, back, sizeof(back)) == 0);

	CHECK(BtAdvDecrypt(nullptr, buf, (size_t)pkt.Len, back, sizeof(back)) == 0);
	CHECK(BtAdvDecrypt(&kEadKey, nullptr, 4, back, sizeof(back)) == 0);
	CHECK(BtAdvDecrypt(&kEadKey, buf, (size_t)pkt.Len, nullptr, sizeof(back)) == 0);
	CHECK(BtAdvDecrypt(&kEadKey, buf, (size_t)pkt.Len, back, 0) == 0);

	CHECK(BtAdvEadKeySet(nullptr));
}

// A fresh randomizer per call is what CSS Part A 1.23.4 asks for whenever the
// payload changes. Two encryptions of the same payload must not match, or a
// scanner could follow the device across address changes by the ciphertext.
void TestEachWrapUsesAFreshRandomizer()
{
	uint8_t a[64];
	uint8_t b[64];
	BtAdvPacket_t pa = { static_cast<int>(sizeof(a)), 0, a };
	BtAdvPacket_t pb = { static_cast<int>(sizeof(b)), 0, b };

	CHECK(BtAdvEadKeySet(&kEadKey));

	BuildPlain(a, sizeof(a), &pa);
	BuildPlain(b, sizeof(b), &pb);
	CHECK(pa.Len == pb.Len);

	CHECK(BtAdvEncrypt(&pa));
	CHECK(BtAdvEncrypt(&pb));

	CHECK(pa.Len == pb.Len);
	CHECK(std::memcmp(a, b, pa.Len) != 0);

	CHECK(BtAdvEadKeySet(nullptr));
}

// Nothing is written unless the whole structure fits, so a packet with no
// room is left as the plaintext it was rather than half encrypted.
void TestNoRoomLeavesThePlaintext()
{
	uint8_t buf[64];
	BtAdvPacket_t pkt = { static_cast<int>(sizeof(buf)), 0, buf };

	int len = BuildPlain(buf, sizeof(buf), &pkt);
	uint8_t before[64];

	std::memcpy(before, buf, len);

	CHECK(BtAdvEadKeySet(&kEadKey));

	// One octet short of the overhead the structure adds.
	pkt.MaxLen = len + 2 + BTEAD_OVERHEAD - 1;
	CHECK(BtAdvEncrypt(&pkt) == false);
	CHECK(pkt.Len == len);
	CHECK(std::memcmp(buf, before, len) == 0);

	// Exactly enough is accepted.
	pkt.MaxLen = len + 2 + BTEAD_OVERHEAD;
	CHECK(BtAdvEncrypt(&pkt));

	CHECK(BtAdvEadKeySet(nullptr));
}

void TestExtendedSelection()
{
	CHECK(!BtAdvUseExtended(0, 0));
	CHECK(!BtAdvUseExtended(BT_ADV_LEGACY_DATA_MAX, BT_ADV_LEGACY_DATA_MAX));
	CHECK(BtAdvUseExtended(BT_ADV_LEGACY_DATA_MAX + 1U, 0));
	CHECK(BtAdvUseExtended(0, BT_ADV_LEGACY_DATA_MAX + 1U));
}

void TestDataMutation()
{
	uint8_t data[31] = {};
	BtAdvPacket_t pkt = { static_cast<int>(sizeof(data)), 0, data };

	uint8_t flags = BT_GAP_DATA_TYPE_FLAGS_GENERAL_DISCOVERABLE |
					BT_GAP_DATA_TYPE_FLAGS_NO_BREDR;
	CHECK(BtAdvDataAdd(&pkt, BT_GAP_DATA_TYPE_FLAGS, &flags, 1));
	CHECK(pkt.Len == 3);

	const uint8_t firstMan[] = { 0x34, 0x12, 0xA5 };
	CHECK(BtAdvDataAdd(&pkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA,
					   const_cast<uint8_t *>(firstMan), sizeof(firstMan)));
	CHECK(pkt.Len == 8);

	const uint8_t replacement[] = { 0x78, 0x56, 0x11, 0x22 };
	CHECK(BtAdvDataAdd(&pkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA,
					   const_cast<uint8_t *>(replacement), sizeof(replacement)));
	CHECK(pkt.Len == 9);

	uint8_t out[8] = {};
	size_t outLen = BtAdvDataGetManData(pkt.pData, pkt.Len, out, sizeof(out));
	CHECK(outLen == sizeof(replacement));
	CHECK(std::memcmp(out, replacement, sizeof(replacement)) == 0);

	BtAdvDataRemove(&pkt, BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA);
	CHECK(pkt.Len == 3);
	CHECK(FindAd(pkt.pData, pkt.Len, BT_GAP_DATA_TYPE_FLAGS) != nullptr);
}

void TestNameEncoding()
{
	uint8_t data[31] = {};
	BtAdvPacket_t pkt = { static_cast<int>(sizeof(data)), 0, data };

	const char longName[] = "IOsonata-advertising-name-that-is-too-long";
	CHECK(BtAdvDataSetDevName(&pkt, longName));

	const BtAdvData_t *pShort = FindAd(pkt.pData, pkt.Len,
									  BT_GAP_DATA_TYPE_SHORT_LOCAL_NAME);
	CHECK(pShort != nullptr);
	CHECK(FindAd(pkt.pData, pkt.Len,
				 BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME) == nullptr);

	char name[64] = {};
	size_t nameLen = BtAdvDataGetDevName(pkt.pData, pkt.Len, name, sizeof(name));
	CHECK(nameLen == 29U);
	CHECK(std::strncmp(name, longName, nameLen) == 0);
	CHECK(name[nameLen] == '\0');

	CHECK(BtAdvDataSetDevName(&pkt, "IOsonata"));
	CHECK(FindAd(pkt.pData, pkt.Len,
				 BT_GAP_DATA_TYPE_SHORT_LOCAL_NAME) == nullptr);
	CHECK(FindAd(pkt.pData, pkt.Len,
				 BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME) != nullptr);

	std::memset(name, 0, sizeof(name));
	nameLen = BtAdvDataGetDevName(pkt.pData, pkt.Len, name, sizeof(name));
	CHECK(nameLen == 8U);
	CHECK(std::strcmp(name, "IOsonata") == 0);
}

void TestMalformedRecords()
{
	char name[8] = { 'x', 0 };

	uint8_t zeroLength[] = { 0x00, BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME };
	CHECK(BtAdvDataGetDevName(zeroLength, sizeof(zeroLength),
							  name, sizeof(name)) == 0U);
	CHECK(name[0] == '\0');

	uint8_t oversized[] = { 0x08, BT_GAP_DATA_TYPE_COMPLETE_LOCAL_NAME };
	name[0] = 'x';
	CHECK(BtAdvDataGetDevName(oversized, sizeof(oversized),
							  name, sizeof(name)) == 0U);
	CHECK(name[0] == '\0');
}

void TestUuidRoundTrip()
{
	uint8_t uuid[16] = {
		0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE,
		0x01, 0x23, 0x45, 0x67, 0xCD, 0xAB, 0x89, 0xEF
	};
	uint8_t original[sizeof(uuid)];
	std::memcpy(original, uuid, sizeof(uuid));

	BtUuid16_t shortUuid = {};
	int baseIdx = BtUuid128To16(&shortUuid, uuid);
	CHECK(baseIdx > 0);
	CHECK(shortUuid.BaseIdx == baseIdx);
	CHECK(shortUuid.Type == BT_UUID_TYPE_16);
	CHECK(shortUuid.Uuid == 0xABCDU);
	CHECK(std::memcmp(uuid, original, sizeof(uuid)) == 0);

	uint8_t rebuilt[16] = {};
	CHECK(BtUuid16To128(&shortUuid, rebuilt));
	CHECK(std::memcmp(rebuilt, original, sizeof(rebuilt)) == 0);

	BtUuid_t invalid = {};
	invalid.BaseIdx = 63;
	invalid.Type = BT_UUID_TYPE_16;
	invalid.Uuid16 = 0x180FU;
	CHECK(!BtUuidTo128(&invalid, rebuilt));
}

void TestUuidNullInputs()
{
	uint8_t uuid[16] = BLUETOOTH_SIG_BASE_UUID;
	uint8_t rebuilt[16] = {};
	BtUuid_t uuidAny = {};
	uuidAny.BaseIdx = 0;
	uuidAny.Type = BT_UUID_TYPE_16;
	uuidAny.Uuid16 = 0x180FU;
	BtUuid16_t uuid16 = { 0, BT_UUID_TYPE_16, 0x180FU };
	BtUuid32_t uuid32 = { 0, BT_UUID_TYPE_32, 0x11223344U };

	CHECK(BtUuidFindBase(nullptr) == -1);
	CHECK(BtUuidAddBase(nullptr) == -1);
	CHECK(!BtUuidGetBase(0, nullptr));
	CHECK(!BtUuidTo128(nullptr, rebuilt));
	CHECK(!BtUuidTo128(&uuidAny, nullptr));
	CHECK(!BtUuid16To128(nullptr, rebuilt));
	CHECK(!BtUuid16To128(&uuid16, nullptr));
	CHECK(!BtUuid32To128(nullptr, rebuilt));
	CHECK(!BtUuid32To128(&uuid32, nullptr));
	CHECK(BtUuid128To16(nullptr, uuid) == -1);
	CHECK(BtUuid128To16(&uuid16, nullptr) == -1);
}

void TestUuidAdvertising()
{
	alignas(BtUuidArr_t) uint8_t uuidMem[sizeof(BtUuidArr_t) + sizeof(uint16_t)] = {};
	BtUuidArr_t *pUuid = reinterpret_cast<BtUuidArr_t *>(uuidMem);
	pUuid->BaseIdx = 0;
	pUuid->Type = BT_UUID_TYPE_16;
	pUuid->Count = 2;
	const uint16_t uuids[2] = { BT_UUID_GATT_SERVICE_DEVICE_INFORMATION,
								BT_UUID_GATT_SERVICE_BATTERY };
	std::memcpy(uuidMem + offsetof(BtUuidArr_t, Uuid16), uuids, sizeof(uuids));

	uint8_t data[31] = {};
	BtAdvPacket_t pkt = { static_cast<int>(sizeof(data)), 0, data };
	CHECK(BtAdvDataAddUuid(&pkt, pUuid, true));

	const BtAdvData_t *p = FindAd(pkt.pData, pkt.Len,
								  BT_GAP_DATA_TYPE_COMPLETE_SRVC_UUID16);
	CHECK(p != nullptr);
	if (p != nullptr)
	{
		CHECK(p->Hdr.Len == 5U);
		CHECK(p->Data[0] == 0x0A && p->Data[1] == 0x18);
		CHECK(p->Data[2] == 0x0F && p->Data[3] == 0x18);
	}
}

void TestLegacyEncode()
{
	BtAppCfg_t cfg = {};
	cfg.Role = BTAPP_ROLE_PERIPHERAL;
	cfg.pDevName = "IOsonata";
	cfg.VendorId = 0x1234;

	uint8_t advData[255] = {};
	uint8_t srData[255] = {};
	BtAdvPacket_t adv = { static_cast<int>(sizeof(advData)), 0, advData };
	BtAdvPacket_t sr = { static_cast<int>(sizeof(srData)), 0, srData };
	bool extended = true;
	bool scannable = true;

	CHECK(BtAdvEncode(&cfg, &adv, &sr, &extended, &scannable));
	CHECK(!extended);
	CHECK(!scannable);
	CHECK(adv.Len <= BT_ADV_LEGACY_DATA_MAX);
	CHECK(sr.Len == 0);
	CHECK(FindAd(adv.pData, adv.Len, BT_GAP_DATA_TYPE_FLAGS) != nullptr);

	char name[32] = {};
	CHECK(BtAdvDataGetDevName(adv.pData, adv.Len, name, sizeof(name)) == 8U);
	CHECK(std::strcmp(name, "IOsonata") == 0);
}

void TestScanResponseEncode()
{
	uint8_t scanMan[20];
	for (size_t i = 0; i < sizeof(scanMan); ++i)
	{
		scanMan[i] = static_cast<uint8_t>(i + 1U);
	}

	BtAppCfg_t cfg = {};
	cfg.Role = BTAPP_ROLE_PERIPHERAL;
	cfg.pDevName = "Tag";
	cfg.VendorId = 0x1234;
	cfg.pSrManData = scanMan;
	cfg.SrManDataLen = static_cast<int>(sizeof(scanMan));

	uint8_t advData[255] = {};
	uint8_t srData[255] = {};
	BtAdvPacket_t adv = { static_cast<int>(sizeof(advData)), 0, advData };
	BtAdvPacket_t sr = { static_cast<int>(sizeof(srData)), 0, srData };
	bool extended = true;
	bool scannable = false;

	CHECK(BtAdvEncode(&cfg, &adv, &sr, &extended, &scannable));
	CHECK(!extended);
	CHECK(scannable);
	CHECK(sr.Len > 0);

	uint8_t out[32] = {};
	size_t outLen = BtAdvDataGetManData(sr.pData, sr.Len, out, sizeof(out));
	CHECK(outLen == sizeof(scanMan) + 2U);
	CHECK(out[0] == 0x34 && out[1] == 0x12);
	CHECK(std::memcmp(&out[2], scanMan, sizeof(scanMan)) == 0);
}

void TestExtendedEncode()
{
	const char longName[] = "IOsonata Bluetooth host extended advertising";

	BtAppCfg_t cfg = {};
	cfg.Role = BTAPP_ROLE_PERIPHERAL;
	cfg.pDevName = longName;
	cfg.VendorId = 0x1234;

	uint8_t advData[255] = {};
	uint8_t srData[255] = {};
	BtAdvPacket_t adv = { static_cast<int>(sizeof(advData)), 0, advData };
	BtAdvPacket_t sr = { static_cast<int>(sizeof(srData)), 0, srData };
	bool extended = false;
	bool scannable = true;

	CHECK(BtAdvEncode(&cfg, &adv, &sr, &extended, &scannable));
	CHECK(extended);
	CHECK(!scannable);
	CHECK(adv.Len > BT_ADV_LEGACY_DATA_MAX);
	CHECK(sr.Len == 0);

	char name[80] = {};
	size_t nameLen = BtAdvDataGetDevName(adv.pData, adv.Len, name, sizeof(name));
	CHECK(nameLen == std::strlen(longName));
	CHECK(std::strcmp(name, longName) == 0);
}

} // namespace

void SetupEad()
{
	s_EadAes.Enable();
	s_EadRng.Enable();
	BtEadInit(&s_EadAes, &s_EadRng);
}

int main()
{
	SetupEad();
	TestExtendedSelection();
	TestNotArmedLeavesThePacketAlone();
	TestArmedWrapsThePacket();
	TestAReportIsDecrypted();
	TestEachWrapUsesAFreshRandomizer();
	TestNoRoomLeavesThePlaintext();
	TestDataMutation();
	TestNameEncoding();
	TestMalformedRecords();
	TestUuidRoundTrip();
	TestUuidNullInputs();
	TestUuidAdvertising();
	TestLegacyEncode();
	TestScanResponseEncode();
	TestExtendedEncode();

	if (s_Failures != 0)
	{
		std::printf("Bluetooth host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("Bluetooth host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
