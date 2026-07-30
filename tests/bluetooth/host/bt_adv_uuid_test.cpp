#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

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

void TestUuidAdvertising()
{
    alignas(BtUuidArr_t) uint8_t uuidMem[sizeof(BtUuidArr_t) + sizeof(uint16_t)] = {};
    BtUuidArr_t *pUuid = reinterpret_cast<BtUuidArr_t *>(uuidMem);
    pUuid->BaseIdx = 0;
    pUuid->Type = BT_UUID_TYPE_16;
    pUuid->Count = 2;
    pUuid->Uuid16[0] = BT_UUID_GATT_SERVICE_DEVICE_INFORMATION;
    pUuid->Uuid16[1] = BT_UUID_GATT_SERVICE_BATTERY;

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

int main()
{
    TestExtendedSelection();
    TestDataMutation();
    TestNameEncoding();
    TestMalformedRecords();
    TestUuidRoundTrip();
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
