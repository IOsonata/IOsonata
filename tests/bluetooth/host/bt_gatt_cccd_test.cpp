// Coverage for the GATT layer's Client Characteristic Configuration checks.
// A CCCD holds two defined bits, and each one is only meaningful when the
// characteristic declares the matching property (Vol 3 Part G 3.3.3.3). Bits
// 2..15 are reserved. BtGattCccdSet used to store whatever 16-bit value it was
// handed, so a client could subscribe to notifications on a characteristic that
// never sends any, and a reserved bit would be kept and mirrored back on a read.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_peer.h"

// Externals bt_gatt.cpp reaches for. Only the CCCD paths are driven here, so
// the peer pool is a single stub link and the rest are inert.
extern "C" {

static BtDevice_t s_StubPeer;

BtDevice_t *BtPeerFindByHdl(uint16_t Hdl)
{
	if (s_StubPeer.Conn.Hdl != Hdl)
	{
		return nullptr;
	}
	return &s_StubPeer;
}

uint16_t BtPeerCount(void) { return 1; }
BtDevice_t *BtPeerSlot(uint16_t Idx) { return Idx == 0 ? &s_StubPeer : nullptr; }
size_t BtPeerGetConnectedHandles(uint16_t *pHdl, size_t MaxCount)
{
	if (pHdl == nullptr || MaxCount < 1)
	{
		return 0;
	}
	pHdl[0] = s_StubPeer.Conn.Hdl;
	return 1;
}

void BtSmpBondCccdSave(uint16_t, uint16_t, uint16_t) {}
uint8_t BtSmpBondCccdGet(uint16_t, uint16_t *, uint16_t *, uint8_t) { return 0; }

BtAttDBEntry_t *BtAttDBAddEntry(BtUuid16_t *, int) { return nullptr; }
BtAttDBEntry_t *BtAttDBFindHandle(uint16_t) { return nullptr; }
void BtAttDBMark(BtAttDBMark_t *) {}
void BtAttDBUnwind(const BtAttDBMark_t *) {}
int BtUuidAddBase(uint8_t const [16]) { return 0; }
void BtGattSrvcEvtHandler(BtGattSrvc_t * const, uint32_t, void * const) {}
uint32_t BtHciSendAcl(BtHciDevice_t * const, BtHciACLDataPacket_t * const) { return 0; }

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

constexpr uint16_t kConnHdl   = 0x0040;
constexpr uint16_t kNotifyCccd = 0x0010;
constexpr uint16_t kIndicCccd  = 0x0020;
constexpr uint16_t kBothCccd   = 0x0030;
constexpr uint16_t kPlainCccd  = 0x0040;

BtGattChar_t s_Chars[4];
BtGattSrvc_t s_Srvc;

// One service with four characteristics: notify only, indicate only, both, and
// one with neither. Each has a CCCD handle so the lookup by handle resolves.
void Setup()
{
	std::memset(s_Chars, 0, sizeof(s_Chars));
	std::memset(&s_Srvc, 0, sizeof(s_Srvc));
	std::memset(&s_StubPeer, 0, sizeof(s_StubPeer));

	s_Chars[0].Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY;
	s_Chars[0].CccdHdl  = kNotifyCccd;
	s_Chars[1].Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_INDICATE;
	s_Chars[1].CccdHdl  = kIndicCccd;
	s_Chars[2].Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY |
						  BT_GATT_CHAR_PROP_INDICATE;
	s_Chars[2].CccdHdl  = kBothCccd;
	s_Chars[3].Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE;
	s_Chars[3].CccdHdl  = kPlainCccd;

	s_Srvc.NbChar     = 4;
	s_Srvc.pCharArray = s_Chars;
	BtGattInsertSrvcList(&s_Srvc);

	s_StubPeer.Conn.Hdl = kConnHdl;
	s_StubPeer.Conn.NbCccd = 0;
}

// ---- value checking -------------------------------------------------------

void TestValueErrorByProperty()
{
	// Disabling is always allowed.
	CHECK(BtGattCccdWriteError(kNotifyCccd, 0) == 0);
	CHECK(BtGattCccdWriteError(kIndicCccd, 0) == 0);
	CHECK(BtGattCccdWriteError(kPlainCccd, 0) == 0);

	// Each defined bit needs its property.
	CHECK(BtGattCccdWriteError(kNotifyCccd,
			BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) == 0);
	CHECK(BtGattCccdWriteError(kNotifyCccd,
			BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) == BT_ATT_ERROR_CCCD_IMPROPER_CFG);

	CHECK(BtGattCccdWriteError(kIndicCccd,
			BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) == 0);
	CHECK(BtGattCccdWriteError(kIndicCccd,
			BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) == BT_ATT_ERROR_CCCD_IMPROPER_CFG);

	// Both bits together need both properties.
	CHECK(BtGattCccdWriteError(kBothCccd,
			BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION |
			BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) == 0);
	CHECK(BtGattCccdWriteError(kNotifyCccd,
			BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION |
			BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) == BT_ATT_ERROR_CCCD_IMPROPER_CFG);

	// A characteristic with neither property accepts neither bit.
	CHECK(BtGattCccdWriteError(kPlainCccd,
			BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) == BT_ATT_ERROR_CCCD_IMPROPER_CFG);
	CHECK(BtGattCccdWriteError(kPlainCccd,
			BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) == BT_ATT_ERROR_CCCD_IMPROPER_CFG);
}

void TestValueErrorReservedBits()
{
	// Any of bits 2..15 makes the value one this version cannot honour, even
	// alongside a bit the characteristic does support.
	CHECK(BtGattCccdWriteError(kBothCccd, 0x0004) == BT_ATT_ERROR_CCCD_IMPROPER_CFG);
	CHECK(BtGattCccdWriteError(kBothCccd, 0x8000) == BT_ATT_ERROR_CCCD_IMPROPER_CFG);
	CHECK(BtGattCccdWriteError(kBothCccd, 0xFFFF) == BT_ATT_ERROR_CCCD_IMPROPER_CFG);
	CHECK(BtGattCccdWriteError(kBothCccd,
			BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION | 0x0100) ==
		  BT_ATT_ERROR_CCCD_IMPROPER_CFG);

	// A handle that belongs to no characteristic is an invalid handle, not a
	// configuration problem.
	CHECK(BtGattCccdWriteError(0x7777, 0) == BT_ATT_ERROR_INVALID_HANDLE);
	CHECK(BtGattCccdValueError(nullptr, 0) == BT_ATT_ERROR_INVALID_HANDLE);
}

// ---- the stored state follows the check -----------------------------------

void TestSetRejectsUnsupportedValue()
{
	s_StubPeer.Conn.NbCccd = 0;

	// A supported value is stored and readable.
	CHECK(BtGattCccdSet(kConnHdl, kNotifyCccd,
						BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION));
	CHECK(BtGattCccdGet(kConnHdl, kNotifyCccd) ==
		  BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);
	CHECK(s_Chars[0].bNotify);
	CHECK(s_Chars[0].bIndic == false);

	// An unsupported bit is refused and leaves the stored value alone.
	CHECK(BtGattCccdSet(kConnHdl, kNotifyCccd,
						BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) == false);
	CHECK(BtGattCccdGet(kConnHdl, kNotifyCccd) ==
		  BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);
	CHECK(s_Chars[0].bNotify);

	// A reserved bit is refused the same way.
	CHECK(BtGattCccdSet(kConnHdl, kNotifyCccd, 0x0040) == false);
	CHECK(BtGattCccdGet(kConnHdl, kNotifyCccd) ==
		  BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION);

	// Disabling always works and clears the aggregate.
	CHECK(BtGattCccdSet(kConnHdl, kNotifyCccd, 0));
	CHECK(BtGattCccdGet(kConnHdl, kNotifyCccd) == 0);
	CHECK(s_Chars[0].bNotify == false);

	// A characteristic that sends nothing cannot be subscribed to at all.
	CHECK(BtGattCccdSet(kConnHdl, kPlainCccd,
						BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) == false);
	CHECK(BtGattCccdGet(kConnHdl, kPlainCccd) == 0);
	CHECK(s_Chars[3].bNotify == false);

	// An unknown connection handle is still refused.
	CHECK(BtGattCccdSet(kConnHdl + 1, kNotifyCccd,
						BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) == false);
}

} // namespace

int main()
{
	Setup();

	TestValueErrorByProperty();
	TestValueErrorReservedBits();
	TestSetRejectsUnsupportedValue();

	if (s_Failures != 0)
	{
		std::printf("GATT CCCD host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("GATT CCCD host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
