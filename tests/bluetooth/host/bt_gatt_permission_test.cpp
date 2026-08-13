/**-------------------------------------------------------------------------
@file	bt_gatt_permission_test.cpp

@brief	Host tests for the Attribute Permissions BtGattSrvcAdd assigns.

		Core Vol 3 Part F 3.2.5: "Attribute permissions are a combination of
		access permissions, encryption permissions, authentication permissions
		and authorization permissions." Vol 3 Part G then fixes the permissions
		of the declarations and leaves the value and its descriptors to the
		service:

		  3.1     service declaration        read only, no authentication,
		                                     no authorization
		  3.3.1   characteristic declaration readable, no authentication,
		                                     no authorization
		  3.3.2   characteristic value       specified by the service
		  3.3.3.2 user description           specified by the profile
		  3.3.3.3 CCCD                       specified by the profile

		The declaration rows are the ones that would break discovery if this
		stack applied a service's security to every attribute in it, so they
		are tested under every security type, not only the open one.

@author	Hoang Nguyen Hoan
@date	Aug. 13, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
----------------------------------------------------------------------------*/
#include <cstring>

#include "bt_test_harness.h"

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_uuid.h"

// BtGattSrvcAdd resolves the device wide default security type from here when
// a service and its characteristics name none. The real definition lives in
// bt_app.cpp, which this test does not link.
BtAppData_t g_BtAppData;

namespace {

bttest::Context g_Ctx("bt_gatt_permission_test");

#define CHECK(expr)	BT_CHECK(g_Ctx, expr)

const uint16_t kConnHdl = 0x0010;

// Link security the ATT access path reads. Strong definition, so the weak one
// in bt_gap.cpp does not take part.
BtConnSec_t g_LinkSec;

uint8_t g_ValueStore[8];
uint8_t g_NotifyStore[8];

// BtGattSrvcAdd keeps registered services on a list and returns early when it
// is handed one that is already on it, and the list has no public way to be
// emptied. So every registration in this file uses a slot of its own. The
// slots left behind point into a database that has since been reinitialised,
// which nothing here walks.
const int kSrvcSlots = 16;
BtGattChar_t g_CharPool[kSrvcSlots][3];
BtGattSrvc_t g_SrvcPool[kSrvcSlots];
int g_SlotNext;

BtGattChar_t *g_Char;
BtGattSrvc_t *g_pSrvc;

// Read and write both, so the two directions can be told apart.
enum {
	CHAR_IDX_RW = 0,		// read and write
	CHAR_IDX_RO_NOTIFY = 1,	// read only, notifies, so it has a CCCD
	CHAR_IDX_WO = 2,		// write only
};

void BuildService(uint8_t AppSecType, uint8_t SrvcSecType, uint8_t CharSecType)
{
	BtAttDBInit(2048);

	g_pSrvc = &g_SrvcPool[g_SlotNext % kSrvcSlots];
	g_Char = g_CharPool[g_SlotNext % kSrvcSlots];
	g_SlotNext++;

	std::memset(g_pSrvc, 0, sizeof(*g_pSrvc));
	std::memset(g_Char, 0, sizeof(g_CharPool[0]));
	std::memset(g_ValueStore, 0, sizeof(g_ValueStore));
	std::memset(g_NotifyStore, 0, sizeof(g_NotifyStore));

	g_BtAppData.SecType = (BTGAP_SECTYPE)AppSecType;

	g_Char[CHAR_IDX_RW].Uuid = 0xFF01;
	g_Char[CHAR_IDX_RW].MaxDataLen = sizeof(g_ValueStore);
	g_Char[CHAR_IDX_RW].Property = BT_GATT_CHAR_PROP_READ |
								   BT_GATT_CHAR_PROP_WRITE;
	g_Char[CHAR_IDX_RW].pDesc = "rw";
	g_Char[CHAR_IDX_RW].SecType = CharSecType;

	g_Char[CHAR_IDX_RO_NOTIFY].Uuid = 0xFF02;
	g_Char[CHAR_IDX_RO_NOTIFY].MaxDataLen = sizeof(g_NotifyStore);
	g_Char[CHAR_IDX_RO_NOTIFY].Property = BT_GATT_CHAR_PROP_READ |
										  BT_GATT_CHAR_PROP_NOTIFY;

	g_Char[CHAR_IDX_WO].Uuid = 0xFF03;
	g_Char[CHAR_IDX_WO].MaxDataLen = sizeof(g_ValueStore);
	g_Char[CHAR_IDX_WO].Property = BT_GATT_CHAR_PROP_WRITE;

	g_pSrvc->bCustom = false;
	g_pSrvc->UuidSrvc = 0xFF00;
	g_pSrvc->NbChar = 3;
	g_pSrvc->pCharArray = g_Char;
	g_pSrvc->SecType = SrvcSecType;

	std::memset(&g_LinkSec, 0, sizeof(g_LinkSec));
	g_LinkSec.Level = BT_GAP_SEC_LEVEL_NONE;
}

bool Register(uint8_t AppSecType, uint8_t SrvcSecType = BT_GAP_SECTYPE_NONE,
			  uint8_t CharSecType = BT_GAP_SECTYPE_NONE)
{
	BuildService(AppSecType, SrvcSecType, CharSecType);
	return BtGattSrvcAdd(g_pSrvc);
}

uint32_t PermOf(uint16_t Hdl)
{
	return BtAttDBEntryGetPermission(BtAttDBFindHandle(Hdl));
}

// The characteristic declaration sits immediately before the value.
uint32_t CharDeclarPerm(const BtGattChar_t *pChar)
{
	return PermOf((uint16_t)(pChar->ValHdl - 1));
}

const uint32_t kSecMask =
		BT_ATT_PERMISSION_READ_ENCRYPT | BT_ATT_PERMISSION_WRITE_ENCRYPT |
		BT_ATT_PERMISSION_READ_AUTHEN | BT_ATT_PERMISSION_WRITE_AUTHEN |
		BT_ATT_PERMISSION_READ_AUTHOR | BT_ATT_PERMISSION_WRITE_AUTHOR |
		BT_ATT_PERMISSION_READ_KEY_SIZE | BT_ATT_PERMISSION_WRITE_KEY_SIZE;

const uint8_t kAllSecTypes[] = {
	BT_GAP_SECTYPE_NONE,
	BT_GAP_SECTYPE_OPEN,
	BT_GAP_SECTYPE_STATICKEY_NO_MITM,
	BT_GAP_SECTYPE_STATICKEY_MITM,
	BT_GAP_SECTYPE_LESC_MITM,
};

//-----------------------------------------------------------------------------
// The declarations, which the specification fixes.
//-----------------------------------------------------------------------------

// Vol 3 Part G 3.1 and 3.3.1. Under every security type this stack offers,
// including the strongest, both declarations stay readable and demand nothing.
// A client that cannot read these cannot discover the service at all.
void TestDeclarationsStayOpenUnderEverySecurityType()
{
	for (size_t i = 0; i < sizeof(kAllSecTypes); i++)
	{
		CHECK(Register(kAllSecTypes[i]));

		CHECK(PermOf(g_pSrvc->Hdl) == BT_ATT_PERMISSION_READ);

		for (int c = 0; c < g_pSrvc->NbChar; c++)
		{
			CHECK(CharDeclarPerm(&g_Char[c]) == BT_ATT_PERMISSION_READ);
		}
	}
}

// The same, driven from the service rather than the application default.
void TestServiceSecurityDoesNotReachTheDeclarations()
{
	CHECK(Register(BT_GAP_SECTYPE_NONE, BT_GAP_SECTYPE_LESC_MITM));

	CHECK((PermOf(g_pSrvc->Hdl) & kSecMask) == 0);
	CHECK((CharDeclarPerm(&g_Char[CHAR_IDX_RW]) & kSecMask) == 0);
}

//-----------------------------------------------------------------------------
// The value, which the service specifies.
//-----------------------------------------------------------------------------

// An open device leaves the value with access permissions only.
void TestOpenDeviceSetsAccessOnly()
{
	CHECK(Register(BT_GAP_SECTYPE_OPEN));

	CHECK(PermOf(g_Char[CHAR_IDX_RW].ValHdl) ==
		  (BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE));
}

// Unauthenticated pairing requires an encrypted link, both directions.
void TestStaticKeyNoMitmRequiresEncryption()
{
	CHECK(Register(BT_GAP_SECTYPE_STATICKEY_NO_MITM));

	CHECK(PermOf(g_Char[CHAR_IDX_RW].ValHdl) ==
		  (BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE |
		   BT_ATT_PERMISSION_READ_ENCRYPT | BT_ATT_PERMISSION_WRITE_ENCRYPT));
}

void TestStaticKeyMitmRequiresAuthentication()
{
	CHECK(Register(BT_GAP_SECTYPE_STATICKEY_MITM));

	CHECK(PermOf(g_Char[CHAR_IDX_RW].ValHdl) ==
		  (BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE |
		   BT_ATT_PERMISSION_READ_AUTHEN | BT_ATT_PERMISSION_WRITE_AUTHEN));
}

// Vol 3 Part C 10.2.1: level 4 is "Authenticated LE Secure Connections pairing
// with encryption using a 128-bit strength encryption key", so the key length
// belongs to it.
void TestLescMitmRequiresAuthenticationAndKeySize()
{
	CHECK(Register(BT_GAP_SECTYPE_LESC_MITM));

	CHECK(PermOf(g_Char[CHAR_IDX_RW].ValHdl) ==
		  (BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_WRITE |
		   BT_ATT_PERMISSION_READ_AUTHEN | BT_ATT_PERMISSION_WRITE_AUTHEN |
		   BT_ATT_PERMISSION_READ_KEY_SIZE | BT_ATT_PERMISSION_WRITE_KEY_SIZE));
}

// A direction the characteristic does not offer gets no permission bits at
// all, because it is refused before any security question is asked.
void TestUnofferedDirectionHasNoPermission()
{
	CHECK(Register(BT_GAP_SECTYPE_STATICKEY_NO_MITM));

	uint32_t ro = PermOf(g_Char[CHAR_IDX_RO_NOTIFY].ValHdl);
	CHECK((ro & BT_ATT_PERMISSION_READ) != 0);
	CHECK((ro & BT_ATT_PERMISSION_WRITE) == 0);
	CHECK((ro & BT_ATT_PERMISSION_WRITE_ENCRYPT) == 0);
	CHECK((ro & BT_ATT_PERMISSION_READ_ENCRYPT) != 0);

	uint32_t wo = PermOf(g_Char[CHAR_IDX_WO].ValHdl);
	CHECK((wo & BT_ATT_PERMISSION_WRITE) != 0);
	CHECK((wo & BT_ATT_PERMISSION_READ) == 0);
	CHECK((wo & BT_ATT_PERMISSION_READ_ENCRYPT) == 0);
	CHECK((wo & BT_ATT_PERMISSION_WRITE_ENCRYPT) != 0);
}

// A characteristic that names its own security wins over the service, and its
// neighbours are unaffected.
void TestCharacteristicSecurityOverridesTheService()
{
	CHECK(Register(BT_GAP_SECTYPE_NONE, BT_GAP_SECTYPE_STATICKEY_NO_MITM,
				   BT_GAP_SECTYPE_LESC_MITM));

	CHECK((PermOf(g_Char[CHAR_IDX_RW].ValHdl) &
		   BT_ATT_PERMISSION_READ_AUTHEN) != 0);
	CHECK((PermOf(g_Char[CHAR_IDX_RO_NOTIFY].ValHdl) &
		   BT_ATT_PERMISSION_READ_ENCRYPT) != 0);
	CHECK((PermOf(g_Char[CHAR_IDX_RO_NOTIFY].ValHdl) &
		   BT_ATT_PERMISSION_READ_AUTHEN) == 0);
}

//-----------------------------------------------------------------------------
// The descriptors, which the profile specifies.
//-----------------------------------------------------------------------------

// A read only characteristic that notifies still has a writable CCCD, and the
// write side has to demand the characteristic's security even though the value
// itself is never written. Taking the CCCD write permission from the value's
// write permission would leave it open, which is the trap this covers.
void TestCccdWriteRequiresSecurityOnAReadOnlyCharacteristic()
{
	CHECK(Register(BT_GAP_SECTYPE_STATICKEY_MITM));

	uint32_t cccd = PermOf(g_Char[CHAR_IDX_RO_NOTIFY].CccdHdl);
	CHECK((cccd & BT_ATT_PERMISSION_WRITE) != 0);
	CHECK((cccd & BT_ATT_PERMISSION_WRITE_AUTHEN) != 0);
	// Readable without meeting anything, so a client can see its own
	// subscription state.
	CHECK((cccd & BT_ATT_PERMISSION_READ) != 0);
	CHECK((cccd & BT_ATT_PERMISSION_READ_AUTHEN) == 0);
	CHECK((cccd & BT_ATT_PERMISSION_READ_ENCRYPT) == 0);
}

void TestUserDescriptionIsReadOnlyAndOpen()
{
	CHECK(Register(BT_GAP_SECTYPE_LESC_MITM));

	CHECK(PermOf(g_Char[CHAR_IDX_RW].DescHdl) == BT_ATT_PERMISSION_READ);
}


//-----------------------------------------------------------------------------
// Entry reuse.
//-----------------------------------------------------------------------------

// A service registration that fails partway unwinds the allocator, which
// rewinds without clearing. The next entry to take that memory has to start
// with no permissions, or it inherits whatever the attempt that did not make
// it had asked for.
void TestUnwoundEntryStartsWithNoPermission()
{
	BtAttDBInit(2048);

	BtUuid16_t uuid = {0, BT_UUID_TYPE_16, 0xFF10};
	BtAttDBMark_t mark;
	BtAttDBMark(&mark);

	BtAttDBEntry_t *first = BtAttDBAddEntry(&uuid, 8);
	CHECK(first != nullptr);
	BtAttDBEntrySetPermission(first,
		BT_ATT_PERMISSION_READ | BT_ATT_PERMISSION_READ_AUTHEN |
		BT_ATT_PERMISSION_WRITE | BT_ATT_PERMISSION_WRITE_AUTHEN);
	CHECK(BtAttDBEntryGetPermission(first) != 0);

	BtAttDBUnwind(&mark);

	uuid.Uuid = 0xFF11;
	BtAttDBEntry_t *second = BtAttDBAddEntry(&uuid, 8);
	CHECK(second != nullptr);
	// Same memory, so this is the case that matters.
	CHECK(second == first);
	CHECK(BtAttDBEntryGetPermission(second) == 0);
}

} // namespace

// Link security the ATT access path reads back. Strong, so the weak default in
// bt_gap.cpp is not linked in.
bool BtGapConnSecGet(uint16_t ConnHdl, BtConnSec_t *pSec)
{
	if (ConnHdl != kConnHdl || pSec == nullptr)
	{
		return false;
	}
	*pSec = g_LinkSec;
	return true;
}

int main(void)
{
	g_Ctx.Run("declarations stay open under every security type",
			  TestDeclarationsStayOpenUnderEverySecurityType);
	g_Ctx.Run("service security does not reach the declarations",
			  TestServiceSecurityDoesNotReachTheDeclarations);
	g_Ctx.Run("open device sets access only", TestOpenDeviceSetsAccessOnly);
	g_Ctx.Run("static key no MITM requires encryption",
			  TestStaticKeyNoMitmRequiresEncryption);
	g_Ctx.Run("static key MITM requires authentication",
			  TestStaticKeyMitmRequiresAuthentication);
	g_Ctx.Run("LESC MITM requires authentication and key size",
			  TestLescMitmRequiresAuthenticationAndKeySize);
	g_Ctx.Run("unoffered direction has no permission",
			  TestUnofferedDirectionHasNoPermission);
	g_Ctx.Run("characteristic security overrides the service",
			  TestCharacteristicSecurityOverridesTheService);
	g_Ctx.Run("CCCD write requires security on a read only characteristic",
			  TestCccdWriteRequiresSecurityOnAReadOnlyCharacteristic);
	g_Ctx.Run("user description is read only and open",
			  TestUserDescriptionIsReadOnlyAndOpen);
	g_Ctx.Run("unwound entry starts with no permission",
			  TestUnwoundEntryStartsWithNoPermission);

	return g_Ctx.Finish();
}
