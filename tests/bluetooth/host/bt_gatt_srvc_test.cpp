// Coverage for BtGattSrvcAdd against a real attribute database.
//
// Registering a service adds one attribute per declaration, value and
// descriptor. Any of those can fail when the pool runs out, and the earlier
// ones used to stay in the database with no owner: the handles were spent, the
// memory was gone, and the characteristics kept handles pointing at attributes
// no service claimed. A failed call now leaves the database exactly as it was.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_peer.h"

// bt_gatt.cpp and bt_att.cpp are both linked in, so only the layers outside
// them need stubbing.
extern "C" {

static BtDevice_t s_StubPeer;

BtDevice_t *BtPeerFindByHdl(uint16_t) { return nullptr; }
uint16_t BtPeerCount(void) { return 1; }
BtDevice_t *BtPeerSlot(uint16_t Idx) { return Idx == 0 ? &s_StubPeer : nullptr; }
size_t BtPeerGetConnectedHandles(uint16_t *, size_t) { return 0; }

void BtSmpBondCccdSave(uint16_t, uint16_t, uint16_t) {}
uint8_t BtSmpBondCccdGet(uint16_t, uint16_t *, uint16_t *, uint8_t) { return 0; }
bool BtSmpSignVerify(uint16_t, const uint8_t *, size_t, const uint8_t *) { return false; }
bool BtGapConnSecGet(uint16_t, BtConnSec_t *) { return false; }
void BtGattSrvcEvtHandler(BtGattSrvc_t * const, uint32_t, void * const) {}
uint32_t BtHciSendAcl(BtHciDevice_t * const, BtHciACLDataPacket_t * const) { return 0; }
void BtGattClientNotified(uint16_t, uint16_t, uint8_t *, uint16_t) {}

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

// Fill the database with entries of DataLen bytes until no more fit, so the
// service registration below is forced to fail part way through. Returns the
// number of filler entries added.
int FillUntilFull(int DataLen)
{
	BtUuid16_t uuid = { 0, BT_UUID_TYPE_16, 0xFEE0 };
	int n = 0;

	while (BtAttDBAddEntry(&uuid, DataLen) != nullptr)
	{
		++n;
	}

	return n;
}

// Services stay registered in a list that outlives any one test, so the objects
// have to outlive the function that adds them.
BtGattSrvc_t s_OkSrvc;
BtGattChar_t s_OkChars[2];
BtGattSrvc_t s_KeepSrvc;
BtGattChar_t s_KeepChars[1];
BtGattSrvc_t s_FailSrvc;
BtGattChar_t s_FailChars[4];

// Two characteristics, the second one notifiable so it needs a CCCD too.
void BuildSrvc(BtGattSrvc_t *pSrvc, BtGattChar_t *pChars, int NbChar,
			   uint16_t MaxDataLen)
{
	std::memset(pSrvc, 0, sizeof(*pSrvc));
	std::memset(pChars, 0, sizeof(pChars[0]) * (size_t)NbChar);

	for (int i = 0; i < NbChar; i++)
	{
		pChars[i].Uuid = (uint16_t)(0xFF00 + i);
		pChars[i].MaxDataLen = MaxDataLen;
		pChars[i].Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY;
	}

	pSrvc->UuidSrvc = 0xFE00;
	pSrvc->NbChar = NbChar;
	pSrvc->pCharArray = pChars;
}

// ---- a service that fits is registered whole ------------------------------

void TestSrvcAddSucceeds()
{
	BtAttDBInit(2048);

	BtGattSrvc_t &srvc = s_OkSrvc;
	BtGattChar_t *chars = s_OkChars;
	BuildSrvc(&srvc, chars, 2, 16);

	CHECK(BtGattSrvcAdd(&srvc));
	CHECK(srvc.Hdl != BT_ATT_HANDLE_INVALID);

	for (int i = 0; i < 2; i++)
	{
		CHECK(chars[i].ValHdl != BT_ATT_HANDLE_INVALID);
		CHECK(chars[i].CccdHdl != BT_ATT_HANDLE_INVALID);
		CHECK(chars[i].pValue != nullptr);
		CHECK(chars[i].pSrvc == &srvc);
		CHECK(BtAttDBFindHandle(chars[i].ValHdl) != nullptr);
		CHECK(BtAttDBFindHandle(chars[i].CccdHdl) != nullptr);
	}

	// A second call with the same service is a no-op, not a second copy.
	uint16_t hdl = srvc.Hdl;
	CHECK(BtGattSrvcAdd(&srvc));
	CHECK(srvc.Hdl == hdl);
}

// ---- a service that does not fit leaves nothing behind --------------------

void TestSrvcAddRollsBackOnFailure()
{
	BtAttDBInit(2048);

	// One small service that must survive the failed attempt untouched.
	BtGattSrvc_t &keep = s_KeepSrvc;
	BtGattChar_t *keepChars = s_KeepChars;
	BuildSrvc(&keep, keepChars, 1, 4);
	keep.UuidSrvc = 0xFD00;
	CHECK(BtGattSrvcAdd(&keep));

	uint16_t keepHdl = keep.Hdl;
	uint16_t keepValHdl = keepChars[0].ValHdl;
	uint16_t keepCccdHdl = keepChars[0].CccdHdl;

	// Leave room for a couple of attributes, far less than the service below
	// needs, so the registration fails part way through.
	int filler = FillUntilFull(200);
	CHECK(filler > 0);

	BtAttDBMark_t before;
	BtAttDBMark(&before);

	BtGattSrvc_t &srvc = s_FailSrvc;
	BtGattChar_t *chars = s_FailChars;
	BuildSrvc(&srvc, chars, 4, 256);

	CHECK(BtGattSrvcAdd(&srvc) == false);

	// The allocator is back where it was.
	BtAttDBMark_t after;
	BtAttDBMark(&after);
	CHECK(after.MemUsed == before.MemUsed);
	CHECK(after.LastHdl == before.LastHdl);

	// The failed service claims nothing.
	CHECK(srvc.Hdl == BT_ATT_HANDLE_INVALID);
	for (int i = 0; i < 4; i++)
	{
		CHECK(chars[i].ValHdl == BT_ATT_HANDLE_INVALID);
		CHECK(chars[i].CccdHdl == BT_ATT_HANDLE_INVALID);
		CHECK(chars[i].DescHdl == BT_ATT_HANDLE_INVALID);
		CHECK(chars[i].pValue == nullptr);
		CHECK(chars[i].pSrvc == nullptr);
	}

	// The service registered earlier is untouched and still reachable.
	CHECK(keep.Hdl == keepHdl);
	CHECK(keepChars[0].ValHdl == keepValHdl);
	CHECK(keepChars[0].CccdHdl == keepCccdHdl);
	CHECK(BtAttDBFindHandle(keepHdl) != nullptr);
	CHECK(BtAttDBFindHandle(keepValHdl) != nullptr);
	CHECK(BtAttDBFindHandle(keepCccdHdl) != nullptr);

	// And the freed room is usable again: the next handle is the one the
	// failed attempt would have taken, not one beyond the attributes it added.
	BtUuid16_t uuid = { 0, BT_UUID_TYPE_16, 0xFEE1 };
	BtAttDBEntry_t *e = BtAttDBAddEntry(&uuid, 4);
	CHECK(e != nullptr);
	if (e != nullptr)
	{
		CHECK(e->Hdl == (uint16_t)(before.LastHdl + 1));
		CHECK(BtAttDBFindHandle(e->Hdl) == e);
	}
}

// ---- the mark API on its own ----------------------------------------------

void TestMarkAndUnwind()
{
	BtAttDBInit(1024);

	BtUuid16_t uuid = { 0, BT_UUID_TYPE_16, 0xFEE2 };
	BtAttDBEntry_t *first = BtAttDBAddEntry(&uuid, 8);
	CHECK(first != nullptr);

	BtAttDBMark_t mark;
	BtAttDBMark(&mark);

	BtAttDBEntry_t *second = BtAttDBAddEntry(&uuid, 8);
	BtAttDBEntry_t *third = BtAttDBAddEntry(&uuid, 8);
	CHECK(second != nullptr);
	CHECK(third != nullptr);
	CHECK(BtAttDBFindHandle(3) == third);

	BtAttDBUnwind(&mark);

	// The dropped handles are gone and the list ends at the first entry.
	CHECK(BtAttDBFindHandle(2) == nullptr);
	CHECK(BtAttDBFindHandle(3) == nullptr);
	CHECK(BtAttDBFindHandle(1) == first);
	CHECK(first->pNext != nullptr);
	if (first->pNext != nullptr)
	{
		CHECK(first->pNext->pNext == nullptr);
		CHECK(first->pNext->pPrev == first);
	}

	// The next entry reuses the handle and the memory.
	BtAttDBEntry_t *again = BtAttDBAddEntry(&uuid, 8);
	CHECK(again == second);
	CHECK(again != nullptr && again->Hdl == 2);
	CHECK(BtAttDBFindHandle(2) == again);

	// A mark ahead of the current position is refused rather than acted on.
	BtAttDBMark_t ahead;
	BtAttDBMark(&ahead);
	ahead.MemUsed += 64;
	ahead.LastHdl += 4;
	BtAttDBUnwind(&ahead);
	CHECK(BtAttDBFindHandle(2) == again);
	CHECK(BtAttDBAddEntry(&uuid, 8) != nullptr);
	CHECK(BtAttDBFindHandle(3) != nullptr);

	// Unwinding to the start empties the database without breaking it.
	BtAttDBMark_t start;
	start.MemUsed = 0;
	start.LastHdl = 0;
	BtAttDBUnwind(&start);
	CHECK(BtAttDBFindHandle(1) == nullptr);
	CHECK(BtAttDBAddEntry(&uuid, 8) != nullptr);
	CHECK(BtAttDBFindHandle(1) != nullptr);

	BtAttDBUnwind(nullptr);
	CHECK(BtAttDBFindHandle(1) != nullptr);
}

} // namespace

int main()
{
	TestMarkAndUnwind();
	TestSrvcAddSucceeds();
	TestSrvcAddRollsBackOnFailure();

	if (s_Failures != 0)
	{
		std::printf("GATT service host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("GATT service host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
