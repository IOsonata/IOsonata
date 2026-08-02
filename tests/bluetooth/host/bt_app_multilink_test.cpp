// Coverage for the per-link application send API in src/bluetooth/bt_app.cpp.
//
// Notify, indicate and disconnect used to read one active connection handle
// when the caller did not name a link, which quietly picked a link once more
// than one was up. The no-handle forms are now defined only while exactly one
// link is up and refuse otherwise; the Conn forms name a link; the All forms
// walk every connected one and report how many accepted.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_peer.h"

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

constexpr size_t kMaxCall = 16;

// Links the stub peer pool reports as connected.
uint16_t s_Links[8] = {};
size_t   s_LinkCount = 0;

// Handle the stub send refuses, standing in for a client that did not
// subscribe or a link with an indication already outstanding. 0 accepts all.
uint16_t s_RefuseHdl = 0;

uint16_t s_NotifyHdl[kMaxCall] = {};
size_t   s_NotifyCount = 0;
uint16_t s_IndicateHdl[kMaxCall] = {};
size_t   s_IndicateCount = 0;
uint16_t s_DisconnectHdl[kMaxCall] = {};
size_t   s_DisconnectCount = 0;
int      s_SetValueCount = 0;

void Reset()
{
	s_RefuseHdl = 0;
	s_NotifyCount = 0;
	s_IndicateCount = 0;
	s_DisconnectCount = 0;
	s_SetValueCount = 0;
	std::memset(s_NotifyHdl, 0, sizeof(s_NotifyHdl));
	std::memset(s_IndicateHdl, 0, sizeof(s_IndicateHdl));
	std::memset(s_DisconnectHdl, 0, sizeof(s_DisconnectHdl));
}

void SetLinks(const uint16_t *pHdl, size_t Count)
{
	s_LinkCount = Count > (sizeof(s_Links) / sizeof(s_Links[0]))
				  ? (sizeof(s_Links) / sizeof(s_Links[0])) : Count;
	for (size_t i = 0; i < s_LinkCount; i++)
	{
		s_Links[i] = pHdl[i];
	}
}

} // namespace

// The layers below bt_app.cpp, stubbed so the walk over links is what is
// under test rather than the GATT send itself.
extern "C" {

size_t BtPeerGetConnectedHandles(uint16_t *pHdl, size_t MaxCount)
{
	size_t n = s_LinkCount < MaxCount ? s_LinkCount : MaxCount;
	for (size_t i = 0; i < n; i++)
	{
		pHdl[i] = s_Links[i];
	}
	return n;
}

BtDevice_t *BtPeerGetActive(void) { return nullptr; }

bool BtGattCharSetValue(BtGattChar_t *, void * const, size_t)
{
	++s_SetValueCount;
	return true;
}

bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *, void * const, size_t)
{
	if (s_NotifyCount < kMaxCall)
	{
		s_NotifyHdl[s_NotifyCount++] = ConnHdl;
	}
	return ConnHdl != s_RefuseHdl;
}

bool BtGattCharIndicate(uint16_t ConnHdl, BtGattChar_t *, void * const, size_t)
{
	if (s_IndicateCount < kMaxCall)
	{
		s_IndicateHdl[s_IndicateCount++] = ConnHdl;
	}
	return ConnHdl != s_RefuseHdl;
}

void BtAppDisconnectConn(uint16_t ConnHdl)
{
	if (s_DisconnectCount < kMaxCall)
	{
		s_DisconnectHdl[s_DisconnectCount++] = ConnHdl;
	}
}

} // extern "C"

namespace {

BtGattChar_t s_Char;
uint8_t s_Value[4] = { 1, 2, 3, 4 };

// ---- one link is named, not inferred --------------------------------------

void TestNotifyTargetsNamedLink()
{
	const uint16_t links[] = { 0x0010, 0x0020, 0x0030 };
	SetLinks(links, 3);
	Reset();

	CHECK(BtAppNotifyConn(0x0020, &s_Char, s_Value, sizeof(s_Value)));
	CHECK(s_NotifyCount == 1);
	CHECK(s_NotifyHdl[0] == 0x0020);

	// A handle that is not in the pool is still passed through: whether the
	// link exists is the sending layer's call, not this one's.
	Reset();
	CHECK(BtAppNotifyConn(0x0099, &s_Char, s_Value, sizeof(s_Value)));
	CHECK(s_NotifyCount == 1);
	CHECK(s_NotifyHdl[0] == 0x0099);

	// Indicate names its link the same way.
	Reset();
	CHECK(BtAppIndicateConn(0x0030, &s_Char, s_Value, sizeof(s_Value)));
	CHECK(s_IndicateCount == 1);
	CHECK(s_IndicateHdl[0] == 0x0030);
	CHECK(s_NotifyCount == 0);
}

// ---- the no-handle forms mean the one link, and only that ------------------

// Code written for a single link keeps working unchanged. With no link, and
// with several, there is no single link to mean, so the call refuses rather
// than picking one. That refusal is the whole point: the old behaviour was to
// pick whichever link was current.
void TestSoleLinkForms()
{
	// Exactly one link: the send goes to it without naming it.
	const uint16_t one[] = { 0x0021 };
	SetLinks(one, 1);
	Reset();

	CHECK(BtAppNotify(&s_Char, s_Value, sizeof(s_Value)));
	CHECK(s_NotifyCount == 1);
	CHECK(s_NotifyHdl[0] == 0x0021);

	Reset();
	CHECK(BtAppIndicate(&s_Char, s_Value, sizeof(s_Value)));
	CHECK(s_IndicateCount == 1);
	CHECK(s_IndicateHdl[0] == 0x0021);

	Reset();
	BtAppDisconnect();
	CHECK(s_DisconnectCount == 1);
	CHECK(s_DisconnectHdl[0] == 0x0021);

	// That one link refusing is reported as a failed send, not as no link.
	Reset();
	s_RefuseHdl = 0x0021;
	CHECK(BtAppNotify(&s_Char, s_Value, sizeof(s_Value)) == false);
	CHECK(s_NotifyCount == 1);

	// No link: nothing is sent.
	SetLinks(nullptr, 0);
	Reset();
	CHECK(BtAppNotify(&s_Char, s_Value, sizeof(s_Value)) == false);
	CHECK(BtAppIndicate(&s_Char, s_Value, sizeof(s_Value)) == false);
	BtAppDisconnect();
	CHECK(s_NotifyCount == 0);
	CHECK(s_IndicateCount == 0);
	CHECK(s_DisconnectCount == 0);

	// Two links: no single link to mean, so nothing is sent and no link is
	// picked. A caller that has to reach both uses the Conn or All form.
	const uint16_t two[] = { 0x0021, 0x0022 };
	SetLinks(two, 2);
	Reset();
	CHECK(BtAppNotify(&s_Char, s_Value, sizeof(s_Value)) == false);
	CHECK(BtAppIndicate(&s_Char, s_Value, sizeof(s_Value)) == false);
	BtAppDisconnect();
	CHECK(s_NotifyCount == 0);
	CHECK(s_IndicateCount == 0);
	CHECK(s_DisconnectCount == 0);
}

void TestSendRejectsBadArguments()
{
	const uint16_t links[] = { 0x0010 };
	SetLinks(links, 1);
	Reset();

	CHECK(BtAppNotifyConn(0x0010, nullptr, s_Value, sizeof(s_Value)) == false);
	CHECK(BtAppIndicateConn(0x0010, nullptr, s_Value, sizeof(s_Value)) == false);
	CHECK(BtAppNotify(nullptr, s_Value, sizeof(s_Value)) == false);
	// A length with no data is refused; a zero length with no data is fine.
	CHECK(BtAppNotifyConn(0x0010, &s_Char, nullptr, 4) == false);
	CHECK(s_NotifyCount == 0);

	CHECK(BtAppNotifyConn(0x0010, &s_Char, nullptr, 0));
	CHECK(s_NotifyCount == 1);

	Reset();
	CHECK(BtAppNotifyAll(nullptr, s_Value, sizeof(s_Value)) == 0);
	CHECK(BtAppIndicateAll(nullptr, s_Value, sizeof(s_Value)) == 0);
	CHECK(s_NotifyCount == 0);
	CHECK(s_IndicateCount == 0);
}

// ---- the All forms cover every link ---------------------------------------

void TestNotifyAllWalksEveryLink()
{
	const uint16_t links[] = { 0x0010, 0x0020, 0x0030 };
	SetLinks(links, 3);
	Reset();

	CHECK(BtAppNotifyAll(&s_Char, s_Value, sizeof(s_Value)) == 3);
	CHECK(s_NotifyCount == 3);
	CHECK(s_NotifyHdl[0] == 0x0010);
	CHECK(s_NotifyHdl[1] == 0x0020);
	CHECK(s_NotifyHdl[2] == 0x0030);

	// The value is stored once for the whole broadcast, not once per link.
	CHECK(s_SetValueCount == 1);

	// A link that refuses is still visited but not counted.
	Reset();
	s_RefuseHdl = 0x0020;
	CHECK(BtAppNotifyAll(&s_Char, s_Value, sizeof(s_Value)) == 2);
	CHECK(s_NotifyCount == 3);

	// Indicate behaves the same way.
	Reset();
	s_RefuseHdl = 0x0010;
	CHECK(BtAppIndicateAll(&s_Char, s_Value, sizeof(s_Value)) == 2);
	CHECK(s_IndicateCount == 3);
	CHECK(s_IndicateHdl[0] == 0x0010);
	CHECK(s_SetValueCount == 1);
}

void TestAllFormsWithNoLinks()
{
	SetLinks(nullptr, 0);
	Reset();

	CHECK(BtAppNotifyAll(&s_Char, s_Value, sizeof(s_Value)) == 0);
	CHECK(BtAppIndicateAll(&s_Char, s_Value, sizeof(s_Value)) == 0);
	CHECK(BtAppDisconnectAll() == 0);
	CHECK(s_NotifyCount == 0);
	CHECK(s_IndicateCount == 0);
	CHECK(s_DisconnectCount == 0);
}

void TestDisconnectAllCoversEveryLink()
{
	const uint16_t links[] = { 0x0041, 0x0042 };
	SetLinks(links, 2);
	Reset();

	CHECK(BtAppDisconnectAll() == 2);
	CHECK(s_DisconnectCount == 2);
	CHECK(s_DisconnectHdl[0] == 0x0041);
	CHECK(s_DisconnectHdl[1] == 0x0042);
}

} // namespace

int main()
{
	std::memset(&s_Char, 0, sizeof(s_Char));

	TestNotifyTargetsNamedLink();
	TestSoleLinkForms();
	TestSendRejectsBadArguments();
	TestNotifyAllWalksEveryLink();
	TestAllFormsWithNoLinks();
	TestDisconnectAllCoversEveryLink();

	if (s_Failures != 0)
	{
		std::printf("App multi-link host tests: %d failure(s), %d checks\n",
					s_Failures, s_Checks);
		return 1;
	}

	std::printf("App multi-link host tests: PASS (%d checks)\n", s_Checks);
	return 0;
}
