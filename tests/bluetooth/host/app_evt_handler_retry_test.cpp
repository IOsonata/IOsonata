#include <cstddef>
#include <cstdint>

#include "app_evt_handler.h"
#include "bt_test_harness.h"

namespace {

bttest::Context s_Test("Application event retry tests");
int s_NormalCount;
int s_DeferredCount;
int s_IdleCount;
bool s_DeferredPending;
bool s_DeferredQueued;

void NormalHandler(uint32_t, void *)
{
	s_NormalCount++;
}

void DeferredHandler(uint32_t, void *)
{
	s_DeferredQueued = false;
	s_DeferredPending = false;
	s_DeferredCount++;
}

void IdlePump()
{
	s_IdleCount++;
	if (!s_DeferredPending || s_DeferredQueued)
	{
		return;
	}

	if (AppEvtHandlerQue(0, nullptr, DeferredHandler))
	{
		s_DeferredQueued = true;
	}
}

void TestFullQueueRetriesAfterDrain()
{
	BT_CHECK(s_Test, AppEvtHandlerInit(nullptr, 0));
	BT_CHECK(s_Test, AppEvtHandlerIdleRegister(IdlePump));
	BT_CHECK(s_Test, AppEvtHandlerIdleRegister(IdlePump));
	BT_CHECK(s_Test, !AppEvtHandlerIdleRegister(nullptr));

	s_NormalCount = 0;
	s_DeferredCount = 0;
	s_IdleCount = 0;
	s_DeferredPending = true;
	s_DeferredQueued = false;

	int queued = 0;
	while (AppEvtHandlerQue((uint32_t)queued, nullptr, NormalHandler))
	{
		queued++;
		BT_CHECK(s_Test, queued <= APPEVT_HANDLER_QUE_DEFAULT_SIZE);
	}
	BT_CHECK(s_Test, queued > 0);
	BT_CHECK(s_Test, !AppEvtHandlerQue(0, nullptr, DeferredHandler));

	AppEvtHandlerExec();
	BT_CHECK(s_Test, s_NormalCount == queued);
	BT_CHECK(s_Test, s_DeferredCount == 0);
	BT_CHECK(s_Test, s_DeferredQueued);
	BT_CHECK(s_Test, s_IdleCount == 1);

	AppEvtHandlerExec();
	BT_CHECK(s_Test, s_DeferredCount == 1);
	BT_CHECK(s_Test, !s_DeferredPending);
	BT_CHECK(s_Test, !s_DeferredQueued);
	BT_CHECK(s_Test, s_IdleCount == 2);
}

} // namespace

int main()
{
	s_Test.Run("full queue retries after drain", TestFullQueueRetriesAfterDrain);
	return s_Test.Finish();
}
