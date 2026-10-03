#include <cstddef>
#include <cstdint>

#include "app_evt_handler.h"
#include "bt_test_harness.h"

namespace {

bttest::Context s_Test("Application event retry tests");
int s_NormalCount;
int s_DeferredCount;

void NormalHandler(uint32_t, void *)
{
	s_NormalCount++;
}

void DeferredHandler(uint32_t, void *)
{
	s_DeferredCount++;
}

// A full queue refuses the event. The caller keeps its work marked and
// queues it again after the main loop has run the queue.
void TestFullQueueRetriesAfterDrain()
{
	BT_CHECK(s_Test, AppEvtHandlerInit(nullptr, 0));

	s_NormalCount = 0;
	s_DeferredCount = 0;

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

	BT_CHECK(s_Test, AppEvtHandlerQue(0, nullptr, DeferredHandler));
	AppEvtHandlerExec();
	BT_CHECK(s_Test, s_DeferredCount == 1);

	// Nothing queued: nothing runs.
	AppEvtHandlerExec();
	BT_CHECK(s_Test, s_NormalCount == queued);
	BT_CHECK(s_Test, s_DeferredCount == 1);
}

} // namespace

int main()
{
	s_Test.Run("full queue retries after drain", TestFullQueueRetriesAfterDrain);
	return s_Test.Finish();
}
