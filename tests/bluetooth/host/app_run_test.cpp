// Exercise the actual AppRun loop, returning only through the test AppWait.
#include <setjmp.h>

#include "app_evt_handler.h"
#include "bt_test_harness.h"

static bttest::Context s_Test("Application status checks");
static jmp_buf s_Wait;
static bool s_Owed;
#if !defined(APP_STATUS_NO_SUBSYSTEM)
static int s_NormalCount;
static int s_NormalQueued;
static const int s_NormalTarget = APPEVT_HANDLER_EXEC_MAX_COUNT + 5;
static int s_DeferredCount;
static int s_StatusCount;
#if !defined(APP_STATUS_OVERRIDE)
static int s_BtStatusCount;
#endif
#endif

#if !defined(APP_STATUS_NO_SUBSYSTEM)
static void DeferredHandler(uint32_t, void *)
{
	s_DeferredCount++;
}

static void NormalHandler(uint32_t, void *)
{
	s_NormalCount++;
	if (s_NormalQueued < s_NormalTarget)
	{
		BT_CHECK(s_Test, AppEvtHandlerQue(0, nullptr, NormalHandler));
		s_NormalQueued++;
	}
}

static void CheckStatus(void)
{
	s_StatusCount++;
	if (s_Owed)
	{
		// More than one bounded dispatch pass is needed. The status hook
		// must wait until the queue is empty.
		BT_CHECK(s_Test, s_NormalCount == s_NormalTarget);
		s_Owed = !AppEvtHandlerQue(0, nullptr, DeferredHandler);
	}
}
#endif

#if defined(APP_STATUS_OVERRIDE)
bool AppCheckStatus(void)
{
	CheckStatus();
	// First check reports idle after queuing work: AppRun must recheck the
	// queue. Second check is busy even though the queue is empty.
	return s_StatusCount != 2;
}
#elif !defined(APP_STATUS_NO_SUBSYSTEM)
extern "C" void UsbCheckStatus(void)
{
	CheckStatus();
}

extern "C" void BtAppCheckStatus(void)
{
	s_BtStatusCount++;
}
#endif

void AppWait(void)
{
	BT_CHECK(s_Test, !AppEvtHandlerPending());
	BT_CHECK(s_Test, !s_Owed);
#if !defined(APP_STATUS_NO_SUBSYSTEM)
	// Status checking queued this work. It must finish before the first wait.
	BT_CHECK(s_Test, s_DeferredCount == 1);
#if defined(APP_STATUS_OVERRIDE)
	BT_CHECK(s_Test, s_StatusCount == 3);
#else
	BT_CHECK(s_Test, s_StatusCount == 2);
	BT_CHECK(s_Test, s_BtStatusCount == 2);
#endif
#endif
	longjmp(s_Wait, 1);
}

int main(void)
{
	BT_CHECK(s_Test, AppEvtHandlerInit(nullptr, 0));
	BT_CHECK(s_Test, !AppEvtHandlerPending());
#if !defined(APP_STATUS_NO_SUBSYSTEM)
	for (int i = 0; i < APPEVT_HANDLER_QUE_DEFAULT_SIZE; i++)
	{
		BT_CHECK(s_Test, AppEvtHandlerQue(0, nullptr, NormalHandler));
		s_NormalQueued++;
	}
	s_Owed = !AppEvtHandlerQue(0, nullptr, DeferredHandler);
	BT_CHECK(s_Test, s_Owed);
#endif
	if (setjmp(s_Wait) == 0)
	{
		AppRun();
	}
	return s_Test.Finish();
}
