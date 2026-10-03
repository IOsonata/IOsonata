// Strong hook linked with the production AppEvt queue and CFifo.
#include <assert.h>
#include <stdio.h>
#include "cfifo.h"
#include "app_evt_handler.h"

static unsigned wakes, seen, context;
static bool dispatchOnWake;

void AppEvtHandlerNotify(void)
{
	++wakes;
	// Model a consumer scheduled immediately after notification.
	if (dispatchOnWake)
		AppEvtHandlerExec();
}

static void Record(uint32_t event, void *ctx)
{
	assert(event == 123 && ctx == &context);
	++seen;
}

int main()
{
	assert(AppEvtHandlerInit(nullptr, 0));
	assert(!AppEvtHandlerQue(123, &context, nullptr));
	assert(wakes == 0);
	for (unsigned i = 0; i < APPEVT_HANDLER_QUE_DEFAULT_SIZE; ++i)
	{
		assert(AppEvtHandlerQue(123, &context, Record));
		assert(wakes == i + 1);
	}
	assert(!AppEvtHandlerQue(123, &context, Record));
	assert(wakes == APPEVT_HANDLER_QUE_DEFAULT_SIZE);
	AppEvtHandlerExec();
	assert(seen == wakes);
	dispatchOnWake = true;
	assert(AppEvtHandlerQue(123, &context, Record));
	assert(seen == wakes); // Consumer sees the complete payload.
	dispatchOnWake = false;

	alignas(4) static uint8_t memory[APPEVT_HANDLER_QUE_MEMSIZE(APPEVT_HANDLER_EXEC_MAX_COUNT + 3)];
	assert(AppEvtHandlerInit(memory, sizeof(memory)));
	for (unsigned i = 0; i < APPEVT_HANDLER_EXEC_MAX_COUNT + 3; ++i)
		assert(AppEvtHandlerQue(123, &context, Record));
	const unsigned before = seen, beforeWake = wakes;
	AppEvtHandlerExec();
	assert(seen == before + APPEVT_HANDLER_EXEC_MAX_COUNT);
	assert(wakes == beforeWake); // Dispatch has no notification policy.
	AppEvtHandlerDispatch();
	AppEvtHandlerExec();
	assert(seen == before + APPEVT_HANDLER_EXEC_MAX_COUNT + 3);
	assert(wakes == beforeWake);
	puts("AppEvt weak/strong hook: successful publication only, no dispatch policy: PASS");
}
