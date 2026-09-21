// Deterministic interrupt-producer tests using the real AppEvt and CFifo.
// Only AppEvt's CFifoPeek/Get calls are redirected to the hooks below.
#include <stdio.h>

#include "app_evt_handler.h"
#include "cfifo.h"

#define EVENT_COUNT (APPEVT_HANDLER_QUE_DEFAULT_SIZE + 1U)

enum PreemptPoint {
	PREEMPT_NONE,
	PREEMPT_PEEK,
	PREEMPT_GET,
};

static PreemptPoint s_Preempt;
static bool s_Injected;
static bool s_Accepted;
static bool s_Requeue;
static unsigned s_SeenCount;
static unsigned s_Seen[EVENT_COUNT + 1U];
static unsigned s_Contexts[EVENT_COUNT + 1U];
static int s_Fail;

#define CHECK(c) do { if (!(c)) { \
	printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); s_Fail++; } } while (0)

static void Record(uint32_t Evt, void *pCtx)
{
	CHECK(Evt > 0U && Evt <= EVENT_COUNT);
	if (Evt > EVENT_COUNT)
	{
		return;
	}
	CHECK(pCtx == &s_Contexts[Evt]);
	CHECK(s_SeenCount < EVENT_COUNT + 1U);
	if (s_SeenCount < EVENT_COUNT + 1U)
	{
		s_Seen[s_SeenCount++] = Evt;
	}
}

static void InterruptHandler(uint32_t Evt, void *pCtx)
{
	CHECK(Evt == EVENT_COUNT);
	Record(Evt, pCtx);
}

static void NormalHandler(uint32_t Evt, void *pCtx)
{
	CHECK(Evt < EVENT_COUNT);
	Record(Evt, pCtx);
	if (s_Requeue)
	{
		s_Requeue = false;
		// The consumed slot must be available before the callback runs.
		CHECK(AppEvtHandlerQue(EVENT_COUNT, &s_Contexts[EVENT_COUNT],
			InterruptHandler));
	}
}

static void Preempt(PreemptPoint Point, uint8_t *pSlot)
{
	if (pSlot != nullptr && s_Preempt == Point)
	{
		s_Preempt = PREEMPT_NONE;
		s_Injected = true;
		s_Accepted = AppEvtHandlerQue(EVENT_COUNT,
			&s_Contexts[EVENT_COUNT], InterruptHandler);
	}
}

extern "C" uint8_t *PreemptCFifoPeek(hCFifo_t pFifo)
{
	uint8_t *p = CFifoPeek(pFifo);
	Preempt(PREEMPT_PEEK, p);
	return p;
}

extern "C" uint8_t *PreemptCFifoGet(hCFifo_t pFifo)
{
	uint8_t *p = CFifoGet(pFifo);
	// Run the producer immediately after the slot is released, before the
	// caller can copy it. A full queue reuses exactly this physical slot.
	Preempt(PREEMPT_GET, p);
	return p;
}

static void Drain(bool Exec)
{
	if (Exec)
	{
		AppEvtHandlerExec();
	}
	else
	{
		for (unsigned i = 0; i <= EVENT_COUNT; i++)
		{
			AppEvtHandlerDispatch();
		}
	}
}

static void TestPreemption(bool Exec, unsigned Head, PreemptPoint Point)
{
	CHECK(AppEvtHandlerInit(nullptr, 0));
	s_Preempt = PREEMPT_NONE;
	s_Requeue = false;
	for (unsigned i = 0; i < Head; i++)
	{
		s_SeenCount = 0;
		CHECK(AppEvtHandlerQue(1U, &s_Contexts[1], NormalHandler));
		AppEvtHandlerDispatch();
	}

	s_SeenCount = 0;
	s_Injected = false;
	s_Accepted = false;
	for (unsigned i = 1; i < EVENT_COUNT; i++)
	{
		CHECK(AppEvtHandlerQue(i, &s_Contexts[i], NormalHandler));
	}
	CHECK(!AppEvtHandlerQue(EVENT_COUNT, &s_Contexts[EVENT_COUNT],
		InterruptHandler));
	s_Preempt = Point;
	Drain(Exec);
	CHECK(s_Injected);
	if (Point == PREEMPT_PEEK)
	{
		// An IRQ cannot overwrite the head while the foreground owns it.
		CHECK(!s_Accepted);
		CHECK(AppEvtHandlerQue(EVENT_COUNT, &s_Contexts[EVENT_COUNT],
			InterruptHandler));
		Drain(Exec);
	}
	else
	{
		CHECK(s_Accepted);
	}
	CHECK(s_SeenCount == EVENT_COUNT);
	for (unsigned i = 0; i < s_SeenCount; i++)
	{
		if (s_Seen[i] != i + 1U)
		{
			printf("%s head %u: event %u was %u, expected %u\n",
				Exec ? "Exec" : "Dispatch", Head, i,
				s_Seen[i], i + 1U);
			s_Fail++;
		}
	}
	Drain(Exec);
	CHECK(s_SeenCount == EVENT_COUNT);
}

static void TestCallbackRequeue(bool Exec)
{
	CHECK(AppEvtHandlerInit(nullptr, 0));
	s_Preempt = PREEMPT_NONE;
	s_SeenCount = 0;
	Drain(Exec);
	CHECK(s_SeenCount == 0U);
	for (unsigned i = 1; i < EVENT_COUNT; i++)
	{
		CHECK(AppEvtHandlerQue(i, &s_Contexts[i], NormalHandler));
	}
	s_Requeue = true;
	Drain(Exec);
	CHECK(s_SeenCount == EVENT_COUNT);
	for (unsigned i = 0; i < s_SeenCount; i++)
	{
		CHECK(s_Seen[i] == i + 1U);
	}
}

int main()
{
	for (unsigned exec = 0; exec < 2; exec++)
	{
		for (unsigned head = 0; head < APPEVT_HANDLER_QUE_DEFAULT_SIZE; head++)
		{
			TestPreemption(exec, head, PREEMPT_GET);
			TestPreemption(exec, head, PREEMPT_PEEK);
		}
		TestCallbackRequeue(exec);
	}
	printf("AppEvt interrupt ownership: %s\n", s_Fail ? "FAIL" : "PASS");
	return s_Fail ? 1 : 0;
}
