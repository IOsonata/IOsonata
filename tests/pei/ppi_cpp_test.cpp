#include <cassert>
#include <cstring>

#include "nrf.h"
#include "coredev/pei.h"

extern "C" {
NRF_PPI_Type g_TestPpi;
int g_critical_depth;
}

static int CountRoutes()
{
	int i;
	int count = 0;

	for (i = 0; i < TEST_PPI_CHANNEL_COUNT; i++)
	{
		if (g_TestPpi.CH[i].EEP != 0U)
		{
			count++;
		}
	}
	return count;
}

int main()
{
	PeiEvent_t evt1 = 0U;
	PeiEvent_t evt2 = 0U;
	PeiEvent_t stale;
	const PeiEndpoint_t event1 = 0x40008100U;
	const PeiEndpoint_t event2 = 0x40009100U;
	const PeiEndpoint_t replacement = 0x4000A100U;
	const PeiEndpoint_t task1 = 0x40006000U;
	const PeiEndpoint_t task2 = 0x40007000U;
	const PeiEndpoint_t task3 = 0x40008000U;

	std::memset(&g_TestPpi, 0, sizeof(g_TestPpi));
	{
		Pei pei;
		assert(pei.BindEvent(evt1, event1));
		assert(evt1 != 0U);
		assert(pei.Connect(evt1, task1));
		assert(pei.Connect(evt1, task2));
		assert(pei.Connect(evt1, task2));
		assert(pei.Enable(evt1));

		assert(pei.BindEvent(evt2, event2));
		assert(evt2 != evt1);
		assert(pei.Connect(evt2, task2));
		assert(pei.Connect(evt2, task3));
		assert(pei.Enable(evt2));
		assert(CountRoutes() == 4);

		assert(pei.Disable(evt1));
		assert(CountRoutes() == 4);
		assert(pei.UnbindEvent(evt1, event1));
		assert(CountRoutes() == 2);
		assert(!pei.UnbindEvent(evt1, event1));
		assert(!pei.Enable(evt1));

		// Replacing the event source retains both of its task links.
		assert(pei.BindEvent(evt1, replacement));
		assert(CountRoutes() == 4);
		assert(pei.Enable(evt1));
		assert(pei.Disconnect(evt2, task2));
		assert(CountRoutes() == 3);
		assert(pei.Disconnect(evt1, task2));
		assert(CountRoutes() == 2);

		stale = evt1;
		assert(pei.ReleaseEvent(evt1));
		assert(evt1 == 0U);
		assert(!pei.Enable(stale));
		assert(!pei.Connect(stale, task2));
		assert(CountRoutes() == 1);
		// Destructor releases the remaining event.
	}
	assert(CountRoutes() == 0);
	assert(g_critical_depth == 0);
	return 0;
}
