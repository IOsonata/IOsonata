#include <cassert>
#include <cstring>
#include <sys/mman.h>

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

	// Software event through the C++ wrapper, on EGU registers mapped at
	// their nRF52 addresses
	void *egu = mmap((void *)TEST_EGU_BASE, 0x1000, PROT_READ | PROT_WRITE,
					 MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
	assert(egu == (void *)TEST_EGU_BASE);
	{
		Pei pei;
		PeiEvent_t sw = 0U;

		assert(pei.BindSwEvent(sw));
		assert(pei.EventEndpoint(sw) == TEST_EGU_BASE + 0x100U);
		assert(pei.SwEventTask(sw) == TEST_EGU_BASE);
		assert(pei.Connect(sw, task1));
		assert(pei.Connect(sw, task2));
		assert(pei.Enable(sw));
		assert(CountRoutes() == 2);
		assert(pei.Trigger(sw));
		assert(NRF_EGU0->TASKS_TRIGGER[0] == 1U);
		assert(!pei.Trigger(evt2));
		// Destructor releases the event and its EGU channel.
	}
	assert(CountRoutes() == 0);
	{
		Pei pei;
		PeiEvent_t sw = 0U;

		assert(pei.BindSwEvent(sw));
		assert(pei.EventEndpoint(sw) == TEST_EGU_BASE + 0x100U);
	}

	// The C++ Pei wrapper manages the same C group object and matrix.
	{
		Pei pei;
		PeiGroup_t group = {};
		PeiEvent_t evt = 0U;

		assert(pei.BindEvent(evt, event1));
		assert(pei.Connect(evt, task1));
		assert(pei.Connect(evt, task2));
		assert(pei.GroupInit(group));
		assert(pei.GroupAddEvent(group, evt));
		assert(pei.GroupEnableTask(group) != 0U);
		assert(pei.GroupDisableTask(group) != 0U);
		assert(pei.GroupRemoveEvent(group, evt));
		assert(pei.GroupDeinit(group));
		assert(pei.ReleaseEvent(evt));
	}

	assert(g_critical_depth == 0);
	return 0;
}
