#include "bt_virtual_clock.h"

#include <cstring>

namespace btcompliance {

VirtualClock::VirtualClock()
{
	Reset();
}

void VirtualClock::Reset()
{
	std::memset(vEvents, 0, sizeof(vEvents));
	vNowUs = 0;
	vNextSequence = 0;
}

bool VirtualClock::ScheduleUs(uint64_t DelayUs, Callback Fn, void *pContext)
{
	if (Fn == nullptr)
	{
		return false;
	}

	for (size_t i = 0; i < EVENT_CAPACITY; i++)
	{
		if (!vEvents[i].Used)
		{
			vEvents[i].Used = true;
			vEvents[i].DueUs = vNowUs + DelayUs;
			vEvents[i].Sequence = vNextSequence++;
			vEvents[i].Fn = Fn;
			vEvents[i].pContext = pContext;
			return true;
		}
	}

	return false;
}

void VirtualClock::AdvanceUs(uint64_t DeltaUs)
{
	vNowUs += DeltaUs;
	RunReady();
}

void VirtualClock::RunReady()
{
	for (;;)
	{
		size_t selected = EVENT_CAPACITY;

		for (size_t i = 0; i < EVENT_CAPACITY; i++)
		{
			if (!vEvents[i].Used || vEvents[i].DueUs > vNowUs)
			{
				continue;
			}

			if (selected == EVENT_CAPACITY ||
				vEvents[i].DueUs < vEvents[selected].DueUs ||
				(vEvents[i].DueUs == vEvents[selected].DueUs &&
				 vEvents[i].Sequence < vEvents[selected].Sequence))
			{
				selected = i;
			}
		}

		if (selected == EVENT_CAPACITY)
		{
			break;
		}

		Callback fn = vEvents[selected].Fn;
		void *pContext = vEvents[selected].pContext;
		vEvents[selected].Used = false;
		vEvents[selected].Fn = nullptr;
		vEvents[selected].pContext = nullptr;
		fn(pContext);
	}
}

size_t VirtualClock::PendingCount() const
{
	size_t count = 0;
	for (size_t i = 0; i < EVENT_CAPACITY; i++)
	{
		if (vEvents[i].Used)
		{
			count++;
		}
	}
	return count;
}

} // namespace btcompliance
