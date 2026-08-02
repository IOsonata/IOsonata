#ifndef __BT_VIRTUAL_CLOCK_H__
#define __BT_VIRTUAL_CLOCK_H__

#include <cstddef>
#include <cstdint>

namespace btcompliance {

class VirtualClock {
public:
	typedef void (*Callback)(void *pContext);

	VirtualClock();

	void Reset();
	uint64_t NowUs() const { return vNowUs; }
	bool ScheduleUs(uint64_t DelayUs, Callback Fn, void *pContext);
	void AdvanceUs(uint64_t DeltaUs);
	void RunReady();
	size_t PendingCount() const;

private:
	struct Event {
		bool Used;
		uint64_t DueUs;
		uint32_t Sequence;
		Callback Fn;
		void *pContext;
	};

	static const size_t EVENT_CAPACITY = 64;
	Event vEvents[EVENT_CAPACITY];
	uint64_t vNowUs;
	uint32_t vNextSequence;
};

} // namespace btcompliance

#endif // __BT_VIRTUAL_CLOCK_H__
