#include <assert.h>
#include "coredev/iopincfg.h"

static int s_Calls;
static int s_FailAt = -1;
static const IOPinCoreCfg_t s_Pins[] = {
	{ 0, 2, 7 },
	{ 1, 0, 19 },
	{ 2, 3, 4 }
};

bool IOPinAssignCore(int PortNo, int PinNo, int CoreNo)
{
	int i = s_Calls++;

	assert(i < 3);
	assert(CoreNo == s_Pins[i].CoreNo);
	assert(PortNo == s_Pins[i].PortNo);
	assert(PinNo == s_Pins[i].PinNo);
	return i != s_FailAt;
}

int main(void)
{
	int i;

	assert(IOPinCoreCfg(NULL, 0));
	assert(IOPinCoreCfg(s_Pins, 0));
	assert(!IOPinCoreCfg(NULL, 1));
	assert(!IOPinCoreCfg(s_Pins, -1));
	assert(s_Calls == 0);
	assert(IOPinCoreCfg(s_Pins, 3));
	assert(s_Calls == 3);
	for (i = 0; i < 3; i++)
	{
		s_Calls = 0;
		s_FailAt = i;
		assert(!IOPinCoreCfg(s_Pins, 3));
		assert(s_Calls == i + 1);
	}
	return 0;
}
