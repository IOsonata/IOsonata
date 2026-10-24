#include "nrf.h"
#include "cc3xx_intrf.h"
#include <cassert>
TestWrapper g_Wrapper;
int main()
{
	assert(Cc3xxBase() == TEST_CC_BASE);
	assert(Cc3xxEnable());
	assert(g_Wrapper.ENABLE.Value == 1 && g_Wrapper.ENABLE.Writes == 1);
	assert(Cc3xxEnable());
	assert(g_Wrapper.ENABLE.Writes == 1);
	Cc3xxDisable();
	assert(g_Wrapper.ENABLE.Value == 0 && g_Wrapper.ENABLE.Writes == 2);
	g_Wrapper.ENABLE.Fail = true;
	assert(!Cc3xxEnable());
	assert(g_Wrapper.ENABLE.Writes == 3);
	g_Wrapper.ENABLE.Fail = false;
	assert(Cc3xxEnable());
	assert(g_Wrapper.ENABLE.Writes == 4);
	Cc3xxDisable();
}
