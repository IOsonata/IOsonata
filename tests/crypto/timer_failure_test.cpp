// Timing is required: failed initialization must fail the shared MCU test.
#include <cassert>
extern bool g_CryptoTimerFail;
extern bool g_CryptoTimerStopped;
int CryptoEngineTest(void);

int main()
{
	g_CryptoTimerFail = true;
	assert(CryptoEngineTest() == 1);
	assert(g_CryptoTimerStopped);
	return 0;
}
