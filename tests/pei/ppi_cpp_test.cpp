#include <cassert>
#include <cstring>

#include "nrf.h"
#include "coredev/pei.h"

extern "C" {
NRF_PPI_Type g_TestPpi;
int g_critical_depth;
}

int main()
{
	std::memset(&g_TestPpi, 0, sizeof(g_TestPpi));
	const PeiCfg_t config = {0, 0x40008100U, 0x40006000U, 0U};
	{
		Pei conn;
		assert(conn.Connect(config));
		assert(conn.Enable());
		assert(conn.Disable());
		// Destructor releases connection without a manual disconnect.
	}
	assert(g_TestPpi.CH[2].EEP == 0U);
	assert(g_TestPpi.CH[2].TEP == 0U);
	assert(g_critical_depth == 0);
	return 0;
}
