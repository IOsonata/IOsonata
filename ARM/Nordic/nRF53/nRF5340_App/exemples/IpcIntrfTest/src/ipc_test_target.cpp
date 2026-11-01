/* Network-core reset control for the nRF5340 IPC test peer. */
#include "nrf.h"
#include "board.h"

void IpcTestPeerRun(bool bRun)
{
#if defined(NRF_RESET_S) && !defined(NRF_TRUSTZONE_NONSECURE)
	NRF_RESET_S->NETWORK.FORCEOFF = bRun ? RESET_NETWORK_FORCEOFF_FORCEOFF_Release
										  : RESET_NETWORK_FORCEOFF_FORCEOFF_Hold;
#else
	NRF_RESET_NS->NETWORK.FORCEOFF = bRun ? RESET_NETWORK_FORCEOFF_FORCEOFF_Release
										   : RESET_NETWORK_FORCEOFF_FORCEOFF_Hold;
#endif
}
