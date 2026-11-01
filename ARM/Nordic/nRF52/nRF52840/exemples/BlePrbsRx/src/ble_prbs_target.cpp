/* Target controller memory for the Nordic PRBS receiver. */
#if defined(NRFXLIB_SDC)
#include "bluetooth/bt_hci_ctlr.h"

alignas(8) static uint8_t s_CentralCtlrMem[16384];
extern "C" const BtHciCtlrMemPool_t g_BtHciCtlrMemPool = {
	s_CentralCtlrMem,
	sizeof(s_CentralCtlrMem)
};
#endif

