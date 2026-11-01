/* Controller storage for the Nordic UART central targets. */
#if defined(NRFXLIB_SDC)
#include "bluetooth/bt_hci_ctlr.h"

// Central streaming reserves ten RX/TX packets and scanning buffers.
// The target reserves storage for the configured controller buffers.
alignas(8) static uint8_t s_CentralCtlrMem[16384];
extern "C" const BtHciCtlrMemPool_t g_BtHciCtlrMemPool = {
	s_CentralCtlrMem, sizeof(s_CentralCtlrMem)
};
#endif
