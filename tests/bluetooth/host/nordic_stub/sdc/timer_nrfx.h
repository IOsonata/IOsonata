// Host stand-in for the Nordic timer port header. bt_hci_ctlr_sdc.cpp starts
// the low frequency timer the link timeouts read through nRFxLFTimerInit; the
// real header needs the MDK, which a host build has not. The test defines
// the function.
#ifndef __TIMER_NRFX_STUB_H__
#define __TIMER_NRFX_STUB_H__

#include "coredev/timer.h"

#ifdef __cplusplus
extern "C" {
#endif

bool nRFxLFTimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);

#ifdef __cplusplus
}
#endif

#endif // __TIMER_NRFX_STUB_H__
