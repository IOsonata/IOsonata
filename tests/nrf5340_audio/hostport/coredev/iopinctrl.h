#ifndef __NRF5340_AUDIO_TEST_IOPINCTRL_H__
#define __NRF5340_AUDIO_TEST_IOPINCTRL_H__

#include "coredev/iopincfg.h"

#ifdef __cplusplus
extern "C" {
#endif

int IOPinRead(int PortNo, int PinNo);
void IOPinSet(int PortNo, int PinNo);
void IOPinClear(int PortNo, int PinNo);

#ifdef __cplusplus
}
#endif

#endif
