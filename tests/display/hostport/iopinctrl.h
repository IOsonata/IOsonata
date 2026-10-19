#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__
#include "coredev/iopincfg.h"
extern "C" void IOPinSet(int Port, int Pin);
extern "C" void IOPinClear(int Port, int Pin);
#endif
