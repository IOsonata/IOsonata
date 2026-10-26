#pragma once
#include "coredev/iopincfg.h"
extern bool g_CsHigh;
inline void IOPinSet(int, int) { g_CsHigh = true; }
inline void IOPinClear(int, int) { g_CsHigh = false; }
inline void IOPinDisable(int, int) {}
