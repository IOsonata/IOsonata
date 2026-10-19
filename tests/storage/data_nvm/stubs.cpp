// Unused GPIO/timer dependencies of the generic Device/Nvm classes.
#include "coredev/iopincfg.h"
#include "coredev/timer.h"
extern "C" void IOPinConfig(int, int, int, IOPINDIR, IOPINRES, IOPINTYPE) {}
#ifndef RA4M1
extern "C" { uint32_t SystemMicroSecLoopCnt = 2; }
#endif
