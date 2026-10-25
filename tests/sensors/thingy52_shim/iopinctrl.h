#pragma once
#include "coredev/iopincfg.h"

void IOPinSet(int PortNo, int PinNo);
void IOPinClear(int PortNo, int PinNo);
int IOPinRead(int PortNo, int PinNo);
