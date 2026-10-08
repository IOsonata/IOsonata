#pragma once
#include "coredev/iopincfg.h"
void IOPinSet(int Port, int Pin);
void IOPinClear(int Port, int Pin);
int IOPinRead(int Port, int Pin);
void IOPinSetDir(int Port, int Pin, IOPINDIR Dir);
