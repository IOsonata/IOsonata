/**-------------------------------------------------------------------------
@file	nvm_demo_checks.h

@brief	NVM test setup and verification shared by the standalone and radio tests.

@author	Hoang Nguyen Hoan
@date	July 23, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __NVM_DEMO_CHECKS_H__
#define __NVM_DEMO_CHECKS_H__

#include "board.h"
#include "coredev/uart.h"
#include "storage/nvm.h"

#ifndef NVM_DEMO_MEDIUM
#define NVM_DEMO_MEDIUM		0
#endif
#ifndef NVM_DEMO_ASYNC
#define NVM_DEMO_ASYNC		1
#endif
#define NVM_DEMO_MAGIC		0x4E564D31UL	// "NVM1"

extern UART g_Uart;
extern const UARTCfg_t g_NvmDemoUartCfg;
extern const char g_NvmMediumName[];
extern Nvm g_Nvm;
extern int g_NvmFailures;
extern volatile uint32_t g_NvmEvtWrite;
extern volatile uint32_t g_NvmEvtErase;
extern volatile uint32_t g_NvmEvtError;
bool NvmDemoSetup(void);
void NvmDemoVerify(Nvm &Mem);
int NvmDemoWrite(Nvm &Mem, uint64_t Off, const void *pData, uint32_t Len);
int NvmDemoErase(Nvm &Mem, uint64_t Off, uint32_t Len);
uint32_t NvmDemoUnitSize(Nvm &Mem);

#endif
