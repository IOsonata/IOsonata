/**-------------------------------------------------------------------------
@file	scan_test.cpp

@brief	Host test of the shared I2C scan example.

Exercise the shared scanner when StartRx succeeds even on an absent address.

@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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

#include <cassert>
#include "board.h"
#undef UART_RETARGET
#define main ScannerMain
#include "../../exemples/i2c/i2c_scan.cpp"
#undef main
struct Done {};
void ScanIdle() { throw Done{}; }
static unsigned selected, starts, stops, reads;
static bool Start(DevIntrf_t *, uint32_t addr) { selected = addr; starts++; return true; }
static void Stop(DevIntrf_t *) { stops++; }
static void Disable(DevIntrf_t *) {}
static int Read(DevIntrf_t *, uint8_t *p, int n) {
 assert(n == 1); reads++; *p = 0xab;
 return selected == 0x08 || selected == 0x48 || selected == 0x77 ? 1 : 0;
}
extern "C" bool I2CInit(I2CDev_t *d, const I2CCfg_t *c) {
 assert(!c->bDmaEn && !c->bIntEn && c->Rate == 100000);
 d->Cfg = *c; d->DevIntrf.StartRx = Start; d->DevIntrf.StopRx = Stop;
 d->DevIntrf.RxData = Read; d->DevIntrf.Disable = Disable;
 atomic_flag_clear(&d->DevIntrf.bBusy); return true;
}
extern "C" bool UARTInit(UARTDev_t *, const UARTCfg_t *) { assert(false); return false; }
extern "C" void UARTSetCtrlLineState(UARTDev_t *, uint32_t) { assert(false); }
extern "C" void I2CSetReadRqstData(I2CDev_t *, int, uint8_t *, int) { assert(false); }
extern "C" void I2CSetWriteRqstBuffer(I2CDev_t *, int, uint8_t *, int) { assert(false); }
int main() {
 try { ScannerMain(); } catch (Done) {}
 assert(starts == stops && reads == starts);
 assert(reads >= 112 && reads <= 224);
}
