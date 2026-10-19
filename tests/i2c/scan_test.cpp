// Exercise the shared scanner when StartRx succeeds even on an absent address.
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
