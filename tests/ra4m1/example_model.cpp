/* Run the shared example source against API test doubles, not MCU hardware.
 * EXAMPLE_KIND: 1=timer, 2=loopback, 3=PRBS. EXAMPLE_SOURCE is a quoted path.
 * Copyright (c) 2026 I-SYST inc. MIT License.
 */
#define RA4M1_HOST_TEST 1
#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>
#include "example_api.h"
// The package masks (Ra4m1PinValid) of the port. Its IOPinToggle is renamed:
// the examples get the test double of example_include/iopinctrl.h.
#define IOPinToggle Ra4m1PortIOPinToggle
#include "../../ARM/Renesas/RA4/RA4M1/include/iopinctrl.h"
#undef IOPinToggle
SCB_Type test_scb;
uint32_t test_primask, test_disabled, test_cleared;
struct Finished {};
static bool fail_init, fail_uart;
static int uart_inits;
static int fail_trigger=-1, trigger_count, configured, disabled, removed, prints;
static int tx_calls, rx_calls;
static uint64_t ticks;
static TimerDev_t *active_timer;
static std::vector<int> toggles;
static std::vector<uint8_t> received, echoed;
static size_t rx_offset;
#define main ExampleMain
#include EXAMPLE_SOURCE
#undef main

void ExampleWait(void) {throw Finished{};}
void ExampleToggle(int p,int n) {assert(Ra4m1PinValid(p,n));toggles.push_back(p*16+n);}
extern "C" void IOPinConfig(int p,int n,int,IOPINDIR,IOPINRES,IOPINTYPE) {assert(Ra4m1PinValid(p,n));++configured;}
static uint64_t count_ticks(TimerDev_t *) {return ticks;}
static int max_triggers(TimerDev_t *d) {return d->DevNo<2 ? 1 : 6;}
static void disable_timer(TimerDev_t *) {++disabled;}
static void remove_trigger(TimerDev_t *,int) {++removed;}
static uint64_t enable_trigger(TimerDev_t *d,int n,uint64_t ns,TIMER_TRIG_TYPE t,TimerTrigEvtHandler_t,void *) {
	assert(n>=0 && n<max_triggers(d));assert(t==TIMER_TRIG_TYPE_CONTINUOUS);
	static const uint64_t expected[]={100000000ULL,1000000000ULL,250000000ULL,500000000ULL};
	assert(n<4 && ns==expected[n]);++trigger_count;
	return n==fail_trigger ? 0 : ns;
}
extern "C" bool TimerInit(TimerDev_t *d,const TimerCfg_t *c) {
	if(fail_init) return false;
	assert(c->DevNo>=0 && c->DevNo<10);
	// Model default frequency selection; hardware range/quantization is tested
	// by timer_model.cpp, not by this application API test double.
	d->DevNo=c->DevNo;d->Freq=c->Freq ? c->Freq : (c->DevNo<2 ? 32768U : 48000000U);
	d->EvtHandler=c->EvtHandler;
	d->GetTickCount=count_ticks;d->GetMaxTrigger=max_triggers;
	d->Disable=disable_timer;d->DisableTrigger=remove_trigger;d->EnableTrigger=enable_trigger;
	assert(!c->bTickInt);active_timer=d;return true;
}
extern "C" bool UARTInit(UARTDev_t *,const UARTCfg_t *c) {
	++uart_inits;
	if(fail_uart) return false;
	// The pins of board.h, RX then TX (example_pin_probe.cpp checks them)
	assert(c->DevNo==UART_DEVNO && c->NbIOPins==2);
	auto pins=(const IOPINCFG *)c->pIOPinMap;
	assert(pins[0].PortNo==UART_RX_PORT && pins[0].PinNo==UART_RX_PIN && pins[0].PinOp==UART_RX_PINOP && pins[0].PinDir==IOPINDIR_INPUT);
	assert(pins[1].PortNo==UART_TX_PORT && pins[1].PinNo==UART_TX_PIN && pins[1].PinOp==UART_TX_PINOP && pins[1].PinDir==IOPINDIR_OUTPUT);
	assert(!c->bDMAMode && c->bFifoBlocking && c->DataBits==8 && c->Parity==UART_PARITY_NONE);
	assert(c->FlowControl==UART_FLWCTRL_NONE);
#if EXAMPLE_KIND==1
	// Console of the timer demo: 1 Mbaud, interrupt, TX FIFO only
	assert(c->Rate==1000000 && c->bIntMode && c->StopBits==1);
	assert(!c->pRxMem && c->pTxMem && c->TxMemSize==CFIFO_MEMSIZE(256) && !((uintptr_t)c->pTxMem&3));
#elif EXAMPLE_KIND==2
	// Loopback: the example default rate and mode, RX and TX FIFO
	assert(c->Rate==UART_BAUDRATE && c->bIntMode==UART_INT_MODE && c->bIntMode);
	assert(c->pRxMem && c->pTxMem && c->pRxMem!=c->pTxMem);
	assert(c->RxMemSize==CFIFO_MEMSIZE(256) && c->TxMemSize==CFIFO_MEMSIZE(256));
	assert(!((uintptr_t)c->pRxMem&3) && !((uintptr_t)c->pTxMem&3));
#else
	// PRBS: the example default rate and stop bits, polling, no FIFO memory
	assert(c->Rate==UART_BAUDRATE && c->StopBits==UART_STOPBITS && c->bIntMode==UART_INT_MODE && !c->bIntMode);
	assert(!c->pRxMem && !c->pTxMem && c->RxMemSize==0 && c->TxMemSize==0);
#endif
	return true;
}
extern "C" int UARTRx(UARTDev_t *,uint8_t *p,int n) {
	++rx_calls;
	// A new read must not discard the previously received unsent suffix.
	assert(echoed.size()==rx_offset);
	if(rx_offset==received.size()) throw Finished{};
	int got=(int)(received.size()-rx_offset);
	if(got>n) got=n;
	memcpy(p,received.data()+rx_offset,(size_t)got);rx_offset+=(size_t)got;return got;
}
extern "C" int UARTTx(UARTDev_t *,uint8_t *p,int n) {
	++tx_calls;
#if EXAMPLE_KIND==3
	if(echoed.size()>=1024) throw Finished{};
#endif
	const int limits[]={0,1,3,0,7,2};
	int sent=limits[(tx_calls-1)%6];if(sent>n)sent=n;
	echoed.insert(echoed.end(),p,p+sent);return sent;
}
extern "C" void UARTprintf(UARTDev_t *,const char *,...) {++prints;}
extern "C" uint8_t Prbs8(uint8_t d) {return (uint8_t)(((d<<1)|(((d>>6)^(d>>5))&1))&0x7f);}
int main() {
#if EXAMPLE_KIND==1
	// The console UART is started first. On failure the demo waits in __WFE
	// and does not touch the timer.
	fail_uart=true;
	try {ExampleMain();assert(false);} catch(const Finished &) {}
	assert(uart_inits==1 && !active_timer && trigger_count==0);
	fail_uart=false;
	fail_init=true;assert(ExampleMain()==1);assert(uart_inits==2 && trigger_count==0);
#else
	fail_uart=true;assert(ExampleMain()==1);assert(uart_inits==1 && tx_calls==0 && rx_calls==0 && prints==0 && trigger_count==0);
	fail_uart=false;
#endif
	fail_init=false;
#if EXAMPLE_KIND==1
	for(int n=0;n<(TIMER_DEMO_DEVNO<2?1:4);++n) {
		fail_trigger=n;trigger_count=disabled=removed=0;
		assert(ExampleMain()==1);assert(disabled==1 && removed==n && trigger_count==n+1);
	}
	fail_trigger=-1;trigger_count=0;
	try {ExampleMain();assert(false);} catch(const Finished &) {}
	assert(trigger_count==(TIMER_DEMO_DEVNO<2?1:4));assert(g_TimerInitOk);
	assert(g_TriggerPeriod[0]==100000000ULL);
	if(TIMER_DEMO_DEVNO>=2) {
		assert(g_TriggerPeriod[1]==1000000000ULL);
		assert(g_TriggerPeriod[2]==250000000ULL && g_TriggerPeriod[3]==500000000ULL);
	}
	ticks=active_timer->Freq;
	TimerHandler(active_timer,TIMER_EVT_TRIGGER(0)|TIMER_EVT_TRIGGER(1)|TIMER_EVT_TRIGGER(2)|TIMER_EVT_TRIGGER(3)|TIMER_EVT_COUNTER_OVR);
	assert(toggles.size()==3);assert(g_TickCount==1000000000ULL);
	for (int i=0;i<5;++i) assert(g_Period[i]==1000000000);
	for (int i=0;i<4;++i) assert(g_TriggerCount[i]==1);
	puts("PASS: console UART and timer init failure, trigger failure cleanup, capabilities, periods, callbacks");
#elif EXAMPLE_KIND==2
	for(int i=0;i<513;++i)received.push_back((uint8_t)(i*37));
	try {ExampleMain();assert(false);} catch(const Finished &) {}
	assert(g_UartInitOk && echoed==received && rx_offset==received.size());
	// No banner any more: every byte sent is echoed data
	assert(prints==0 && tx_calls>rx_calls);
	puts("PASS: UART loopback init failure, configuration, short/zero writes, 513 exact bytes");
#else
	try {ExampleMain();assert(false);} catch(const Finished &) {}
	uint8_t d=0xff;for(auto c:echoed){assert(c==d);d=Prbs8(d);}
	assert(echoed.size()==1024 && tx_calls>1024 && prints==0 && g_UartInitOk);
	puts("PASS: UART PRBS init failure, configuration, retry sequence, 1024 exact bytes");
#endif
}
