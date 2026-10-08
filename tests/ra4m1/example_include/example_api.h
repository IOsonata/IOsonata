/* REDUCED EXAMPLE TEST CONTRACT ONLY. No production project uses this header.
 * Extends the existing reduced GPIO/timer/UART contracts with the wrappers
 * exercised by shared examples. This is not a production CMSIS/newlib build.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "ra4m1xxx.h"
#include "coredev/iopincfg.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
typedef IOPINCFG IOPinCfg_t;
#ifndef IRQ_PRIO_NORMAL
#define IRQ_PRIO_NORMAL 3
#endif
void ExampleWait(void);
void ExampleToggle(int, int);
#define __WFE ExampleWait
static inline uint64_t TimerTickToTime(TimerDev_t *d,uint64_t c,uint32_t u) {
 return d->Freq ? c/d->Freq*u + c%d->Freq*u/d->Freq : 0;
}
static inline uint64_t TimerGetNanosecond(TimerDev_t *d) {return TimerTickToTime(d,d->GetTickCount(d),1000000000);}
static inline uint32_t TimerGetMilisecond(TimerDev_t *d) {return (uint32_t)TimerTickToTime(d,d->GetTickCount(d),1000);}
static inline int TimerGetMaxTrigger(TimerDev_t *d) {return d->GetMaxTrigger(d);}
static inline void TimerDisable(TimerDev_t *d) {d->Disable(d);}
static inline void TimerDisableTrigger(TimerDev_t *d,int n) {d->DisableTrigger(d,n);}
static inline uint64_t nsTimerEnableTrigger(TimerDev_t *d,int n,uint64_t p,TIMER_TRIG_TYPE t,TimerTrigEvtHandler_t h,void *c) {return d->EnableTrigger(d,n,p,t,h,c);}
class Timer {
 TimerDev_t vTimer{};
public:
 Timer() {vTimer.pObj=this;}
 operator TimerDev_t *() {return &vTimer;}
 bool Init(const TimerCfg_t &c) {return TimerInit(&vTimer,&c);}
 uint64_t EnableTimerTrigger(int n,uint64_t p,TIMER_TRIG_TYPE t,TimerTrigEvtHandler_t h=nullptr,void *c=nullptr) {return vTimer.EnableTrigger(&vTimer,n,p,t,h,c);}
 uint32_t EnableTimerTrigger(int n,uint32_t p,TIMER_TRIG_TYPE t,TimerTrigEvtHandler_t h=nullptr,void *c=nullptr) {return (uint32_t)(EnableTimerTrigger(n,(uint64_t)p*1000000,t,h,c)/1000000);}
 uint32_t mSecond() {return TimerGetMilisecond(&vTimer);}
};
extern "C" {
 int UARTRx(UARTDev_t *,uint8_t *,int);
 int UARTTx(UARTDev_t *,uint8_t *,int);
 void UARTprintf(UARTDev_t *,const char *,...);
 uint8_t Prbs8(uint8_t);
}
class UART {
 UARTDev_t vDev{};
public:
 UART() {vDev.pObj=this;}
 bool Init(const UARTCfg_t &c) {return UARTInit(&vDev,&c);}
 int Rx(uint8_t *p,int n) {return UARTRx(&vDev,p,n);}
 int Tx(uint8_t *p,uint32_t n) {return UARTTx(&vDev,p,(int)n);}
 void printf(const char *s,...) {UARTprintf(&vDev,s);}
};
