// Register semantics model using the production Timer API and all three sources.
#include <cassert>
#include <cstdint>
#include <cstdio>
#define __SAM4LXXX_H__
#define __ASSEMBLY__
#include "component/component_ast.h"
#include "component/component_tc.h"
#include "component/component_pm.h"
#include "component/component_bpm.h"
#include "component/component_bscif.h"
#undef __ASSEMBLY__
#define __NVIC_PRIO_BITS 4
using IRQn_Type = int;
enum {AST_ALARM_IRQn=39,AST_OVF_IRQn=41,TC00_IRQn=55,TC01_IRQn,TC02_IRQn,TC10_IRQn,TC11_IRQn,TC12_IRQn};
struct { uint32_t ISER[2]={}; } nvic;
#define NVIC (&nvic)
uint64_t pending;
void NVIC_EnableIRQ(int n) { nvic.ISER[n/32] |= 1u<<(n%32); }
void NVIC_DisableIRQ(int n) { nvic.ISER[n/32] &= ~(1u<<(n%32)); }
void NVIC_ClearPendingIRQ(int n) { pending &= ~(1ULL<<n); }
void NVIC_SetPendingIRQ(int n) { pending |= 1ULL<<n; }
void NVIC_SetPriority(int, int) {}
struct Status {
	uint32_t value=0, wrapAfterRead=0;
	operator uint32_t() { uint32_t v=value; value &= TC_SR_CLKSTA; if(wrapAfterRead) { value|=TC_SR_COVFS; wrapAfterRead=0; } return v; }
};
struct Command {
	uint32_t *cv; Status *sr;
	void operator=(uint32_t v) {
		if(v&TC_CCR_CLKDIS) sr->value &= ~TC_SR_CLKSTA;
		else if(v&TC_CCR_CLKEN) sr->value |= TC_SR_CLKSTA;
		if(v&TC_CCR_SWTRG) *cv=0;
	}
};
struct MaskWrite {
	uint32_t *mask; bool enable;
	void operator=(uint32_t v) {if(enable)*mask|=v;else *mask&=~v;}
};
struct TcChannel {
	uint32_t TC_CV=0,TC_CMR=0,TC_SMMR=0,TC_IMR=0,TC_RA=0,TC_RB=0,TC_RC=0;
	Status TC_SR;
	Command TC_CCR{&TC_CV,&TC_SR};
	MaskWrite TC_IER{&TC_IMR,true},TC_IDR{&TC_IMR,false};
};
struct Tc { TcChannel TC_CHANNEL[3]; } tc[2];
#define SAM4L_TC0 (&tc[0])
#define SAM4L_TC1 (&tc[1])
struct {uint32_t PM_UNLOCK=0,PM_PBAMASK=0,PM_PBDMASK=0,PM_PBADIVMASK=0;} pm;
#define SAM4L_PM (&pm)
struct {uint32_t BPM_PMCON=0;} bpm;
#define SAM4L_BPM (&bpm)
struct {uint32_t BSCIF_PCLKSR=BSCIF_PCLKSR_OSC32RDY,BSCIF_OSCCTRL32=BSCIF_OSCCTRL32_EN32K,BSCIF_RC32KCR=0;} bscif;
#define SAM4L_BSCIF (&bscif)
struct AstStatus {uint32_t value=0; operator uint32_t();};
struct {
	uint32_t AST_CR=0,AST_CLOCK=0,AST_DTR=0,AST_CV=0,AST_SCR=0,AST_AR0=0,AST_IMR=0;
	AstStatus AST_SR;
	MaskWrite AST_IER{&AST_IMR,true},AST_IDR{&AST_IMR,false};
} ast;
#define SAM4L_AST (&ast)
AstStatus::operator uint32_t() {
	// Hardware clears flags on a synchronized SCR write.
	value &= ~ast.AST_SCR; ast.AST_SCR=0;
	static uint32_t clock=0;
	uint32_t diff=clock^ast.AST_CLOCK;
	assert(!((diff&AST_CLOCK_CEN)&&(diff&AST_CLOCK_CSSEL_Msk)));
	clock=ast.AST_CLOCK;
	return value;
}
#include "../../ARM/Microchip/SAM4L/src/timer_sam4l.cpp"
#include "../../ARM/Microchip/SAM4L/src/timer_sam4l_ast.cpp"
#include "../../ARM/Microchip/SAM4L/src/timer_sam4l_tc.cpp"
McuOsc_t g_McuOsc = {};
extern "C" uint32_t SystemPeriphClockGet(int) { return 48000000; }
unsigned events[7][3]={},overflows[7]={},callbacks=0;
bool cancelNext=false,resetInCallback=false;
void Event(TimerDev_t *t,uint32_t e) {
	if(e&TIMER_EVT_COUNTER_OVR) ++overflows[t->DevNo];
	for(int n=0;n<3;++n) if(e&TIMER_EVT_TRIGGER(n)) ++events[t->DevNo][n];
	if(cancelNext && (e&TIMER_EVT_TRIGGER(0))) t->DisableTrigger(t,1);
	if(resetInCallback && (e&TIMER_EVT_TRIGGER(0))) t->Reset(t);
}
void Callback(TimerDev_t *t,int n,void *p) {
	assert(n==2 && p==&callbacks); ++callbacks;
	assert(!g_Sam4lTimerData[t->DevNo].Trigger[2].Active); // one shot freed before callback
}
void Service(int dev) {
	auto &d=g_Sam4lTimerData[dev];
	if (!d.Running || !(nvic.ISER[d.Irq/32]&(1u<<(d.Irq%32)))) return;
	uint32_t status=d.TcReg ? d.TcReg->TC_SR.value & d.TcReg->TC_IMR : ast.AST_SR.value & ast.AST_IMR;
	if(status || (pending&(1ULL<<d.Irq))) {NVIC_ClearPendingIRQ(d.Irq);Sam4lTimerIRQ(dev);}
}
void Advance(int dev,uint32_t ticks,bool service=true) {
	auto &d=g_Sam4lTimerData[dev];
	for(uint32_t i=0;i<ticks;++i) {
		if(d.TcReg) {
			auto &r=*d.TcReg;
			if(r.TC_SR.value&TC_SR_CLKSTA) {
				r.TC_CV=(r.TC_CV+1)&65535;
				if(!r.TC_CV)r.TC_SR.value|=TC_SR_COVFS;
				if(r.TC_CV==r.TC_RA)r.TC_SR.value|=TC_SR_CPAS;
				if(r.TC_CV==r.TC_RB)r.TC_SR.value|=TC_SR_CPBS;
				if(r.TC_CV==r.TC_RC)r.TC_SR.value|=TC_SR_CPCS;
			}
		} else if(ast.AST_CR&AST_CR_EN) {
			if(++ast.AST_CV==0)ast.AST_SR.value|=AST_SR_OVF;
			if(ast.AST_CV==ast.AST_AR0)ast.AST_SR.value|=AST_SR_ALARM0;
		}
		if(service)Service(dev);
	}
}
int main() {
	g_McuOsc.LowPwrOsc.Freq=32768;
	// AST is only reset by POR (42023H table 10-12), not by debugger reset.
	// Model a previous firmware's running timer with no live software owner.
	ast.AST_CLOCK=AST_CLOCK_CSSEL_32KHZCLK;
	(void)(uint32_t)ast.AST_SR;
	ast.AST_CLOCK |= AST_CLOCK_CEN;
	(void)(uint32_t)ast.AST_SR;
	ast.AST_CR=AST_CR_EN | AST_CR_PSEL(7);
	ast.AST_CV=123456;
	ast.AST_SR.value=AST_SR_OVF | AST_SR_ALARM0;
	ast.AST_IMR=AST_SR_ALARM0;
	TimerDev_t timers[7]={},other={};
	TimerCfg_t cfg={0,TIMER_CLKSRC_DEFAULT,0,7,Event,false};
	assert(TimerGetLowFreqDevCount()==1&&TimerGetHighFreqDevCount()==6&&TimerGetHighFreqDevNo()==1);
	cfg.bTickInt=true; assert(!TimerInit(&other,&cfg)); cfg.bTickInt=false;
	cfg.ClkSrc=TIMER_CLKSRC_EXT; assert(!TimerInit(&other,&cfg)); cfg.ClkSrc=TIMER_CLKSRC_DEFAULT;
	cfg.IntPrio=16; assert(!TimerInit(&other,&cfg)); cfg.IntPrio=7;
	assert(!TimerInit(nullptr,&cfg)); cfg.DevNo=7; assert(!TimerInit(&other,&cfg));
	cfg.DevNo=0;
	NVIC_EnableIRQ(AST_ALARM_IRQn);
	assert(!TimerInit(&other,&cfg));
	assert(g_Sam4lTimerInitStage==SAM4L_TIMER_INIT_IRQ);
	assert(ast.AST_CR&AST_CR_EN); // Do not take a live interrupt owner.
	NVIC_DisableIRQ(AST_ALARM_IRQn);
	for(int dev=0;dev<7;++dev) {
		cfg.DevNo=dev; timers[dev].pObj=&timers[dev];
		assert(TimerInit(&timers[dev],&cfg));
		assert(g_Sam4lTimerInitStage==SAM4L_TIMER_INIT_OK);
		assert(timers[dev].GetTickCount(&timers[dev])==0);
		assert(timers[dev].pObj==&timers[dev]);
		assert(!TimerInit(&other,&cfg));
		assert(g_Sam4lTimerInitStage==SAM4L_TIMER_INIT_OWNER);
		assert(!TimerInit(&timers[dev],&cfg));
		assert(timers[dev].Freq==(dev?375000u:1024u));
		assert(timers[dev].GetMaxTrigger(&timers[dev])==(dev?3:1));
		assert(!timers[dev].EnableExtTrigger(&timers[dev],0,TIMER_EXTTRIG_SENSE_TOGGLE));
	}
	assert(pm.PM_PBADIVMASK==64 && (pm.PM_PBAMASK & (PM_PBAMASK_TC0|PM_PBAMASK_TC1)));
	for(int dev=0;dev<7;++dev) {
		auto &t=timers[dev];
		assert(t.EnableTrigger(&t,0,100000000,TIMER_TRIG_TYPE_CONTINUOUS,nullptr,nullptr));
		uint32_t period=g_Sam4lTimerData[dev].Trigger[0].Ticks;
		Advance(dev,period*3); assert(events[dev][0]==3);
		uint64_t count=t.GetTickCount(&t); assert(count==period*3);
		t.Disable(&t); Advance(dev,period);assert(t.GetTickCount(&t)==count);
		assert(t.Enable(&t));Advance(dev,period);assert(events[dev][0]==4);
		t.Reset(&t);assert(t.GetTickCount(&t)==0);
		Advance(dev,period,false);assert(t.GetTickCount(&t)==period); // reading status must not lose compare
		Service(dev);assert(events[dev][0]==5);
		t.DisableTrigger(&t,0);Advance(dev,period);assert(events[dev][0]==5);
		assert(t.FindAvailTrigger(&t)==0);
		assert(!t.EnableTrigger(&t,0,UINT64_MAX,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
		assert(!t.EnableTrigger(&t,-1,100000000,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
	}
	auto &t=timers[1]; t.Reset(&t);
	assert(t.EnableTrigger(&t,0,1000000000,TIMER_TRIG_TYPE_CONTINUOUS,nullptr,nullptr));
	assert(t.EnableTrigger(&t,1,1000000000,TIMER_TRIG_TYPE_CONTINUOUS,nullptr,nullptr));
	assert(t.EnableTrigger(&t,2,1000000000,TIMER_TRIG_TYPE_SINGLE,Callback,&callbacks));
	unsigned old=events[1][0];cancelNext=true;
	Advance(1,375000); assert(events[1][0]==old+1&&events[1][1]==0&&callbacks==1);
	cancelNext=false;
	assert(t.GetTickCount(&t)==375000); // trigger spanning five hardware wraps
	// Several missed periods, but service each wrap for the 16-bit extension.
	t.DisableTrigger(&t,0);t.Reset(&t);
	assert(t.EnableTrigger(&t,0,10000000,TIMER_TRIG_TYPE_CONTINUOUS,nullptr,nullptr));
	old=events[1][0]; Advance(1,40000,false);Service(1);
	assert(events[1][0]==old+1&&g_Sam4lTimerData[1].Trigger[0].Deadline==41250);
	assert(t.SetFrequency(&t,1000000)==1500000); // nearest supported PBA divisor
	assert(t.GetTickCount(&t)==0 && g_Sam4lTimerData[1].Trigger[0].Ticks==15000);
	// Rollover arriving between status and CV reads is counted once.
	t.DisableTrigger(&t,0);t.Reset(&t);
	tc[0].TC_CHANNEL[0].TC_CV=2;tc[0].TC_CHANNEL[0].TC_SR.wrapAfterRead=1;
	assert(t.GetTickCount(&t)==65538);Service(1);assert(t.GetTickCount(&t)==65538);
	// AST rollover and W1C acknowledgement, including a foreground read.
	auto &a=timers[0];a.Reset(&a);ast.AST_CV=0xFFFFFFFEu;
	Advance(0,4,false);assert(a.GetTickCount(&a)==0x100000002ULL);Service(0);
	assert(a.GetTickCount(&a)==0x100000002ULL&&overflows[0]==1);
	assert(!(ast.AST_SR.value&AST_SR_OVF));
	// Callback reset invalidates the remaining due triggers.
	t.Reset(&t);assert(t.EnableTrigger(&t,0,10000000,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
	assert(t.EnableTrigger(&t,1,10000000,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
	resetInCallback=true;Advance(1,15000);resetInCallback=false;assert(events[1][1]==0);
	// Synchronization fault is bounded and makes enable fail closed.
	ast.AST_SR.value|=AST_SR_BUSY;a.Reset(&a);assert(!a.Enable(&a));
	puts("SAM4L timer register-model tests passed");
}
