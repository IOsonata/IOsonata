#include <cassert>
#include <cstdio>
#include <cstdint>
#include "stm32f0xx.h"
// Real vendor bit definitions with a model of rc_w0 status semantics.
struct Status {
	uint32_t value=0;
	operator uint32_t() const { return value; }
	void operator=(uint32_t v) { value &= v; }
};
struct TimerRegs {
	uint32_t CR1=0,CR2=0,SMCR=0,DIER=0,EGR=0,CCMR1=0,CCMR2=0,CCER=0;
	uint32_t CNT=0,PSC=0,ARR=0,CCR1=0,CCR2=0,CCR3=0,CCR4=0,RCR=0,BDTR=0;
	Status SR;
} regs[7];
auto &tim=regs[5];
struct {uint32_t CFGR=0,APB1ENR=0,APB1RSTR=0,APB2ENR=0,APB2RSTR=0;} rcc;
struct {uint32_t ISER[1]={0};} nvic;
unsigned irqState=0,priority=99;uint32_t pending=0;
void ClearPending(IRQn_Type n) { pending&=~(1u<<n); }
void SetPending(IRQn_Type n) { pending|=1u<<n; }
void EnableIrq(IRQn_Type n) { nvic.ISER[0]|=1u<<n; }
void DisableIrq(IRQn_Type n) { nvic.ISER[0]&=~(1u<<n); }
void Priority(IRQn_Type n,unsigned p) { priority=p; }
#undef TIM3
#undef RCC
#undef TIM6
#undef TIM14
#undef TIM16
#undef TIM17
#undef TIM15
#undef TIM1
#define TIM_TypeDef TimerRegs
#define TIM6 (&regs[0])
#define TIM14 (&regs[1])
#define TIM16 (&regs[2])
#define TIM17 (&regs[3])
#define TIM15 (&regs[4])
#define TIM3 (&regs[5])
#define TIM1 (&regs[6])
#define RCC (&rcc)
#define NVIC (&nvic)
#define NVIC_ClearPendingIRQ ClearPending
#define NVIC_SetPendingIRQ SetPending
#define NVIC_EnableIRQ EnableIrq
#define NVIC_DisableIRQ DisableIrq
#define NVIC_SetPriority Priority
#include "../../../ARM/ST/STM32F0xx/src/timer_stm32f030x8.cpp"
uint32_t pclk=48000000;
extern "C" uint32_t SystemPeriphClockGet(int n) { assert(n==0);return pclk; }
unsigned events[4]={},overflows=0,callbacks=0;bool cancelNext=false;
void Event(TimerDev_t *t,uint32_t e) {
	if(e&TIMER_EVT_COUNTER_OVR)overflows++;
	for(int n=0;n<4;n++)if(e&TIMER_EVT_TRIGGER(n))events[n]++;
	if(cancelNext && (e&TIMER_EVT_TRIGGER(0)))t->DisableTrigger(t,1);
}
void Callback(TimerDev_t *,int n,void *p) {assert(n==3&&p==&callbacks);callbacks++;}
void Service() {
	if((tim.CR1&TIM_CR1_CEN)&&(nvic.ISER[0]&(1u<<TIM3_IRQn))&&
		((pending&(1u<<TIM3_IRQn))||(tim.SR.value&tim.DIER))) {pending&=~(1u<<TIM3_IRQn);TIM3_IRQHandler();}
}
void Advance(unsigned ticks,bool service=true) {
	for(unsigned i=0;i<ticks;i++) {
		if(!(tim.CR1&TIM_CR1_CEN))continue;
		tim.CNT=(tim.CNT+1)&0xffff;
		if(!tim.CNT)tim.SR.value|=TIM_SR_UIF;
		uint32_t cc[]={tim.CCR1,tim.CCR2,tim.CCR3,tim.CCR4};
		for(unsigned n=0;n<4;n++)if(tim.CNT==cc[n])tim.SR.value|=TIM_SR_CC1IF<<n;
		if(service)Service();
	}
}
void Step(int id,unsigned ticks,bool service=true) {
	auto &r=regs[id];auto &d=s_Devices[id];
	for(unsigned i=0;i<ticks;i++) {
		if(!(r.CR1&TIM_CR1_CEN))continue;
		if(++r.CNT>r.ARR){r.CNT=0;r.SR.value|=TIM_SR_UIF;}
		uint32_t cc[]={r.CCR1,r.CCR2,r.CCR3,r.CCR4};
		for(int n=0;n<d.Channels;n++)if(r.CNT==cc[n])r.SR.value|=TIM_SR_CC1IF<<n;
		if(service&&(nvic.ISER[0]&(1u<<d.Irq))&&(r.SR.value&r.DIER)) {
			void (*handlers[])()={TIM6_IRQHandler,TIM14_IRQHandler,TIM16_IRQHandler,TIM17_IRQHandler,TIM15_IRQHandler,TIM3_IRQHandler,TIM1_BRK_UP_TRG_COM_IRQHandler};
			if(id==6 && !(r.SR.value&TIM_SR_UIF))TIM1_CC_IRQHandler();else handlers[id]();
		}
	}
}
#ifdef TEST_IRQ_OVERRIDE
extern "C" void TIM17_IRQHandler(void) {}
#endif
int main() {
	TimerDev_t t{},other{};TimerCfg_t cfg{5,TIMER_CLKSRC_DEFAULT,1000000,2,Event,false};
	assert(TimerGetLowFreqDevCount()==0&&TimerGetHighFreqDevCount()==7&&TimerGetHighFreqDevNo()==0);
	for(int n:{-1,7,8}){cfg.DevNo=n;assert(!TimerInit(&t,&cfg));}cfg.DevNo=5;
	cfg.bTickInt=true;assert(!TimerInit(&t,&cfg));cfg.bTickInt=false;
	cfg.ClkSrc=TIMER_CLKSRC_LFRC;assert(!TimerInit(&t,&cfg));cfg.ClkSrc=TIMER_CLKSRC_DEFAULT;
	cfg.IntPrio=4;assert(!TimerInit(&t,&cfg));cfg.IntPrio=2;
	assert(!t.Enable&&rcc.APB1ENR==0);
	nvic.ISER[0]=1u<<TIM3_IRQn;assert(!TimerInit(&t,&cfg));nvic.ISER[0]=0;
	assert(TimerInit(&t,&cfg));assert(t.Freq==1000000&&tim.PSC==47&&priority==2);
	assert(!TimerInit(&other,&cfg));assert(!TimerInit(&t,&cfg));
	assert(t.GetMaxTrigger(&t)==4&&t.FindAvailTrigger(&t)==0);
	// rc_w0 writes must preserve a different flag that arrived after the snapshot.
	tim.SR.value=TIM_SR_CC1IF|TIM_SR_CC2IF;ClearFlags(s_Devices[5],TIM_SR_CC1IF);
	assert(tim.SR.value==TIM_SR_CC2IF);tim.SR.value=0;
	assert(!t.EnableExtTrigger(&t,0,TIMER_EXTTRIG_SENSE_TOGGLE));
	assert(!t.EnableTrigger(&t,-1,10000,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
	assert(!t.EnableTrigger(&t,0,UINT64_MAX,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
	assert(!t.EnableTrigger(&t,0,1000,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
	for(int n=0;n<4;n++)assert(t.EnableTrigger(&t,n,1000000ULL*(n+1),TIMER_TRIG_TYPE_CONTINUOUS,
		n==3?Callback:nullptr,n==3?&callbacks:nullptr));
	assert(t.FindAvailTrigger(&t)==-1);Advance(12000);
	assert(events[0]==12&&events[1]==6&&events[2]==4&&callbacks==3);
	for(int n=0;n<4;n++)t.DisableTrigger(&t,n);
	t.Reset(&t);Advance(65530);Advance(10,false);
	assert(t.GetTickCount(&t)==65540);assert(t.GetTickCount(&t)==65540);Service();
	assert(t.GetTickCount(&t)==65540&&t.Rollover==65536);
	// Long deadlines must not fire on the earlier matching 16-bit compare.
	t.Reset(&t);unsigned prior=events[0];
	assert(t.EnableTrigger(&t,0,200000000,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr)==200000000);
	Advance(199999);assert(events[0]==prior);Advance(1);assert(events[0]==prior+1);
	Advance(200000);assert(events[0]==prior+1);
	// Delayed periodic service coalesces missed periods and retains phase.
	t.Reset(&t);prior=events[0];t.EnableTrigger(&t,0,1000000,TIMER_TRIG_TYPE_CONTINUOUS,nullptr,nullptr);
	Advance(3500,false);Service();assert(events[0]==prior+1);assert(tim.CCR1==4000);
	t.Disable(&t);uint64_t paused=t.GetTickCount(&t);Advance(5000);assert(t.GetTickCount(&t)==paused);
	assert(t.Enable(&t));Advance(500);assert(events[0]==prior+2);
	// A callback can cancel a second channel at the same deadline.
	t.Reset(&t);t.EnableTrigger(&t,1,1000000,TIMER_TRIG_TYPE_CONTINUOUS,nullptr,nullptr);
	unsigned second=events[1];cancelNext=true;Advance(1000);assert(events[1]==second);cancelNext=false;
	// Reconfiguration preserves trigger duration, resets and restarts the timer.
	assert(t.SetFrequency(&t,2000000)==2000000);assert(tim.PSC==23&&tim.CCR1==2000);
	assert(t.GetTickCount(&t)==0);t.DisableTrigger(&t,0);
	pclk=12000000;rcc.CFGR=RCC_CFGR_PPRE_DIV4;assert(t.SetFrequency(&t,1000000)==1000000);assert(tim.PSC==23);
	assert(t.SetFrequency(&t,0)==24000000&&tim.PSC==0);
	assert(t.SetFrequency(&t,1)==24000000/65536&&tim.PSC==65535);
	TimerDev_t devices[7]{};
	const int channels[]={1,1,1,1,2,4,4};
	pclk=48000000;rcc.CFGR=0;
	for(int id=0;id<7;id++) {
		if(id==5)continue;
#ifdef TEST_IRQ_OVERRIDE
		if(id==3){cfg.DevNo=id;assert(!TimerInit(&devices[id],&cfg));continue;}
#endif
		cfg.DevNo=id;auto &dev=devices[id];auto &r=regs[id];
		assert(TimerInit(&dev,&cfg));assert(dev.GetMaxTrigger(&dev)==channels[id]);
		for(int n=0;n<channels[id];n++)assert(dev.EnableTrigger(&dev,n,1000000,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
		assert(dev.FindAvailTrigger(&dev)==-1);
		unsigned saved[4];for(int n=0;n<4;n++)saved[n]=events[n];
		Step(id,1000);
		for(int n=0;n<channels[id];n++)assert(events[n]==saved[n]+1);
		dev.Reset(&dev);
		assert((s_Devices[id].Apb2?rcc.APB2ENR:rcc.APB1ENR)&s_Devices[id].ClockMask);
		assert(!dev.EnableTrigger(&dev,channels[id],1000000,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
		assert(!dev.EnableTrigger(&dev,0,UINT64_MAX,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
		if(id==0)assert(!dev.EnableTrigger(&dev,0,65537000,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
		unsigned before=events[0];
		assert(dev.EnableTrigger(&dev,0,1000000,TIMER_TRIG_TYPE_CONTINUOUS,nullptr,nullptr)==1000000);
		Step(id,3000);assert(events[0]==before+3);assert(dev.GetTickCount(&dev)==3000);
		dev.Disable(&dev);Step(id,500);assert(dev.GetTickCount(&dev)==3000);dev.Enable(&dev);
		Step(id,1000);assert(events[0]==before+4);
		dev.DisableTrigger(&dev,0);uint64_t count=dev.GetTickCount(&dev);Step(id,17);assert(dev.GetTickCount(&dev)==count+17);
		dev.Reset(&dev);assert(dev.GetTickCount(&dev)==0);
		assert(dev.EnableTrigger(&dev,0,1000000,TIMER_TRIG_TYPE_SINGLE,nullptr,nullptr));
		before=events[0];Step(id,5000);assert(events[0]==before+1);assert(dev.GetTickCount(&dev)==5000);
		dev.Reset(&dev);Step(id,65530);Step(id,10,false);assert(dev.GetTickCount(&dev)==65540);
		STM32F030TimerIRQHandler(id);assert(dev.GetTickCount(&dev)==65540);
		assert(dev.SetFrequency(&dev,2000000)==2000000&&r.PSC==23);
		if(id==0) {
			assert(dev.EnableTrigger(&dev,0,30000000,TIMER_TRIG_TYPE_CONTINUOUS,nullptr,nullptr));
			assert(dev.SetFrequency(&dev,4000000)==0);assert(dev.Freq==2000000&&r.PSC==23);
			dev.DisableTrigger(&dev,0);
		}
		// Pause with an outstanding update: the next enable must retain it.
		dev.Reset(&dev);Step(id,65540,false);dev.Disable(&dev);
		assert(dev.GetTickCount(&dev)==65540);dev.Enable(&dev);
		STM32F030TimerIRQHandler(id);assert(dev.GetTickCount(&dev)==65540);
	}
	assert(!irqState);puts("PASS F030x8: all seven timers, clock gates, rollover, triggers, lifecycle and callback cancellation");
}
