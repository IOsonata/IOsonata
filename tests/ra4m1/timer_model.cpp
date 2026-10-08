/* Actual timer and ICU implementations with an explicit register/event model.
 * This does not simulate the ARM pipeline, electrical clocks or ISR latency.
 */
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <initializer_list>
#define RA4M1_HOST_TEST 1
#define RA4M1_IO_TEST 1
#define RA4M1_TIMER_READY_POLLS 16U
static uint8_t read8(uintptr_t);
static uint16_t read16(uintptr_t);
static uint32_t read32(uintptr_t);
static void write8(uintptr_t,uint8_t);
static void write16(uintptr_t,uint16_t);
static void write32(uintptr_t,uint32_t);
#define RA4M1_RD8(a) read8(a)
#define RA4M1_RD16(a) read16(a)
#define RA4M1_RD32(a) read32(a)
#define RA4M1_WR8(a,v) write8(a,v)
#define RA4M1_WR16(a,v) write16(a,v)
#define RA4M1_WR32(a,v) write32(a,v)
#include "../../ARM/Renesas/RA4M1/src/interrupt_ra4m1.cpp"
#include "../../ARM/Renesas/RA4M1/src/timer_ra4m1.cpp"
SCB_Type test_scb;
uint32_t test_primask,test_disabled,test_cleared,test_enabled,test_active,test_pending,test_priority[32];
uint32_t SystemCoreClock=48000000;
static uint32_t clocks[4]={48000000,24000000,48000000,48000000};
McuOsc_t g_McuOsc={{OSC_TYPE_RC,48000000,0,0},{OSC_TYPE_RC,32768,0,0},false};
uint32_t SystemCoreClockGet(void){return SystemCoreClock;}
uint32_t SystemPeriphClockGet(int n){assert(n>=0&&n<4);return clocks[n];}
#define V(n) Ra4m1DefaultIEL##n
extern "C" void (* const __Vectors[48])(void)={
 nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,
 nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,
 V(0),V(1),V(2),V(3),V(4),V(5),V(6),V(7),V(8),V(9),V(10),V(11),
 V(12),V(13),V(14),V(15),V(16),V(17),V(18),V(19),V(20),V(21),V(22),V(23),
 V(24),V(25),V(26),V(27),V(28),V(29),V(30),V(31)};
#undef V
struct Ag {uint8_t ctrl[16];uint16_t counter,reload,cma,cmb;int settling;bool goal;};
static Ag ag[2];
static uint32_t gp[8][64],iel[32],del[4],mstp;
static uint16_t prcr;
static uint8_t src,loco,sosc;
static uintptr_t fail_address;
static bool fail_once,start_fail,stop_fail;
static int step_on_count,step_on_flags;
static uint64_t injected_ticks, step_on_compare_write;
static unsigned writes,checks,overflow_calls,trigger_calls[6],last_event;
static void *last_context;
static void step(unsigned,uint64_t);
#define CHECK(x) do{assert(x);++checks;}while(0)
static uint32_t gate(unsigned dev){return 1U<<(dev<2?3-dev:dev<4?5:6);}
static unsigned agidx(uintptr_t a){unsigned n=(a-0x40084000UL)/256;assert(n<2&&!(mstp&gate(n)));return n;}
static unsigned gpidx(uintptr_t a){unsigned n=(a-0x40078000UL)/256;assert(n<8&&!(mstp&gate(n+2)));return n;}
static void event(unsigned e)
{
 for(unsigned i=0;i<32;++i)if((iel[i]&255U)==e){iel[i]|=RA4M1_IELS_IR;test_pending|=1U<<i;}
}
static bool fail(uintptr_t a){if(a!=fail_address)return false;if(fail_once)fail_address=0;return true;}
static uint8_t read8(uintptr_t a)
{
 if(a==RA4M1_LOCOCR)return loco;if(a==RA4M1_SOSCCR)return sosc;if(a==RA4M1_SCKSCR)return src;
 if(a>=RA4M1_IRQCR(0)&&a<=RA4M1_IRQCR(15))return 0;
 unsigned n=agidx(a),o=a&255U;Ag &g=ag[n];assert(o>=8&&o<=15);
 if(g.settling){assert(o==RA4M1_AGTCR);if(--g.settling==0){
  if(g.goal&&!start_fail)g.ctrl[o]|=2;if(!g.goal&&!stop_fail)g.ctrl[o]&=~2U;
 }}
 if(o==RA4M1_AGTCR&&step_on_flags==(int)n){step_on_flags=-1;step(n,injected_ticks);}
 return g.ctrl[o];
}
static uint16_t read16(uintptr_t a)
{
 if(a==RA4M1_PRCR)return prcr;
 unsigned n=agidx(a),o=a&255U;Ag &g=ag[n];assert(!g.settling);
 if(o==0){uint16_t c=g.counter;if(step_on_count==(int)n){step_on_count=-1;step(n,injected_ticks);}return c;}
 if(o==2)return g.cma;if(o==4)return g.cmb;assert(false);return 0;
}
static uint32_t read32(uintptr_t a)
{
 assert(!(a&3U));
 if(a==RA4M1_TIMER_MSTPCRD)return mstp;
 if(a>=RA4M1_IELSR(0)&&a<=RA4M1_IELSR(31))return iel[(a-RA4M1_IELSR(0))/4];
 if(a>=RA4M1_DELSR(0)&&a<=RA4M1_DELSR(3))return del[(a-RA4M1_DELSR(0))/4];
 unsigned n=gpidx(a),o=a&255U;assert(o<=0xA0);uint32_t v=gp[n][o/4];
 if(o==RA4M1_GTCNT&&step_on_count==(int)n+2){step_on_count=-1;step(n+2,injected_ticks);}
 if(o==RA4M1_GTST&&step_on_flags==(int)n+2){step_on_flags=-1;step(n+2,injected_ticks);v=gp[n][o/4];}
 return v;
}
static void write8(uintptr_t a,uint8_t v)
{
 assert(test_primask);++writes;if(fail(a))return;
 unsigned n=agidx(a),o=a&255U;Ag &g=ag[n];assert(!g.settling);assert(o>=8&&o<=15);
 if(o==RA4M1_AGTCR){
  assert(!(v&~1U));bool change=(v&1U)!=(g.ctrl[o]&1U);
  g.ctrl[o]=(g.ctrl[o]&2U)|(v&1U);if(change){g.goal=v&1;g.settling=((g.ctrl[o]&2U)!=((v&1U)<<1))?3:0;}
 }else{assert(!(g.ctrl[RA4M1_AGTCR]&3U));g.ctrl[o]=v;}
}
static void write16(uintptr_t a,uint16_t v)
{
 assert(test_primask);++writes;if(fail(a))return;
 if(a==RA4M1_PRCR){assert((v&0xFF00)==0xA500);prcr=v&15U;return;}
 unsigned n=agidx(a),o=a&255U;Ag &g=ag[n];assert(!g.settling&&!(g.ctrl[8]&3U));
 if(o==0)g.counter=g.reload=v;else if(o==2)g.cma=v;else if(o==4)g.cmb=v;else assert(false);
}
static void write32(uintptr_t a,uint32_t v)
{
 assert(test_primask);assert(!(a&3U));++writes;if(fail(a))return;
 if(a==RA4M1_TIMER_MSTPCRD){assert(prcr&2U);mstp=v;return;}
 if(a>=RA4M1_IELSR(0)&&a<=RA4M1_IELSR(31)){assert(!(v&~255U));iel[(a-RA4M1_IELSR(0))/4]=v;return;}
 unsigned n=gpidx(a),o=a&255U;assert(o<=0xA0);
 if(o==RA4M1_GTWP){assert((v&~1U)==0xA500);gp[n][0]=v&1U;return;}
 assert(!gp[n][0]);
 if(o==RA4M1_GTST){assert(v==0);gp[n][o/4]=0;return;}
 if(o==RA4M1_GTCR){assert(!(v&~0x07000001U));if(start_fail&&(v&1U))return;if(stop_fail&&!(v&1U))return;}
 if(o==RA4M1_GTCNT||o==RA4M1_GTPR)assert(!(gp[n][RA4M1_GTCR/4]&1U));
 if(o==RA4M1_GTINTAD||o==RA4M1_GTUPSR||o==RA4M1_GTDNSR||o==RA4M1_GTIOR||o==RA4M1_GTBER)assert(v==0);
 if(n>=2 && (o==RA4M1_GTCNT||o==RA4M1_GTPR||o==RA4M1_GTPBR||(o>=0x4C&&o<=0x60)))assert(!(v&0xFFFF0000U));
 gp[n][o/4]=v;
 if(o>=0x4C&&o<=0x60&&step_on_compare_write){uint64_t advance=step_on_compare_write;step_on_compare_write=0;step(n+2,advance);}
}
static void step(unsigned dev,uint64_t n)
{
 if(dev<2){Ag &g=ag[dev];if(!(g.ctrl[8]&2U))return;
  uint64_t cycle=(uint64_t)g.reload+1,pos=g.reload-g.counter;
  if(pos+n>=cycle){g.ctrl[8]|=0x20;event(0x1EU +3*dev);}
  g.counter=(uint16_t)(g.reload-(pos+n)%cycle);
 }else{
  unsigned h=dev-2;if(mstp&gate(dev)||!(gp[h][RA4M1_GTCR/4]&1U))return;
  uint64_t c=gp[h][RA4M1_GTCNT/4],cycle=(uint64_t)gp[h][RA4M1_GTPR/4]+1U;
  for(unsigned k=0;k<6;++k){uint32_t match=gp[h][s_Ra4m1GtccrOffset[k]/4];uint64_t dist=(match+cycle-c)%cycle;
   if(!dist)dist=cycle;if(n>=dist){gp[h][RA4M1_GTST/4]|=1U<<k;event(0x57+8*h+k);}
  }
  if(c+n>=cycle){gp[h][RA4M1_GTST/4]|=0x40;event(0x5D+8*h);}
  gp[h][RA4M1_GTCNT/4]=(uint32_t)((c+n)%cycle);
 }
}
static void dispatch(int irq)
{
 assert(irq>=0&&irq<32);if(!(test_enabled&(1U<<irq))||!(iel[irq]&RA4M1_IELS_IR))return;
 test_pending&=~(1U<<irq);test_active|=1U<<irq;__Vectors[16+irq]();test_active&=~(1U<<irq);
}
static void serve(unsigned dev)
{
 dispatch(s_Timer[dev].Irq);
 if(dev>=2)for(unsigned i=0;i<6;++i)if(s_Timer[dev].Trigger[i].Irq>=0)dispatch(s_Timer[dev].Trigger[i].Irq);
}
static void reset_model()
{
 memset(ag,0,sizeof(ag));memset(gp,0,sizeof(gp));memset(iel,0,sizeof(iel));memset(del,0,sizeof(del));
 memset(s_Timer,0,sizeof(s_Timer));memset(s_IntHook,0,sizeof(s_IntHook));memset(s_AgtTrigger,0,sizeof(s_AgtTrigger));memset(s_GptTrigger,0,sizeof(s_GptTrigger));
 for(auto &g:gp)g[0]=1;
 for(auto &g:ag)g.counter=g.reload=65535;
 memset(test_priority,0,sizeof(test_priority));memset(trigger_calls,0,sizeof(trigger_calls));
 test_primask=test_disabled=test_cleared=test_enabled=test_active=test_pending=0;
 mstp=0xFFFFFFFF;prcr=8;src=0;loco=0;sosc=1;fail_address=0;fail_once=start_fail=stop_fail=false;
 step_on_count=step_on_flags=-1;injected_ticks=step_on_compare_write=0;writes=overflow_calls=last_event=0;last_context=nullptr;
 SystemCoreClock=48000000;clocks[0]=clocks[2]=clocks[3]=48000000;clocks[1]=24000000;
 g_McuOsc={{OSC_TYPE_RC,48000000,0,0},{OSC_TYPE_RC,32768,0,0},false};
 for(unsigned i=0;i<10;++i)g_Ra4m1TimerError[i]=0;
}
static void evt(TimerDev_t *,uint32_t e){assert(!test_primask);last_event=e;if(e==TIMER_EVT_COUNTER_OVR)++overflow_calls;else for(unsigned i=0;i<6;++i)if(e==TIMER_EVT_TRIGGER(i))++trigger_calls[i];}
static void cb(TimerDev_t *,int n,void *ctx){assert(!test_primask);++trigger_calls[n];last_context=ctx;}
static bool init(TimerDev_t &t,int n,uint32_t f=0,TIMER_CLKSRC clk=TIMER_CLKSRC_DEFAULT){TimerCfg_t c={n,clk,f,3,evt,false};return TimerInit(&t,&c);}
static uint64_t ns(unsigned ticks,uint32_t f){return (1000000000ULL*ticks+f/2U)/f;}
static void fresh_cb(TimerDev_t *t,int n,void *ctx){cb(t,n,ctx);step(t->DevNo,s_Timer[t->DevNo].Trigger[n].Ticks);}
static void rearm_cb(TimerDev_t *t,int n,void *ctx){cb(t,n,ctx);CHECK(t->EnableTrigger(t,n,ns(19,t->Freq),TIMER_TRIG_TYPE_SINGLE,cb,ctx)>0);}
static void disable_cb(TimerDev_t *t,int n,void *ctx){cb(t,n,ctx);t->Disable(t);}
static void cancel_evt(TimerDev_t *t,uint32_t e){evt(t,e);if(e==TIMER_EVT_COUNTER_OVR)t->DisableTrigger(t,0);}
int main()
{
 CHECK(TimerGetLowFreqDevCount()==2&&TimerGetHighFreqDevCount()==8&&TimerGetHighFreqDevNo()==2);
 reset_model();TimerDev_t t={};TimerCfg_t bad={-1,TIMER_CLKSRC_DEFAULT,0,3,evt,false};
 CHECK(!TimerInit(&t,&bad)&&!TimerInit(nullptr,&bad)&&!TimerInit(&t,nullptr));bad.DevNo=10;CHECK(!TimerInit(&t,&bad));
 bad.DevNo=0;bad.bTickInt=true;CHECK(!TimerInit(&t,&bad));bad.bTickInt=false;bad.IntPrio=16;CHECK(!TimerInit(&t,&bad));
 CHECK(writes==0);CHECK(!init(t,0,0,TIMER_CLKSRC_EXT)&&!init(t,0,0,TIMER_CLKSRC_HFRC)&&!init(t,2,0,TIMER_CLKSRC_LFRC));
 CHECK(!init(t,0,0,TIMER_CLKSRC_LFXTAL));loco=1;CHECK(!init(t,0));
 reset_model();g_McuOsc.LowPwrOsc.Type=OSC_TYPE_XTAL;sosc=0;CHECK(init(t,1));CHECK(ag[1].ctrl[9]==0x60);
 reset_model();clocks[3]=0;CHECK(!init(t,2));
 reset_model();src=3;g_McuOsc.CoreOsc.Type=OSC_TYPE_XTAL;CHECK(init(t,2,0,TIMER_CLKSRC_HFXTAL));
 reset_model();CHECK(!init(t,2,0,TIMER_CLKSRC_HFXTAL));
 for(int dev=0;dev<10;++dev){reset_model();t={};t.pObj=&checks;CHECK(init(t,dev));CHECK(t.pObj==&checks&&t.DevNo==dev&&prcr==8&&!test_primask);
  CHECK(t.Freq==(dev<2?32768U:48000000U));CHECK(t.GetMaxTrigger(&t)==(dev<2?1:6));CHECK(t.GetTickCount(&t)==0);
  CHECK((test_enabled&(1U<<s_Timer[dev].Irq))!=0);CHECK(test_priority[s_Timer[dev].Irq]==3);
  CHECK(dev<2?(mstp&gate(dev))!=0:!(mstp&gate(dev)));step(dev,19);CHECK(t.GetTickCount(&t)==19);
  uint64_t cycle=s_Timer[dev].Cycle;step(dev,cycle-19);CHECK(t.GetTickCount(&t)==cycle);serve(dev);CHECK(t.GetTickCount(&t)==cycle&&overflow_calls==1);
  step(dev,7);CHECK(t.GetTickCount(&t)==cycle+7);t.Disable(&t);uint64_t saved=t.GetTickCount(&t);step(dev,100);CHECK(t.GetTickCount(&t)==saved);
  CHECK(t.Enable(&t));step(dev,9);CHECK(t.GetTickCount(&t)==saved+9);t.Reset(&t);CHECK(t.GetTickCount(&t)==0);
  CHECK(!t.EnableExtTrigger(&t,0,TIMER_EXTTRIG_SENSE_TOGGLE));t.DisableExtTrigger(&t);
 }
 // A read racing a hardware wrap must count the new epoch exactly once.
 for(int dev:{0,2,4}){reset_model();CHECK(init(t,dev));uint64_t cycle=s_Timer[dev].Cycle;step(dev,cycle-1);
  step_on_count=dev;injected_ticks=2;CHECK(t.GetTickCount(&t)==cycle+1);serve(dev);CHECK(t.GetTickCount(&t)==cycle+1);
 }
 reset_model();CHECK(init(t,2));TimerDev_t other={};CHECK(!init(other,2)&&!init(t,3));
 CHECK(init(other,3));t.Disable(&t);CHECK(other.Enable(&other)&&!(mstp&gate(3)));step(3,31);CHECK(other.GetTickCount(&other)==31);
 // Raw routes and running hardware must not be taken over.
 reset_model();iel[0]=0x57;CHECK(!init(t,2)&&writes==0);
 reset_model();del[1]=0x1F;CHECK(!init(t,0)&&writes==0);
 reset_model();gp[0][RA4M1_GTCR/4]=1;mstp&=~gate(2);CHECK(!init(t,2)&&gp[0][RA4M1_GTCR/4]==1);
 reset_model();fail_address=RA4M1_TIMER_MSTPCRD;CHECK(!init(t,2)&&!s_Timer[2].Timer);
 reset_model();fail_address=RA4M1_GTWP+RA4M1_GPT_BASE(0);CHECK(!init(t,2)&&test_enabled==0);
 reset_model();for(unsigned i=0;i<32;++i)iel[i]=1;CHECK(!init(t,0)&&test_enabled==0&&!s_Timer[0].Timer);CHECK(g_Ra4m1TimerError[0]==RA4M1_TIMER_IRQ_ERROR);
 reset_model();start_fail=true;CHECK(!init(t,0)&&test_enabled==0&&!s_Timer[0].Timer);
 reset_model();CHECK(init(t,0));stop_fail=true;t.Disable(&t);CHECK(g_Ra4m1TimerError[0]==RA4M1_TIMER_STOP_TIMEOUT&&!t.Enable(&t));
 // Frequency quantization and reset/restart contract.
 reset_model();CHECK(init(t,0,1000)&&t.Freq==1024);CHECK(t.SetFrequency(&t,0)==32768);CHECK(t.SetFrequency(&t,1)==256);
 reset_model();CHECK(init(t,4,1000000)&&t.Freq==750000);CHECK(t.SetFrequency(&t,0)==48000000);
 CHECK(t.SetFrequency(&t,1)==46875);step(4,55);CHECK(t.SetFrequency(&t,12000000)==12000000&&t.GetTickCount(&t)==0);
 fail_address=RA4M1_GPT_BASE(2)+RA4M1_GTCR;fail_once=false;CHECK(t.SetFrequency(&t,750000)==0);
 // AGT uses stable hardware reload, not ISR-driven stop/restart compare.
 for(unsigned dev=0;dev<2;++dev){reset_model();CHECK(init(t,dev));step(dev,23);CHECK(t.EnableTrigger(&t,0,ns(101,t.Freq),TIMER_TRIG_TYPE_CONTINUOUS,cb,&checks)>0);
  CHECK(t.GetTickCount(&t)==23&&ag[dev].reload==100);unsigned before=writes;step(dev,101);serve(dev);
  CHECK(trigger_calls[0]==1&&last_context==&checks&&t.GetTickCount(&t)==124);CHECK(ag[dev].reload==100);CHECK(writes>before);
  step(dev,101);serve(dev);CHECK(trigger_calls[0]==2&&t.GetTickCount(&t)==225);CHECK(t.FindAvailTrigger(&t)==-1);
  t.DisableTrigger(&t,0);CHECK(t.FindAvailTrigger(&t)==0&&ag[dev].reload==65535&&t.GetTickCount(&t)==225);
  CHECK(t.EnableTrigger(&t,0,ns(99,t.Freq),TIMER_TRIG_TYPE_SINGLE,cb,nullptr)>0);step(dev,99);serve(dev);step(dev,99);serve(dev);CHECK(trigger_calls[0]==3);
  CHECK(!t.EnableTrigger(&t,1,1000000000,TIMER_TRIG_TYPE_SINGLE,cb,nullptr));CHECK(!t.EnableTrigger(&t,0,UINT64_MAX,TIMER_TRIG_TYPE_SINGLE,cb,nullptr));
 }
 reset_model();CHECK(init(t,0));t.EvtHandler=cancel_evt;CHECK(t.EnableTrigger(&t,0,ns(71,t.Freq),TIMER_TRIG_TYPE_SINGLE,cb,nullptr)>0);step(0,71);serve(0);CHECK(trigger_calls[0]==0);
 // Six independent GPT compares, D/E physical register ordering, rearm phase.
 for(unsigned dev:{2U,4U}){reset_model();CHECK(init(t,dev));for(unsigned k=0;k<6;++k){
  CHECK(t.EnableTrigger(&t,k,ns(101+k,t.Freq),TIMER_TRIG_TYPE_CONTINUOUS,cb,&checks)>0);
  CHECK(gp[dev-2][s_Ra4m1GtccrOffset[k]/4]==101+k);
 }CHECK(t.FindAvailTrigger(&t)==-1);step(dev,113);serve(dev);
  for(unsigned k=0;k<6;++k){CHECK(trigger_calls[k]==1);CHECK(s_Timer[dev].Trigger[k].Deadline==2*(101+k));}
  step(dev,400);serve(dev);for(unsigned k=0;k<6;++k){CHECK(trigger_calls[k]==2);CHECK(s_Timer[dev].Trigger[k].Deadline>513);CHECK(s_Timer[dev].Trigger[k].Deadline%(101+k)==0);}
  t.DisableTrigger(&t,3);CHECK(t.FindAvailTrigger(&t)==3&&s_Timer[dev].Trigger[3].Irq<0);
 }
 // Preserve the non-aligned software origin when resuming a paused counter.
 reset_model();CHECK(init(t,4));step(4,37);CHECK(t.EnableTrigger(&t,0,ns(101,t.Freq),TIMER_TRIG_TYPE_CONTINUOUS,cb,nullptr)>0);
 t.Disable(&t);CHECK(t.Enable(&t));CHECK(gp[2][0x4C/4]==101);step(4,101);serve(4);CHECK(trigger_calls[0]==1&&t.GetTickCount(&t)==138);
 // Compare target crossing wrap; service compare before overflow on purpose.
 reset_model();CHECK(init(t,4));step(4,65520);CHECK(t.EnableTrigger(&t,4,ns(31,t.Freq),TIMER_TRIG_TYPE_SINGLE,cb,nullptr)>0);
 step(4,31);dispatch(s_Timer[4].Trigger[4].Irq);CHECK(trigger_calls[4]==1&&t.GetTickCount(&t)==65551);dispatch(s_Timer[4].Irq);CHECK(t.GetTickCount(&t)==65551);
 // Fresh callback-time events survive dispatcher exit.
 reset_model();CHECK(init(t,2));CHECK(t.EnableTrigger(&t,0,ns(101,t.Freq),TIMER_TRIG_TYPE_CONTINUOUS,fresh_cb,nullptr)>0);
 step(2,101);serve(2);int ir=s_Timer[2].Trigger[0].Irq;CHECK((iel[ir]&RA4M1_IELS_IR)&&(test_pending&(1U<<ir)));CHECK(trigger_calls[0]==1);
 // A one-shot callback may rearm itself without another allocation.
 reset_model();CHECK(init(t,2));CHECK(t.EnableTrigger(&t,2,ns(101,t.Freq),TIMER_TRIG_TYPE_SINGLE,rearm_cb,&checks)>0);
 step(2,101);serve(2);CHECK(trigger_calls[2]==1);step(2,19);serve(2);CHECK(trigger_calls[2]==2);
 reset_model();CHECK(init(t,2));CHECK(t.EnableTrigger(&t,0,ns(101,t.Freq),TIMER_TRIG_TYPE_CONTINUOUS,disable_cb,nullptr)>0);step(2,101);serve(2);CHECK(!s_Timer[2].Running);
 // Invalid requests and allocation/write failures do not invent triggers.
 reset_model();CHECK(init(t,4));CHECK(!t.EnableTrigger(&t,-1,1000,TIMER_TRIG_TYPE_SINGLE,cb,nullptr));CHECK(!t.EnableTrigger(&t,6,1000,TIMER_TRIG_TYPE_SINGLE,cb,nullptr));
 CHECK(!t.EnableTrigger(&t,0,1,TIMER_TRIG_TYPE_SINGLE,cb,nullptr));CHECK(!t.EnableTrigger(&t,0,UINT64_MAX,TIMER_TRIG_TYPE_SINGLE,cb,nullptr));
 for(unsigned i=1;i<32;++i)iel[i]=1;CHECK(!t.EnableTrigger(&t,0,10000,TIMER_TRIG_TYPE_SINGLE,cb,nullptr));CHECK(t.FindAvailTrigger(&t)==0);
 reset_model();CHECK(init(t,4));fail_address=RA4M1_GPT_BASE(2)+0x4C;CHECK(!t.EnableTrigger(&t,0,10000,TIMER_TRIG_TYPE_SINGLE,cb,nullptr));CHECK(t.FindAvailTrigger(&t)==0);
 // Reset and changing frequency retain active trigger durations, reset phase.
 reset_model();CHECK(init(t,2,12000000));CHECK(t.EnableTrigger(&t,0,1000000,TIMER_TRIG_TYPE_CONTINUOUS,nullptr,nullptr)==1000000);
 step(2,12000);serve(2);CHECK(trigger_calls[0]==1);t.Reset(&t);CHECK(t.GetTickCount(&t)==0&&gp[0][0x4C/4]==12000);
 CHECK(t.SetFrequency(&t,3000000)==3000000&&gp[0][0x4C/4]==3000);step(2,3000);serve(2);CHECK(trigger_calls[0]==2);
 reset_model();CHECK(init(t,0));CHECK(t.EnableTrigger(&t,0,1000000000,TIMER_TRIG_TYPE_CONTINUOUS,nullptr,nullptr)==1000000000);
 CHECK(t.SetFrequency(&t,1024)==1024&&ag[0].reload==1023);step(0,1024);serve(0);CHECK(trigger_calls[0]==1);
 // Programming latency: report a missed single shot, skip a missed periodic
 // phase without waiting for the counter to wrap.
 reset_model();CHECK(init(t,2));step_on_compare_write=150;
 CHECK(t.EnableTrigger(&t,0,ns(101,t.Freq),TIMER_TRIG_TYPE_SINGLE,cb,nullptr)==0);
 CHECK(g_Ra4m1TimerError[2]==RA4M1_TIMER_COMPARE_ERROR&&t.FindAvailTrigger(&t)==0);
 reset_model();CHECK(init(t,2));CHECK(t.EnableTrigger(&t,0,ns(101,t.Freq),TIMER_TRIG_TYPE_CONTINUOUS,cb,nullptr)>0);
 step(2,101);step_on_compare_write=150;serve(2);CHECK(trigger_calls[0]==1&&s_Timer[2].Trigger[0].Deadline==303);
 step(2,52);serve(2);CHECK(trigger_calls[0]==2);
 // The AGT interrupt must not erase another underflow produced in its callback.
 reset_model();CHECK(init(t,0));CHECK(t.EnableTrigger(&t,0,ns(101,t.Freq),TIMER_TRIG_TYPE_CONTINUOUS,fresh_cb,nullptr)>0);
 step(0,101);serve(0);CHECK((iel[s_Timer[0].Irq]&RA4M1_IELS_IR)!=0);CHECK(t.GetTickCount(&t)==202);
 // All devices can coexist; their shared GPT gates remain intact.
 reset_model();TimerDev_t all[10]={};for(unsigned n=0;n<10;++n)CHECK(init(all[n],n));
 for(unsigned n=0;n<10;++n){step(n,100+n);CHECK(all[n].GetTickCount(&all[n])==100+n);}
 CHECK((mstp&(gate(2)|gate(4)))==0&&(mstp&(gate(0)|gate(1)))==(gate(0)|gate(1)));
 // Preserve an outer mask; the driver never unconditionally enables IRQs.
 reset_model();test_primask=1;CHECK(init(t,2)&&test_primask==1);t.Disable(&t);CHECK(test_primask==1);CHECK(t.Enable(&t)&&test_primask==1);
 printf("PASS: %u timer/ICU assertions (actual source, reduced headers)\n",checks);
 return 0;
}
