#include <cassert>
#include <cstdio>
#include "miscdev/buzzer.h"
#include "coredev/timer.h"
uint64_t g_DelayUs = 0;
// Base-class C entry points must never be called by these injected test devices.
extern "C" {
bool PWMInit(PwmDev_t*,const PwmCfg_t*) { assert(false); return false; }
bool PWMEnable(PwmDev_t*) { assert(false); return false; }
void PWMDisable(PwmDev_t*) { assert(false); }
bool PWMOpenChannel(PwmDev_t*,const PwmChanCfg_t*,int) { assert(false); return false; }
void PWMCloseChannel(PwmDev_t*,int) { assert(false); }
bool PWMStart(PwmDev_t*,uint32_t) { assert(false); return false; }
void PWMStop(PwmDev_t*) { assert(false); }
bool PWMSetFrequency(PwmDev_t*,uint32_t) { assert(false); return false; }
bool PWMSetDutyCycle(PwmDev_t*,int,int) { assert(false); return false; }
bool TimerInit(TimerDev_t*,const TimerCfg_t*) { assert(false); return false; }
}
struct TestPwm : Pwm {
 uint32_t hz=0; int duty=0; bool running=false; bool fail=false; int starts=0;
 bool Enable() override { return true; }
 bool Frequency(uint32_t f) override { assert(!running); hz=f; return true; }
 bool DutyCycle(int,int d) override { duty=d; return true; }
 bool Start(uint32_t duration=0) override { assert(duration==0); if(fail)return false; running=true; ++starts; return true; }
 void Stop() override { running=false; }
};
struct TestTimer : Timer { uint32_t now=0; uint32_t mSecond() override { return now; } };
int main() {
 TestPwm pwm; TestTimer timer; Buzzer buzzer; BuzzerMelody player;
 buzzer.Stop(); buzzer.Volume(75); assert(!buzzer.Start(440));
 assert(!buzzer.Init(nullptr,0)); assert(!buzzer.Init(&pwm,-1));
 assert(buzzer.Init(&pwm,0)); assert(buzzer.Start(440)); assert(pwm.duty==50);
 buzzer.Play((uint8_t)69,0); assert(pwm.hz==440);
 buzzer.Play((uint8_t)60,0); assert(pwm.hz==262);
 buzzer.Play((uint8_t)5,0); assert(pwm.hz==11);
 buzzer.Play((uint8_t)127,0); assert(pwm.hz==12544);
 buzzer.Play((uint8_t)128,0); assert(!pwm.running);
 buzzer.Play((uint32_t)5000,10); assert(g_DelayUs==10000 && !pwm.running);
 buzzer.Volume(0); assert(buzzer.Start(440) && !pwm.running);
 buzzer.Volume(100); pwm.fail=true; assert(!buzzer.Start(440) && !pwm.running); pwm.fail=false;
 assert(player.Init(&buzzer,&timer));
 const BuzzerNote_t notes[]={{440,24},{0,12},{660,12}};
 assert(player.Play(notes,3,120,1,20)); assert(pwm.hz==440 && pwm.running);
 auto delay=g_DelayUs;
 timer.now=479; player.Process(); assert(pwm.running);
 timer.now=480; player.Process(); assert(!pwm.running && player.IsPlaying());
 timer.now=500; player.Process(); assert(!pwm.running);
 timer.now=750; player.Process(); assert(pwm.running && pwm.hz==660);
 timer.now=1000; player.Process(); assert(!pwm.running && !player.IsPlaying() && !player.Failed());
 assert(g_DelayUs==delay); // Melody playback never calls the blocking delay API.
 const BuzzerNote_t single[]={{440,24}};
 timer.now=0xfffffff0; assert(player.Play(single,1,120,2,0));
 timer.now+=500; player.Process(); assert(player.IsPlaying());
 timer.now+=500; player.Process(); assert(!player.IsPlaying());
 assert(player.Play(single,1,240,0,20));
 for(int i=0;i<5;i++){timer.now+=250;player.Process();assert(player.IsPlaying());}
 player.Stop(); assert(!pwm.running && !player.IsPlaying());
 const BuzzerNote_t bad[]={{440,0}};
 assert(!player.Play(bad,1)); assert(player.Failed() && !pwm.running);
 assert(!player.Play(single,1,0)); assert(player.Failed());
 assert(player.Play(notes,3)); timer.now+=500;player.Process(); // rest
 pwm.fail=true;timer.now+=250;player.Process();assert(player.Failed()&&!player.IsPlaying()&&!pwm.running);
 pwm.fail=false; assert(player.Play(single,1,120,1,1000));timer.now+=1;player.Process();assert(!pwm.running);
 timer.now+=499;player.Process();assert(!player.IsPlaying());
 assert(player.Play(notes,3));auto starts=pwm.starts;timer.now+=100000;player.Process();assert(pwm.starts==starts); // only advances to rest
 puts("PASS: buzzer MIDI, mute, failures; melody timing, rests, tempo, repeat, cancel, wrap and delayed service");
}
