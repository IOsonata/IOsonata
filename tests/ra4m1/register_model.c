/* Host model for the actual system_ra4m1.c. Not a hardware emulator. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#define RA4M1_HOST_TEST 1
#define RA4M1_STARTUP_TIMEOUT 32U
static uint8_t rd8(uintptr_t);
static uint16_t rd16(uintptr_t);
static uint32_t rd32(uintptr_t);
static void wr8(uintptr_t,uint8_t);
static void wr16(uintptr_t,uint16_t);
static void wr32(uintptr_t,uint32_t);
static void delay_us(uint32_t);
#define RA4M1_RD8(a) rd8(a)
#define RA4M1_RD16(a) rd16(a)
#define RA4M1_RD32(a) rd32(a)
#define RA4M1_WR8(a,v) wr8(a,v)
#define RA4M1_WR16(a,v) wr16(a,v)
#define RA4M1_WR32(a,v) wr32(a,v)
#define RA4M1_TEST_DELAY_US(us) delay_us(us)
#include "../../ARM/Renesas/RA4M1/src/system_ra4m1.c"
SCB_Type test_scb;
uint32_t test_primask, test_disabled, test_cleared;
uint32_t SystemMicroSecLoopCnt;
void (*const __Vectors[48])(void) = {0};
static uint8_t sys[0x500];
static uint16_t protect,cache,invalid,usb_cfg;
static uint32_t dividers,links[32],writes,delays,input_hz;
static uintptr_t fail_addr;
static uint8_t stuck_ready;
static bool fail_invalidate;
static unsigned checks;
static uint32_t nominal(void)
{
 unsigned sel=sys[0x26]&7;
 if(sel==1)return 8000000;
 if(sel==0){unsigned h=(sys[0]>>3)&7;return h==0?24000000:h==2?32000000:h==4?48000000:h==5?64000000:0;}
 if(sel==3)return input_hz;
 if(sel==5){unsigned c=sys[0x2B];return (input_hz*((c&31)+1))/(1U<<(c>>6));}
 return 32768;
}
static void check_clocks(void)
{
 uint32_t f=nominal(), ic=f>>((dividers>>24)&7),pa=f>>((dividers>>12)&7);
 uint32_t pb=f>>((dividers>>8)&7),pc=f>>((dividers>>4)&7),pd=f>>(dividers&7),fc=f>>((dividers>>28)&7);
 assert(ic<=48000000 && pa<=48000000 && pb<=32000000 && pc<=64000000 && pd<=64000000 && fc<=32000000);
 assert(ic>=pa && pa>=pb && pd>=pa && ic>=fc);
 assert(((dividers>>16)&7)==((dividers>>8)&7));
 if(ic>32000000){assert(sys[0x31]==1);assert((sys[0xA0]&3)==0);}
}
static uint8_t rd8(uintptr_t a){assert(a>=0x4001E000UL && a<0x4001E500UL);return sys[a-0x4001E000UL];}
static uint16_t rd16(uintptr_t a){if(a==RA4M1_PRCR)return protect;if(a==RA4M1_FCACHEE)return cache;if(a==RA4M1_FCACHEIV)return invalid;assert(a==RA4M1_USB_SYSCFG);return usb_cfg;}
static uint32_t rd32(uintptr_t a){if(a==RA4M1_SCKDIVCR)return dividers;assert(a>=RA4M1_IELSR(0)&&a<=RA4M1_IELSR(31));return links[(a-RA4M1_IELSR(0))/4];}
static void wr8(uintptr_t a,uint8_t v)
{
 ++writes;assert(test_primask==1);assert(a>=0x4001E000UL&&a<0x4001E500UL);
 if(a==fail_addr)return;
 assert(protect & (a==RA4M1_OPCCR || a==RA4M1_SOPCCR?2:1));
 if(a==RA4M1_HOCOCR2){assert(sys[0x36]&1);assert(v==0||v==0x10||v==0x20||v==0x28);}
 if(a==RA4M1_HOCOCR && v==1){assert((sys[0xA0]&3)!=2);assert((sys[0x26]&7)!=0);}
 if(a==RA4M1_HOCOWTCR){assert((sys[0x36]&1)||(sys[0x3C]&1));assert(v==5||v==6);}
 if(a==RA4M1_OPCCR){assert(cache==0);assert((sys[0x36]&1)||(sys[0x3C]&1));assert((sys[0xA0]&0x10)==0);}
 if(a==RA4M1_MEMWAIT){assert(cache==0);assert((sys[0xA0]&3)==0);assert(nominal()>>((dividers>>24)&7)<=32000000);}
 if(a==RA4M1_MOMCR||a==RA4M1_MOSCWTCR)assert((sys[0x32]&1)&&!(sys[0x3C]&8));
 if(a==RA4M1_MOSCWTCR)assert(v<=9);
 if(a==RA4M1_PLLCCR2){assert(sys[0x2A]&1);assert((v&31)>=7&&(v&31)<=30);assert((v>>6)==1||(v>>6)==2);delays=0;}
 if(a==RA4M1_PLLCR && v==0){assert(delays>=1);assert(sys[0x3C]&8);}
 if(a==RA4M1_SOMCR)assert(sys[0x33]&1);
 if(a==RA4M1_USBCKCR)assert(!(usb_cfg&RA4M1_USB_SCKE));
 if(a==RA4M1_SCKSCR){assert(v<=5);if(v==0)assert(sys[0x3C]&1);if(v==3)assert(sys[0x3C]&8);if(v==5)assert(sys[0x3C]&0x20);}
 sys[a-0x4001E000UL]=v;
 unsigned bit=a==RA4M1_HOCOCR?1:a==RA4M1_MOSCCR?8:a==RA4M1_PLLCR?32:0;
 if(bit){if(v&1)sys[0x3C]&=~bit;else if(!(stuck_ready&bit))sys[0x3C]|=bit;}
 if(a==RA4M1_SCKSCR)check_clocks();
}
static void wr16(uintptr_t a,uint16_t v)
{
 ++writes;assert(test_primask==1);
 if(a==fail_addr)return;
 if(a==RA4M1_PRCR){assert((v&0xFF00)==0xA500);protect=v&0xB;return;}
 if(a==RA4M1_FCACHEE){cache=v;return;}
 assert(a==RA4M1_FCACHEIV);assert(cache==0);invalid=fail_invalidate?v:0;
}
static void wr32(uintptr_t a,uint32_t v)
{
 ++writes;assert(test_primask==1);if(a==fail_addr)return;
 if(a==RA4M1_SCKDIVCR){assert(protect&1);dividers=v;check_clocks();return;}
 assert(a>=RA4M1_IELSR(0)&&a<=RA4M1_IELSR(31));links[(a-RA4M1_IELSR(0))/4]=v;
}
static void delay_us(uint32_t us){delays+=us;}
static void reset_model(void)
{
 memset(sys,0,sizeof(sys));memset(&test_scb,0,sizeof(test_scb));
 sys[0x26]=1;sys[0x2A]=sys[0x32]=sys[0x33]=sys[0x34]=1;sys[0x3C]=1;sys[0xA0]=2;sys[0xA5]=5;
 dividers=RA4M1_RESET_DIV;input_hz=12000000;protect=cache=invalid=usb_cfg=0;
 writes=delays=0;fail_addr=0;stuck_ready=0;fail_invalidate=false;s_MainOscFreq=0;
 test_primask=test_disabled=test_cleared=0;
 g_McuOsc=(McuOsc_t){{OSC_TYPE_RC,48000000,0,0},{OSC_TYPE_RC,32768,0,0},false};
 g_Ra4m1StartupError=0;SystemCoreClock=500000;
 for(unsigned i=0;i<32;i++)links[i]=0x0101002B;
}
static void pass(const char *name){++checks;printf("PASS %u: %s\n",checks,name);}
int main(void)
{
 reset_model();SystemInit();assert(SystemCoreClock==48000000);assert(dividers==0x10010100);assert(sys[0x31]==1&&cache==1&&protect==0);
 assert(test_disabled==0xFFFFFFFFU && test_cleared==0xFFFFFFFFU && test_primask==0);
 assert(test_scb.CPACR==0xF00000 && test_scb.VTOR==(uint32_t)(uintptr_t)__Vectors);
 for(unsigned i=0;i<32;i++)assert(links[i]==0);pass("48 MHz cold startup, FPU, VTOR, all 32 IRQs disabled/unrouted");
 assert(SystemPeriphClockGet(0)==48000000&&SystemPeriphClockGet(1)==24000000&&SystemPeriphClockGet(2)==48000000&&SystemPeriphClockGet(3)==48000000&&SystemFlashClockGet()==24000000);pass("48 MHz bus plan and reserved PCKB mirror");
 uint32_t before=writes;SystemCoreClockUpdate();(void)SystemCoreClockGet();for(int i=-1;i<=4;i++)(void)SystemPeriphClockGet(i);(void)SystemFlashClockGet();(void)SystemUsbClockGet();assert(writes==before);pass("all clock queries are hardware-read-only");
 assert(SystemPeriphClockGet(-1)==0&&SystemPeriphClockGet(4)==0&&SystemPeriphClockSet(1,24000000)==24000000&&SystemPeriphClockSet(1,12000000)==0);pass("invalid bus indices and unsupported divider changes");
 const uint32_t rcs[]={8000000,24000000,32000000,48000000};
 for(unsigned i=0;i<4;i++){reset_model();assert(SystemCoreClockSelect(OSC_TYPE_RC,rcs[i]));assert(SystemCoreClock==rcs[i]);assert(sys[0x31]==(rcs[i]>32000000));pass("supported RC source and Flash boundary");}
 reset_model();assert(SystemCoreClockSelect(OSC_TYPE_RC,48000000));assert(SystemCoreClockSelect(OSC_TYPE_RC,24000000));assert(sys[0x31]==0&&SystemCoreClock==24000000);assert(SystemCoreClockSelect(OSC_TYPE_RC,48000000));pass("quiesced 48-to-24-to-48 MHz transition ordering");
 const uint32_t bad[]={0,1,32768,16000000,40000000,64000000,0xFFFFFFFFU};
 for(unsigned i=0;i<7;i++){reset_model();assert(!SystemCoreClockSelect(OSC_TYPE_RC,bad[i]));assert(writes==0&&SystemCoreClock==500000);pass("invalid RC frequency rejected without register writes");}
 reset_model();assert(!SystemCoreClockSelect((OSC_TYPE)99,12000000));assert(writes==0);pass("unknown oscillator type rejected");
 const uint32_t exts[]={4000000,6000000,8000000,12000000};
 for(unsigned i=0;i<4;i++){reset_model();input_hz=exts[i];g_McuOsc.bUSBClk=true;assert(SystemCoreClockSelect(OSC_TYPE_XTAL,input_hz));assert(SystemCoreClock==48000000&&SystemUsbClockGet()==48000000);assert(sys[0xD0]==0);pass("external crystal to exact 48 MHz PLL and USB source");}
 reset_model();input_hz=12000000;assert(SystemCoreClockSelect(OSC_TYPE_TCXO,input_hz));assert(sys[0x413]==0x40&&sys[0xA2]==0&&SystemCoreClock==48000000);pass("external-clock bypass and zero MOSC wait");
 const uint32_t direct[]={1000000,10000000,12288000,16000000,20000000};
 for(unsigned i=0;i<5;i++){reset_model();input_hz=direct[i];assert(SystemCoreClockSelect(OSC_TYPE_XTAL,input_hz));assert(SystemCoreClock==input_hz&&sys[0x26]==3&&sys[0x31]==0);pass("external frequencies without exact PLL solution use MOSC directly");}
 reset_model();g_McuOsc.bUSBClk=true;assert(SystemCoreClockSelect(OSC_TYPE_RC,48000000));assert(sys[0xD0]==1&&SystemUsbClockGet()==48000000);pass("48 MHz HOCO USB source, controller stays disabled");
 reset_model();g_McuOsc.bUSBClk=true;assert(!SystemCoreClockSelect(OSC_TYPE_XTAL,16000000));assert(writes==0);pass("USB request rejects non-48 MHz plan");
 reset_model();g_McuOsc.bUSBClk=true;sys[0x62]=1;assert(!SystemCoreClockSelect(OSC_TYPE_RC,48000000));assert(g_Ra4m1StartupError==RA4M1_STARTUP_USB_CLOCK&&SystemUsbClockGet()==0);pass("nonzero HOCO user trim is not silently used for USB");
 reset_model();usb_cfg=0x400;assert(!SystemCoreClockSelect(OSC_TYPE_RC,48000000));assert(g_Ra4m1StartupError==RA4M1_STARTUP_USB_CLOCK&&dividers==RA4M1_RESET_DIV);pass("clock change refused with USB SCKE enabled");
 for(unsigned mode=1;mode<=3;mode++){reset_model();sys[0xA0]=mode;assert(SystemCoreClockSelect(OSC_TYPE_RC,48000000));assert(sys[0xA0]==0);pass("legal slow-clock operating-mode entry");}
 reset_model();sys[0xAA]=1;assert(!SystemCoreClockSelect(OSC_TYPE_RC,48000000));assert(g_Ra4m1StartupError==RA4M1_STARTUP_BAD_ENTRY);pass("Subosc-mode transition refused");
 reset_model();sys[0x40]=0x80;assert(!SystemCoreClockSelect(OSC_TYPE_RC,48000000));assert(g_Ra4m1StartupError==RA4M1_STARTUP_BAD_ENTRY);pass("enabled clock-stop detector prevents unsafe oscillator stop");
 const uintptr_t errors[]={RA4M1_PRCR,RA4M1_HOCOCR2,RA4M1_MEMWAIT,RA4M1_OPCCR,RA4M1_MOCOCR,RA4M1_SCKDIVCR,RA4M1_FCACHEE};
 for(unsigned i=0;i<sizeof(errors)/sizeof(errors[0]);i++){reset_model();fail_addr=errors[i];if(fail_addr==RA4M1_MOCOCR)sys[0x38]=1;assert(!SystemCoreClockSelect(OSC_TYPE_RC,48000000));assert(g_Ra4m1StartupError!=0);assert(protect==0&&test_primask==0);pass("ignored register write fails with lock/PRIMASK restored");}
 reset_model();stuck_ready=1;assert(!SystemCoreClockSelect(OSC_TYPE_RC,48000000));assert(g_Ra4m1StartupError==RA4M1_STARTUP_HOCO&&SystemCoreClock==500000);pass("HOCO lock timeout never publishes requested 48 MHz");
 reset_model();stuck_ready=8;assert(!SystemCoreClockSelect(OSC_TYPE_XTAL,12000000));assert(g_Ra4m1StartupError==RA4M1_STARTUP_MOSC&&SystemCoreClock==500000);pass("MOSC ready timeout");
 reset_model();stuck_ready=32;assert(!SystemCoreClockSelect(OSC_TYPE_XTAL,12000000));assert(g_Ra4m1StartupError==RA4M1_STARTUP_PLL&&SystemCoreClock==500000);pass("PLL lock timeout");
 reset_model();fail_invalidate=true;assert(!SystemCoreClockSelect(OSC_TYPE_RC,48000000));assert(g_Ra4m1StartupError==RA4M1_STARTUP_FLASH&&cache==0&&sys[0x31]==1);pass("cache invalidation timeout remains correctly timed and uncached");
 reset_model();protect=8;test_primask=1;assert(SystemCoreClockSelect(OSC_TYPE_RC,48000000));assert(protect==8&&test_primask==1);pass("nested PRIMASK and unrelated PRCR bits preserved");
 reset_model();assert(SystemLowFreqClockSelect(OSC_TYPE_RC,32768));assert(sys[0x34]==0&&delays>=100&&protect==0);pass("LOCO low-frequency source startup");
 reset_model();assert(SystemLowFreqClockSelect(OSC_TYPE_XTAL,32768));assert(sys[0x33]==0&&delays>=2000000&&protect==0);pass("SOSC drive setup and configurable stabilization wait");
 reset_model();sys[0x33]=0;sys[0x481]=2;assert(SystemLowFreqClockSelect(OSC_TYPE_XTAL,32768));assert(sys[0x481]==2);pass("running SOSC is preserved across reset/reselection");
 reset_model();assert(!SystemLowFreqClockSelect(OSC_TYPE_TCXO,32768)&&!SystemLowFreqClockSelect(OSC_TYPE_RC,32000));assert(writes==0);pass("unsupported low-frequency descriptors rejected");
 reset_model();fail_addr=RA4M1_SOSCCR;assert(!SystemLowFreqClockSelect(OSC_TYPE_XTAL,32768));assert(protect==0&&test_primask==0);pass("LF start failure restores protection");
 reset_model();dividers=0x77077777;assert(SystemCoreClockGet()==0&&SystemPeriphClockGet(0)==0);pass("prohibited divider encodings report unknown frequency");
 reset_model();sys[0x26]=7;assert(SystemCoreClockGet()==0);pass("reserved system source reports unknown frequency");
 reset_model();assert(Ra4m1PllHz(12000000,0x40|7)==48000000&&Ra4m1PllHz(12000000,0x20|0x40|7)==0);pass("PLL byte-width encoding and reserved-bit rejection");
 reset_model();input_hz=16000000;assert(SystemCoreClockSelect(OSC_TYPE_XTAL,input_hz));sys[0x41]=1;assert(SystemCoreClockGet()==8000000);pass("MOSC stop fallback reports MOCO rather than stale crystal rate");
 reset_model();assert(SystemCoreClockSelect(OSC_TYPE_XTAL,12000000));sys[0x41]=1;assert(SystemCoreClockGet()==0);pass("PLL free-running after input failure reports unknown rate");
 reset_model();sys[0x41]=1;assert(!SystemCoreClockSelect(OSC_TYPE_XTAL,12000000));assert(g_Ra4m1StartupError==RA4M1_STARTUP_BAD_ENTRY);pass("latched oscillator-stop fault requires explicit recovery");
 printf("%u register-model scenarios passed. No hardware execution.\n",checks);
}
