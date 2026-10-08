/* Actual-source host register model. Not a CPU, oscillator, or electrical model. */
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cstdint>
#define RA4M1_HOST_TEST 1
#define RA4M1_IO_TEST 1
#define RA4M1_IO_READY_POLLS 16U
static uint8_t rd8(uintptr_t);
static uint16_t rd16(uintptr_t);
static uint32_t rd32(uintptr_t);
static void wr8(uintptr_t,uint8_t);
static void wr16(uintptr_t,uint16_t);
static void wr32(uintptr_t,uint32_t);
#define RA4M1_RD8(a) rd8(a)
#define RA4M1_RD16(a) rd16(a)
#define RA4M1_RD32(a) rd32(a)
#define RA4M1_WR8(a,v) wr8(a,v)
#define RA4M1_WR16(a,v) wr16(a,v)
#define RA4M1_WR32(a,v) wr32(a,v)
#include "../../ARM/Renesas/RA4M1/src/interrupt_ra4m1.cpp"
#include "../../ARM/Renesas/RA4M1/src/iopincfg_ra4m1.c"
SCB_Type test_scb;
uint32_t test_primask, test_disabled, test_cleared, test_enabled, test_active, test_pending;
uint32_t test_priority[32];
static void override_irq(void) {}
#define V(n) Ra4m1DefaultIEL##n
extern "C" void (* const __Vectors[48])(void) = {
 nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,
 nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,
 V(0),V(1),V(2),V(3),V(4),V(5),V(6),V(7),V(8),V(9),V(10),override_irq,
 V(12),V(13),V(14),V(15),V(16),V(17),V(18),V(19),V(20),V(21),V(22),V(23),
 V(24),V(25),V(26),V(27),V(28),V(29),V(30),V(31)
};
#undef V
static uint32_t links[32],dma[4],wakeup,portregs[10][4],pfs[10][16];
static uint8_t pwpr,irqcr[16],mosc,sosc,vbt,valid,vin,vout;
static uint16_t protect;
static uintptr_t fail_addr;
static unsigned writes,checks,callback_count;
static int callback_arg;
static void *callback_ctx;
static bool pending_source;
static uint8_t rd8(uintptr_t a)
{
#ifdef RA4M1_UART_TEST
 if(a>=0x40070000UL && a<0x40070140UL) return UartRead8(a);
#endif
 if(a==RA4M1_PWPR)return pwpr;
 if(a>=RA4M1_IRQCR(0)&&a<=RA4M1_IRQCR(15)){assert(a!=RA4M1_IRQCR(13));return irqcr[a-RA4M1_IRQCR(0)];}
 if(a==RA4M1_MOSCCR)return mosc;
 if(a==RA4M1_SOSCCR)return sosc;
 if(a==RA4M1_VBTCR1)return vbt;
 if(a==RA4M1_VBTSR)return valid;
 if(a==RA4M1_VBTICTLR){assert(valid&0x10);return vin;}
 if(a==RA4M1_VBTOCTLR){assert(valid&0x10);return vout;}
 assert(false);return 0;
}
static uint16_t rd16(uintptr_t a){
#ifdef RA4M1_UART_TEST
 if(a>=0x40070000UL && a<0x40070040UL) return UartRead16(a);
#endif
 assert(a==RA4M1_PRCR);return protect;
}
static uint32_t rd32(uintptr_t a)
{
#ifdef RA4M1_UART_TEST
 if(a==0x40047000UL) return uart_mstp;
#endif
 assert(!(a&3));
 if(a>=RA4M1_IELSR(0)&&a<=RA4M1_IELSR(31))return links[(a-RA4M1_IELSR(0))/4];
 if(a>=RA4M1_DELSR(0)&&a<=RA4M1_DELSR(3))return dma[(a-RA4M1_DELSR(0))/4];
 if(a==RA4M1_WUPEN)return wakeup;
 if(a>=RA4M1_PFS(0,0)&&a<=RA4M1_PFS(9,15)) {
  unsigned p=(a-RA4M1_PFS(0,0))/0x40,n=((a-RA4M1_PFS(0,0))%0x40)/4;
  assert(Ra4m1PinValid(p,n));return pfs[p][n];
 }
 if(a>=RA4M1_PCNTR1(0)&&a<=RA4M1_PCNTR4(9)){
  unsigned p=(a-RA4M1_PCNTR1(0))/0x20,n=((a-RA4M1_PCNTR1(0))%0x20)/4;
  assert(Ra4m1PortMask(p));assert(n<4);if(n==3)assert(p>=1&&p<=4);return portregs[p][n];
 }
 assert(false);return 0;
}
static void wr8(uintptr_t a,uint8_t value)
{
 assert(test_primask);++writes;if(a==fail_addr)return;
 #ifdef RA4M1_UART_TEST
 if(a>=0x40070000UL && a<0x40070140UL){UartWrite8(a,value);return;}
#endif
 if(a==RA4M1_PWPR){if(pwpr&0x80)assert((pwpr&0x40)==(value&0x40));pwpr=value;return;}
 if(a==RA4M1_VBTCR1){assert(protect&2);vbt=value;return;}
 assert(a>=RA4M1_IRQCR(0)&&a<=RA4M1_IRQCR(15)&&a!=RA4M1_IRQCR(13));
 unsigned line=a-RA4M1_IRQCR(0);
 for(auto reg:links)assert((reg&255)!=line+1);
 for(auto reg:dma)assert((reg&255)!=line+1);
 assert(!(wakeup&(1U<<line)));assert(!(value&0x4C));irqcr[line]=value;
}
static void wr16(uintptr_t a,uint16_t value)
{
 assert(test_primask);++writes;if(a==fail_addr)return;
 #ifdef RA4M1_UART_TEST
 if(a>=0x40070000UL && a<0x40070040UL){UartWrite16(a,value);return;}
#endif
 assert(a==RA4M1_PRCR);assert((value&0xff00)==0xA500);protect=value&15;
}
static void wr32(uintptr_t a,uint32_t value)
{
 ++writes;if(a==fail_addr)return;assert(!(a&3));
#ifdef RA4M1_UART_TEST
 if(a==0x40047000UL){assert(protect&2);uart_mstp=value;return;}
#endif
 if(a>=RA4M1_IELSR(0)&&a<=RA4M1_IELSR(31)){
  assert(test_primask);assert(!(value&RA4M1_IELS_IR));assert(!(value&~0xffU));
  assert(!pending_source);links[(a-RA4M1_IELSR(0))/4]=value;return;
 }
 if(a>=RA4M1_PFS(0,0)&&a<=RA4M1_PFS(9,15)){
  assert(test_primask);assert(pwpr&0x40);
  unsigned p=(a-RA4M1_PFS(0,0))/0x40,n=((a-RA4M1_PFS(0,0))%0x40)/4;
  assert(Ra4m1PinValid(p,n));assert(p!=9);uint32_t old=pfs[p][n];
  if((old^value)&(RA4M1_PFS_PSEL|RA4M1_PFS_ASEL))assert(!(old&RA4M1_PFS_PMR)&&!(value&RA4M1_PFS_PMR));
  assert(!(value&RA4M1_PFS_PIDR));
  if(p==2&&(n==0||n>=14))assert(!(value&(RA4M1_PFS_PDR|RA4M1_PFS_PODR|RA4M1_PFS_PCR|RA4M1_PFS_NCODR)));
  if(p!=4||n!=8)assert(!(value&RA4M1_PFS_DSCR1));
  else assert((value&(RA4M1_PFS_DSCR|RA4M1_PFS_DSCR1))!=(RA4M1_PFS_DSCR|RA4M1_PFS_DSCR1));
  if(p==0)assert(!(value&RA4M1_PFS_NCODR));
  if(value&RA4M1_PFS_ASEL)assert(!(value&(RA4M1_PFS_PMR|RA4M1_PFS_PCR|RA4M1_PFS_PDR)));
  if(p==4&&n>=2&&n<=4)assert(valid&0x10);
  pfs[p][n]=value|(old&RA4M1_PFS_PIDR);
  uint32_t bit=1U<<n;portregs[p][0]&=~(bit|(bit<<16));
  if(value&RA4M1_PFS_PDR)portregs[p][0]|=bit;
  if(value&RA4M1_PFS_PODR)portregs[p][0]|=bit<<16;
  return;
 }
 unsigned p=(a-RA4M1_PCNTR1(0))/0x20;
 assert(p<10&&a==RA4M1_PCNTR3(p));uint32_t set=value&0xffff,clr=value>>16;
 assert(!(set&clr));assert(((set|clr)&~Ra4m1OutputMask(p))==0);
 portregs[p][0]=(portregs[p][0]|(set<<16))&~(clr<<16);
 for(unsigned n=0;n<16;n++){if(set&(1U<<n))pfs[p][n]|=1;if(clr&(1U<<n))pfs[p][n]&=~1U;}
}
static void reset(void)
{
 memset(s_IntHook,0,sizeof(s_IntHook));memset(s_PinHook,0,sizeof(s_PinHook));
 memset(links,0,sizeof(links));memset(dma,0,sizeof(dma));memset(portregs,0,sizeof(portregs));memset(pfs,0,sizeof(pfs));memset(irqcr,0,sizeof(irqcr));
 test_primask=test_disabled=test_cleared=test_enabled=test_active=test_pending=wakeup=0;
 memset(test_priority,0,sizeof(test_priority));pwpr=0x80;protect=8;mosc=sosc=1;vbt=0;valid=0x10;vin=vout=0;fail_addr=0;writes=callback_count=0;callback_arg=-99;callback_ctx=nullptr;pending_source=false;
}
static void cb(int n,void *ctx){assert(!test_primask);++callback_count;callback_arg=n;callback_ctx=ctx;pending_source=false;}
static void cfg(int p,int n,int op=0,IOPINDIR d=IOPINDIR_OUTPUT,IOPINRES r=IOPINRES_NONE,IOPINTYPE t=IOPINTYPE_NORMAL){IOPinConfig(p,n,op,d,r,t);}
static void fire(int irq)
{
 assert(irq>=0&&irq<32);links[irq]|=RA4M1_IELS_IR;test_pending&=~(1U<<irq);test_active|=1U<<irq;
 __Vectors[16+irq]();test_active&=~(1U<<irq);
}
#define CHECK(x) do {assert(x);++checks;}while(0)
static void new_edge_cb(int n,void *ctx)
{
 cb(n,ctx);unsigned irq=(unsigned)s_PinHook[n].Irq;
 CHECK(!(links[irq]&RA4M1_IELS_IR));links[irq]|=RA4M1_IELS_IR;test_pending|=1U<<irq;
}
static int replacement_irq;
static void replace_cb(int n,void *ctx)
{
 cb(n,ctx);Ra4m1UnregisterIntHandler((IRQn_Type)n);
 replacement_irq=Ra4m1RegisterIntHandler(RA4M1_EVTID_SCI1_RXI,3,cb,nullptr);
 CHECK(replacement_irq>=0&&replacement_irq!=n);
 links[replacement_irq]|=RA4M1_IELS_IR;test_pending|=1U<<replacement_irq;
}
int main()
{
 reset();CHECK(Ra4m1PortMask(-1)==0&&Ra4m1PortMask(10)==0);
 CHECK(!Ra4m1PinValid(0,9)&&!Ra4m1PinValid(6,4)&&!Ra4m1PinValid(2,11));
 cfg(-1,0);cfg(1,-1);cfg(10,0);cfg(1,16);cfg(0,9);CHECK(writes==0);
 cfg(1,0);CHECK(pfs[1][0]==RA4M1_PFS_PDR&&pwpr==0x80&&protect==8&&test_primask==0);
 IOPinSet(1,0);CHECK((portregs[1][0]>>16)==1);IOPinClear(1,0);CHECK(!(portregs[1][0]>>16));
 IOPinSet(1,2);IOPinToggle(1,0);CHECK((portregs[1][0]>>16)==5);IOPinToggle(1,0);CHECK((portregs[1][0]>>16)==4);
 IOPinWritePort(1,0xA55A);CHECK((portregs[1][0]>>16)==(0xA55A&Ra4m1PortMask(1)));
 portregs[1][1]=0xCAFEA55A;CHECK(IOPinReadPort(1)==(0xA55A&Ra4m1PortMask(1)));CHECK(IOPinRead(1,1)==1&&IOPinRead(-1,3)==0);
 test_primask=1;IOPinToggle(1,1);CHECK(test_primask==1);test_primask=0;
 pwpr=0x40;cfg(1,0);CHECK(pwpr==0x40);pwpr=0x80;
 cfg(1,0,IOPINOP_FUNC3,IOPINDIR_INPUT,IOPINRES_PULLUP);CHECK((pfs[1][0]&RA4M1_PFS_PSEL)==(4U<<24));
 cfg(1,0,IOPINOP_FUNC4,IOPINDIR_INPUT);CHECK((pfs[1][0]&RA4M1_PFS_PSEL)==(5U<<24));
 cfg(1,0);CHECK(!(pfs[1][0]&RA4M1_PFS_PMR));
 uint32_t old=pfs[1][0];cfg(1,0,0,IOPINDIR_OUTPUT,IOPINRES_PULLDOWN);CHECK(pfs[1][0]==old);
 cfg(1,0,0,IOPINDIR_OUTPUT,IOPINRES_FOLLOW);CHECK(pfs[1][0]==old);
 cfg(1,0,11);CHECK(pfs[1][0]==old);cfg(1,0,33);CHECK(pfs[1][0]==old);
 cfg(2,0);CHECK(pfs[2][0]==0);IOPinSet(2,0);IOPinSetDir(2,14,IOPINDIR_OUTPUT);CHECK(pfs[2][0]==0&&pfs[2][14]==0);
 IOPinWritePort(2,0xFFFF);CHECK((portregs[2][0]>>16)==(Ra4m1PortMask(2)&~0xC001));
 mosc=0;cfg(2,12);CHECK(!(pfs[2][12]&RA4M1_PFS_PDR));mosc=1;cfg(2,12);CHECK(pfs[2][12]&RA4M1_PFS_PDR);
 sosc=0;old=pfs[2][15];cfg(2,15,0,IOPINDIR_INPUT);CHECK(pfs[2][15]==old);sosc=1;
 cfg(0,0,0,IOPINDIR_OUTPUT,IOPINRES_NONE,IOPINTYPE_OPENDRAIN);CHECK(pfs[0][0]==0);
 cfg(1,0,0,IOPINDIR_OUTPUT,IOPINRES_NONE,IOPINTYPE_OPENDRAIN);CHECK(pfs[1][0]&RA4M1_PFS_NCODR);
 cfg(1,0,IOPINOP_FUNC31,IOPINDIR_INPUT);CHECK((pfs[1][0]&~1U)==RA4M1_PFS_ASEL);
 old=pfs[1][4];cfg(1,4,IOPINOP_FUNC31,IOPINDIR_INPUT);CHECK(pfs[1][4]==old);
 cfg(5,0,IOPINOP_FUNC31,IOPINDIR_INPUT);CHECK(pfs[5][0]&RA4M1_PFS_ASEL);
 cfg(4,8);IOPinSetStrength(4,8,IOPINSTRENGTH_STRONG);CHECK((pfs[4][8]&0xC00)==0x400);
 IOPinSetStrength(4,8,IOPINSTRENGTH_REGULAR);CHECK(!(pfs[4][8]&0xC00));
 old=pfs[4][8];IOPinSetSpeed(4,8,IOPINSPEED_TURBO);CHECK(pfs[4][8]==old);
 portregs[1][3]=(1U<<3)|(1U<<20);old=portregs[1][0];IOPinWritePort(1,0);CHECK(((old^portregs[1][0])&((1U<<19)|(1U<<20)))==0);
 old=pfs[1][3];cfg(1,3);CHECK(pfs[1][3]==old);portregs[1][3]=0;
 old=pfs[9][14]=RA4M1_PFS_PMR;cfg(9,14);IOPinClear(9,14);IOPinSet(9,15);CHECK(pfs[9][14]==old&&pfs[9][15]==0);
 reset();cfg(4,2);CHECK(pfs[4][2]==RA4M1_PFS_PDR&&vbt==0&&protect==8);
 reset();valid=0;cfg(4,2);CHECK(pfs[4][2]==0&&vbt==0&&protect==8);
 reset();vin=1;cfg(4,2);CHECK(pfs[4][2]==0&&vin==1);
 reset();vout=2;cfg(4,3);CHECK(pfs[4][3]==0&&vout==2);
 reset();cfg(1,0);IOPinSetDir(1,0,IOPINDIR_INPUT);CHECK(!(pfs[1][0]&RA4M1_PFS_PDR));
 IOPinSetDir(1,0,IOPINDIR_OUTPUT);CHECK(pfs[1][0]&RA4M1_PFS_PDR);
 IOPinSetSense(1,0,IOPINSENSE_TOGGLE);CHECK((pfs[1][0]&0x3000)==0x3000);
 IOPinSetSense(1,0,IOPINSENSE_HIGH_TRANSITION);CHECK((pfs[1][0]&0x3000)==0x1000);
 IOPinSetSense(0,0,IOPINSENSE_TOGGLE);CHECK(!(pfs[0][0]&0x3000));
 IOPinDisable(1,0);CHECK(!(pfs[1][0]&~RA4M1_PFS_PODR));
 reset();CHECK(Ra4m1RegisterIntHandler(0,3,cb,nullptr)==-1);
 CHECK(Ra4m1RegisterIntHandler(14,3,cb,nullptr)==-1);CHECK(Ra4m1RegisterIntHandler(0xDB,3,cb,nullptr)==-1);
 CHECK(Ra4m1RegisterIntHandler(1,-1,cb,nullptr)==-1);CHECK(Ra4m1RegisterIntHandler(1,16,cb,nullptr)==-1);CHECK(Ra4m1RegisterIntHandler(1,3,nullptr,nullptr)==-1);
 int irq=Ra4m1RegisterIntHandler(RA4M1_EVTID_SCI0_RXI,3,cb,&checks);CHECK(irq==0&&links[0]==0x98&&test_priority[0]==3);
 CHECK(Ra4m1RegisterIntHandler(RA4M1_EVTID_SCI0_RXI,3,cb,nullptr)==-1);
 pending_source=true;fire(irq);CHECK(callback_count==1&&callback_arg==irq&&callback_ctx==&checks&&links[0]==0x98);
 Ra4m1UnregisterIntHandler((IRQn_Type)irq);CHECK(links[0]==0&&test_enabled==0);
 Ra4m1UnregisterIntHandler((IRQn_Type)-1);Ra4m1UnregisterIntHandler((IRQn_Type)-32); // valid enum representation, invalid CPU slot
 reset();links[0]=0x44;test_enabled|=2;test_active|=4;irq=Ra4m1RegisterIntHandler(1,3,cb,nullptr);CHECK(irq==3&&links[0]==0x44);
 reset();dma[0]=1;CHECK(Ra4m1RegisterIntHandler(1,3,cb,nullptr)==-1);
 reset();fail_addr=RA4M1_IELSR(0);CHECK(Ra4m1RegisterIntHandler(1,3,cb,nullptr)==-1&&test_enabled==0&&!s_IntHook[0].Handler);
 reset();for(int i=0;i<31;i++){irq=Ra4m1RegisterIntHandler((uint8_t)(0x57+i),3,cb,nullptr);CHECK(irq>=0&&irq!=11);}CHECK(Ra4m1RegisterIntHandler(1,3,cb,nullptr)==-1);
 CHECK(!links[11]); // strong vector reserved
 reset();CHECK(!IOPinEnableInterrupt(13,3,1,0,IOPINSENSE_TOGGLE,cb,nullptr));
 CHECK(!IOPinEnableInterrupt(1,3,1,0,IOPINSENSE_TOGGLE,cb,nullptr));CHECK(writes==0);
 cfg(1,0,0,IOPINDIR_INPUT,IOPINRES_PULLUP);
 int line=IOPinAllocateInterrupt(3,1,0,IOPINSENSE_TOGGLE,cb,&checks);CHECK(line==2);
 irq=s_PinHook[line].Irq;CHECK(links[irq]==3&&irqcr[line]==2&&(pfs[1][0]&RA4M1_PFS_ISEL));
 fire(irq);CHECK(callback_count==1&&callback_arg==2&&callback_ctx==&checks&&!(links[irq]&RA4M1_IELS_IR));
 CHECK(!IOPinEnableInterrupt(2,3,0,2,IOPINSENSE_TOGGLE,cb,nullptr));
 old=pfs[1][0];cfg(1,0);CHECK(pfs[1][0]==old);IOPinSetDir(1,0,IOPINDIR_OUTPUT);CHECK(pfs[1][0]==old);
 IOPinDisableInterrupt(2);CHECK(!links[irq]&&!(pfs[1][0]&RA4M1_PFS_ISEL)&&(pfs[1][0]&RA4M1_PFS_PCR));
 reset();wakeup=4;CHECK(IOPinAllocateInterrupt(3,1,0,IOPINSENSE_TOGGLE,cb,nullptr)==-1&&writes==0);
 reset();dma[0]=3;CHECK(IOPinAllocateInterrupt(3,1,0,IOPINSENSE_TOGGLE,cb,nullptr)==-1&&writes==0);
 reset();pfs[0][2]=RA4M1_PFS_ISEL;CHECK(IOPinAllocateInterrupt(3,1,0,IOPINSENSE_TOGGLE,cb,nullptr)==-1&&writes==0);
 reset();for(int i=0;i<32;i++)links[i]=0x57+i;irqcr[2]=0xB1;old=pfs[1][0]=RA4M1_PFS_PCR;
 CHECK(IOPinAllocateInterrupt(3,1,0,IOPINSENSE_TOGGLE,cb,nullptr)==-1);CHECK(pfs[1][0]==old&&irqcr[2]==0xB1&&!s_PinHook[2].Handler);
 reset();fail_addr=RA4M1_PFS(1,0);CHECK(IOPinAllocateInterrupt(3,1,0,IOPINSENSE_TOGGLE,cb,nullptr)==-1&&test_enabled==0);
 reset();fail_addr=RA4M1_IRQCR(2);CHECK(IOPinAllocateInterrupt(3,1,0,IOPINSENSE_TOGGLE,cb,nullptr)==-1&&test_enabled==0);
 reset();line=IOPinAllocateInterrupt(3,1,0,IOPINSENSE_TOGGLE,new_edge_cb,nullptr);irq=s_PinHook[line].Irq;test_cleared=0;
 fire(irq);CHECK(links[irq]&RA4M1_IELS_IR);CHECK(test_pending&(1U<<irq));CHECK(test_cleared==0);
 reset();irq=Ra4m1RegisterIntHandler(0x98,3,replace_cb,nullptr);fire(irq);CHECK(links[replacement_irq]&RA4M1_IELS_IR);CHECK(test_pending&(1U<<replacement_irq));
 reset();line=IOPinAllocateInterrupt(3,1,0,IOPINSENSE_TOGGLE,cb,nullptr);irq=s_PinHook[line].Irq;
 test_active=1U<<irq;IOPinDisableInterrupt(line);
 CHECK(IOPinAllocateInterrupt(3,1,0,IOPINSENSE_TOGGLE,cb,nullptr)==line);
 CHECK(s_PinHook[line].Irq!=irq);Ra4m1PinCallback(irq,&s_PinHook[line]);CHECK(callback_count==0);
 reset();for(auto pin:s_PinIrqMap){int p=(pin&255)>>4,n=pin&15,l=pin>>8;if(Ra4m1PinValid(p,n)){
  CHECK(Ra4m1PinIrq(p,n)==l);CHECK(IOPinAllocateInterrupt(3,p,n,IOPINSENSE_HIGH_TRANSITION,cb,nullptr)==l);IOPinDisableInterrupt(l);
 }}
 reset();test_primask=1;irq=Ra4m1RegisterIntHandler(1,3,cb,nullptr);CHECK(test_primask==1);Ra4m1UnregisterIntHandler((IRQn_Type)irq);CHECK(test_primask==1);
 printf("PASS: %u GPIO/ICU assertions, package %d\n",checks,RA4M1_PACKAGE_PINS);
 return 0;
}
