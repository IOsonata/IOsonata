/* Actual UART + ICU + GPIO sources, with a non-FIFO SCI register model and
 * CFifo test double. Not electrical/timing simulation or production ABI proof.
 */
#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>
#include "../../ARM/Renesas/RA4M1/src/ra4m1_uart_regs.h"
static uint32_t uart_mstp;
static uint8_t UartRead8(uintptr_t);
static uint16_t UartRead16(uintptr_t);
static void UartWrite8(uintptr_t,uint8_t);
static void UartWrite16(uintptr_t,uint16_t);
#define RA4M1_UART_TEST 1
#define main GpioModelMain
#include "io_model.cpp"
#undef main
#include "../../ARM/Renesas/RA4M1/src/uart_ra4m1.cpp"

uint32_t SystemCoreClock=48000000;
static uint32_t pclk=48000000;
uint32_t SystemPeriphClockGet(int) {return pclk;}
static uint8_t sci[4][32];
static bool tdr_full[4];
static std::vector<uint8_t> output[4];
static unsigned callback_rx,callback_tx,callback_error;
static uint8_t last_error;
static bool inject_rx,close_in_callback,rearm_tx;
static unsigned last_tx_space;
static unsigned assertions;
#define UTEST(x) do {assert(x);++assertions;} while(0)
static int sci_index(uintptr_t addr)
{
 unsigned channel=(unsigned)((addr-0x40070000UL)/32);
 assert(channel==0||channel==1||channel==2||channel==9);
 int idx=channel==9?3:(int)channel;
 assert(!(uart_mstp&(1UL<<(31-channel))));
 return idx;
}
static uint8_t UartRead8(uintptr_t addr)
{
 int idx=sci_index(addr);unsigned reg=addr&31U;
 assert(reg<=SCI_SIMR1||reg==SCI_SPMR||reg==SCI_MDDR||reg==SCI_DCCR||reg==SCI_SPTR);
 uint8_t val=sci[idx][reg];
 if(reg==SCI_RDR)sci[idx][SCI_SSR]&=~SCI_SSR_RDRF;
 return val;
}
static uint16_t UartRead16(uintptr_t addr)
{
 int idx=sci_index(addr);assert(idx<2&&(addr&31)==SCI_FCR);return sci[idx][SCI_FCR];
}
static void UartWrite8(uintptr_t addr,uint8_t value)
{
 int idx=sci_index(addr);unsigned reg=addr&31U;assert(test_primask);
 assert(reg<=SCI_SIMR1||reg==SCI_SPMR||reg==SCI_MDDR||reg==SCI_DCCR||reg==SCI_SPTR);
 if(reg==SCI_SMR||reg==SCI_SEMR||reg==SCI_BRR||reg==SCI_MDDR||reg==SCI_SCMR)
  assert(!(sci[idx][SCI_SCR]&(SCI_SCR_TE|SCI_SCR_RE)));
 if(reg==SCI_MDDR)assert(value>=128);
 if(reg==SCI_DCCR)assert(value==0x40); // address match off, reset IDSEL
 if(reg==SCI_SSR){
  assert((value&(SCI_SSR_RDRF|SCI_SSR_TDRE))==(SCI_SSR_RDRF|SCI_SSR_TDRE));
  sci[idx][reg]&=(uint8_t)(value|~SCI_SSR_ERRORS);return;
 }
 if(reg==SCI_TDR){
  assert(sci[idx][SCI_SCR]&SCI_SCR_TE);
  assert(sci[idx][SCI_SSR]&SCI_SSR_TDRE);assert(!tdr_full[idx]);
  tdr_full[idx]=true;sci[idx][SCI_SSR]&=~(SCI_SSR_TDRE|SCI_SSR_TEND);
 }
 if(reg==SCI_SCR&&!(value&SCI_SCR_TE)){
  tdr_full[idx]=false;sci[idx][SCI_SSR]|=SCI_SSR_TDRE|SCI_SSR_TEND;
 }
 if(reg==SCI_SPTR)value=(value&~1U)|(sci[idx][reg]&1U);
 sci[idx][reg]=value;
}
static void UartWrite16(uintptr_t addr,uint16_t value)
{
 int idx=sci_index(addr);assert(idx<2&&(addr&31)==SCI_FCR&&value==0);
 assert(!(sci[idx][SCI_SCR]&(SCI_SCR_TE|SCI_SCR_RE)));sci[idx][SCI_FCR]=0;
}
/* Test double models the production FIFO contract, including push-out mode.
 * The validation runner can substitute the unchanged production CFifo source.
 */
#ifndef RA4M1_TEST_REAL_CFIFO
hCFifo_t CFifoInit(uint8_t *mem,uint32_t size,uint32_t block,bool blocking)
{
 if(!mem||block!=1||size<=sizeof(CFifo_t))return nullptr;
 CFifo_t *q=(CFifo_t*)mem;q->PutIdx=q->GetIdx=q->DropCnt=0;
 q->BlkSize=1;q->pMemStart=mem+sizeof(CFifo_t);q->MaxIdxCnt=(int)(size-sizeof(CFifo_t));
 q->Mask=0;q->bBlocking=blocking;return q;
}
uint8_t *CFifoGet(hCFifo_t q){
 if(!q||q->GetIdx==q->PutIdx)return nullptr;
 return &q->pMemStart[(q->GetIdx++)%(uint32_t)q->MaxIdxCnt];
}
uint8_t *CFifoPut(hCFifo_t q){
 if(!q)return nullptr;
 if(CFifoAvail(q)==0){if(q->bBlocking)return nullptr;++q->GetIdx;++q->DropCnt;}
 return &q->pMemStart[(q->PutIdx++)%(uint32_t)q->MaxIdxCnt];
}
void CFifoFlush(hCFifo_t q){q->PutIdx=q->GetIdx=0;}
int CFifoUsed(hCFifo_t q){return (int)(q->PutIdx-q->GetIdx);}
int CFifoAvail(hCFifo_t q){return q->MaxIdxCnt-CFifoUsed(q);}
#endif
static void fresh()
{
 reset();memset(s_Uart,0,sizeof(s_Uart));memset(sci,0,sizeof(sci));
 for(unsigned i=0;i<4;i++){sci[i][SCI_SSR]=0x84;sci[i][SCI_SPTR]=1;tdr_full[i]=false;output[i].clear();}
 uart_mstp=0xFFFFFFFF;pclk=SystemCoreClock=48000000;
 callback_rx=callback_tx=callback_error=0;last_error=0;inject_rx=close_in_callback=rearm_tx=false;last_tx_space=0;
}
static void request(unsigned dev,unsigned event)
{
 int irq=(int)s_Uart[dev].Irq[event];assert(irq>=0);
 links[irq]|=RA4M1_IELS_IR;test_pending|=1U<<irq;
 if((test_enabled&(1U<<irq))&&!(test_active&(1U<<irq))&&!test_primask)fire(irq);
}
static void rx(unsigned dev,uint8_t byte,uint8_t errors=0)
{
 assert(sci[dev][SCI_SCR]&SCI_SCR_RE);
 sci[dev][SCI_RDR]=byte;sci[dev][SCI_SSR]|=SCI_SSR_RDRF|errors;
 if(sci[dev][SCI_SCR]&SCI_SCR_RIE)request(dev,errors?3:0);
}
static void shift(unsigned dev)
{
 assert(tdr_full[dev]);tdr_full[dev]=false;output[dev].push_back(sci[dev][SCI_TDR]);
 sci[dev][SCI_SSR]|=SCI_SSR_TDRE;
 if(sci[dev][SCI_SCR]&SCI_SCR_TIE)request(dev,1);
}
static void finish(unsigned dev)
{
 assert(!tdr_full[dev]);sci[dev][SCI_SSR]|=SCI_SSR_TEND;
 if(sci[dev][SCI_SCR]&SCI_SCR_TEIE)request(dev,2);
}
static int uart_callback(UARTDEV *u,UART_EVT event,uint8_t *buffer,int len)
{
 assert(!test_primask);assert(len>0);
 if(event==UART_EVT_RXDATA){++callback_rx;assert(!buffer);assert(len==CFifoUsed(u->hRxFifo));}
 if(event==UART_EVT_TXREADY){++callback_tx;last_tx_space=(unsigned)len;assert(!buffer);assert(len==CFifoAvail(u->hTxFifo));}
 if(event==UART_EVT_LINESTATE){++callback_error;assert(buffer&&len==1);last_error=*buffer;}
 if(event==UART_EVT_TXREADY&&rearm_tx){
  rearm_tx=false;uint8_t byte=0xA7;assert(u->DevIntrf.TxData(&u->DevIntrf,&byte,1)==1);
 }
 if(inject_rx){inject_rx=false;rx(0,0x62);}
 if(close_in_callback){close_in_callback=false;u->DevIntrf.PowerOff(&u->DevIntrf);}
 return 0;
}
static IOPINCFG uart_pins[2]={{1,0,4,IOPINDIR_INPUT,IOPINRES_PULLUP,IOPINTYPE_NORMAL},
 {1,1,4,IOPINDIR_OUTPUT,IOPINRES_NONE,IOPINTYPE_NORMAL}};
static UARTCFG config(int dev=0,bool interrupt=true)
{
 UARTCFG c={};c.DevNo=dev;c.pIOPinMap=uart_pins;c.NbIOPins=2;c.Rate=115200;
 c.DataBits=8;c.StopBits=1;c.Parity=UART_PARITY_NONE;c.bIntMode=interrupt;c.IntPrio=3;
 c.EvtCallback=uart_callback;c.bFifoBlocking=true;return c;
}
static uint64_t brute(uint32_t clock,uint32_t rate)
{
 const unsigned divs[]={32,16,8,6};uint64_t best=UINT64_MAX;
 for(auto div:divs)for(unsigned c=0;c<4;c++)for(unsigned n=1;n<=256;n++)for(unsigned m=128;m<=256;m++){
  uint64_t real=(uint64_t)clock*m*1000000/((uint64_t)div*(1U<<(2*c))*n*256);
  uint64_t target=(uint64_t)rate*1000000;uint64_t err=real>target?real-target:target-real;
  best=std::min(best,err);
 }
 return best;
}
int main()
{
 // Keep the 188 existing assertions as an actual-source regression.
 GpioModelMain();
 fresh();Ra4m1UartBaud b;
 UTEST(!Ra4m1UartPlanBaud(0,115200,&b));UTEST(!Ra4m1UartPlanBaud(48000000,0,&b));
 UTEST(!Ra4m1UartPlanBaud(48000000,8000001,&b));UTEST(!Ra4m1UartPlanBaud(48000000,1,&b));
 for(unsigned clk:{8000000U,24000000U,48000000U})for(unsigned rate:{110U,9600U,115200U,250000U,921600U,1000000U}){
  UTEST(Ra4m1UartPlanBaud(clk,rate,&b));
  unsigned divisor=b.Semr&SCI_SEMR_ABCSE?6:(32U>>((b.Semr&SCI_SEMR_BGDM?1:0)+(b.Semr&SCI_SEMR_ABCS?1:0)));
  uint64_t num=(uint64_t)clk*(b.Semr&SCI_SEMR_BRME?b.Mddr:256);
  uint64_t den=(uint64_t)divisor*(1U<<(b.Cks*2))*(b.Brr+1U)*256;
  uint64_t scaled=num*1000000/den,target=(uint64_t)rate*1000000;
  UTEST((scaled>target?scaled-target:target-scaled)==brute(clk,rate));
 }
 UTEST(Ra4m1UartPlanBaud(48000000,1000000,&b)&&b.Actual==1000000);
 UTEST(Ra4m1UartPlanBaud(48000000,45,&b)&&b.Brr==255&&b.Cks==3);
 UTEST(!Ra4m1UartPlanBaud(48000000,44,&b));
 UTEST(Ra4m1UartPlanBaud(48000000,8000000,&b)&&b.Brr==0&&b.Actual==8000000);
 for(int channel=0;channel<4;channel++){
  fresh();UARTDEV u={};UARTCFG c=config(channel);u.pObj=&c;
  UTEST(UARTInit(&u,&c));UTEST(UARTGetInstance(channel)==&u&&u.pObj==&c);
  UTEST(s_Uart[channel].Base==0x40070000UL+32U*s_Channel[channel]);
  UTEST((uart_mstp^(0xFFFFFFFFU))==(1U<<(31-s_Channel[channel])));
  UTEST(__builtin_popcount(test_enabled)==4);UTEST((sci[channel][SCI_SCR]&0xFC)==0x70);
  UTEST(sci[channel][SCI_DCCR]==0x40);UTEST(u.DevIntrf.GetHandle(&u.DevIntrf)==&u);
  UTEST(!UARTInit(&u,&c));UTEST(u.DevIntrf.Reset&&u.DevIntrf.TxSrData&&u.DevIntrf.StopRx);
  rx(channel,0xC5);uint8_t data=0;UTEST(u.DevIntrf.RxData(&u.DevIntrf,&data,1)==1&&data==0xC5);
  uint8_t tx[]={1,2,3};UTEST(u.DevIntrf.TxData(&u.DevIntrf,tx,3)==3);
  while(tdr_full[channel])shift(channel);
  UTEST(!u.bTxReady&&!u.DevIntrf.bTxReady);finish(channel);UTEST(u.bTxReady&&u.DevIntrf.bTxReady);
  UTEST(output[channel]==std::vector<uint8_t>({1,2,3}));
  u.DevIntrf.PowerOff(&u.DevIntrf);UTEST(!UARTGetInstance(channel)&&test_enabled==0&&uart_mstp==0xFFFFFFFF);
  UTEST(pfs[1][0]==0&&pfs[1][1]==0&&protect==8&&pwpr==0x80);
 }
 fresh();UARTDEV u={};UARTCFG c=config();
 unsigned before=writes;UTEST(!UARTInit(nullptr,&c));c.DevNo=4;UTEST(!UARTInit(&u,&c));
 c=config();c.bDMAMode=true;UTEST(!UARTInit(&u,&c));c=config();c.DataBits=9;UTEST(!UARTInit(&u,&c));
 c=config();c.FlowControl=UART_FLWCTRL_HW;UTEST(!UARTInit(&u,&c));c=config();c.bIrDAMode=true;UTEST(!UARTInit(&u,&c));
 c=config();c.Rate=0;UTEST(!UARTInit(&u,&c));c=config();c.NbIOPins=1;UTEST(!UARTInit(&u,&c));UTEST(writes==before);
 c=config();UTEST(UARTInit(&u,&c));
 uint8_t values[64];for(unsigned i=0;i<64;i++)values[i]=(uint8_t)i;
 // Hardware holding byte + 16 FIFO entries, no silent loss in blocking mode.
 UTEST(u.DevIntrf.TxData(&u.DevIntrf,values,64)==17);UTEST(u.TxDropCnt==0);
 while(tdr_full[0])shift(0);finish(0);UTEST(output[0].size()==17);
 for(unsigned i=0;i<17;i++)UTEST(output[0][i]==i);
 for(unsigned i=0;i<20;i++)rx(0,(uint8_t)i);
 UTEST(u.RxDropCnt==4&&!(sci[0][SCI_SSR]&SCI_SSR_RDRF));
 uint8_t buf[32];UTEST(u.DevIntrf.RxData(&u.DevIntrf,buf,32)==16);
 for(unsigned i=0;i<16;i++)UTEST(buf[i]==i);
 rx(0,0xFF,SCI_SSR_PER);UTEST(u.ParErrCnt==1&&(last_error&UART_LINESTATE_PARERR));
 rx(0,0xFF,SCI_SSR_ORER);UTEST(u.RxOvrErrCnt==1&&(last_error&UART_LINESTATE_OVR));
 sci[0][SCI_SPTR]&=~1U;rx(0,0,SCI_SSR_FER);UTEST(u.FramErrCnt==1&&(last_error&UART_LINESTATE_BRK));
 rx(0,0x55);UTEST(u.DevIntrf.RxData(&u.DevIntrf,buf,1)==1&&buf[0]==0x55);
 // Fresh byte raised during user callback survives ICU's outer dispatcher.
 inject_rx=true;rx(0,0x61);int irq=s_Uart[0].Irq[0];
 UTEST(links[irq]&RA4M1_IELS_IR);UTEST(test_pending&(1U<<irq));fire(irq);
 UTEST(u.DevIntrf.RxData(&u.DevIntrf,buf,2)==2&&buf[0]==0x61&&buf[1]==0x62);
 int rate=u.Rate;UTEST(u.DevIntrf.SetRate(&u.DevIntrf,9600)==0&&u.Rate==rate);
 u.DevIntrf.Disable(&u.DevIntrf);UTEST(!s_Uart[0].Enabled&&test_enabled==0);
 UTEST(u.DevIntrf.SetRate(&u.DevIntrf,9600)>0);UTEST(u.Rate!=rate);
 UTEST(!u.DevIntrf.StartRx(&u.DevIntrf,0));u.DevIntrf.Enable(&u.DevIntrf);UTEST(s_Uart[0].Enabled);
 // Release from application callback; no post-callback source access.
 close_in_callback=true;rx(0,0x33);UTEST(!UARTGetInstance(0)&&test_enabled==0);
 UTEST(UARTInit(&u,&c));u.DevIntrf.PowerOff(&u.DevIntrf);
 // Polling takes no ICU routes and never strands accepted TX in CFifo.
 fresh();c=config(0,false);UTEST(UARTInit(&u,&c));UTEST(test_enabled==0);
 UTEST(u.DevIntrf.TxData(&u.DevIntrf,values,4)==1);UTEST(CFifoUsed(u.hTxFifo)==0);
 shift(0);UTEST(u.DevIntrf.TxData(&u.DevIntrf,values+1,3)==1);shift(0);finish(0);
 rx(0,0x42);UTEST(u.DevIntrf.RxData(&u.DevIntrf,buf,1)==1&&buf[0]==0x42);
 UTEST(callback_rx==0&&callback_tx==0);u.DevIntrf.PowerOff(&u.DevIntrf);
 // Push-out mode counts both RX and TX eviction; no CFifo policy changes.
 fresh();c=config();c.bFifoBlocking=false;UTEST(UARTInit(&u,&c));
 UTEST(u.DevIntrf.TxData(&u.DevIntrf,values,20)==20&&u.TxDropCnt==3);
 for(unsigned i=0;i<20;i++)rx(0,(uint8_t)i);UTEST(u.RxDropCnt==4);
 UTEST(u.DevIntrf.RxData(&u.DevIntrf,buf,16)==16&&buf[0]==4&&buf[15]==19);
 // Partial ICU allocation, mux readback and baud-write failures roll back.
 fresh();for(int i=0;i<30;i++)links[i]=0x57+i;c=config();
 UTEST(!UARTInit(&u,&c));UTEST(uart_mstp==0xFFFFFFFF&&test_enabled==0&&links[30]==0);
 fresh();fail_addr=0x40070001;c=config();UTEST(!UARTInit(&u,&c));UTEST(!UARTGetInstance(0)&&uart_mstp==0xFFFFFFFF);
 fresh();fail_addr=RA4M1_PFS(1,1);UTEST(!UARTInit(&u,&c));UTEST(pfs[1][0]==0&&test_enabled==0);
 fresh();alignas(4) uint8_t shared[CFIFO_MEMSIZE(32)];c=config();c.pRxMem=shared;c.pTxMem=shared;c.RxMemSize=c.TxMemSize=sizeof(shared);
 UTEST(!UARTInit(&u,&c));UTEST(writes==0);
 // Frame format and byte masks are not inherited from a prior owner.
 fresh();c=config();c.DataBits=7;c.Parity=UART_PARITY_ODD;c.StopBits=2;UTEST(UARTInit(&u,&c));
 UTEST((sci[0][SCI_SMR]&~3U)==0x78U);rx(0,0xFF);UTEST(u.DevIntrf.RxData(&u.DevIntrf,buf,1)==1&&buf[0]==0x7F);
 uint8_t byte=0xFF;UTEST(u.DevIntrf.TxData(&u.DevIntrf,&byte,1)==1);shift(0);UTEST(output[0][0]==0x7F);
 // Refill during TEI callback: new TDR and pending TXI survive dispatcher exit.
 fresh();c=config();UTEST(UARTInit(&u,&c));byte=0xA6;
 UTEST(u.DevIntrf.TxData(&u.DevIntrf,&byte,1)==1);shift(0);rearm_tx=true;finish(0);
 UTEST(tdr_full[0]&&!u.bTxReady&&!u.DevIntrf.bTxReady);shift(0);finish(0);
 UTEST(output[0]==std::vector<uint8_t>({0xA6,0xA7})&&u.bTxReady);
 // All partial-init failure points leave no owned channel/interrupt/pin.
 for(uintptr_t address:{(uintptr_t)0x40070006,(uintptr_t)0x40070013,(uintptr_t)0x40047000}){
  fresh();c=config();fail_addr=address;UTEST(!UARTInit(&u,&c));
  UTEST(!UARTGetInstance(0)&&test_enabled==0&&uart_mstp==0xFFFFFFFF);
 }
 fresh();c=config();UTEST(UARTInit(&u,&c));rx(0,0x19);u.DevIntrf.Disable(&u.DevIntrf);
 int oldrate=u.Rate;fail_addr=0x40070001;
 UTEST(u.DevIntrf.SetRate(&u.DevIntrf,9600)==0&&u.Rate==oldrate);
 fail_addr=0;u.DevIntrf.Enable(&u.DevIntrf);
 UTEST(u.DevIntrf.RxData(&u.DevIntrf,buf,1)==1&&buf[0]==0x19);
 rx(0,0x1A);u.DevIntrf.Reset(&u.DevIntrf);UTEST(CFifoUsed(u.hRxFifo)==0&&s_Uart[0].Enabled);
 // TX-only and RX-only descriptors retain conventional RX/TX array ordering.
 fresh();IOPINCFG txonly[2]={uart_pins[0],uart_pins[1]};txonly[0].PortNo=txonly[0].PinNo=-1;
 c=config();c.pIOPinMap=txonly;UTEST(UARTInit(&u,&c));UTEST(__builtin_popcount(test_enabled)==2&&!s_Uart[0].Rx);
 fresh();IOPINCFG rxonly[2]={uart_pins[0],uart_pins[1]};rxonly[1].PortNo=rxonly[1].PinNo=-1;
 c=config();c.pIOPinMap=rxonly;UTEST(UARTInit(&u,&c));UTEST(__builtin_popcount(test_enabled)==2&&!s_Uart[0].Tx);
 UTEST(!u.DevIntrf.StartTx(&u.DevIntrf,0));rx(0,0x21);
 UTEST(u.DevIntrf.RxData(&u.DevIntrf,buf,1)==1&&buf[0]==0x21);
 // Two different SCI channels preserve each other's module and ICU ownership.
 fresh();c=config();UARTDEV other={};UTEST(UARTInit(&u,&c));
 IOPINCFG otherpins[2]={{1,2,4,IOPINDIR_INPUT,IOPINRES_NONE,IOPINTYPE_NORMAL},
  {1,3,4,IOPINDIR_OUTPUT,IOPINRES_NONE,IOPINTYPE_NORMAL}};
 UARTCFG c2=config(1);c2.pIOPinMap=otherpins;UTEST(UARTInit(&other,&c2));
 UTEST(__builtin_popcount(test_enabled)==8);u.DevIntrf.PowerOff(&u.DevIntrf);
 UTEST(UARTGetInstance(1)==&other&&__builtin_popcount(test_enabled)==4);
 rx(1,0x22);UTEST(other.DevIntrf.RxData(&other.DevIntrf,buf,1)==1&&buf[0]==0x22);
 printf("PASS: %u UART assertions; actual UART/ICU/GPIO with test contracts\n",assertions);
 return 0;
}
