#!/usr/bin/env python3
"""Focused production-function regressions; host models do not validate bus timing."""
from pathlib import Path
import re, subprocess, tempfile, os, shlex
ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'ARM/ST/STM32L4xx'
def function(s, name):
    m = re.search(r'^.*\b'+name+r'\([^;]*?\)\s*\{', s, re.M)
    a=s.index('{',m.start()); depth=1; b=a+1
    while depth:
        depth += (s[b]=='{')-(s[b]=='}'); b+=1
    return s[m.start():b]
header=(BASE/'include/stm32l476xx.h').read_text()
prefixes=['RCC_PLLCFGR_PLL','RCC_CFGR_SWS','RCC_CR_MSI','RCC_CSR_MSI','RCC_APB1ENR1_LPTIM1','RCC_APB1ENR2_LPTIM2','RCC_APB1SMENR1_LPTIM1','RCC_APB1SMENR2_LPTIM2','LPTIM_CR_','LPTIM_IER_','USART_ISR_','USART_CR1_','USART_CR2_STOP']
macros='\n'.join(l for l in header.splitlines() if l.startswith('#define ') and any(l.split()[1].startswith(p) for p in prefixes))
common='#include <cstdint>\n#include <cassert>\n#include <cstdio>\n#include <cstring>\n#include <algorithm>\n'+macros+'\n'
models={}
for name in ['system_stm32l4xx.c','system_stm32l4plus.c']:
    models[name]=common+r'''
struct {uint32_t CR=0,CSR=0,CFGR=0,PLLCFGR=0;} rcc; auto RCC=&rcc;
struct {uint32_t CR5=0;} pwr;auto PWR=&pwr;
constexpr uint32_t PWR_CR5_R1MODE=1;
uint32_t SystemCoreClock;
struct {struct {uint32_t Freq;} CoreOsc;} g_McuOsc{{24000000}};
const uint32_t MSIRangeTable[]={100000,200000,400000,800000,1000000,2000000,4000000,8000000,16000000,24000000,32000000,48000000};
void SetFlashWaitState(uint32_t){} void SystemPeriphClockSet(int,uint32_t){}
'''+function((BASE/'src'/name).read_text(),'SystemCoreClockUpdate')+r'''
int main(){
 rcc.CR=RCC_CR_MSIRGSEL|(11<<RCC_CR_MSIRANGE_Pos);
 rcc.PLLCFGR=RCC_PLLCFGR_PLLSRC_HSE|(10<<RCC_PLLCFGR_PLLN_Pos);
 rcc.CFGR=RCC_CFGR_SWS_HSE;SystemCoreClockUpdate();assert(SystemCoreClock==24000000);
 rcc.CFGR=RCC_CFGR_SWS_HSI;SystemCoreClockUpdate();assert(SystemCoreClock==16000000);
 rcc.CFGR=RCC_CFGR_SWS_MSI;SystemCoreClockUpdate();assert(SystemCoreClock==48000000);
 rcc.CR=0;rcc.CSR=6<<RCC_CSR_MSISRANGE_Pos;SystemCoreClockUpdate();assert(SystemCoreClock==4000000);
 rcc.CFGR=RCC_CFGR_SWS_PLL;
 for(unsigned src=1;src<=3;src++) {
  rcc.PLLCFGR=src|(10<<RCC_PLLCFGR_PLLN_Pos);
  SystemCoreClockUpdate();assert(SystemCoreClock==(src==1?20000000:src==2?80000000:120000000));
 }
 // Exercise non-default PLL M and R divisors: 48 MHz /6 x40 /4.
 rcc.CR=RCC_CR_MSIRGSEL|(11<<RCC_CR_MSIRANGE_Pos);
 rcc.PLLCFGR=RCC_PLLCFGR_PLLSRC_MSI|(5<<RCC_PLLCFGR_PLLM_Pos)|(40<<RCC_PLLCFGR_PLLN_Pos)|(1<<RCC_PLLCFGR_PLLR_Pos);
 SystemCoreClockUpdate();assert(SystemCoreClock==80000000);
 puts("PASS clock: direct sources, PLL sources/dividers and both MSI range selectors");
}
'''
s=(BASE/'src/timer_lptim_stm32l4xx.cpp').read_text()
models['timer']=common+r'''
struct Reg {uint32_t CR=0,IER=0;} regs[2];
struct TimerDev_t {int DevNo;};
struct STM32L4XX_TimerData_t {Reg *pLPTimReg;TimerDev_t *pTimer;} g_Stm32l4TimerData[2];
struct {uint32_t APB1ENR1=0,APB1ENR2=0;} rcc;auto RCC=&rcc;
enum {LPTIM1_IRQn,LPTIM2_IRQn}; void NVIC_ClearPendingIRQ(int){}
'''+function(s,'Stm32l4LptEnable')+'\n'+function(s,'Stm32l4LptDisable')+r'''
int main(){TimerDev_t timers[2]={{0},{1}};for(int i=0;i<2;i++){
 g_Stm32l4TimerData[i]={&regs[i],&timers[i]};regs[i].IER=LPTIM_IER_ARRMIE|LPTIM_IER_CMPMIE;
 for(int cycle=0;cycle<3;cycle++){
  Stm32l4LptDisable(&timers[i]);assert(!(regs[i].IER&LPTIM_IER_ARRMIE));
  assert(Stm32l4LptEnable(&timers[i]));assert(regs[i].IER==(LPTIM_IER_ARRMIE|LPTIM_IER_CMPMIE));
  assert(regs[i].CR==(LPTIM_CR_ENABLE|LPTIM_CR_CNTSTRT));
  assert(Stm32l4LptEnable(&timers[i]));
 }
}puts("PASS LPTIM: both devices restore overflow interrupt across repeated enable cycles");}
'''
s=(BASE/'src/uart_stm32l4xx.cpp').read_text()
a=s.index('\t// Hardware word length');b=s.index('\n\t// Swap',a)
models['framing']=common+r'''
enum {UART_PARITY_NONE,UART_PARITY_EVEN,UART_PARITY_ODD};
struct Config {int DataBits,Parity,StopBits;};struct Reg {uint32_t CR1,CR2;};
void frame(Reg *reg,Config *pCfg){
'''+s[a:b]+r'''
}
int main(){for(int bits=7;bits<=8;bits++)for(int parity=0;parity<3;parity++)for(int stop=1;stop<=2;stop++){
 Reg reg{0xffffffff,0xffffffff};Config cfg{bits,parity,stop};frame(&reg,&cfg);
 assert(!!(reg.CR1&USART_CR1_PCE)==(parity!=0));assert(!!(reg.CR1&USART_CR1_PS)==(parity==2));
 unsigned length=bits+(parity!=0);assert(!!(reg.CR1&USART_CR1_M0)==(length==9));
 assert(!!(reg.CR1&(1u<<28))==(length==7));assert((reg.CR2&USART_CR2_STOP_Msk)==(stop==2?USART_CR2_STOP_1:0));
 assert((reg.CR2&~USART_CR2_STOP_Msk)==(0xffffffff&~USART_CR2_STOP_Msk));
}puts("PASS UART framing: 7/8 payload bits, none/even/odd parity, 1/2 stops");}
'''
models['rx']=common+r'''
struct Fifo {uint8_t bytes[2];int used=0;};
uint8_t *CFifoPut(Fifo *f){return f->used==2?nullptr:&f->bytes[f->used++];}
uint8_t *CFifoGet(Fifo *){return nullptr;}
uint8_t *CFifoGetMultiple(Fifo *f,int *n){if(!f->used)return nullptr;*n=std::min(*n,f->used);f->used-=*n;return f->bytes;}
int CFifoUsed(Fifo *f){return f->used;}
struct Reg;
struct DataReg {Reg *reg;uint8_t value=0;int reads=0;operator uint16_t();};
struct Reg {uint32_t ISR=0,ICR=0,TDR=0;DataReg RDR{this};};
DataReg::operator uint16_t(){reads++;reg->ISR&=~USART_ISR_RXNE;return value;}
struct UARTDev_t {Fifo *hRxFifo,*hTxFifo;bool bRxReady=false,bTxReady=false;int DataBits=8;void (*EvtCallback)(UARTDev_t*,int,void*,int)=nullptr;};
struct STM32L4X_UARTDEV {UARTDev_t *pUartDev;Reg *pReg;int RxDropCnt=0,ErrCnt=0;uint32_t ErrFlag=0;};
struct DevIntrf_t {void *pDevData;};
constexpr int STM32L4X_UART_HWFIFO_SIZE=8,UART_EVT_RXDATA=0,UART_EVT_TXREADY=1,UART_EVT_RXTIMEOUT=2;
uint32_t DisableInterrupt(){return 0;}void EnableInterrupt(uint32_t){}
'''+function(s,'UART_IRQHandler')+'\n'+function(s,'STM32L4xUARTRxData')+r'''
int main(){Fifo rx{},tx{};Reg reg{};UARTDev_t uart{&rx,&tx};STM32L4X_UARTDEV dev{&uart,&reg};DevIntrf_t intrf{&dev};
 for(int i=0;i<3;i++){reg.RDR.value=10+i;reg.ISR=USART_ISR_RXNE;UART_IRQHandler(&dev);}
 assert(dev.RxDropCnt==1);assert(reg.RDR.reads==3);assert(uart.bRxReady);
 uint8_t out[4]{};assert(STM32L4xUARTRxData(&intrf,out,4)==2);assert(out[0]==10&&out[1]==11);
 assert(!uart.bRxReady);assert(STM32L4xUARTRxData(&intrf,out,4)==0);assert(reg.RDR.reads==3);
 uart.DataBits=7;reg.RDR.value=0xff;reg.ISR=USART_ISR_RXNE;UART_IRQHandler(&dev);
 assert(STM32L4xUARTRxData(&intrf,out,4)==1);assert(out[0]==0x7f);assert(reg.RDR.reads==4);
 puts("PASS UART RX: full FIFO drops once, no stale RDR replay, seven-bit masking");
}
'''
with tempfile.TemporaryDirectory(prefix='stm32l4-runtime-') as d:
    for name,code in models.items():
        p=Path(d)/name;p=p.with_suffix('.cpp');p.write_text(code);exe=p.with_suffix('.out')
        subprocess.run(shlex.split(os.environ.get('CXX','g++'))+['-std=c++17','-O2','-fsanitize=undefined','-fno-sanitize-recover=all',str(p),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
