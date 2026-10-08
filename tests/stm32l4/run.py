#!/usr/bin/env python3
"""Compile production function bodies against focused host register/dispatch models.

This checks virtual-device routing and unsupported-mode refusal, not peripheral
bus timing, a complete firmware build, or hardware operation.
"""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / 'ARM/ST/STM32L4xx/src'
def source(name): return (SRC / name).read_text()
def function(text, name):
    match = re.search(r'^.*\b' + re.escape(name) + r'\([^;]*?\)\s*\{', text, re.M)
    assert match, name
    start = text.index('{', match.start())
    depth, end = 1, start + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[match.start():end]
def table(text, name):
    start = text.index('static STM32L4X_UARTDEV '+name) if name.startswith('s_Stm') else text.index('STM32L4XX_SPIDev_t '+name)
    end = text.index('\n};', start) + 3
    return text[start:end]
COMMON = '#include <cassert>\n#include <cstdint>\n#include <cstdio>\n#include <cstring>\n'
models = {}
u = source('uart_stm32l4xx.cpp')
models['uart'] = COMMON + r'''
struct Reg { uint32_t CR1=0, CR2=0; } regs[6];
auto LPUART1=&regs[0], USART1=&regs[1], USART2=&regs[2], USART3=&regs[3], UART4=&regs[4], UART5=&regs[5];
struct IOPinCfg_t { int PortNo, PinNo; };
struct UARTDev { int hTxFifo=0; bool bTxReady=false; };
struct STM32L4X_UARTDEV { int DevNo; Reg *pReg; UARTDev *pUartDev=nullptr; unsigned ErrCnt=0,RxTimeoutCnt=0,RxDropCnt=0; const IOPinCfg_t *pIOPinMap=nullptr; int NbPins=0; };
struct DevIntrf_t { void *pDevData; int bBusy=0; };
struct Rcc { uint32_t APB2ENR=0, APB1ENR1=0, APB1ENR2=0, APB2RSTR=0, APB1RSTR1=0, APB1RSTR2=0, CCIPR=0; } rcc;
auto RCC=&rcc;
constexpr uint32_t RCC_APB2ENR_USART1EN=1u<<14, RCC_APB1ENR1_USART2EN=1u<<17, RCC_APB1ENR2_LPUART1EN=1;
constexpr uint32_t RCC_APB1ENR1_USART3EN=1u<<18, RCC_APB1ENR1_UART4EN=1u<<19, RCC_APB1ENR1_UART5EN=1u<<20;
constexpr uint32_t RCC_APB2RSTR_USART1RST=1u<<14, RCC_APB1RSTR1_USART2RST=1u<<17, RCC_APB1RSTR2_LPUART1RST=1;
constexpr uint32_t RCC_CCIPR_USART1SEL_Msk=3, RCC_CCIPR_LPUART1SEL_Msk=3u<<10;
constexpr uint32_t USART_CR1_UE=1, USART_CR1_RE=4, USART_CR1_TE=8, USART_CR2_RTOEN=1u<<23;
constexpr int IOPINSPEED_TURBO=0;
enum { LPUART1_IRQn, USART1_IRQn, USART2_IRQn, USART3_IRQn, UART4_IRQn, UART5_IRQn };
int selected=-1, cleared=-1, enabled=-1, priority=-1;
void UART_IRQHandler(STM32L4X_UARTDEV *p) { selected=int(p->pReg-regs); }
void NVIC_ClearPendingIRQ(int irq) { cleared=irq; }
void NVIC_SetPriority(int irq,int) { priority=irq; }
void NVIC_EnableIRQ(int irq) { enabled=irq; }
void IOPinDis(const IOPinCfg_t*,int) {}
void IOPinCfg(const IOPinCfg_t*,int) {}
void IOPinSetSpeed(int,int,int) {}
void CFifoFlush(int) {}
void atomic_flag_clear(int*) {}
Rcc resetSnapshot;
void usDelay(int) { resetSnapshot=rcc; }
''' + table(u, 's_Stm32l4xUartDev') + '\n'
for name in ['USART1','USART2','USART3','UART4','UART5','LPUART1']:
    models['uart'] += function(u,name+'_IRQHandler')+'\n'
for name in ['Disable','Enable','PowerOff','Reset']:
    models['uart'] += function(u,'STM32L4xUART'+name)+'\n'
# Exercise the actual IRQ-selection switch in UARTInit, without simulating FIFOs.
start = u.index('switch (devno)', u.index('if (pCfg->bIntMode)'))
# Match the switch itself using balanced braces.
i=u.index('{',start); d=1; end=i+1
while d:
    d+=(u[end]=='{')-(u[end]=='}'); end+=1
models['uart'] += 'void init_irq(int devno) { struct { int IntPrio; } cfg{3}; auto pCfg=&cfg; '+u[start:end]+'}\n'
models['uart'] += r'''
int main() {
 void (*irqs[])()={LPUART1_IRQHandler,USART1_IRQHandler,USART2_IRQHandler,USART3_IRQHandler,UART4_IRQHandler,UART5_IRQHandler};
 UARTDev devs[6];
 for(int i=0;i<6;i++) {
  auto &entry=s_Stm32l4xUartDev[i]; entry.pUartDev=&devs[i]; DevIntrf_t dev{&entry};
  irqs[i](); assert(selected==i && cleared==i);
  init_irq(i); assert(enabled==i && priority==i && cleared==i);
  rcc={}; STM32L4xUARTReset(&dev);
  uint32_t a=i==1 ? (1u<<14):0, b=i>=2 ? (1u<<(17+i-2)):0, c=i==0 ? 1:0;
  assert(resetSnapshot.APB2RSTR==a && resetSnapshot.APB1RSTR1==b && resetSnapshot.APB1RSTR2==c);
  assert(rcc.APB2RSTR==0 && rcc.APB1RSTR1==0 && rcc.APB1RSTR2==0);
  STM32L4xUARTEnable(&dev);
  assert(rcc.APB2ENR==a && rcc.APB1ENR1==b && rcc.APB1ENR2==c);
  STM32L4xUARTDisable(&dev); assert(!(rcc.APB2ENR|rcc.APB1ENR1|rcc.APB1ENR2));
  STM32L4xUARTEnable(&dev); rcc.CCIPR=0xfff;
  STM32L4xUARTPowerOff(&dev);
  assert(rcc.CCIPR==(0xfffu & ~(3u<<(i==0 ? 10:2*(i-1)))));
  assert(!(rcc.APB2ENR|rcc.APB1ENR1|rcc.APB1ENR2));
 }
 puts("PASS UART: six IRQ, NVIC, reset and clock mappings");
}
'''
s = source('spi_stm32l4xx.cpp')
models['spi'] = COMMON + r'''
#define STM32L4XX_SPI_DEV_COUNT 3
#ifdef STM32L4S9xx
#define STM32L4XX_SPI_MAXDEV 5
#else
#define STM32L4XX_SPI_MAXDEV 4
#endif
struct Reg { uint32_t CR2=0; } regs[5];
auto SPI1=&regs[0], SPI2=&regs[1], SPI3=&regs[2], OCTOSPI1=&regs[3], OCTOSPI2=&regs[4], QUADSPI=&regs[3];
using SPI_TypeDef=Reg;
struct Pin { int PortNo, PinNo; };
struct SPICFG { int DevNo,Mode; const Pin *pIOPinMap; int NbIOPins; bool bIntEn,bDmaEn; int IntPrio; };
struct SPIDEV { SPICFG Cfg; struct { void *pDevData; } DevIntrf; };
struct STM32L4XX_SPIDev_t { int DevNo; SPIDEV *pSpiDev; union { Reg *pReg; Reg *pOReg; Reg *pQReg; }; };
constexpr int SPIMODE_MASTER=0,SPIMODE_SLAVE=1,IOPINSPEED_TURBO=0;
constexpr int SPI_CR2_RXNEIE=1,SPI_CR2_ERRIE=2;
enum {SPI1_IRQn,SPI2_IRQn,SPI3_IRQn,OCTOSPI1_IRQn,OCTOSPI2_IRQn,QUADSPI_IRQn};
int selected=-1, pins=0;
void IOPinCfg(const Pin*,int) { pins++; }
void IOPinSetSpeed(int,int,int) {}
void NVIC_ClearPendingIRQ(int) {} void NVIC_SetPriority(int,int) {} void NVIC_EnableIRQ(int) {}
bool STM32L4xxSPIInit(SPIDEV*,const SPICFG *c) { selected=c->DevNo; return true; }
bool STM32L4xxOctoSPIInit(SPIDEV*,const SPICFG *c) { selected=10+c->DevNo; return true; }
bool STM32L4xxQuadSPIInit(SPIDEV*,const SPICFG *c) { selected=20+c->DevNo; return true; }
''' + table(s,'s_STM32L4xxSPIDev')+'\n'+function(s,'SPIInit')+r'''
int main() {
 SPIDEV dev{}; Pin pin{0,0}; SPICFG c{0,SPIMODE_MASTER,&pin,1,false,false,0};
 for(int i=0;i<STM32L4XX_SPI_MAXDEV;i++) {
  c.DevNo=i; assert(SPIInit(&dev,&c));
#ifdef STM32L4S9xx
  assert(selected==(i<3 ? i:10+i));
#else
  assert(selected==(i<3 ? i:20+i));
#endif
 }
 for(int i: {-1,STM32L4XX_SPI_MAXDEV}) { c.DevNo=i; int before=pins; assert(!SPIInit(&dev,&c)); assert(pins==before); }
 for(int i=3;i<STM32L4XX_SPI_MAXDEV;i++) {
  c.DevNo=i;
  for(int mode=0;mode<3;mode++) { c.Mode=mode==0 ? SPIMODE_SLAVE:SPIMODE_MASTER; c.bIntEn=mode==1; c.bDmaEn=mode==2;
   int before=pins; assert(!SPIInit(&dev,&c)); assert(pins==before);
  }
 }
 puts("PASS SPI: ordinary/extended dispatch and refusal before side effects");
}
'''
# I2C preflight uses the actual function prefix through the first mutation.
s=source('i2c_stm32l4xx.cpp'); prefix=function(s,'I2CInit').split('\t// Save config data')[0]
models['i2c'] = COMMON + r'''
#define STM32L4XX_I2C_MAXDEV 4
struct I2C_TypeDef {};
struct I2CDev_t {};
struct I2CCfg_t { int DevNo,Mode; bool bIntEn,bDmaEn; };
constexpr int I2CMODE_MASTER=0;
''' + prefix + 'return true;\n}\n' + r'''
int main() {
 I2CDev_t dev; I2CCfg_t cfg{0,0,false,false};
 assert(!I2CInit(nullptr,&cfg)); assert(!I2CInit(&dev,nullptr));
 for(int i=-1;i<=4;i++) { cfg.DevNo=i; assert(I2CInit(&dev,&cfg)==(i>=0 && i<4)); }
 cfg.DevNo=0; cfg.Mode=1; assert(!I2CInit(&dev,&cfg)); cfg.Mode=0;
 cfg.bIntEn=true; assert(!I2CInit(&dev,&cfg)); cfg.bIntEn=false;
 cfg.bDmaEn=true; assert(!I2CInit(&dev,&cfg));
 puts("PASS I2C: invalid/unfinished modes rejected before mutation");
}
'''
s=source('timer_stm32l4x.cpp')
models['timer']=COMMON+r'''
#define STM32L4XX_LPTIM_CNT 2
#define STM32L4XX_TIMER_MAXCNT 13
struct TimerDev_t { int sentinel=123; }; struct TimerCfg_t { int DevNo; };
struct STM32L4XX_TimerData_t { TimerDev_t *pTimer=nullptr; } g_Stm32l4TimerData[13];
int chosen=-1;
bool Stm32l4LPTimInit(STM32L4XX_TimerData_t *p,const TimerCfg_t *c) { chosen=c->DevNo; return true; }
''' + function(source('timer_tim_stm32l4xx.cpp'),'Stm32l4TimInit')+'\n'+function(s,'TimerInit')+'\n'+function(s,'TimerGetHighFreqDevCount')+r'''
int main() {
 TimerDev_t timer;
 for(int i=-1;i<=13;i++) { TimerCfg_t c{i}; assert(TimerInit(&timer,&c)==(i>=0 && i<2)); }
 for(int i=2;i<13;i++) assert(!g_Stm32l4TimerData[i].pTimer);
 assert(chosen==1 && timer.sentinel==123 && TimerGetHighFreqDevCount()==0);
 assert(!Stm32l4TimInit(nullptr,nullptr)); assert(!TimerInit(nullptr,nullptr));
 puts("PASS timer: LPTIM dispatch preserved, TIM refusal without publication");
}
'''
s=source('octospi_stm32l4xx.cpp')
models['ospi']=COMMON+r'''
#define STM32L4XX_OSPI_DEVNO_START 3
#define STM32L4XX_SPI_MAXDEV 5
struct Reg { uint32_t CR=0, DCR2=0; } regs[2];
struct Rcc { uint32_t CFGR=0, AHB3RSTR=0, AHB3ENR=0; } rcc;
auto RCC=&rcc;
constexpr uint32_t RCC_CFGR_HPRE_Msk=0xf0, RCC_CFGR_HPRE_Pos=4;
constexpr uint32_t RCC_AHB3RSTR_OSPI1RST_Msk=1u<<8, RCC_AHB3ENR_OSPI1EN=1u<<8;
constexpr uint32_t OCTOSPI_DCR2_PRESCALER_Msk=255, OCTOSPI_DCR2_PRESCALER_Pos=0;
constexpr int IOPINDIR_INPUT=0, IOPINRES_PULLUP=0, IOPINTYPE_NORMAL=0;
uint32_t SystemCoreClock=120000000;
struct Pin { int PortNo,PinNo; };
struct DevIntrf_t { void *pDevData; };
struct SPIDEV { struct { int DevNo; uint32_t Rate; int NbIOPins; Pin *pIOPinMap; } Cfg; DevIntrf_t DevIntrf; };
struct STM32L4XX_SPIDev_t { int DevNo; Reg *pOReg; SPIDEV *pSpiDev; };
uint32_t resetMask;
void msDelay(int) { resetMask=rcc.AHB3RSTR; }
void STM32L4xxOSPIDisable(DevIntrf_t*) {}
void IOPinConfig(int,int,int,int,int,int) {}
int commandDevice=-1;
bool STM32L4xxOSPISendCmd(DevIntrf_t *p,uint8_t,uint32_t,uint8_t,uint32_t,uint8_t) {
 commandDevice=static_cast<STM32L4XX_SPIDev_t*>(p->pDevData)->DevNo; return true;
}
''' + '\n'.join(function(s,n) for n in ['STM32L4xxOSPISetRate','STM32L4xxOSPIReset','STM32L4xxOSPIPowerOff','QuadSPISendCmd'])+r'''
int main() {
 for(int i=3;i<5;i++) {
  SPIDEV spi{}; spi.Cfg.DevNo=i; STM32L4XX_SPIDev_t entry{i,&regs[i-3],&spi}; spi.DevIntrf.pDevData=&entry;
  STM32L4xxOSPIReset(&spi.DevIntrf); assert(resetMask==(1u<<(8+i-3))); assert(!rcc.AHB3RSTR);
  rcc.AHB3ENR=(1u<<8)|(1u<<9); STM32L4xxOSPIPowerOff(&spi.DevIntrf);
  assert(rcc.AHB3ENR==((1u<<8)|(1u<<9))-(1u<<(8+i-3)));
  assert(STM32L4xxOSPISetRate(&spi.DevIntrf,120000000)==120000000); assert(entry.pOReg->DCR2==0);
  assert(STM32L4xxOSPISetRate(&spi.DevIntrf,30000000)==30000000); assert(entry.pOReg->DCR2==3);
  assert(STM32L4xxOSPISetRate(&spi.DevIntrf,1)==120000000/256); assert(entry.pOReg->DCR2==255);
  assert(STM32L4xxOSPISetRate(&spi.DevIntrf,0)==0); assert(entry.pOReg->DCR2==255);
  assert(QuadSPISendCmd(&spi,0,0,0,0,0)); assert(commandDevice==i);
 }
 puts("PASS OSPI: both virtual devices, reset/clock masks and prescaler limits");
}
'''

with tempfile.TemporaryDirectory(prefix='stm32l4-') as directory:
    work=Path(directory)
    for name, code in models.items():
        file=work/(name+'.cpp'); file.write_text('#include <initializer_list>\n'+code)
        for variant in (['STM32L476xx','STM32L496xx','STM32L4S9xx'] if name=='spi' else ['host']):
            exe=work/(name+'-'+variant)
            subprocess.run(shlex.split(os.environ.get('CXX','g++'))+['-std=c++17','-O2','-g','-fsanitize=undefined','-fno-sanitize-recover=all','-D'+variant,str(file),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

# Startup, framing, RX overflow and timer lifecycle regressions.
import sys
subprocess.run([sys.executable, str(Path(__file__).with_name("runtime.py"))], check=True)
