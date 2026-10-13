#!/usr/bin/env python3
"""Compile production function bodies against focused host register/dispatch models.

This checks virtual-device routing and unsupported-mode refusal, not peripheral
bus timing, a complete firmware build, or hardware operation. The UART driver,
ARM/ST/src/uart_stm32.cpp, is tested by tests/stm32/run_uart.py.
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
    start = text.index('STM32L4XX_SPIDev_t '+name)
    end = text.index('\n};', start) + 3
    return text[start:end]
COMMON = '#include <cassert>\n#include <cstdint>\n#include <cstdio>\n#include <cstring>\n'
models = {}
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
// The device headers define each instance as a macro. STM32L43x/L44x have no SPI2.
#ifndef STM32L432xx
#define SPI2 SPI2
#endif
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
  c.DevNo=i;
#ifdef STM32L432xx
  if(i==1) { int before=pins; assert(!SPIInit(&dev,&c)); assert(pins==before); continue; }
#endif
  assert(SPIInit(&dev,&c));
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
 puts("PASS SPI: ordinary/extended dispatch, absent instance and refusal before side effects");
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
struct STM32L4XX_I2CDEV { int DevNo; I2CDev_t *pI2cDev; I2C_TypeDef *pReg; };
I2C_TypeDef i2cregs[4];
STM32L4XX_I2CDEV s_STM32L4xxI2CDev[4]={{0,nullptr,&i2cregs[0]},{1,nullptr,&i2cregs[1]},{2,nullptr,&i2cregs[2]},{3,nullptr,&i2cregs[3]}};
''' + prefix + 'return true;\n}\n' + r'''
int main() {
 I2CDev_t dev; I2CCfg_t cfg{0,0,false,false};
 assert(!I2CInit(nullptr,&cfg)); assert(!I2CInit(&dev,nullptr));
 for(int i=-1;i<=4;i++) { cfg.DevNo=i; assert(I2CInit(&dev,&cfg)==(i>=0 && i<4)); }
 cfg.DevNo=0; cfg.Mode=1; assert(!I2CInit(&dev,&cfg)); cfg.Mode=0;
 cfg.bIntEn=true; assert(!I2CInit(&dev,&cfg)); cfg.bIntEn=false;
 cfg.bDmaEn=true; assert(!I2CInit(&dev,&cfg)); cfg.bDmaEn=false;
 // An instance the MCU does not have (I2C2 on STM32L43x/L44x) has no register
 s_STM32L4xxI2CDev[1].pReg=nullptr; cfg.DevNo=1; assert(!I2CInit(&dev,&cfg));
 cfg.DevNo=2; assert(I2CInit(&dev,&cfg));
 puts("PASS I2C: invalid/unfinished modes and absent instance rejected before mutation");
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
        for variant in (['STM32L432xx','STM32L476xx','STM32L496xx','STM32L4S9xx'] if name=='spi' else ['host']):
            exe=work/(name+'-'+variant)
            subprocess.run(shlex.split(os.environ.get('CXX','g++'))+['-std=c++17','-O2','-g','-fsanitize=undefined','-fno-sanitize-recover=all','-D'+variant,str(file),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

# Startup, framing, RX overflow and timer lifecycle regressions.
import sys
subprocess.run([sys.executable, str(Path(__file__).with_name("runtime.py"))], check=True)
