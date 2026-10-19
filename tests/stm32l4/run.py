#!/usr/bin/env python3
"""Compile production function bodies against focused host register/dispatch models.

This checks virtual-device routing and unsupported-mode refusal, not peripheral
bus timing, a complete firmware build, or hardware operation. The UART driver,
ARM/ST/src/uart_stm32.cpp, is tested by tests/stm32/run_uart.py, the timer
driver, ARM/ST/src timer_*stm32.cpp, by tests/stm32/run_timer.py.
"""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
def function(text, name):
    match = re.search(r'^.*\b' + re.escape(name) + r'\([^;]*?\)\s*\{', text, re.M)
    assert match, name
    start = text.index('{', match.start())
    depth, end = 1, start + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[match.start():end]
COMMON = '#include <cassert>\n#include <cstdint>\n#include <cstdio>\n#include <cstring>\n'
models = {}
# QUADSPI and OCTOSPI, ARM/ST/src. SPI1 to SPI3 and I2C use the shared STM32
# drivers, tested by tests/stm32wba/run_spi.py and run_i2c.py.
SRCST = ROOT / 'ARM/ST/src'
s=(SRCST/'octospi_stm32.cpp').read_text()
models['ospi']=COMMON+r'''
#define STM32_QOSPI_DEVNO_START 3
#define STM32_OSPI_MAXDEV 2
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
struct SPIDev_t { struct { int DevNo; uint32_t Rate; int NbIOPins; Pin *pIOPinMap; } Cfg; DevIntrf_t DevIntrf; };
struct Stm32OspiDev_t { int Idx; SPIDev_t *pSpiDev; Reg *pReg; };
Stm32OspiDev_t s_Stm32OspiDev[STM32_OSPI_MAXDEV]={{0,nullptr,&regs[0]},{1,nullptr,&regs[1]}};
uint32_t resetMask;
void msDelay(int) { resetMask=rcc.AHB3RSTR; }
void Stm32OspiDisable(DevIntrf_t*) {}
void IOPinConfig(int,int,int,int,int,int) {}
int commandDevice=-1;
bool Stm32OspiSendCmd(Stm32OspiDev_t *d,uint32_t,uint32_t,uint8_t,uint32_t,uint8_t) { commandDevice=d->Idx; return true; }
''' + '\n'.join(function(s,n) for n in ['Stm32OspiDev','Stm32OspiSetRate','Stm32OspiReset','Stm32OspiPowerOff','QuadSPISendCmd'])+r'''
int main() {
 for(int i=0;i<2;i++) {
  SPIDev_t spi{}; spi.Cfg.DevNo=3+i; Stm32OspiDev_t &entry=s_Stm32OspiDev[i]; spi.DevIntrf.pDevData=&entry;
  commandDevice=-1; assert(!QuadSPISendCmd(&spi,0,0,0,0,0)); assert(commandDevice==-1);
  entry.pSpiDev=&spi;
  Stm32OspiReset(&spi.DevIntrf); assert(resetMask==(1u<<(8+i))); assert(!rcc.AHB3RSTR);
  rcc.AHB3ENR=(1u<<8)|(1u<<9); Stm32OspiPowerOff(&spi.DevIntrf);
  assert(rcc.AHB3ENR==((1u<<8)|(1u<<9))-(1u<<(8+i)));
  assert(Stm32OspiSetRate(&spi.DevIntrf,120000000)==120000000); assert(entry.pReg->DCR2==0);
  assert(Stm32OspiSetRate(&spi.DevIntrf,30000000)==30000000); assert(entry.pReg->DCR2==3);
  assert(Stm32OspiSetRate(&spi.DevIntrf,1)==120000000/256); assert(entry.pReg->DCR2==255);
  assert(Stm32OspiSetRate(&spi.DevIntrf,0)==0); assert(entry.pReg->DCR2==255);
  assert(QuadSPISendCmd(&spi,0,0,0,0,0)); assert(commandDevice==i);
 }
 SPIDev_t other{}; other.Cfg.DevNo=2; assert(!QuadSPISendCmd(&other,0,0,0,0,0));
 other.Cfg.DevNo=5; assert(!QuadSPISendCmd(&other,0,0,0,0,0));
 puts("PASS OSPI: both instances, reset/clock masks, prescaler limits and command routing");
}
'''
s=(SRCST/'quadspi_stm32.cpp').read_text()
models['qspi']=COMMON+r'''
#define STM32_QOSPI_DEVNO_START 3
struct Reg { uint32_t CR=0; } reg;
struct Rcc { uint32_t CFGR=0; } rcc;
auto RCC=&rcc;
constexpr uint32_t RCC_CFGR_HPRE_Msk=0xf0, RCC_CFGR_HPRE_Pos=4;
constexpr uint32_t QUADSPI_CR_PRESCALER_Pos=24, QUADSPI_CR_PRESCALER_Msk=255u<<24;
uint32_t SystemCoreClock=80000000;
struct DevIntrf_t { void *pDevData; };
struct SPIDev_t { struct { int DevNo; uint32_t Rate; } Cfg; DevIntrf_t DevIntrf; };
struct Stm32QspiDev_t { SPIDev_t *pSpiDev; Reg *pReg; };
Stm32QspiDev_t s_Stm32QspiDev={nullptr,&reg};
int commands=0;
bool Stm32QspiSendCmd(DevIntrf_t *p,uint8_t,uint32_t,uint8_t,uint32_t,uint8_t) { assert(p->pDevData==&s_Stm32QspiDev); commands++; return true; }
''' + '\n'.join(function(s,n) for n in ['Stm32QspiIsDev','Stm32QspiSetRate','QuadSPISendCmd'])+r'''
int main() {
 SPIDev_t spi{}; spi.Cfg.DevNo=3; spi.DevIntrf.pDevData=&s_Stm32QspiDev;
 assert(!QuadSPISendCmd(&spi,0,0,0,0,0)); assert(commands==0);
 s_Stm32QspiDev.pSpiDev=&spi;
 // SCK = HCLK / (PRESCALER + 1), 80 MHz HCLK
 assert(Stm32QspiSetRate(&spi.DevIntrf,80000000)==80000000); assert(reg.CR==0);
 assert(Stm32QspiSetRate(&spi.DevIntrf,20000000)==20000000); assert(reg.CR==(3u<<24));
 assert(Stm32QspiSetRate(&spi.DevIntrf,1)==80000000/256); assert(reg.CR==(255u<<24));
 assert(Stm32QspiSetRate(&spi.DevIntrf,0)==0); assert(reg.CR==(255u<<24));
 assert(QuadSPISendCmd(&spi,0,0,0,0,0)); assert(commands==1);
 SPIDev_t other{}; other.Cfg.DevNo=2; assert(!QuadSPISendCmd(&other,0,0,0,0,0)); assert(commands==1);
 puts("PASS QSPI: prescaler limits and command routing");
}
'''

with tempfile.TemporaryDirectory(prefix='stm32l4-') as directory:
    work=Path(directory)
    for name, code in models.items():
        file=work/(name+'.cpp'); file.write_text('#include <initializer_list>\n'+code)
        for variant in ['host']:
            exe=work/(name+'-'+variant)
            subprocess.run(shlex.split(os.environ.get('CXX','g++'))+['-std=c++17','-O2','-g','-fsanitize=undefined','-fno-sanitize-recover=all','-D'+variant,str(file),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

# Startup, framing, RX overflow and timer lifecycle regressions.
import sys
subprocess.run([sys.executable, str(Path(__file__).with_name("runtime.py"))], check=True)
