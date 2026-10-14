#!/usr/bin/env python3
"""Focused production-function regressions; host models do not validate bus timing.

Clock. The UART driver is tested by tests/stm32/run_uart.py, the timer driver by
tests/stm32/run_timer.py.
"""
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
prefixes=['RCC_PLLCFGR_PLL','RCC_CFGR_SWS','RCC_CR_MSI','RCC_CSR_MSI']
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
with tempfile.TemporaryDirectory(prefix='stm32l4-runtime-') as d:
    for name,code in models.items():
        p=Path(d)/name;p=p.with_suffix('.cpp');p.write_text(code);exe=p.with_suffix('.out')
        subprocess.run(shlex.split(os.environ.get('CXX','g++'))+['-std=c++17','-O2','-fsanitize=undefined','-fno-sanitize-recover=all',str(p),'-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
