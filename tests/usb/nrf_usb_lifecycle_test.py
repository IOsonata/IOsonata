#!/usr/bin/env python3
"""Exercise production controller lifecycle and clock ownership on the host."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
src = (ROOT / 'ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp').read_text()


def function(name):
    match = re.search(r'(?:bool|void)\s+' + name + r'\([^;{}]*\)\s*\{', src)
    assert match, name
    brace = src.index('{', match.start())
    end, depth = brace + 1, 1
    while depth:
        depth += (src[end] == '{') - (src[end] == '}')
        end += 1
    return src[match.start():end]


code = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
bool s_UsbdInitialized, s_UsbdStarted;
uint8_t s_UsbdIntPrio;
bool cable, clockOK, readyOK;
unsigned requests, releases, clockRefs, starts, resets, waits, dispatches;
unsigned irqDisables, irqPriority;
constexpr int USBD_IRQn=7;
struct {uint32_t INTEN,USBPULLUP,ENABLE,LOWPOWER;} regs;
auto *NRF_USBD=&regs;
bool UsbCtrlrVbusDetected(int dev){assert(dev==0);return cable;}
bool UsbdXtalRequest(){++requests;if(clockOK)++clockRefs;return clockOK;}
void UsbdXtalRelease(){assert(clockRefs==1);--clockRefs;++releases;}
bool UsbdStartCtrlr(){++starts;return readyOK;}
void NVIC_SetPriority(int irq,uint8_t p){assert(irq==USBD_IRQn);irqPriority=p;}
void NVIC_DisableIRQ(int irq){assert(irq==USBD_IRQn);++irqDisables;}
void nRFUsbdDmaWait(){++waits;}
void nRFUsbdResetState(){++resets;}
void __ISB(){}
void __DSB(){}
void AppEvtHandlerExec(){++dispatches;}
void init(){
 s_UsbdInitialized=true;s_UsbdStarted=false;s_UsbdIntPrio=6;
 cable=clockOK=readyOK=true;
 requests=releases=clockRefs=starts=resets=waits=dispatches=irqDisables=0;
 irqPriority=0;regs={0xFFFF,1,1,0};
}
'''
code += '\n'.join(function(n) for n in ['UsbCtrlrStart', 'UsbCtrlrStop', 'UsbCtrlrProcess'])
code += r'''
int main(){
 for(unsigned initial=0;initial<2;++initial)
 for(unsigned attached=0;attached<2;++attached)
 for(unsigned clock=0;clock<2;++clock)
 for(unsigned ready=0;ready<2;++ready){
  init();s_UsbdInitialized=initial;cable=attached;clockOK=clock;readyOK=ready;
  const bool success=initial && attached && clock && ready;
  assert(UsbCtrlrStart(0)==success);
  assert(s_UsbdStarted==success && clockRefs==unsigned(success));
  assert(requests==unsigned(initial && attached));
  assert(starts==unsigned(initial && attached && clock));
  assert(releases==unsigned(initial && attached && clock && !ready));
  unsigned req=requests,rel=releases;
  if(success){assert(UsbCtrlrStart(0));assert(requests==req);}
  UsbCtrlrStop(0);
  assert(!s_UsbdStarted && clockRefs==0);
  assert(releases==rel+unsigned(success));
  assert(irqDisables==unsigned(success));
  if(success)assert(regs.INTEN==0 && regs.USBPULLUP==0 && regs.ENABLE==0);
  rel=releases;UsbCtrlrStop(0);assert(releases==rel);
 }
 init();
 for(unsigned n=0;n<8;++n){
  assert(UsbCtrlrStart(0));assert(clockRefs==1);
  UsbCtrlrStop(0);assert(clockRefs==0);
 }
 assert(requests==8 && releases==8);
 init();assert(!UsbCtrlrStart(1));UsbCtrlrStop(1);UsbCtrlrProcess(1);
 assert(requests==0 && releases==0 && resets==0 && dispatches==0);
 for(unsigned started=0;started<2;++started)for(unsigned low=0;low<2;++low){
  init();s_UsbdStarted=started;regs.LOWPOWER=low;
  UsbCtrlrProcess(0);
  assert(dispatches==1 && regs.LOWPOWER==low && requests==0 && releases==0);
 }
 puts("PASS: lifecycle balances clock ownership on success/failure, repeated start/stop, and invalid controller calls");
 puts("PASS: foreground processing dispatches AppEvt without a second peripheral power owner");
}
'''
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / 'lifecycle.cpp'
    path.write_text(code)
    target = Path(directory) / 'lifecycle'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    str(path), '-o', str(target)], check=True)
    subprocess.run([str(target)], check=True)
