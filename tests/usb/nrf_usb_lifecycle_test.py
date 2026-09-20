#!/usr/bin/env python3
"""Exercise production controller lifecycle and clock ownership on the host."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
src = (ROOT / 'ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp').read_text()

# Start/stop pairing ownership moved to the usb core: UsbCtrlrStart is not
# idempotent and UsbCtrlrStop pairs with one successful start, so the core
# must guard both behind its started flag.
core = (ROOT / 'src/usb/usb.cpp').read_text()
assert 'if (s_UsbDevStarted)' in core
assert 'if (!s_UsbDevStarted)' in core
assert core.index('UsbCtrlrStart(') < core.index('s_UsbDevStarted = true;')


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
struct {uint8_t IntPrio;bool LowPowerSuspend;} s_Usbd;
bool cable, clockOK, readyOK;
unsigned requests, releases, clockRefs, starts, resets, waits, dispatches;
unsigned irqDisables, irqPriority;
constexpr int USBD_IRQn=7;
struct {uint32_t INTEN,USBPULLUP,ENABLE,LOWPOWER,EPDATASTATUS;} regs;
auto *NRF_USBD=&regs;
bool UsbCtrlrVbusDetected(int dev){(void)dev;return cable;}
bool UsbdXtalRequest(){++requests;if(clockOK)++clockRefs;return clockOK;}
void UsbdXtalRelease(){assert(clockRefs==1);--clockRefs;++releases;}
bool UsbdStartCtrlr(){++starts;return readyOK;}
void NVIC_SetPriority(int irq,uint8_t p){assert(irq==USBD_IRQn);irqPriority=p;}
void NVIC_DisableIRQ(int irq){assert(irq==USBD_IRQn);++irqDisables;}
void nRFUsbdDmaWait(){++waits;}
void nRFUsbdResetState(){++resets;}
void UsbdSync(){}
void __ISB(){}
void __DSB(){}
void AppEvtHandlerExec(){++dispatches;}
uint32_t DisableInterrupt(){return 0;}
void EnableInterrupt(uint32_t){}
uint32_t nRFUsbdQueueInComplete(uint32_t){assert(false);return 0;}
unsigned __CLZ(uint32_t){assert(false);return 0;}
void nRFUsbdProcessOutData(uint32_t,void*){assert(false);}
void init(){
 s_Usbd={6,false};
 cable=clockOK=readyOK=true;
 requests=releases=clockRefs=starts=resets=waits=dispatches=irqDisables=0;
 irqPriority=0;regs={0xFFFF,1,1,0};
}
'''
code += '\n'.join(function(n) for n in ['UsbCtrlrStart', 'UsbCtrlrStop', 'UsbCtrlrProcess'])
code += r'''
int main(){
 // Start/stop pairing and DevNo validation moved to the usb core
 // (s_UsbDevStarted); the controller owns only clock and peripheral state.
 for(unsigned attached=0;attached<2;++attached)
 for(unsigned clock=0;clock<2;++clock)
 for(unsigned ready=0;ready<2;++ready){
  init();cable=attached;clockOK=clock;readyOK=ready;
  const bool success=attached && clock && ready;
  assert(UsbCtrlrStart(0)==success);
  assert(clockRefs==unsigned(success));
  assert(requests==unsigned(attached));
  assert(starts==unsigned(attached && clock));
  // A failed controller start releases the clock it requested.
  assert(releases==unsigned(attached && clock && !ready));
  if(success){
   assert(irqPriority==6);
   UsbCtrlrStop(0);
   assert(clockRefs==0 && releases==1 && irqDisables==1);
   assert(waits==1 && resets==1);
   assert(regs.INTEN==0 && regs.USBPULLUP==0 && regs.ENABLE==0);
  }
 }
 init();
 for(unsigned n=0;n<8;++n){
  assert(UsbCtrlrStart(0));assert(clockRefs==1);
  UsbCtrlrStop(0);assert(clockRefs==0);
 }
 assert(requests==8 && releases==8);
 for(unsigned low=0;low<2;++low){
  init();regs.LOWPOWER=low;
  UsbCtrlrProcess(0);
  assert(dispatches==1 && regs.LOWPOWER==low && requests==0 && releases==0);
 }
 puts("PASS: lifecycle balances clock ownership on success/failure and repeated start/stop");
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
