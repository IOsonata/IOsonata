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
struct nRFUsbEpReg_t {uint8_t *pBuffer;void *Handler;void *pContext;uint16_t MaxPacketSize;bool bBlocking;};
struct UsbdMock {uint8_t IntPrio;bool LowPowerSuspend;
 nRFUsbEpReg_t EpReg[8][2];} s_Usbd;
bool cable, clockOK, readyOK;
unsigned requests, releases, clockRefs, starts, resets, waits, dispatches;
unsigned irqDisables, irqPriority;
constexpr int USBD_IRQn=7;
struct {uint32_t INTEN,USBPULLUP,ENABLE,LOWPOWER,EPDATASTATUS,EVENTS_EP0SETUP;} regs;
auto *NRF_USBD=&regs;
bool UsbCtrlrVbusDetected(int dev){(void)dev;return cable;}
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
uint32_t DisableInterrupt(){return 0;}
void EnableInterrupt(uint32_t){}
uint32_t nRFUsbdQueueInComplete(uint32_t){assert(false);return 0;}
unsigned __CLZ(uint32_t){assert(false);return 0;}
void nRFUsbdProcessOutData(uint32_t,void*){assert(false);}
void nRFUsbdQueueEp0Setup(){assert(false);}
// Foreground OUT sweep support: no endpoint is registered in this harness,
// so the sweep sees null handlers and emits nothing.
constexpr unsigned NRFX_USBD_EP_COUNT=9;
enum UsbCtrlrEvtType_t {USB_CTRLR_EVT_DRDY=2};
void nRFUsbEpRegisteredEvent(uint8_t,uint8_t,UsbCtrlrEvtType_t,uint16_t){assert(false);}
void init(){
 s_Usbd={6,false,{}};
 cable=clockOK=readyOK=true;
 requests=releases=clockRefs=starts=resets=waits=dispatches=irqDisables=0;
 irqPriority=0;regs={0xFFFF,1,1,0};
}
'''
code += '\n'.join(function(n) for n in ['UsbCtrlrStart', 'UsbCtrlrStop', 'UsbCtrlrProcess'])
code += r'''
namespace startup {
constexpr uint32_t USBD_EVENTCAUSE_READY_Msk=1,POWER_USBREGSTATUS_OUTPUTRDY_Msk=2;
constexpr uint32_t USBD_INTEN_USBRESET_Msk=1,NRFX_USBD_READY_WAIT_LOOPS=3;
struct {uint32_t EVENTCAUSE,ENABLE,EVENTS_USBRESET,INTEN,INTENCLR,INTENSET;} regs;
struct {uint32_t USBREGSTATUS;} power;
auto *NRF_USBD=&regs;
auto *NRF_POWER=&power;
bool peripheralReady,regulatorReady;
unsigned applies,reverts,peripheralWaits,regulatorWaits,clearedIrqs;
void UsbdErrataApply(){++applies;assert(!regs.ENABLE);}
void UsbdErrataRevert(){++reverts;assert(regs.ENABLE && peripheralWaits==1);}
void NVIC_ClearPendingIRQ(int irq){assert(irq==USBD_IRQn);++clearedIrqs;}
bool UsbdWaitReady(const volatile uint32_t *reg,uint32_t mask,uint32_t loops){
 assert(loops==NRFX_USBD_READY_WAIT_LOOPS && regs.ENABLE);
 if(reg==&regs.EVENTCAUSE){
  assert(applies==1 && !reverts && mask==USBD_EVENTCAUSE_READY_Msk);
  ++peripheralWaits;return peripheralReady;
 }
 assert(reg==&power.USBREGSTATUS && mask==POWER_USBREGSTATUS_OUTPUTRDY_Msk);
 assert(reverts==1 && peripheralReady);++regulatorWaits;return regulatorReady;
}
'''
code += function('UsbdStartCtrlr')
code += r'''
void check(){
 for(unsigned controller=0;controller<2;++controller)
 for(unsigned regulator=0;regulator<2;++regulator){
  regs={0,0,1,0xFFFF,0,0};peripheralReady=controller;regulatorReady=regulator;
  applies=reverts=peripheralWaits=regulatorWaits=clearedIrqs=0;
  const bool success=controller && regulator;
  assert(UsbdStartCtrlr()==success && regs.ENABLE==unsigned(success));
  assert(applies==1 && reverts==1 && peripheralWaits==1);
  assert(regulatorWaits==controller && clearedIrqs==unsigned(success));
  if(success)assert(!regs.EVENTS_USBRESET && regs.INTENCLR==0xFFFF &&
   regs.INTENSET==USBD_INTEN_USBRESET_Msk);
  else assert(regs.EVENTS_USBRESET && !regs.INTENCLR && !regs.INTENSET);
 }
 puts("PASS: startup restores errata once, disables on either timeout, and arms IRQ only when both ready");
}
}
namespace clock_request {
#define SOFTDEVICE_PRESENT 1
constexpr uint32_t NRF_SUCCESS=0,NRFX_USBD_XTAL_WAIT_LOOPS=3;
constexpr uint32_t CLOCK_HFCLKSTAT_STATE_Msk=2,CLOCK_HFCLKSTAT_SRC_Msk=1;
constexpr uint32_t CLOCK_HFCLKSTAT_SRC_Xtal=1,CLOCK_HFCLKSTAT_SRC_Pos=0;
struct {uint32_t HFCLKSTAT,EVENTS_HFCLKSTARTED,TASKS_HFCLKSTART;} clock;
auto *NRF_CLOCK=&clock;
bool requestOK,pollOK,running;
unsigned requested,released,polls;
bool UsbdSdRunning(){return true;}
uint32_t sd_clock_hfclk_request(){++requested;return requestOK?NRF_SUCCESS:1;}
uint32_t sd_clock_hfclk_release(){++released;return NRF_SUCCESS;}
uint32_t sd_clock_hfclk_is_running(uint32_t *value){
 ++polls;*value=running;return pollOK?NRF_SUCCESS:1;
}
bool UsbdWaitReady(const volatile uint32_t*,uint32_t,uint32_t){assert(false);return false;}
'''
code += function('UsbdXtalRequest')
code += r'''
#undef SOFTDEVICE_PRESENT
void check(){
 for(unsigned request=0;request<2;++request)
 for(unsigned poll=0;poll<2;++poll)
 for(unsigned run=0;run<2;++run){
  requestOK=request;pollOK=poll;running=run;requested=released=polls=0;
  const bool success=request && poll && run;
  assert(UsbdXtalRequest()==success && requested==1);
  assert(released==unsigned(request && !success));
  assert(polls==(request?(poll && !run?NRFX_USBD_XTAL_WAIT_LOOPS:1):0));
 }
 puts("PASS: SoftDevice clock request releases on poll error or timeout, but not on success or request failure");
}
}
'''
code += r'''
int main(){
 startup::check();clock_request::check();
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
