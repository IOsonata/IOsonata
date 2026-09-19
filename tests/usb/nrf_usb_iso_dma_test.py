#!/usr/bin/env python3
"""Exercise production ISO scheduling with simulated registers and real AppEvt.

This is a host ownership/ordering test, not a model of USB electrical timing.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
BASE_SOURCE = ROOT / 'ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp'
ISO_SOURCE = ROOT / 'ARM/Nordic/nRF52/src/usb_ctrlr_nrf52_iso.cpp'
PRIV_SOURCE = ROOT / 'ARM/Nordic/nRF52/src/usb_ctrlr_nrf52_priv.h'
src = '\n'.join((PRIV_SOURCE.read_text(), BASE_SOURCE.read_text(), ISO_SOURCE.read_text()))

def function(name):
    import re
    match = re.search(r'(?:void|bool|uint8_t|uint16_t|uint32_t|volatile uint32_t \*|nRFUsbdXfer_t \*|nRFUsbEpReg_t \*)\s*'
                      + name + r'\([^;{}]*\)\s*\{', src)
    assert match, name
    brace = src.index('{', match.start())
    depth = 1
    end = brace + 1
    while depth:
        depth += (src[end] == '{') - (src[end] == '}')
        end += 1
    return src[match.start():end]

preamble = r'''
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include "usb/usb.h"
#include "app_evt_handler.h"
using namespace std;
constexpr uint8_t NRFX_USBD_ISO_EP_NO=8;
constexpr uint_fast8_t NRFX_USBD_ISO_IN_OPEN=2, NRFX_USBD_ISO_OUT_OPEN=1;
constexpr uint32_t USBD_SIZE_ISOOUT_ZERO_Msk=1UL<<16;
constexpr uint32_t USBD_INTENCLR_SOF_Msk=1, USBD_INTEN_SOF_Msk=1;
constexpr unsigned USBD_INTEN_ENDISOIN_Pos=11;
constexpr uint32_t USBD_INTEN_ENDISOIN_Msk=1U<<11, USBD_INTEN_ENDISOOUT_Msk=1U<<20;
constexpr unsigned USBD_INTEN_ENDEPIN0_Pos=2, USBD_INTEN_ENDEPOUT0_Pos=12;
constexpr unsigned NRFX_USBD_EP_COUNT=9;
constexpr unsigned NRFX_USBD_MAX_PACKET_SIZE=64;
constexpr unsigned NRFX_USBD_EASYDMA_BUSY_REG_BUSY=0x82, NRFX_USBD_EASYDMA_BUSY_REG_CLEAR=0;
uint32_t dmaBusy=0;
#define NRFX_USBD_EASYDMA_BUSY_REG dmaBusy
struct W1C {uint32_t bits=0; operator uint32_t()const{return bits;}
 void operator=(uint32_t value){bits&=~value;}};
struct Task {uint32_t value=0;void operator=(uint32_t);};
struct Endpoint {uint32_t PTR=0,MAXCNT=0,AMOUNT=0;};
struct Registers {
 uint32_t EVENTS_USBRESET=0,EVENTS_STARTED=0,EVENTS_ENDEPIN[8]={};
 uint32_t EVENTS_EP0DATADONE=0,EVENTS_ENDISOIN=0;
 uint32_t EVENTS_ENDEPOUT[8]={},EVENTS_ENDISOOUT=0;
 uint32_t TASKS_STARTEPIN[8]={},TASKS_STARTEPOUT[8]={};
 // Task is modeled through the barrier below: hardware latches at task issue.
 uint32_t TASKS_STARTISOIN=0,TASKS_STARTISOOUT=0;
 Endpoint ISOIN,ISOOUT;W1C EPSTATUS,EPDATASTATUS;
 uint32_t EPINEN=0,EPOUTEN=0,INTENSET=0,INTENCLR=0,FRAMECNTR=0;
 struct {uint32_t ISOOUT=0,EPOUT[8]={};} SIZE;
} regs;
auto *NRF_USBD=&regs;
struct nRFUsbdXfer_t {uint8_t *pBuffer;uint16_t TotalLen;volatile uint16_t ActualLen;};
struct nRFUsbEpReg_t {uint8_t *pBuffer;UsbCtrlrEpHandler_t Handler;void *pContext;uint16_t Mps;bool bBlocking;};
typedef Endpoint USBD_ISOIN_Type;
typedef Endpoint USBD_ISOOUT_Type;
FLAG_ENUM
// One state block, as in the driver; the flag word carries the former
// atomic fields at the same OUT-low/IN-high bit pairing.
struct {
 volatile uint32_t Flags=0;
 uint32_t IsoGeneration[2]={};uint16_t IsoOutSize=0;
 struct {nRFUsbdXfer_t Ep0[2],Iso[2];bool SofEnabled;} Ctrlr;
 nRFUsbEpReg_t EpReg[9][2];
} s_Usbd;
#define ISO_OPEN() ((s_Usbd.Flags / USBD_FLAG_ISO_OUT_OPEN) & 3u)
#define ISO_BUSY() ((s_Usbd.Flags / USBD_FLAG_ISO_OUT_BUSY) & 3u)
#define ISO_CMPL() ((s_Usbd.Flags / USBD_FLAG_ISO_OUT_CMPL) & 3u)
unsigned irqMask=0,isoStarts[2]={},regularStarts=0;
unsigned activeDir=0;
uint8_t inBuffer[512],outBuffer[512],wireIn[512],hostOut[512];
unsigned callbacks[2]={};uint16_t lengths[2]={};
bool interruptCopy=false,chainIn=false;
uint32_t DisableInterrupt(){auto old=irqMask;irqMask=1;return old;}
void EnableInterrupt(uint32_t old){irqMask=old;}
void __ISB(){}
void __DSB(){
 if(regs.TASKS_STARTISOIN || regs.TASKS_STARTISOOUT){
  activeDir=regs.TASKS_STARTISOIN?1:0;
  assert(dmaBusy==0x82);assert(regs.EPSTATUS.bits==0);
  assert(s_Usbd.Flags & ((uint32_t)USBD_FLAG_ISO_OUT_BUSY<<activeDir));
  regs.EPSTATUS.bits=1UL<<(activeDir?8:24);
  ++isoStarts[activeDir];
  if(activeDir) memcpy(wireIn,inBuffer,regs.ISOIN.MAXCNT);
  else memcpy(outBuffer,hostOut,regs.ISOOUT.MAXCNT);
  regs.TASKS_STARTISOIN=regs.TASKS_STARTISOOUT=0;
 }
}
void nRFUsbdHostResumeDetected(){}
void nRFUsbdEmit(const UsbCtrlrEvt_t*){}
void nRFUsbdRetryIsoComplete();
void nRFUsbdResumeQueuedDmaLocked();
void nRFUsbdDmaWait();
bool nRFUsbRegDataEpXfer(uint8_t,uint16_t);
'''
names = ['UsbdSync','nRFUsbdDmaEndBit','nRFUsbdDmaEndEvent','nRFUsbdDir','nRFUsbEpDir',
         'nRFUsbGetEpReg','nRFUsbEpRegisteredEvent','nRFUsbdDmaActive','nRFUsbdDmaUnlock',
         'nRFUsbdDmaStartLocked','nRFUsbdEpHwEnable','nRFUsbdSofRelease',
         'nRFIsoDir','nRFIsoReg','nRFIsoRegisteredEvent','nRFUsbdStartIsoNow',
         'nRFUsbdServiceIso','nRFUsbRegIsoXfer','nRFUsbdProcessIsoComplete',
         'nRFUsbdRetryIsoComplete','nRFUsbdFinishIsoDma','nRFUsbdIsoStart',
         'nRFUsbdIsoService','nRFUsbdIsoFinishDma','nRFUsbdIsoSof',
         'nRFUsbdIsoEpClose','UsbCtrlrEpClose','nRFUsbdHandleSof']
import re as _re
flag_enum = _re.search(r'enum\s*\{[^}]*USBD_FLAG_ISO_IN_CMPL[^}]*\};', src)
assert flag_enum, 'USBD_FLAG enum not found in driver source'
code = preamble.replace('FLAG_ENUM', flag_enum.group(0))
code += '\n'.join(function(n) for n in names)
code += r'''
bool nRFUsbRegDataEpXfer(uint8_t ep,uint16_t length){return nRFUsbRegIsoXfer(ep,length);}
void nRFUsbdDmaWait(){if(dmaBusy)assert(nRFUsbdFinishIsoDma(activeDir!=0));}
void nRFUsbdResumeQueuedDmaLocked(){
 const uint32_t gate=s_Usbd.Flags&
  (USBD_FLAG_HOST_RESUME|USBD_FLAG_SUSPENDED|USBD_FLAG_SUSPEND_PEND);
 if(dmaBusy||(gate&USBD_FLAG_HOST_RESUME)||gate==USBD_FLAG_SUSPENDED)return;
 if(!nRFUsbdStartIsoNow())++regularStarts;
}
void finish(bool in){
 assert(dmaBusy && activeDir==unsigned(in));
 if(in){regs.ISOIN.AMOUNT=regs.ISOIN.MAXCNT;regs.EVENTS_ENDISOIN=1;}
 else{regs.ISOOUT.AMOUNT=regs.ISOOUT.MAXCNT;regs.EVENTS_ENDISOOUT=1;}
 assert(nRFUsbdFinishIsoDma(in));assert(!dmaBusy && regs.EPSTATUS.bits==0);
 nRFUsbdResumeQueuedDmaLocked();
}
void frame(uint16_t length=0,bool zero=false){
 ++regs.FRAMECNTR;regs.SIZE.ISOOUT=zero?USBD_SIZE_ISOOUT_ZERO_Msk:length;
 nRFUsbdHandleSof();
}
void callback(uint8_t ep,UsbCtrlrEvtType_t event,uint16_t length,UsbCtrlrXferResult_t,void*){
 assert(event==USB_CTRLR_EVT_XFER_CMPL && !irqMask);
 unsigned dir=USB_ENDPADDR_IS_IN(ep)?1:0;++callbacks[dir];lengths[dir]=length;
 if(!dir){
  assert(ISO_BUSY()&1);uint8_t copy[512];memcpy(copy,outBuffer,length);
  if(interruptCopy){
   auto before=isoStarts[0];memset(hostOut,0xDD,sizeof(hostOut));frame(17);
   assert(isoStarts[0]==before && !memcmp(copy,outBuffer,length));
  }
 }else if(chainIn){chainIn=false;assert(nRFUsbRegIsoXfer(0x88,9));}
}
void init(){
 regs={};s_Usbd.Ctrlr={};memset(s_Usbd.EpReg,0,sizeof(s_Usbd.EpReg));
 // Both ISO directions open; every other flag (busy, complete, ready,
 // suspend group) cleared, exactly the former per-field init.
 s_Usbd.Flags=USBD_FLAG_ISO_OUT_OPEN|USBD_FLAG_ISO_IN_OPEN;
 dmaBusy=0;irqMask=0;isoStarts[0]=isoStarts[1]=regularStarts=0;
 callbacks[0]=callbacks[1]=0;chainIn=interruptCopy=false;
 ++s_Usbd.IsoGeneration[0];++s_Usbd.IsoGeneration[1];
 s_Usbd.EpReg[8][0]={outBuffer,callback,nullptr,512,false};
 s_Usbd.EpReg[8][1]={inBuffer,callback,nullptr,512,false};
 memset(inBuffer,0xA5,sizeof(inBuffer));memset(hostOut,0x5A,sizeof(hostOut));
 assert(AppEvtHandlerInit(nullptr,0));assert(AppEvtHandlerIdleRegister(nRFUsbdRetryIsoComplete));
}
void dummy(uint32_t,void*){}
int main(){
 for(unsigned queued=0;queued<=4;++queued)for(unsigned masked=0;masked<2;++masked){
  init();for(unsigned n=0;n<queued;++n)assert(AppEvtHandlerQue(n,nullptr,dummy));
  // Both END events retired while publication was deferred.
  s_Usbd.Flags|=USBD_FLAG_ISO_OUT_BUSY|USBD_FLAG_ISO_IN_BUSY|
   USBD_FLAG_ISO_OUT_CMPL|USBD_FLAG_ISO_IN_CMPL;
  irqMask=masked;
  nRFUsbdRetryIsoComplete();
  assert(irqMask==masked && ISO_BUSY()==3);
  assert(ISO_CMPL()==(queued==4?3U:queued==3?2U:0U));
  assert(callbacks[0]==0 && callbacks[1]==0);
  irqMask=0;AppEvtHandlerExec();AppEvtHandlerExec();
  assert(callbacks[0]==1 && callbacks[1]==1);
  assert(ISO_BUSY()==0 && ISO_CMPL()==0);
 }
 puts("PASS: paired ISO completion retry preserves partial/full queue state and delivers each callback once");

 init();frame();assert(isoStarts[1]==0); // no unsolicited IN ZLP/DMA
 assert(nRFUsbRegIsoXfer(0x88,9));assert(dmaBusy && isoStarts[1]==1);
 assert(!nRFUsbdFinishIsoDma(true));assert(dmaBusy); // matching END required
 finish(true);assert(callbacks[1]==0);frame();assert(isoStarts[1]==1);
 AppEvtHandlerExec();assert(callbacks[1]==1 && lengths[1]==9 && !(ISO_BUSY()&2));
 frame();assert(isoStarts[1]==1); // never retransmit previous payload
 puts("PASS: ISO IN requires a request, retires only at END, completes once, never repeats old data");

 init();assert(nRFUsbRegIsoXfer(0x88,33));assert(!dmaBusy);
 frame(17);assert(activeDir==1 && isoStarts[1]==1 && isoStarts[0]==0);
 finish(true);assert(dmaBusy && activeDir==0);finish(false);
 AppEvtHandlerExec();assert(callbacks[0]==1 && callbacks[1]==1);
 assert(lengths[0]==17 && lengths[1]==33 && ISO_BUSY()==0 && regularStarts>0);
 puts("PASS: duplex ISO uses one DMA; both completions delivered; shared scheduler resumes");

 init();frame(17);finish(false);interruptCopy=true;
 for(int i=0;i<3;++i)frame(17);
 assert(isoStarts[0]==1 && callbacks[0]==0);
 AppEvtHandlerExec();assert(callbacks[0]==1 && ISO_BUSY()==0 && isoStarts[0]==1);
 frame(9);assert(isoStarts[0]==2);finish(false);interruptCopy=false;AppEvtHandlerExec();
 puts("PASS: delayed OUT callback and SOF during copy cannot overwrite the RX buffer");

 init();for(int i=0;i<4;++i)assert(AppEvtHandlerQue(i,nullptr,dummy));
 frame(25);finish(false);assert(ISO_CMPL()==1 && ISO_BUSY()==1 && !dmaBusy);
 AppEvtHandlerExec();assert(callbacks[0]==0 && ISO_CMPL()==0 && ISO_BUSY()==1);
 AppEvtHandlerExec();assert(callbacks[0]==1 && ISO_BUSY()==0);
 puts("PASS: full real AppEvt queue retains completion, retries, and does not retain EasyDMA");

 init();frame(17);finish(false);UsbCtrlrEpClose(0,8);
 s_Usbd.Flags|=USBD_FLAG_ISO_OUT_OPEN;s_Usbd.EpReg[8][0].Mps=9;frame(9);finish(false);
 AppEvtHandlerExec();assert(callbacks[0]==1 && lengths[0]==9 && ISO_BUSY()==0);
 puts("PASS: close/reopen discards old callback without releasing the new transfer");

 init();assert(nRFUsbRegIsoXfer(0x88,0));frame(0,true);
 finish(true);finish(false);AppEvtHandlerExec();
 assert(callbacks[0]==1 && callbacks[1]==1 && lengths[0]==0 && lengths[1]==0);
 init();assert(nRFUsbRegIsoXfer(0x88,512));frame();finish(true);
 chainIn=true;AppEvtHandlerExec();assert(ISO_BUSY()&2);frame();finish(true);AppEvtHandlerExec();
 assert(callbacks[1]==2 && lengths[1]==9);
 puts("PASS: explicit ZLPs, 512-byte IN and callback submission are supported");

 init();s_Usbd.Flags|=USBD_FLAG_SUSPENDED;assert(nRFUsbRegIsoXfer(0x88,17));frame();assert(!dmaBusy);
 s_Usbd.Flags&=~(uint32_t)USBD_FLAG_SUSPENDED;frame();assert(dmaBusy);finish(true);AppEvtHandlerExec();
 assert(callbacks[1]==1);
 puts("PASS: suspended submission waits until resume");

 for(unsigned ep=1;ep<8;++ep)for(unsigned dir=0;dir<2;++dir)for(unsigned masked=0;masked<2;++masked){
  init();irqMask=masked;regs.EPINEN=regs.EPOUTEN=0x1FF;
  regs.EPDATASTATUS.bits=0x00FF00FF;
  for(unsigned n=0;n<8;++n){
   regs.EVENTS_ENDEPIN[n]=regs.EVENTS_ENDEPOUT[n]=1;
   regs.SIZE.EPOUT[n]=64;s_Usbd.EpReg[n][0].Mps=s_Usbd.EpReg[n][1].Mps=64;
  }
  UsbCtrlrEpClose(0,ep|(dir?0x80:0));
  assert(irqMask==masked && ISO_OPEN()==3 && ISO_BUSY()==0);
  assert(regs.EPINEN==(dir?(0x1FFU&~(1U<<ep)):0x1FFU));
  assert(regs.EPOUTEN==(!dir?(0x1FFU&~(1U<<ep)):0x1FFU));
  assert(regs.INTENCLR==(1U<<((dir?USBD_INTEN_ENDEPIN0_Pos:USBD_INTEN_ENDEPOUT0_Pos)+ep)));
  assert(regs.EPDATASTATUS.bits==(0x00FF00FFU&~(1U<<(ep+(dir?0:16)))));
  for(unsigned n=0;n<8;++n){
   assert(regs.EVENTS_ENDEPIN[n]==unsigned(!(n==ep&&dir)));
   assert(regs.EVENTS_ENDEPOUT[n]==unsigned(!(n==ep&&!dir)));
   assert(regs.SIZE.EPOUT[n]==((n==ep&&!dir)?0U:64U));
   for(unsigned d=0;d<2;++d)assert(s_Usbd.EpReg[n][d].Mps==((n==ep&&d==dir)?0U:64U));
  }
 }
 puts("PASS: regular close affects only the selected endpoint/direction and preserves IRQ state");
}
'''
with tempfile.TemporaryDirectory(prefix='iosonata-iso-') as temp:
    path = Path(temp) / 'iso_test.cpp'
    path.write_text(code)
    compiler = os.environ.get('CXX', 'g++')
    subprocess.run([compiler, '-std=c++17', '-O1', '-Wall', '-Wextra',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    '-I'+str(ROOT/'include'), '-I'+str(ROOT/'tests/usb/hostport'),
                    '-x', 'c++', str(path), str(ROOT/'src/app_evt_handler.cpp'),
                    str(ROOT/'src/cfifo.c'), '-o', str(Path(temp)/'iso_test')], check=True)
    subprocess.run([str(Path(temp)/'iso_test')], check=True)
