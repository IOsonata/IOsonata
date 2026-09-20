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
HEADER_SOURCE = ROOT / 'ARM/Nordic/include/usb_ctrlr.h'
src = '\n'.join((HEADER_SOURCE.read_text(), BASE_SOURCE.read_text(), ISO_SOURCE.read_text()))

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
#include "cfifo.h"
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
unsigned dmaLocks=0,dmaUnlocks=0;
struct BusyRegister {
 operator uint32_t() const{return dmaBusy;}
 void operator=(uint32_t value){
  if(value==0x82){assert(!dmaBusy);++dmaLocks;}else{assert(value==0);++dmaUnlocks;}
  dmaBusy=value;
 }
} dmaRegister;
#define NRFX_USBD_EASYDMA_BUSY_REG dmaRegister
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
struct nRFUsbEpReg_t {uint8_t *pBuffer;UsbCtrlrEpHandler_t Handler;void *pContext;uint16_t MaxPacketSize;bool bBlocking;};
typedef Endpoint USBD_ISOIN_Type;
typedef Endpoint USBD_ISOOUT_Type;
FLAG_ENUM
QUEUE_TYPES
// One state block, as in the driver; the flag word carries the former
// atomic fields at the same OUT-low/IN-high bit pairing.
struct {
 volatile uint32_t Flags=0;
 uint32_t IsoGeneration[2]={};uint16_t IsoOutSize=0;
 struct {nRFUsbdXfer_t Ep0[2],Iso[2];bool SofEnabled;} Ctrlr;
 nRFUsbEpReg_t EpReg[8][2];
 hCFifo_t hQue;
} s_Usbd;
alignas(8) uint8_t queueMemory[CFIFO_TOTAL_MEMSIZE(16,sizeof(nRFUsbdQue_t))];
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
bool nRFUsbdQueXferDir(uint8_t,bool,uint16_t){assert(false);return false;}
bool productionEpInXfer(int,uint8_t,uint8_t*,uint16_t);
'''
names = ['UsbdSync','nRFUsbdDmaEndBit','nRFUsbdDmaEndEvent',
         'nRFUsbGetEpReg','nRFUsbEpRegisteredEvent','nRFUsbdDmaActive','nRFUsbdDmaLock','nRFUsbdDmaUnlock',
         'nRFUsbdDmaStartLocked','nRFUsbdEpHwEnable','nRFUsbdSofRelease',
         'nRFUsbdResumeQueuedDma',
         'nRFIsoDir','nRFIsoReg','nRFIsoHwEnable','nRFUsbdStartIsoNow',
         'nRFUsbdServiceIso','nRFUsbRegIsoXfer','nRFUsbdProcessIsoComplete',
         'nRFUsbdRetryIsoComplete','nRFUsbdFinishIsoDma','nRFUsbdIsoStart',
         'nRFUsbdIsoService','nRFUsbdIsoFinishDma','nRFUsbdIsoSof',
         'nRFUsbdIsoEpClose','UsbCtrlrEpClose','nRFUsbdHandleSof',
         'nRFUsbdIsoXfer','UsbCtrlrEpXfer']
import re as _re
flag_enum = _re.search(r'enum\s*\{[^}]*USBD_FLAG_ISO_IN_CMPL[^}]*\};', src)
assert flag_enum, 'USBD_FLAG enum not found in driver source'
code = preamble.replace('FLAG_ENUM', flag_enum.group(0))
queue_enum = _re.search(r'enum\s*\{[^}]*NRFX_USBD_QUE_IN_SCRATCH[^}]*\};', src)
queue_type = _re.search(r'typedef struct __nRF_Usbd_Que \{.*?\} nRFUsbdQue_t;',
                        src, _re.S)
assert queue_enum and queue_type
code = code.replace('QUEUE_TYPES', queue_enum.group(0) + '\n#pragma pack(push,4)\n' +
                    queue_type.group(0) + '\n#pragma pack(pop)\n')
code += '\n'.join(function(n) for n in names)
# Exercise the production entry point instead of the hostport inline adapter.
code += function('UsbCtrlrEpInXfer').replace('UsbCtrlrEpInXfer(', 'productionEpInXfer(')
code += r'''
bool nRFUsbRegDataEpXfer(uint8_t ep,uint16_t length){return nRFUsbRegIsoXfer(ep,length);}
void nRFUsbdDmaWait(){
 if(dmaBusy){assert(nRFUsbdFinishIsoDma(activeDir!=0));nRFUsbdDmaUnlock();}
}
void nRFUsbdResumeQueuedDmaLocked(){
 const uint32_t gate=s_Usbd.Flags&
  (USBD_FLAG_HOST_RESUME|USBD_FLAG_SUSPENDED|USBD_FLAG_SUSPEND_PEND);
 if(dmaBusy||(gate&USBD_FLAG_HOST_RESUME)||gate==USBD_FLAG_SUSPENDED)return;
 nRFUsbdDmaLock();
 if(!nRFUsbdStartIsoNow()){++regularStarts;nRFUsbdDmaUnlock();}
}
void finish(bool in){
 assert(dmaBusy && activeDir==unsigned(in));
 if(in){regs.ISOIN.AMOUNT=regs.ISOIN.MAXCNT;regs.EVENTS_ENDISOIN=1;}
 else{regs.ISOOUT.AMOUNT=regs.ISOOUT.MAXCNT;regs.EVENTS_ENDISOOUT=1;}
 const unsigned locks=dmaLocks,unlocks=dmaUnlocks;
 assert(nRFUsbdFinishIsoDma(in));assert(dmaBusy && regs.EPSTATUS.bits==0);
 if(!nRFUsbdStartIsoNow()){++regularStarts;nRFUsbdDmaUnlock();}
 assert(dmaLocks==locks && dmaUnlocks==unlocks+unsigned(!dmaBusy));
}
void frame(uint16_t length=0,bool zero=false){
 ++regs.FRAMECNTR;regs.SIZE.ISOOUT=zero?USBD_SIZE_ISOOUT_ZERO_Msk:length;
 nRFUsbdHandleSof();
}
void callback(uint8_t ep,UsbCtrlrEvtType_t event,uint16_t length,UsbCtrlrXferResult_t result,void*){
 assert(event==USB_CTRLR_EVT_XFER_CMPL && !irqMask);
 assert(USB_ENDPADDR_NUM(ep)==8 && result==USB_CTRLR_XFER_SUCCESS);
 unsigned dir=USB_ENDPADDR_IS_IN(ep)?1:0;++callbacks[dir];lengths[dir]=length;
 if(!dir){
  assert(ISO_BUSY()&1);uint8_t copy[512];memcpy(copy,outBuffer,length);
  if(interruptCopy){
   auto before=isoStarts[0];memset(hostOut,0xDD,sizeof(hostOut));frame(17);
   assert(isoStarts[0]==before && !memcmp(copy,outBuffer,length));
  }
 }else if(chainIn){chainIn=false;assert(productionEpInXfer(0,8,inBuffer,9));}
}
void init(){
 regs={};s_Usbd.Ctrlr={};memset(s_Usbd.EpReg,0,sizeof(s_Usbd.EpReg));
 s_Usbd.hQue=CFifoInit(queueMemory,sizeof(queueMemory),sizeof(nRFUsbdQue_t),true);
 assert(s_Usbd.hQue);
 // Both ISO directions open; every other flag (busy, complete, ready,
 // suspend group) cleared, exactly the former per-field init.
 s_Usbd.Flags=USBD_FLAG_ISO_OUT_OPEN|USBD_FLAG_ISO_IN_OPEN;
 dmaBusy=0;dmaLocks=dmaUnlocks=0;irqMask=0;isoStarts[0]=isoStarts[1]=regularStarts=0;
 callbacks[0]=callbacks[1]=0;chainIn=interruptCopy=false;
 ++s_Usbd.IsoGeneration[0];++s_Usbd.IsoGeneration[1];
 s_Usbd.EpReg[7][0]={outBuffer,callback,nullptr,512,false};
 s_Usbd.EpReg[7][1]={inBuffer,callback,nullptr,512,false};
 memset(inBuffer,0xA5,sizeof(inBuffer));memset(hostOut,0x5A,sizeof(hostOut));
 assert(AppEvtHandlerInit(nullptr,0));assert(AppEvtHandlerIdleRegister(nRFUsbdRetryIsoComplete));
}
void dummy(uint32_t,void*){}
int main(){
 init();unsigned ready=0;
 s_Usbd.EpReg[7][0].bBlocking=true;
 s_Usbd.EpReg[7][0].pContext=&ready;
 s_Usbd.EpReg[7][0].Handler=[](uint8_t ep,UsbCtrlrEvtType_t event,uint16_t length,
  UsbCtrlrXferResult_t result,void *context){
  assert(ep==8 && event==USB_CTRLR_EVT_DRDY && length==0);
  assert(result==USB_CTRLR_XFER_SUCCESS);++*(unsigned*)context;
 };
 frame(9);
 assert(ready==1 && !isoStarts[0] && !dmaBusy);
 puts("PASS: ISO DRDY dispatch preserves endpoint, event, success and registered context");

 alignas(8) uint8_t txMemory[CFIFO_TOTAL_MEMSIZE(128,1)];
 const uint16_t regularLengths[]={1,2,3,4,9,64};
 for(unsigned masked=0;masked<2;++masked)for(uint8_t ep=1;ep<8;++ep){
  init();irqMask=masked;
  assert(productionEpInXfer(0,ep,inBuffer,0));
  auto *entry=(nRFUsbdQue_t*)CFifoGet(s_Usbd.hQue);
  assert(entry && entry->EpNum==ep && entry->Dir==NRFX_USBD_QUE_IN_BUFFER);
  assert(entry->Len==0 && entry->pBuffer==inBuffer && irqMask==masked);
  for(unsigned offset=0;offset<4;++offset)for(uint16_t length:regularLengths){
   init();irqMask=masked;
   auto fifo=CFifoInit(txMemory,sizeof(txMemory),1,true);
   int count=100;auto *data=CFifoPutMultiple(fifo,&count);
   assert(data && count==100);
   for(int i=0;i<count;++i)data[i]=uint8_t(i);
   int skip=offset;if(skip)assert(CFifoGetMultiple(fifo,&skip)==data);
   data=CFifoPeek(fifo);assert((uintptr_t(data)&3U)==offset);
   const auto get=fifo->GetIdx,put=fifo->PutIdx;
   s_Usbd.EpReg[ep-1][1].pBuffer=(uint8_t*)fifo;
   assert(productionEpInXfer(0,ep,nullptr,length));
   assert(irqMask==masked && CFifoUsed(s_Usbd.hQue)==1);
   assert(fifo->GetIdx==get && fifo->PutIdx==put && CFifoPeek(fifo)==data);
   entry=(nRFUsbdQue_t*)CFifoGet(s_Usbd.hQue);
   assert(entry->EpNum==ep);
   if(offset){
    assert(entry->Dir==NRFX_USBD_QUE_IN_SCRATCH);
    assert(entry->Len==(length<4-offset?length:4-offset));
    assert(!memcmp(&entry->Scratch,data,entry->Len));
   }else{
    assert(entry->Dir==NRFX_USBD_QUE_IN_FIFO);
    assert(entry->Len==length && entry->hFifo==fifo);
   }
   assert(productionEpInXfer(0,ep,inBuffer,length));
   entry=(nRFUsbdQue_t*)CFifoGet(s_Usbd.hQue);
   assert(entry->EpNum==ep && entry->Dir==NRFX_USBD_QUE_IN_BUFFER);
   assert(entry->Len==length && entry->pBuffer==inBuffer && irqMask==masked);
  }
 }
 puts("PASS: regular IN preserves direct/FIFO/scratch sources, FIFO ownership and IRQ state");

 const uint16_t inLengths[]={0,9,17,33,512};
 for(uint16_t length : inLengths){
  init();s_Usbd.EpReg[7][1].pBuffer=nullptr;
  assert(productionEpInXfer(0,8,inBuffer,length));
  assert(CFifoUsed(s_Usbd.hQue)==0 && (ISO_BUSY()&2));
  assert(s_Usbd.EpReg[7][1].pBuffer==inBuffer && !dmaBusy);
  frame();assert(isoStarts[1]==1 && regs.ISOIN.MAXCNT==length);
  assert(regs.ISOIN.PTR==uint32_t(uintptr_t(inBuffer)));
  assert(!memcmp(wireIn,inBuffer,length));
  finish(true);AppEvtHandlerExec();
  assert(callbacks[1]==1 && lengths[1]==length && ISO_BUSY()==0);
 }
 puts("PASS: public IN entry point routes EP8 through ISO DMA without using the regular queue");

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
 s_Usbd.Flags|=USBD_FLAG_ISO_OUT_OPEN;s_Usbd.EpReg[7][0].MaxPacketSize=9;frame(9);finish(false);
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
   regs.SIZE.EPOUT[n]=64;s_Usbd.EpReg[n][0].MaxPacketSize=s_Usbd.EpReg[n][1].MaxPacketSize=64;
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
   for(unsigned d=0;d<2;++d)assert(s_Usbd.EpReg[n][d].MaxPacketSize==((n+1==ep&&d==dir)?0U:64U));
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
