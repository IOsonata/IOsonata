#!/usr/bin/env python3
"""Exercise production nRF52 ISO and DMA scheduling on simulated registers.

The controller's own functions are extracted from the driver sources and
compiled against a register model: START tasks capture EPSTATUS and copy the
buffer at the barrier that follows them, END events are raised by the test.
The state struct and enums come from the real header, so a layout change in
the driver shows up here as a compile error rather than a silent mismatch.

This is a host ownership and ordering test, not a model of USB timing.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
BASE_SOURCE = ROOT / 'ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp'
ISO_SOURCE = ROOT / 'ARM/Nordic/nRF52/src/usb_ctrlr_nrf52_iso.cpp'
HEADER_SOURCE = ROOT / 'ARM/Nordic/include/usb_ctrlr.h'
header = HEADER_SOURCE.read_text()
base = BASE_SOURCE.read_text()
iso = ISO_SOURCE.read_text()
# Strong ISO definitions come before the base's weak defaults.
src = iso + '\n' + base


def function(name, rename=None):
    """Definition of name, the first non-declaration match in src."""
    match = re.search(r'^[^\n;{}]*\b' + name + r'\s*\([^;{}]*\)\s*\{', src, re.M)
    assert match, name
    start = match.start()
    brace = src.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (src[end] == '{') - (src[end] == '}')
        end += 1
    text = src[start:end]
    text = re.sub(r'^\s*extern "C"\s*', '', text)
    if rename:
        text = text.replace(name + '(', rename + '(', 1)
    head = text[:text.index('{')].strip()
    return head + ';', text


def block(source, pattern, what):
    match = re.search(pattern, source, re.S)
    assert match, what + ' not found in driver source'
    return match.group(0)


# Types and enums, verbatim from the driver.
types = '\n'.join([
    block(header, r'enum\s*\{[^}]*NRF_USB_EP_COUNT[^}]*\};', 'endpoint count enum'),
    block(header, r'typedef struct __nRF_Usb_Ep_Registration\s*\{.*?\}\s*nRFUsbEpReg_t;', 'nRFUsbEpReg_t'),
    block(header, r'enum\s*\{[^}]*USBD_FLAG_SUSPENDED[^}]*\};', 'flag enum'),
    block(header, r'enum\s*\{[^}]*NRFUSBD_ISO_IN_READY[^}]*\};', 'ISO ready enum'),
    block(header, r'typedef struct __nRF_Usbd_State\s*\{.*?\}\s*nRFUsbdState_t;', 'nRFUsbdState_t'),
    block(base, r'#define NRFUSBD_QUE_DEPTH[^\n]*', 'queue depth'),
    block(base, r'#define NRFUSBD_EP0_QUE_DEPTH[^\n]*', 'EP0 queue depth'),
    block(base, r'enum\s*\{[^}]*NRFX_USBD_QUE_IN_SCRATCH[^}]*\};', 'queue enum'),
    '#pragma pack(push, 4)',
    block(base, r'typedef struct __nRF_Usbd_Que \{.*?\} nRFUsbdQue_t;', 'nRFUsbdQue_t'),
    block(base, r'typedef struct __nRF_Ep_Packet \{.*?\} nRFEPPkt_t;', 'nRFEPPkt_t'),
    '#pragma pack(pop)',
])

names = [
    ('nRFUsbGetEpReg', None), ('nRFUsbEpRegisteredEvent', None),
    ('nRFUsbdDmaActive', None), ('nRFUsbdDmaLock', None),
    ('nRFUsbdDmaUnlock', None), ('nRFUsbdDmaStartLocked', None),
    ('nRFUsbdDmaAllowed', None), ('nRFUsbdEpHwEnable', None),
    ('nRFUsbdGetCompletedXfer', None), ('nRFUsbdDmaWait', None),
    ('nRFUsbdEp0InStart', None), ('nRFUsbdStartDmaNow', None),
    ('nRFUsbdStartQueuedDma', None), ('nRFUsbdResumeQueuedDmaLocked', None),
    ('nRFUsbdHandleSof', None),
    ('UsbCtrlrIsoRxBuffer', None), ('nRFIsoHwEnable', None),
    ('nRFUsbdIsoStart', None), ('UsbCtrlrIsoSend', None),
    ('nRFUsbdIsoComplete', None), ('nRFUsbdIsoEpClose', None),
    ('UsbCtrlrIsoOpen', 'productionIsoOpen'),
    ('UsbCtrlrEpSend', 'productionEpSend'),
    ('UsbCtrlrEpOpenData', 'productionEpOpenData'),
    ('UsbCtrlrEpClose', 'productionEpClose'),
    ('UsbCtrlrEpClearStall', None),
]
extracted = [function(n, r) for n, r in names]
prototypes = '\n'.join(p for p, _ in extracted)
bodies = '\n\n'.join(b for _, b in extracted)
# The static buffer the ISO member owns.
iso_rx = block(iso, r'alignas\(4\) static uint8_t s_IsoRxBuffer\[[^\]]*\];', 's_IsoRxBuffer')

preamble = r'''
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <initializer_list>
#include "usb/usb.h"
#include "app_evt_handler.h"
#include "cfifo.h"
using namespace std;

// Register model ----------------------------------------------------------
constexpr uint32_t USBD_SIZE_ISOOUT_ZERO_Msk=1UL<<16;
constexpr uint32_t USBD_INTENCLR_SOF_Msk=1, USBD_INTEN_SOF_Msk=1;
constexpr uint32_t USBD_INTEN_ENDISOIN_Msk=1U<<11, USBD_INTEN_ENDISOOUT_Msk=1U<<20;
constexpr uint32_t USBD_INTEN_ENDEPIN0_Msk=1U<<2, USBD_INTEN_ENDEPOUT0_Msk=1U<<12;
constexpr unsigned USBD_INTEN_ENDEPIN0_Pos=2, USBD_INTEN_ENDEPOUT0_Pos=12;
constexpr uint32_t USBD_SHORTS_EP0DATADONE_EP0STATUS_Msk=1;
constexpr unsigned USBD_EPSTALL_STALL_UnStall=0, USBD_EPSTALL_STALL_Pos=8;
constexpr unsigned USBD_DTOGGLE_VALUE_Data0=1, USBD_DTOGGLE_VALUE_Pos=8;
constexpr unsigned USBD_ISOSPLIT_SPLIT_HalfIN=0x80, USBD_ISOSPLIT_SPLIT_Pos=0;
constexpr unsigned USBD_ISOINCONFIG_RESPONSE_ZeroData=1, USBD_ISOINCONFIG_RESPONSE_Pos=0;
constexpr uint32_t NRFX_USBD_EASYDMA_BUSY_REG_BUSY=0x82, NRFX_USBD_EASYDMA_BUSY_REG_CLEAR=0;
#define NRFUSBD_ISO_TRACE 0
#define ISO_TRACE_FLAG(f) ((void)0)
#define NRFX_USBD_REG32(a) (*(volatile uint32_t *)(a))
#define NRFX_USBD_ERRATA_166_REG_A 0
#define NRFX_USBD_ERRATA_166_REG_B 0

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
typedef struct {uint32_t PTR=0,MAXCNT=0,AMOUNT=0;} USBD_EPIN_Type;
typedef USBD_EPIN_Type USBD_EPOUT_Type;
typedef USBD_EPIN_Type USBD_ISOIN_Type;
typedef USBD_EPIN_Type USBD_ISOOUT_Type;
struct Registers {
 uint32_t EVENTS_USBRESET=0,EVENTS_ENDEPIN[8]={},EVENTS_EP0DATADONE=0,EVENTS_ENDISOIN=0;
 uint32_t EVENTS_ENDEPOUT[8]={},EVENTS_ENDISOOUT=0,EVENTS_SOF=0,EVENTS_USBEVENT=0;
 uint32_t TASKS_STARTEPIN[8]={},TASKS_STARTISOIN=0,TASKS_STARTEPOUT[8]={},TASKS_STARTISOOUT=0;
 USBD_EPIN_Type EPIN[8];USBD_ISOIN_Type ISOIN;USBD_EPOUT_Type EPOUT[8];USBD_ISOOUT_Type ISOOUT;
 W1C EPSTATUS,EPDATASTATUS;
 uint32_t EPINEN=0,EPOUTEN=0,INTEN=0,INTENSET=0,INTENCLR=0,FRAMECNTR=0;
 uint32_t EPSTALL=0,DTOGGLE=0,SHORTS=0,BMREQUESTTYPE=0,ISOSPLIT=0,ISOINCONFIG=0;
 struct {uint32_t EPOUT[8]={},ISOOUT=0;} SIZE;
} regs;
using NRF_USBD_Type = Registers;
auto *NRF_USBD=&regs;

// Memory the simulated EasyDMA copies from or into.
uint8_t hostOut[512];
uint8_t wireIn[512];
unsigned isoStarts[2]={},ep0InStarts=0,regularStarts=0;
int activeBit=-1;
// The barrier after a START task is where the hardware takes the transfer:
// it captures EPSTATUS and moves the data. One DMA at a time.
static void capture(int bit){
 assert(dmaBusy==0x82 && "START with the channel unlocked");
 assert(regs.EPSTATUS.bits==0 && "START while a DMA is still captured");
 regs.EPSTATUS.bits=1UL<<bit;activeBit=bit;
}
// The driver writes PTR as a 32-bit address, right for the target and
// truncated on a 64-bit host. Map it back to the buffer it came from.
uint8_t *dmaBuffers[8];unsigned dmaBufferCnt=0;
void dmaBuffer(uint8_t *p){dmaBuffers[dmaBufferCnt++]=p;}
uint8_t *resolve(uint32_t ptr){
 for(unsigned i=0;i<dmaBufferCnt;++i)
  if(uint32_t(uintptr_t(dmaBuffers[i]))==ptr)return dmaBuffers[i];
 assert(!"DMA pointer not in a known buffer");return nullptr;
}
void __DSB(){
 if(regs.TASKS_STARTISOIN){regs.TASKS_STARTISOIN=0;capture(8);++isoStarts[1];
  memcpy(wireIn,resolve(regs.ISOIN.PTR),regs.ISOIN.MAXCNT);}
 if(regs.TASKS_STARTISOOUT){regs.TASKS_STARTISOOUT=0;capture(24);++isoStarts[0];
  memcpy(resolve(regs.ISOOUT.PTR),hostOut,regs.ISOOUT.MAXCNT);}
 for(unsigned n=0;n<8;++n){
  if(regs.TASKS_STARTEPIN[n]){regs.TASKS_STARTEPIN[n]=0;capture(n);
   if(n==0)++ep0InStarts;else ++regularStarts;}
  if(regs.TASKS_STARTEPOUT[n]){regs.TASKS_STARTEPOUT[n]=0;capture(16+n);
   if(n)++regularStarts;}
 }
}
void __ISB(){}
static inline uint32_t __CLZ(uint32_t v){return v?__builtin_clz(v):32;}
unsigned irqMask=0;
uint32_t DisableInterrupt(){auto old=irqMask;irqMask=1;return old;}
void EnableInterrupt(uint32_t old){irqMask=old;}
uint32_t __get_PRIMASK(){return irqMask;}
void __disable_irq(){irqMask=1;}
void __set_PRIMASK(uint32_t v){irqMask=v;}
void nRFUsbdHostResume(){}

@@DRIVER_TYPES@@
nRFUsbdState_t s_Usbd;
alignas(4) uint8_t s_QueMem[CFIFO_TOTAL_MEMSIZE(NRFUSBD_QUE_DEPTH, sizeof(nRFUsbdQue_t))];
alignas(4) uint8_t s_Ep0QueMem[CFIFO_TOTAL_MEMSIZE(NRFUSBD_EP0_QUE_DEPTH, sizeof(nRFEPPkt_t))];
@@ISO_RX@@

// Core stand-in: at SOF offer whatever IN frame the test staged.
uint8_t *pendingIn=nullptr;
uint16_t pendingInLen=0;
bool UsbCtrlrIsoSend(int,uint8_t,uint8_t*,uint16_t);
void UsbDevProcessEvent(int,const UsbCtrlrEvt_t *evt){
 if(evt->Type==USB_CTRLR_EVT_SOF)
  (void)UsbCtrlrIsoSend(0,8,pendingIn,pendingIn?pendingInLen:0);
}
void nRFUsbdEmitXfer(uint8_t,uint16_t){}

@@PROTOTYPES@@
'''

tests = r'''
// Harness -----------------------------------------------------------------
unsigned callbacks[2]={};uint16_t lengths[2]={};
uint8_t lastOut[512];
void isoCallback(UsbCtrlrEvtType_t event,uint16_t length,void *context){
 assert(event==USB_CTRLR_EVT_XFER_CMPL);
 unsigned dir=(unsigned)(uintptr_t)context;++callbacks[dir];lengths[dir]=length;
 if(!dir)memcpy(lastOut,UsbCtrlrIsoRxBuffer(0),length);
}
uint8_t inBuffer[512],inBuffer2[512];
void init(){
 if(!dmaBufferCnt){dmaBuffer(UsbCtrlrIsoRxBuffer(0));dmaBuffer(inBuffer);dmaBuffer(inBuffer2);}
 regs={};memset(&s_Usbd,0,sizeof(s_Usbd));
 s_Usbd.hQue=CFifoInit(s_QueMem,sizeof(s_QueMem),sizeof(nRFUsbdQue_t),true);
 s_Usbd.hEp0Que=CFifoInit(s_Ep0QueMem,sizeof(s_Ep0QueMem),sizeof(nRFEPPkt_t),true);
 s_Usbd.SofEnabled=true;
 dmaBusy=0;dmaLocks=dmaUnlocks=0;irqMask=0;activeBit=-1;
 isoStarts[0]=isoStarts[1]=ep0InStarts=regularStarts=0;
 callbacks[0]=callbacks[1]=0;lengths[0]=lengths[1]=0;
 pendingIn=nullptr;pendingInLen=0;
 s_Usbd.EpReg[7][0]={nullptr,isoCallback,(void*)0,512,false};
 s_Usbd.EpReg[7][1]={nullptr,isoCallback,(void*)1,512,false};
 assert(productionIsoOpen(0,8,true,512) && productionIsoOpen(0,8,false,512));
 assert(s_Usbd.IsoOpen);
 memset(inBuffer,0xA5,sizeof(inBuffer));memset(inBuffer2,0x3C,sizeof(inBuffer2));
 for(unsigned i=0;i<sizeof(hostOut);++i)hostOut[i]=uint8_t(i*7+1);
 assert(AppEvtHandlerInit(nullptr,0));
}
// Host sends OUT (length, or nothing when negative) and the SOF follows.
void frame(int outLength=-1){
 ++regs.FRAMECNTR;
 regs.SIZE.ISOOUT=outLength<0?0:(outLength==0?USBD_SIZE_ISOOUT_ZERO_Msk:uint32_t(outLength));
 nRFUsbdHandleSof();
}
// END event for the running DMA, then what the interrupt does with it:
// retire, report, hand the channel on.
void finish(){
 assert(dmaBusy && activeBit>=0);
 const int bit=activeBit;
 if(bit==8){regs.ISOIN.AMOUNT=regs.ISOIN.MAXCNT;regs.EVENTS_ENDISOIN=1;}
 else if(bit==24){regs.ISOOUT.AMOUNT=regs.ISOOUT.MAXCNT;regs.EVENTS_ENDISOOUT=1;}
 else if(bit==0){regs.EVENTS_ENDEPIN[0]=1;regs.EVENTS_EP0DATADONE=1;}
 else if(bit<16){regs.EPIN[bit].AMOUNT=regs.EPIN[bit].MAXCNT;regs.EVENTS_ENDEPIN[bit]=1;}
 else{regs.EPOUT[bit-16].AMOUNT=regs.EPOUT[bit-16].MAXCNT;regs.EVENTS_ENDEPOUT[bit-16]=1;}
 const int done=nRFUsbdGetCompletedXfer();
 assert(done==bit && regs.EPSTATUS.bits==0);
 activeBit=-1;
 if(done==8||done==24)nRFUsbdIsoComplete(done==8);
 else if(done>0&&done<8)(void)CFifoGet(s_Usbd.hQue);
 nRFUsbdStartQueuedDma();
}

int main(){
 // OUT: the SOF offer starts a DMA into the controller's own buffer, sized
 // by the received packet; its END completes it with the byte count.
 init();frame(17);
 assert(dmaBusy && activeBit==24 && isoStarts[0]==1);
 assert(regs.ISOOUT.PTR==uint32_t(uintptr_t(UsbCtrlrIsoRxBuffer(0))) && regs.ISOOUT.MAXCNT==17);
 finish();
 assert(callbacks[0]==1 && lengths[0]==17 && !memcmp(lastOut,hostOut,17));
 assert(!dmaBusy && s_Usbd.IsoDataFlag==0);
 puts("PASS: ISO OUT lands in the controller buffer and completes at ENDISOOUT");

 // No OUT packet this frame: nothing starts, the request is dropped, the
 // channel is released.
 init();frame(-1);
 assert(!dmaBusy && isoStarts[0]==0 && s_Usbd.IsoDataFlag==0 && dmaLocks==dmaUnlocks);
 // A packet larger than the endpoint is not read.
 init();s_Usbd.EpReg[7][0].MaxPacketSize=9;frame(17);
 assert(!dmaBusy && isoStarts[0]==0 && s_Usbd.IsoDataFlag==0);
 // Zero-length packet (SIZE.ISOOUT ZERO set): no data, no DMA.
 init();frame(0);
 assert(!dmaBusy && isoStarts[0]==0 && s_Usbd.IsoDataFlag==0 && dmaLocks==dmaUnlocks);
 puts("PASS: an empty, zero-length or oversize OUT frame starts no DMA and holds no channel");

 // IN: an offered frame is moved at SOF, IN before OUT, one DMA at a time.
 for(uint16_t length:{0,9,63,512}){
  init();pendingIn=inBuffer;pendingInLen=length;frame(33);
  assert(activeBit==8 && isoStarts[1]==1 && isoStarts[0]==0);
  assert(regs.ISOIN.PTR==uint32_t(uintptr_t(inBuffer)) && regs.ISOIN.MAXCNT==length);
  assert(!memcmp(wireIn,inBuffer,length));
  finish();
  assert(callbacks[1]==1 && lengths[1]==length);
  assert(activeBit==24 && isoStarts[0]==1);
  finish();
  assert(callbacks[0]==1 && lengths[0]==33 && !dmaBusy && s_Usbd.IsoDataFlag==0);
 }
 puts("PASS: ISO IN is staged first, OUT follows on the same channel");

 // A new offer does not replace an IN frame that has not started yet.
 init();dmaBusy=0x82;regs.EPSTATUS.bits=1U<<3;activeBit=3;
 regs.EPIN[3].MAXCNT=8;
 auto *q=(nRFUsbdQue_t*)CFifoPut(s_Usbd.hQue);*q={3,NRFX_USBD_QUE_IN_BUFFER,8,{inBuffer}};
 pendingIn=inBuffer;pendingInLen=9;frame(-1);
 pendingIn=inBuffer2;pendingInLen=5;frame(-1);
 assert(s_Usbd.EpReg[7][1].pBuffer==inBuffer && s_Usbd.IsoInDmaLen==9);
 finish();
 assert(activeBit==8 && regs.ISOIN.PTR==uint32_t(uintptr_t(inBuffer)) && regs.ISOIN.MAXCNT==9);
 finish();
 puts("PASS: a pending IN frame is not overwritten by the next offer");

 // Scheduler order when a regular DMA holds the channel at SOF and EP0 IN
 // and a regular transfer are also waiting: ISO IN, ISO OUT, EP0, regular.
 init();dmaBusy=0x82;regs.EPSTATUS.bits=1U<<2;activeBit=2;regs.EPIN[2].MAXCNT=8;
 q=(nRFUsbdQue_t*)CFifoPut(s_Usbd.hQue);*q={2,NRFX_USBD_QUE_IN_BUFFER,8,{inBuffer}};
 q=(nRFUsbdQue_t*)CFifoPut(s_Usbd.hQue);*q={5,NRFX_USBD_QUE_IN_BUFFER,4,{inBuffer}};
 auto *pkt=(nRFEPPkt_t*)CFifoPut(s_Usbd.hEp0Que);pkt->Len=18;
 pendingIn=inBuffer;pendingInLen=9;frame(17);
 assert(activeBit==2 && isoStarts[0]==0 && isoStarts[1]==0);
 finish();assert(activeBit==8);
 finish();assert(activeBit==24);
 finish();assert(activeBit==0 && ep0InStarts==1);
 finish();assert(activeBit==5);
 finish();assert(!dmaBusy);
 assert(callbacks[0]==1 && callbacks[1]==1);
 puts("PASS: after a regular DMA the channel goes to ISO IN, ISO OUT, EP0, then regular");

 // Retirement needs the matching END event; EPSTATUS alone is not enough.
 init();pendingIn=inBuffer;pendingInLen=9;frame(-1);
 assert(activeBit==8);
 assert(nRFUsbdGetCompletedXfer()==-1 && regs.EPSTATUS.bits==(1U<<8) && dmaBusy);
 finish();assert(callbacks[1]==1);
 // EP0 IN retires only once the host took the packet as well.
 init();dmaBusy=0x82;pkt=(nRFEPPkt_t*)CFifoPut(s_Usbd.hEp0Que);pkt->Len=8;
 nRFUsbdEp0InStart(pkt);assert(activeBit==0);
 regs.EVENTS_ENDEPIN[0]=1;
 assert(nRFUsbdGetCompletedXfer()==-1 && dmaBusy);
 regs.EVENTS_EP0DATADONE=1;
 assert(nRFUsbdGetCompletedXfer()==0 && regs.EPSTATUS.bits==0);
 assert(CFifoUsed(s_Usbd.hEp0Que)==0);
 puts("PASS: a DMA retires only at its END event, EP0 IN also at EP0DATADONE");

 // Suspended: offers are recorded, nothing starts until the flag clears.
 init();s_Usbd.Flags|=USBD_FLAG_SUSPENDED;
 pendingIn=inBuffer;pendingInLen=17;frame(9);
 assert(!dmaBusy && (s_Usbd.IsoDataFlag&NRFUSBD_ISO_IN_READY));
 s_Usbd.Flags&=(uint8_t)~USBD_FLAG_SUSPENDED;
 nRFUsbdResumeQueuedDmaLocked();assert(activeBit==8);
 finish();finish();assert(callbacks[1]==1 && callbacks[0]==1);
 puts("PASS: a suspended bus holds ISO work until resume");

 // Close with a DMA in flight: the transfer is retired without a callback,
 // the flags drop, the channel is free, and a reopened pair works.
 init();frame(17);assert(activeBit==24);
 regs.ISOOUT.AMOUNT=17;regs.EVENTS_ENDISOOUT=1;
 nRFUsbdIsoEpClose(false);
 assert(callbacks[0]==0 && !dmaBusy && s_Usbd.IsoDataFlag==0 && !s_Usbd.IsoOpen);
 assert(productionIsoOpen(0,8,false,9) && s_Usbd.IsoOpen);
 activeBit=-1;frame(9);finish();
 assert(callbacks[0]==1 && lengths[0]==9);
 puts("PASS: close retires an active ISO DMA silently; reopen starts clean");

 // Open refuses a packet size the hardware cannot hold.
 init();assert(!productionIsoOpen(0,8,true,513) && productionIsoOpen(0,8,true,512));
 assert(regs.ISOSPLIT==USBD_ISOSPLIT_SPLIT_HalfIN && regs.ISOINCONFIG==USBD_ISOINCONFIG_RESPONSE_ZeroData);
 puts("PASS: ISO open is bounded by the 512-byte half buffer");

 // Regular IN keeps word-aligned sources and repairs a misaligned head
 // through the scratch word, under interrupt exclusion.
 alignas(8) uint8_t txMemory[CFIFO_TOTAL_MEMSIZE(128,1)];
 const uint16_t regularLengths[]={1,2,3,4,9,64};
 for(unsigned masked=0;masked<2;++masked)for(uint8_t ep=1;ep<8;++ep)
 for(unsigned offset=0;offset<4;++offset)for(uint16_t length:regularLengths){
  init();irqMask=masked;s_Usbd.Flags|=USBD_FLAG_SUSPENDED;  // hold the queue
  auto fifo=CFifoInit(txMemory,sizeof(txMemory),1,true);
  int count=100;auto *data=CFifoPutMultiple(fifo,&count);
  assert(data && count==100);
  for(int i=0;i<count;++i)data[i]=uint8_t(i);
  int skip=offset;if(skip)assert(CFifoGetMultiple(fifo,&skip)==data);
  data=CFifoPeek(fifo);assert((uintptr_t(data)&3U)==offset);
  assert(productionEpSend(0,ep,data,length));
  assert(irqMask==masked && CFifoUsed(s_Usbd.hQue)==1 && !dmaBusy);
  auto *entry=(nRFUsbdQue_t*)CFifoGet(s_Usbd.hQue);
  assert(entry->EpNum==ep);
  if(offset){
   assert(entry->Dir==NRFX_USBD_QUE_IN_SCRATCH);
   assert(entry->Len==(length<4-offset?length:4-offset));
   assert(!memcmp(&entry->Scratch,data,entry->Len));
  }else{
   assert(entry->Dir==NRFX_USBD_QUE_IN_BUFFER);
   assert(entry->Len==length && entry->pBuffer==data);
  }
 }
 puts("PASS: regular IN keeps aligned sources, repairs misaligned heads, preserves IRQ state");

 // Regular open clears halt and data toggle, arms OUT, leaves the other
 // direction alone.
 for(unsigned ep=1;ep<8;++ep)for(unsigned dir=0;dir<2;++dir)
 for(unsigned mps:{1U,8U,64U}){
  init();const uint8_t address=ep|(dir?0x80:0);
  regs.EPINEN=regs.EPOUTEN=1;
  regs.EVENTS_ENDEPIN[ep]=regs.EVENTS_ENDEPOUT[ep]=1;
  regs.SIZE.EPOUT[ep]=64;regs.EPSTALL=address|0x100;
  assert(productionEpOpenData(0,ep,dir,USB_ENDPATT_TRANS_BULK,mps));
  assert(s_Usbd.EpReg[ep-1][dir].MaxPacketSize==mps);
  assert(regs.EPINEN==(1U|(dir?(1U<<ep):0U)));
  assert(regs.EPOUTEN==(1U|(!dir?(1U<<ep):0U)));
  assert(regs.EVENTS_ENDEPIN[ep]==unsigned(!dir));
  assert(regs.EVENTS_ENDEPOUT[ep]==dir);
  assert(regs.EPSTALL==address && regs.DTOGGLE==(address|0x100U));
  assert(regs.SIZE.EPOUT[ep]==(dir?64U:0U));
 }
 puts("PASS: regular open clears halt/data toggle, arms OUT and preserves the other direction");

 // Regular close affects only the selected endpoint and direction.
 for(unsigned ep=1;ep<8;++ep)for(unsigned dir=0;dir<2;++dir)for(unsigned masked=0;masked<2;++masked){
  init();irqMask=masked;regs.EPINEN=regs.EPOUTEN=0x1FF;
  regs.EPDATASTATUS.bits=0x00FF00FF;
  productionEpClose(0,ep,dir);
  assert(irqMask==masked && s_Usbd.IsoOpen);
  assert(regs.EPINEN==(dir?(0x1FFU&~(1U<<ep)):0x1FFU));
  assert(regs.EPOUTEN==(!dir?(0x1FFU&~(1U<<ep)):0x1FFU));
  assert(regs.INTENCLR==(1U<<((dir?USBD_INTEN_ENDEPIN0_Pos:USBD_INTEN_ENDEPOUT0_Pos)+ep)));
  assert(regs.EPDATASTATUS.bits==(0x00FF00FFU&~(1U<<(ep+(dir?0:16)))));
 }
 puts("PASS: regular close affects only the selected endpoint/direction and preserves IRQ state");
}
'''

code = (preamble.replace('@@DRIVER_TYPES@@', types).replace('@@ISO_RX@@', iso_rx)
        .replace('@@PROTOTYPES@@', prototypes) + '\n' + bodies + '\n' + tests)

with tempfile.TemporaryDirectory(prefix='iosonata-iso-') as temp:
    path = Path(temp) / 'iso_test.cpp'
    path.write_text(code)
    compiler = os.environ.get('CXX', 'g++')
    subprocess.run([compiler, '-std=c++17', '-O1', '-Wall', '-Wextra',
                    '-Wno-unused-function', '-Wno-missing-field-initializers',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'tests/usb/hostport'),
                    '-x', 'c++', str(path), str(ROOT / 'src/app_evt_handler.cpp'),
                    str(ROOT / 'src/cfifo.c'), '-o', str(Path(temp) / 'iso_test')], check=True)
    subprocess.run([str(Path(temp) / 'iso_test')], check=True)
