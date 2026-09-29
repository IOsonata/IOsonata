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
import argparse
import os
import re
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--regular', action='store_true',
                    help='run deferred regular endpoint tests, including IN ordering')
parser.add_argument('--regular-out', action='store_true',
                    help='run deferred OUT DRDY/completion and DMA handoff tests')
args = parser.parse_args()

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
    ('nRFUsbdDmaWait', None),
    ('nRFUsbdEp0InStart', None), ('nRFUsbdStartDmaNow', None),
    ('nRFUsbdStartQueuedDma', None), ('nRFUsbdResumeQueuedDmaLocked', None),
    ('nRFUsbdHandleSof', None), ('USBD_IRQHandler', None),
    ('nRFUsbdAcquireDma', None), ('nRFUsbdProcessQueuedEvent', None),
    ('nRFUsbdInvalidateEvents', None),
    ('nRFUsbdResetState', None), ('UsbCtrlrProcess', None),
    ('nRFUsbdEpDisable', None),
    ('nRFIsoHwEnable', None),
    ('nRFUsbdIsoStart', None), ('UsbCtrlrIsoSend', None),
    ('nRFUsbdIsoComplete', None), ('nRFUsbdIsoEpClose', None),
    ('UsbCtrlrIsoOpen', 'productionIsoOpen'),
    ('UsbCtrlrEpSend', 'productionEpSend'),
    ('UsbCtrlrEpReceive', 'productionEpReceive'),
    ('UsbCtrlrEpOpenData', 'productionEpOpenData'),
    ('UsbCtrlrEpClose', 'productionEpClose'),
    ('UsbCtrlrEpClearStall', None),
]
extracted = [function(n, r) for n, r in names]
prototypes = '\n'.join(p for p, _ in extracted)
bodies = '\n\n'.join(b for _, b in extracted)

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
 W1C EPSTATUS,EPDATASTATUS,EVENTCAUSE;
 uint32_t EVENTS_EP0SETUP=0,EVENTS_EPDATA=0,TASKS_EP0RCVOUT=0;
 uint32_t EPINEN=0,EPOUTEN=0,INTEN=0,INTENSET=0,INTENCLR=0,FRAMECNTR=0;
 uint32_t EPSTALL=0,DTOGGLE=0,SHORTS=0,BMREQUESTTYPE=0,ISOSPLIT=0,ISOINCONFIG=0;
 struct {uint32_t EPOUT[8]={},ISOOUT=0;} SIZE;
} regs;
using NRF_USBD_Type = Registers;
auto *NRF_USBD=&regs;

// Memory the simulated EasyDMA copies from or into.
uint8_t hostOut[512];
uint8_t wireIn[512];
unsigned isoStarts[2]={},ep0InStarts=0,ep0OutStarts=0,regularStarts=0;
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
uint8_t *dmaBuffers[20];unsigned dmaBufferCnt=0;
bool checkInBuffer=false,inUsbBusy[8]={};
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
   if(n==0)++ep0InStarts;else{
    if(checkInBuffer)assert(!inUsbBusy[n] && "IN endpoint still contains the preceding packet");
    inUsbBusy[n]=true;++regularStarts;
    if(regs.EPIN[n].MAXCNT)memcpy(wireIn,resolve(regs.EPIN[n].PTR),regs.EPIN[n].MAXCNT);
   }}
  if(regs.TASKS_STARTEPOUT[n]){regs.TASKS_STARTEPOUT[n]=0;capture(16+n);
   if(n){++regularStarts;memcpy(resolve(regs.EPOUT[n].PTR),hostOut,regs.EPOUT[n].MAXCNT);}
   else ++ep0OutStarts;}
 }
}
void __ISB(){}
static inline uint32_t __CLZ(uint32_t v){return v?__builtin_clz(v):32;}
static inline uint32_t __ROR(uint32_t v,uint32_t n){n&=31U;return n?(v>>n)|(v<<(32U-n)):v;}
unsigned irqMask=0;
bool inIsr=false;
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
// The OUT endpoint owner's buffer (a reserved RX FIFO block in UsbIntrf).
alignas(4) uint8_t outSlot[512];

// Core stand-in: at SOF offer whatever IN frame the test staged.
uint8_t *pendingIn=nullptr;
uint16_t pendingInLen=0;
bool UsbCtrlrIsoSend(int,uint8_t,uint8_t*,uint16_t);
void UsbDevProcessEvent(int,const UsbCtrlrEvt_t *evt){
 if(evt->Type==USB_CTRLR_EVT_SOF)
  (void)UsbCtrlrIsoSend(0,8,pendingIn,pendingIn?pendingInLen:0);
}
unsigned ep0Callbacks[2]={},busEventCalls=0,tailVisits=0;
uint16_t ep0Lengths[2]={};
void nRFUsbdEmitXfer(uint8_t address,uint16_t length){
 assert(!regs.EPSTATUS.bits && dmaBusy);
 const unsigned dir=(address&USB_ENDPADDR_DIR_IN)!=0;
 ++ep0Callbacks[dir];ep0Lengths[dir]=length;
}
// Unchanged reset/setup/power hooks are outside this completion test.
void nRFUsbdBusReset(){assert(!"unexpected bus reset in DMA test");}
void nRFUsbdProcessEP0Setup(){assert(!"unexpected setup in DMA test");}
void nRFUsbdEmitSimple(UsbCtrlrEvtType_t){assert(!"unexpected bus event");}
void nRFUsbdIsoSofMark(){}
bool resumeOnBusEvent=false;
void nRFUsbdHandleBusEvent(uint32_t){
 ++busEventCalls;
 if(resumeOnBusEvent)s_Usbd.Flags&=(uint8_t)~USBD_FLAG_SUSPENDED;
}
void nRFUsbdTryRemoteWake(){}
void nRFUsbdTryEnterLowPower(){++tailVisits;}

@@PROTOTYPES@@
'''

tests = r'''
// Harness -----------------------------------------------------------------
unsigned callbacks[2]={};uint16_t lengths[2]={};
unsigned regularCompletions=0;
uint16_t regularLength=0;
uint8_t lastOut[512];
// DRDY: the owner has no destination submitted; drdyGives says whether it
// submits one when asked.
unsigned drdyCnt=0;bool drdyGives=false;
void isoCallback(UsbCtrlrEvtType_t event,uint16_t length,void *context){
 unsigned dir=(unsigned)(uintptr_t)context;
 if(event==USB_CTRLR_EVT_DRDY){
  assert(dir==0 && s_Usbd.pIsoBuffer[0]==nullptr);
  ++drdyCnt;if(drdyGives)assert(productionEpReceive(0,8,outSlot,512));return;}
 assert(event==USB_CTRLR_EVT_XFER_CMPL);
 ++callbacks[dir];lengths[dir]=length;
 if(!dir)memcpy(lastOut,outSlot,length);
}
uint8_t inBuffer[512],inBuffer2[512];
void init(){
 if(!dmaBufferCnt){dmaBuffer(outSlot);dmaBuffer(inBuffer);dmaBuffer(inBuffer2);}
 regs={};memset(&s_Usbd,0,sizeof(s_Usbd));
 regs.BMREQUESTTYPE=USB_REQTYPE_MASK_DIR;
 ep0Callbacks[0]=ep0Callbacks[1]=busEventCalls=tailVisits=0;
 resumeOnBusEvent=false;
 ep0Lengths[0]=ep0Lengths[1]=0;regularCompletions=0;regularLength=0;
 s_Usbd.hQue=CFifoInit(s_QueMem,sizeof(s_QueMem),sizeof(nRFUsbdQue_t),true);
 s_Usbd.hEp0Que=CFifoInit(s_Ep0QueMem,sizeof(s_Ep0QueMem),sizeof(nRFEPPkt_t),true);
 s_Usbd.SofEnabled=true;
 dmaBusy=0;dmaLocks=dmaUnlocks=0;irqMask=0;activeBit=-1;
 checkInBuffer=false;memset(inUsbBusy,0,sizeof(inUsbBusy));
 isoStarts[0]=isoStarts[1]=ep0InStarts=ep0OutStarts=regularStarts=0;
 callbacks[0]=callbacks[1]=0;lengths[0]=lengths[1]=0;
 pendingIn=nullptr;pendingInLen=0;
 drdyCnt=0;drdyGives=true;
 s_Usbd.EpReg[7][0]={isoCallback,(void*)0};
 s_Usbd.EpReg[7][1]={isoCallback,(void*)1};
 assert(productionIsoOpen(0,8,true,512) && productionIsoOpen(0,8,false,512));
 assert(s_Usbd.IsoOpen);
 memset(inBuffer,0xA5,sizeof(inBuffer));memset(inBuffer2,0x3C,sizeof(inBuffer2));
 for(unsigned i=0;i<sizeof(hostOut);++i)hostOut[i]=uint8_t(i*7+1);
 alignas(8) static uint8_t appEvents[APPEVT_HANDLER_QUE_MEMSIZE(16)];
 assert(AppEvtHandlerInit(appEvents,sizeof(appEvents)));
}
void interrupt(){
 const unsigned savedMask=irqMask;
 assert(!inIsr);inIsr=true;
 USBD_IRQHandler();
 inIsr=false;
 assert(irqMask==savedMask);
 if(!regs.EPSTATUS.bits)activeBit=-1;
}
// Host sends OUT (length, or nothing when negative) and the SOF follows.
void frame(int outLength=-1){
 ++regs.FRAMECNTR;
 regs.SIZE.ISOOUT=outLength<0?0:(outLength==0?USBD_SIZE_ISOOUT_ZERO_Msk:uint32_t(outLength));
 regs.EVENTS_SOF=1;interrupt();
}
// END event for the running DMA, then run the production interrupt handler.
void finish(){
 assert(dmaBusy && activeBit>=0);
 const int bit=activeBit;
 if(bit==8){regs.ISOIN.AMOUNT=regs.ISOIN.MAXCNT;regs.EVENTS_ENDISOIN=1;}
 else if(bit==24){regs.ISOOUT.AMOUNT=regs.ISOOUT.MAXCNT;regs.EVENTS_ENDISOOUT=1;}
 else if(bit==0){regs.EVENTS_ENDEPIN[0]=1;regs.EVENTS_EP0DATADONE=1;}
 else if(bit<16){regs.EPIN[bit].AMOUNT=regs.EPIN[bit].MAXCNT;regs.EVENTS_ENDEPIN[bit]=1;}
 else{regs.EPOUT[bit-16].AMOUNT=regs.EPOUT[bit-16].MAXCNT;regs.EVENTS_ENDEPOUT[bit-16]=1;}
 interrupt();
}

void regularCallback(UsbCtrlrEvtType_t event,uint16_t length,void *context){
 assert(context==outSlot && event==USB_CTRLR_EVT_XFER_CMPL);
 ++regularCompletions;regularLength=length;
}
// Exercise the actual CFifo reservation/publication contract used by UsbIntrf.
struct RxPacket {uint32_t length;uint8_t data[64];};
struct RxEndpoint {
 unsigned ep,completions,drdy;
 alignas(8) uint8_t memory[CFIFO_TOTAL_MEMSIZE(2,sizeof(RxPacket))];
 hCFifo_t fifo;
} receivers[7];
void (*duringCompletion)()=nullptr;
void deferredCallback(UsbCtrlrEvtType_t event,uint16_t length,void *context){
 auto &rx=*static_cast<RxEndpoint*>(context);
 auto *packet=reinterpret_cast<RxPacket*>(CFifoResv(rx.fifo));
 if(event==USB_CTRLR_EVT_DRDY){
  assert(!inIsr && !irqMask);
  ++rx.drdy;
  if(packet)assert(productionEpReceive(0,rx.ep,packet->data,sizeof(packet->data)));
  return;
 }
 assert(event==USB_CTRLR_EVT_XFER_CMPL && !inIsr && !irqMask && packet);
 assert(CFifoUsed(rx.fifo)==0 && "RX slot was published before AppEvt");
 packet->length=length;
 assert(CFifoPut(rx.fifo)==reinterpret_cast<uint8_t*>(packet));
 ++rx.completions;
 if(duringCompletion){auto hook=duringCompletion;duringCompletion=nullptr;hook();}
}
void deferredInit(){
 init();dmaBufferCnt=3;duringCompletion=nullptr;
 regs.EPINEN=regs.EPOUTEN=0xFEU;
 for(unsigned i=0;i<7;++i){
  auto &rx=receivers[i];rx.ep=i+1;rx.completions=rx.drdy=0;
  rx.fifo=CFifoInit(rx.memory,sizeof(rx.memory),sizeof(RxPacket),true);
  auto *first=reinterpret_cast<RxPacket*>(CFifoPut(rx.fifo));dmaBuffer(first->data);
  auto *second=reinterpret_cast<RxPacket*>(CFifoPut(rx.fifo));dmaBuffer(second->data);
  CFifoFlush(rx.fifo);
  s_Usbd.EpReg[i][0]={deferredCallback,&rx};
 }
}
void receive(unsigned ep,uint16_t length){
 regs.SIZE.EPOUT[ep]=length;regs.EPDATASTATUS.bits|=1U<<(ep+16);
 interrupt();
 AppEvtHandlerDispatch();
 assert(activeBit==int(ep+16));
}
void checkPacket(unsigned ep,uint16_t length){
 auto &rx=receivers[ep-1];
 auto *packet=reinterpret_cast<RxPacket*>(CFifoGet(rx.fifo));
 assert(packet && packet->length==length && !memcmp(packet->data,hostOut,length));
}
void ackIn(unsigned ep);
unsigned dummyCalls=0;
void dummyEvent(uint32_t,void*){++dummyCalls;}
void closeEp1(){productionEpClose(0,1,false);}
void completeEp2(){receive(2,13);finish();assert(!receivers[1].completions);}
void queueAnotherOut(){
 regs.SIZE.EPOUT[1]=23;regs.EPDATASTATUS.bits|=1U<<17;interrupt();
 assert(!dmaBusy && receivers[0].drdy==1);
}
void testDeferredOut(){
 // The ISR queues DRDY without calling the owner; the callback supplies
 // the DMA destination. Completion captures AMOUNT before the next DMA.
 for(unsigned ep=1;ep<8;++ep)for(uint16_t length:{0U,1U,17U,64U}){
  deferredInit();regs.SIZE.EPOUT[ep]=length;regs.EPDATASTATUS.bits=1U<<(ep+16);
  interrupt();auto &rx=receivers[ep-1];
  assert(!dmaBusy && !CFifoUsed(s_Usbd.hQue) && !rx.drdy && !regs.EPDATASTATUS.bits);
  for(unsigned i=0;i<3;++i)interrupt();
  AppEvtHandlerDispatch();assert(rx.drdy==1 && activeBit==int(ep+16));
  finish();assert(!dmaBusy && !CFifoUsed(s_Usbd.hQue) && !rx.completions);
  assert(!CFifoUsed(rx.fifo));regs.EPOUT[ep].AMOUNT=99;
  AppEvtHandlerDispatch();assert(rx.completions==1);checkPacket(ep,length);
  AppEvtHandlerExec();assert(rx.drdy==1 && rx.completions==1);
 }
 puts("PASS: all OUT endpoints/ZLP defer DRDY and completion once, with captured completion length");

 // Existing queued DMA starts in the same ISR, before either notification.
 deferredInit();receive(1,17);s_Usbd.EpReg[1][1]={regularCallback,outSlot};
 assert(productionEpSend(0,2,inBuffer,11));
 finish();assert(activeBit==2 && dmaBusy && !dmaUnlocks && !receivers[0].completions);
 finish();assert(!dmaBusy && !regularCompletions);
 ackIn(2);
 AppEvtHandlerDispatch();assert(receivers[0].completions==1 && !regularCompletions);
 AppEvtHandlerDispatch();assert(regularCompletions==1 && regularLength==11);checkPacket(1,17);
 puts("PASS: ISR hands DMA directly to the next queued request; each AppEvt delivers one callback");

 // A following OUT can be latched with END, or arrive after its ISR but
 // before AppEvt. FIFO event order publishes A before DRDY reserves B.
 for(bool sameIsr:{false,true}){
  deferredInit();receive(1,17);
  uint8_t first[17];memcpy(first,hostOut,sizeof(first));
  const uint32_t firstPtr=regs.EPOUT[1].PTR;
  if(!sameIsr)finish();
  for(auto &byte:hostOut)byte^=0xFFU;
  regs.SIZE.EPOUT[1]=31;regs.EPDATASTATUS.bits=1U<<17;
  if(sameIsr)finish();else interrupt();
  assert(!dmaBusy && regularStarts==1 && receivers[0].drdy==1);
  assert(!receivers[0].completions && !regs.EPDATASTATUS.bits);
  for(unsigned i=0;i<3;++i)interrupt();
  AppEvtHandlerDispatch();assert(receivers[0].completions==1 && !dmaBusy);
  AppEvtHandlerDispatch();assert(receivers[0].drdy==2 && activeBit==17);
  assert(regularStarts==2 && regs.EPOUT[1].PTR!=firstPtr);
  auto *packet=reinterpret_cast<RxPacket*>(CFifoGet(receivers[0].fifo));
  assert(packet && packet->length==17 && !memcmp(packet->data,first,sizeof(first)));
  finish();AppEvtHandlerDispatch();checkPacket(1,31);
  AppEvtHandlerExec();assert(receivers[0].drdy==2 && receivers[0].completions==2);
 }
 puts("PASS: completion precedes next DRDY in the same or following ISR; distinct RX slots preserve both payloads");

 // Every regular endpoint can have one completion queued at once. This
 // uses the same 16-entry AppEvt capacity as UsbInit, without coalescing.
 deferredInit();
 for(unsigned ep=1;ep<8;++ep){
  regs.SIZE.EPOUT[ep]=ep*7;regs.EPDATASTATUS.bits|=1U<<(ep+16);
  s_Usbd.EpReg[ep-1][1]={regularCallback,outSlot};
 }
 interrupt();assert(!dmaBusy);AppEvtHandlerExec();
 for(unsigned ep=1;ep<8;++ep)assert(productionEpSend(0,ep,inBuffer,ep));
 for(unsigned i=0;i<14;++i)finish();
 for(unsigned ep=1;ep<8;++ep)ackIn(ep);
 for(auto &rx:receivers)assert(rx.drdy==1 && !rx.completions);
 assert(!regularCompletions && !dmaBusy);dummyCalls=0;
 assert(AppEvtHandlerQue(0,nullptr,dummyEvent));
 assert(AppEvtHandlerQue(0,nullptr,dummyEvent));
 AppEvtHandlerExec();assert(regularCompletions==7 && dummyCalls==2);
 for(unsigned ep=1;ep<8;++ep){assert(receivers[ep-1].completions==1);checkPacket(ep,ep*7);}
 puts("PASS: 16 AppEvt entries hold all 14 regular completions plus two other events");

 // A blocked owner retries the same DRDY after freeing space; readiness
 // was consumed by the ISR when the notification was queued.
 deferredInit();auto &rx=receivers[0];
 assert(CFifoPut(rx.fifo) && CFifoPut(rx.fifo));
 regs.SIZE.EPOUT[1]=19;regs.EPDATASTATUS.bits=1U<<17;interrupt();
 AppEvtHandlerDispatch();assert(rx.drdy==1 && !dmaBusy && !regs.EPDATASTATUS.bits);
 assert(CFifoGet(rx.fifo) && CFifoGet(rx.fifo));
 auto *packet=reinterpret_cast<RxPacket*>(CFifoResv(rx.fifo));
 assert(productionEpReceive(0,1,packet->data,sizeof(packet->data)));
 finish();AppEvtHandlerDispatch();checkPacket(1,19);
 puts("PASS: a blocked RX owner can submit after freeing space without another DRDY latch");

 // If DRDY cannot be queued, its hardware status remains latched. An IRQ
 // after AppEvt drains retries it without a controller bitmap or pump.
 deferredInit();dummyCalls=0;
 for(unsigned i=0;i<16;++i)assert(AppEvtHandlerQue(0,nullptr,dummyEvent));
 regs.SIZE.EPOUT[1]=9;regs.EPDATASTATUS.bits=1U<<17;interrupt();
 assert(regs.EPDATASTATUS.bits==(1U<<17) && !dmaBusy && !receivers[0].drdy);
 UsbCtrlrProcess(0);assert(dummyCalls==16 && !receivers[0].drdy);
 interrupt();assert(!regs.EPDATASTATUS.bits);AppEvtHandlerDispatch();
 assert(receivers[0].drdy==1 && activeBit==17);
 finish();AppEvtHandlerDispatch();checkPacket(1,9);
 puts("PASS: full AppEvt queue leaves DRDY in hardware for a later ISR to queue");

 deferredInit();receive(1,5);finish();duringCompletion=queueAnotherOut;
 AppEvtHandlerDispatch();assert(receivers[0].completions==1 && !dmaBusy);
 checkPacket(1,5);AppEvtHandlerDispatch();assert(receivers[0].drdy==2 && activeBit==17);
 finish();AppEvtHandlerDispatch();checkPacket(1,23);
 puts("PASS: DRDY arriving during a completion callback executes only after that callback returns");

 // Existing hardware enable state suppresses delivery to a closed endpoint.
 for(bool in:{false,true}){
  deferredInit();receive(1,7);finish();productionEpClose(0,1,in);
  AppEvtHandlerExec();assert(receivers[0].completions==unsigned(in));
 }
 deferredInit();regs.SIZE.EPOUT[1]=7;regs.EPDATASTATUS.bits=1U<<17;interrupt();
 productionEpClose(0,1,false);AppEvtHandlerExec();assert(!receivers[0].drdy && !dmaBusy);
 deferredInit();receive(1,7);finish();
 regs.EPINEN=regs.EPOUTEN=0; // USB reset disables the data endpoints in hardware.
 nRFUsbdResetState();AppEvtHandlerExec();assert(!receivers[0].completions);
 puts("PASS: close/hardware reset suppress queued endpoint callbacks without completion bookkeeping");

 // Route each direction to its registered owner and capture its DMA length.
 // EP0, ISO and the opposite direction staying enabled must not keep a
 // closed regular endpoint's event deliverable, including ZLP/odd lengths.
 for(unsigned ep=1;ep<8;++ep)for(bool in:{false,true})
 for(uint16_t length:{0U,1U,17U,64U})for(bool closed:{false,true}){
  deferredInit();regs.EPINEN|=0x101U;regs.EPOUTEN|=0x101U;
  if(!in)receive(ep,length);
  s_Usbd.EpReg[ep-1][in]={regularCallback,outSlot};
  if(in)assert(productionEpSend(0,ep,inBuffer,length));
  finish();assert(!regularCompletions && !dmaBusy);
  if(in)ackIn(ep);
  if(closed)productionEpClose(0,ep,in);
  regs.EPIN[ep].AMOUNT=regs.EPOUT[ep].AMOUNT=99;
  AppEvtHandlerExec();assert(regularCompletions==unsigned(!closed));
  if(!closed)assert(regularLength==length);
 }
 for(unsigned ep=1;ep<8;++ep){
  deferredInit();regs.EPINEN|=0x101U;regs.EPOUTEN|=0x101U;
  regs.SIZE.EPOUT[ep]=7;regs.EPDATASTATUS.bits=1U<<(ep+16);interrupt();
  productionEpClose(0,ep,false);AppEvtHandlerExec();
  assert(!receivers[ep-1].drdy && !dmaBusy);
 }
 puts("PASS: IN/OUT owner routing preserves captured lengths and closed-endpoint suppression with EP0/ISO enabled");
}
void ackIn(unsigned ep){
 assert(inUsbBusy[ep]);inUsbBusy[ep]=false;
 regs.EPDATASTATUS.bits|=1U<<ep;interrupt();
}
alignas(8) uint8_t txMemory[CFIFO_TOTAL_MEMSIZE(128,1)];
hCFifo_t txFifo;
unsigned txCompletions=0;bool completeDuringCallback=false;
void txCallback(UsbCtrlrEvtType_t event,uint16_t length,void*){
 assert(!inIsr && !irqMask && event==USB_CTRLR_EVT_XFER_CMPL);
 ++txCompletions;
 int consumed=length;assert(CFifoGetMultiple(txFifo,&consumed) && consumed==length);
 if(auto *next=CFifoPeek(txFifo)){
  assert(!inUsbBusy[1]);
  assert(productionEpSend(0,1,next,64));
  assert(CFifoUsed(s_Usbd.hQue)==1 && dmaBusy);
  if(completeDuringCallback){completeDuringCallback=false;finish();ackIn(1);}
 }
}
void txInit(){
 deferredInit();checkInBuffer=true;txCompletions=0;completeDuringCallback=false;
 txFifo=CFifoInit(txMemory,sizeof(txMemory),1,true);
 int count=128;auto *data=CFifoPutMultiple(txFifo,&count);assert(data && count==128);
 for(int i=0;i<count;++i)data[i]=uint8_t(i+3);
 dmaBuffer(data);dmaBuffer(data+64);
 s_Usbd.EpReg[0][1]={txCallback,nullptr};
 assert(productionEpSend(0,1,data,64));
}
void testDeferredIn(){
 // Regression: dispatching after END, before EPDATA, must not consume TX
 // data or submit another packet to the same IN endpoint.
 txInit();uint8_t first[64];memcpy(first,wireIn,64);
 finish();assert(!dmaBusy && !txCompletions && !CFifoUsed(s_Usbd.hQue));
 for(unsigned i=0;i<3;++i){interrupt();UsbCtrlrProcess(0);}
 assert(!txCompletions && CFifoUsed(txFifo)==128 && regularStarts==1);
 assert(!memcmp(first,wireIn,64));
 ackIn(1);assert(!dmaBusy && !txCompletions);
 AppEvtHandlerDispatch();
 assert(txCompletions==1 && CFifoUsed(txFifo)==64 && activeBit==1 && regularStarts==2);
 assert(!memcmp(wireIn,CFifoPeek(txFifo),64));
 finish();AppEvtHandlerExec();assert(txCompletions==1);
 ackIn(1);AppEvtHandlerExec();assert(txCompletions==2 && !CFifoUsed(txFifo));
 for(unsigned i=0;i<3;++i){interrupt();AppEvtHandlerExec();}
 assert(txCompletions==2);
 puts("PASS: IN END frees DMA without callback; EPDATA queues exactly one continuation through AppEvt");

 // Queued OUT starts in the IN END ISR even while that IN packet awaits
 // consumption. A different IN endpoint can also use shared DMA.
 txInit();
 regs.SIZE.EPOUT[2]=17;regs.EPDATASTATUS.bits=1U<<18;interrupt();AppEvtHandlerDispatch();
 assert(CFifoUsed(s_Usbd.hQue)==2 && activeBit==1);
 s_Usbd.EpReg[2][1]={regularCallback,outSlot};
 assert(productionEpSend(0,3,inBuffer,9));
 finish();assert(activeBit==18 && !txCompletions && !dmaUnlocks);
 finish();assert(activeBit==3 && !receivers[1].completions);
 finish();AppEvtHandlerExec();checkPacket(2,17);
 assert(!txCompletions && !regularCompletions && !dmaBusy);
 ackIn(3);AppEvtHandlerExec();assert(regularCompletions==1 && regularLength==9);
 ackIn(1);AppEvtHandlerExec();assert(txCompletions==1 && activeBit==1);
 finish();ackIn(1);AppEvtHandlerExec();assert(txCompletions==2);
 puts("PASS: IN END immediately hands DMA to queued OUT; unrelated IN progresses without queue rotation");

 // Host consumption can be latched when END is serviced, or later.
 for(bool together:{false,true}){
  txInit();
  if(together){inUsbBusy[1]=false;regs.EPDATASTATUS.bits=1U<<1;}
  finish();
  if(!together)ackIn(1);
  assert(!txCompletions && !dmaBusy);
  AppEvtHandlerDispatch();assert(txCompletions==1 && activeBit==1 && regularStarts==2);
  finish();ackIn(1);AppEvtHandlerExec();assert(txCompletions==2);
 }
 txInit();finish();ackIn(1);completeDuringCallback=true;
 AppEvtHandlerDispatch();assert(txCompletions==1 && !dmaBusy);
 AppEvtHandlerDispatch();assert(txCompletions==2 && !CFifoUsed(txFifo));
 puts("PASS: END/EPDATA together or separate, and completion during callback, preserve one continuation");

 // Full AppEvt storage must leave the acknowledgement latched. A later
 // interrupt retries after the foreground has drained the older events.
 txInit();dummyCalls=0;
 for(unsigned i=0;i<16;++i)assert(AppEvtHandlerQue(0,nullptr,dummyEvent));
 finish();ackIn(1);
 assert(!dmaBusy && !txCompletions && CFifoUsed(txFifo)==128);
 assert(regs.EPDATASTATUS.bits==(1U<<1));
 UsbCtrlrProcess(0);assert(dummyCalls==16 && !txCompletions);
 interrupt();assert(!regs.EPDATASTATUS.bits);
 AppEvtHandlerDispatch();assert(txCompletions==1 && activeBit==1);
 finish();ackIn(1);AppEvtHandlerExec();assert(txCompletions==2);
 puts("PASS: full AppEvt queue retains IN acknowledgement in hardware until a later ISR queues it");

 // Actual lengths, including ZLP, are captured at EPDATA before dispatch.
 for(unsigned ep=1;ep<8;++ep)for(uint16_t length:{0U,1U,17U,64U}){
  deferredInit();checkInBuffer=true;s_Usbd.EpReg[ep-1][1]={regularCallback,outSlot};
  assert(productionEpSend(0,ep,inBuffer,length));finish();AppEvtHandlerExec();
  assert(!regularCompletions);ackIn(ep);regs.EPIN[ep].AMOUNT=99;
  AppEvtHandlerDispatch();assert(regularCompletions==1 && regularLength==length);
  interrupt();AppEvtHandlerExec();assert(regularCompletions==1);
 }
 for(unsigned cancel=0;cancel<3;++cancel){
  txInit();finish();ackIn(1);
  if(cancel==2){regs.EPINEN=regs.EPOUTEN=0;nRFUsbdResetState();}
  else productionEpClose(0,1,cancel==1);
  AppEvtHandlerExec();assert(txCompletions==unsigned(cancel==0));
 }
 puts("PASS: EP1-7/ZLP preserve captured IN length and suppress closed/reset endpoint delivery");
}
int main(int argc,char **argv){
 testDeferredOut();
 if(argc==2 && !strcmp(argv[1],"--regular-out"))return 0;
 testDeferredIn();
 if(argc==2 && !strcmp(argv[1],"--regular"))return 0;
 // Check every hardware END mapping independently of the decoder. A stale
 // END from another direction must not release the active transfer.
 for(unsigned ep=0;ep<=8;++ep)for(unsigned out=0;out<2;++out){
  init();dmaBusy=0x82;
  const unsigned bit=ep+out*16;
  regs.EPSTATUS.bits=1U<<bit;
  for(unsigned i=0;i<8;++i)regs.EVENTS_ENDEPIN[i]=regs.EVENTS_ENDEPOUT[i]=1;
  regs.EVENTS_ENDISOIN=regs.EVENTS_ENDISOOUT=1;
  uint32_t *end=ep==8?(out?&regs.EVENTS_ENDISOOUT:&regs.EVENTS_ENDISOIN):
   (out?&regs.EVENTS_ENDEPOUT[ep]:&regs.EVENTS_ENDEPIN[ep]);
  if(ep==0 && !out)assert(CFifoPut(s_Usbd.hEp0Que));
  if(ep>0 && ep<8){
   auto *queued=(nRFUsbdQue_t*)CFifoPut(s_Usbd.hQue);
   *queued={uint8_t(ep),NRFX_USBD_QUE_IN_BUFFER,0,{inBuffer}};
   s_Usbd.EpReg[ep-1][!out]={regularCallback,outSlot};
  }
  if(ep==8)s_Usbd.IsoDataFlag=out?NRFUSBD_ISO_OUT_READY:NRFUSBD_ISO_IN_READY;
  *end=0;regs.EVENTS_EP0DATADONE=bit==0;
  interrupt();
  assert(regs.EPSTATUS.bits==(1U<<bit) && dmaBusy);
  assert(!callbacks[0] && !callbacks[1] && !regularCompletions);
  assert(!ep0Callbacks[0] && !ep0Callbacks[1] && tailVisits==1);
  *end=1;
  interrupt();
  assert(*end==0 && !regs.EPSTATUS.bits && !dmaBusy);
  assert(!CFifoUsed(s_Usbd.hQue) && !CFifoUsed(s_Usbd.hEp0Que));
  assert(callbacks[!out]==unsigned(ep==8));
  assert(ep0Callbacks[!out]==unsigned(ep==0));
  assert(!regularCompletions);
  // Regular OUT completion is queued at END; regular IN waits for EPDATA.
  if(ep>0 && ep<8)AppEvtHandlerDispatch();
  assert(regularCompletions==unsigned(ep>0 && ep<8 && out));
  if(ep>0 && ep<8 && !out){
   regs.EPDATASTATUS.bits=1U<<ep;
   interrupt();assert(!regs.EPDATASTATUS.bits && !regularCompletions);
   AppEvtHandlerDispatch();assert(regularCompletions==1);
  }
 }
 puts("PASS: all 18 DMA directions retire only on their own END");
 // A bus event is handled on its own; unless it completes a resume the
 // ISR returns and the latched END and EPDATA work run on the next pass.
 init();dmaBusy=0x82;regs.EPSTATUS.bits=1;regs.EVENTS_ENDEPIN[0]=1;
 assert(CFifoPut(s_Usbd.hEp0Que));
 regs.EVENTS_USBEVENT=1;
 s_Usbd.EpReg[1][1]={regularCallback,outSlot};
 regs.EPIN[2].AMOUNT=11;regs.EPDATASTATUS.bits=1U<<2;
 interrupt();
 assert(busEventCalls==1 && !tailVisits && regs.EPSTATUS.bits==1 && dmaBusy);
 assert(regs.EPDATASTATUS.bits==(1U<<2) && !ep0Callbacks[1]);
 interrupt();
 assert(!regs.EPSTATUS.bits && !dmaBusy && ep0Callbacks[1]==1 && tailVisits==1);
 assert(!regs.EPDATASTATUS.bits && !CFifoUsed(s_Usbd.hEp0Que) && !regularCompletions);
 AppEvtHandlerDispatch();assert(regularCompletions==1 && regularLength==11);
 // Hardware status without software ownership is not retired.
 init();regs.EPSTATUS.bits=1U<<2;regs.EVENTS_ENDEPIN[2]=1;
 interrupt();assert(regs.EPSTATUS.bits==(1U<<2) && regs.EVENTS_ENDEPIN[2]==1);
 puts("PASS: a bus event defers END/EPDATA to the next pass; retirement requires software DMA ownership");

 // EP0 IN chains its next packet under the same ownership and reports only
 // after the last packet's host handshake.
 init();dmaBusy=0x82;
 auto *first=(nRFEPPkt_t*)CFifoPut(s_Usbd.hEp0Que);first->Len=64;
 auto *second=(nRFEPPkt_t*)CFifoPut(s_Usbd.hEp0Que);second->Len=9;
 nRFUsbdEp0InStart(first);
 finish();
 assert(activeBit==0 && dmaBusy && ep0InStarts==2 && !dmaUnlocks);
 assert(CFifoUsed(s_Usbd.hEp0Que)==1 && !ep0Callbacks[1]);
 assert(regs.EPIN[0].MAXCNT==9);
 finish();
 assert(!dmaBusy && !regs.EPSTATUS.bits && ep0Callbacks[1]==1);
 assert(!CFifoUsed(s_Usbd.hEp0Que));
 puts("PASS: EP0 IN chains packets without releasing DMA ownership or reporting an early completion");
 // EP0 OUT readiness exists before any EPOUT0 DMA is captured. It starts
 // from idle, or stays latched behind an active regular/ISO DMA.
 init();regs.BMREQUESTTYPE=0;regs.EVENTS_EP0DATADONE=1;
 interrupt();
 assert(activeBit==16 && ep0OutStarts==1 && !regs.EVENTS_EP0DATADONE);
 assert(regs.EPOUT[0].PTR==uint32_t(uintptr_t(s_Usbd.Ep0Bounce)));
 assert(regs.EPOUT[0].MAXCNT==NRFX_USBD_MAX_PACKET_SIZE);
 regs.EPOUT[0].AMOUNT=17;regs.EVENTS_ENDEPOUT[0]=1;interrupt();
 assert(!dmaBusy && ep0Callbacks[0]==1 && ep0Lengths[0]==17);
 assert(regs.TASKS_EP0RCVOUT==1);
 for(unsigned held:{2U,8U,18U,24U}){
  init();regs.BMREQUESTTYPE=0;regs.EVENTS_EP0DATADONE=1;
  dmaBusy=0x82;regs.EPSTATUS.bits=1U<<held;activeBit=int(held);
  if(held==2 || held==18){
   auto *entry=(nRFUsbdQue_t*)CFifoPut(s_Usbd.hQue);
   *entry={2,NRFX_USBD_QUE_IN_BUFFER,0,{inBuffer}};
   s_Usbd.EpReg[1][0]={regularCallback,outSlot};
  }else s_Usbd.IsoDataFlag=held==8?NRFUSBD_ISO_IN_READY:NRFUSBD_ISO_OUT_READY;
  interrupt();
  assert(regs.EPSTATUS.bits==(1U<<held) && regs.EVENTS_EP0DATADONE==1);
  assert(!ep0OutStarts && dmaBusy);
  finish();assert(activeBit==16 && ep0OutStarts==1 && !regs.EVENTS_EP0DATADONE);
 }
 puts("PASS: EP0 OUT data starts from idle and remains latched behind regular or ISO DMA");

 // IN status/stray handshakes clear even when a different DMA owns the
 // channel. They must not retire that DMA or dequeue its request.
 for(int held:{-1,2,8,16,18,24}){
  init();regs.EVENTS_EP0DATADONE=1;
  if(held>=0){dmaBusy=0x82;regs.EPSTATUS.bits=1U<<held;}
  const uint32_t before=regs.EPSTATUS.bits;
  interrupt();
  assert(!regs.EVENTS_EP0DATADONE && regs.EPSTATUS.bits==before);
  assert(dmaBusy==(held<0?0U:0x82U));
  assert(!ep0Callbacks[0] && !ep0Callbacks[1] && !ep0OutStarts);
 }
 // Pending EP0 IN keeps the handshake until its matching END arrives.
 init();dmaBusy=0x82;regs.EPSTATUS.bits=1;regs.EVENTS_EP0DATADONE=1;
 interrupt();assert(regs.EVENTS_EP0DATADONE==1 && regs.EPSTATUS.bits==1 && dmaBusy);
 // A suspend holds an OUT-ready packet. The bus event that completes the
 // resume falls through to the scheduler and starts it in the same pass.
 init();regs.BMREQUESTTYPE=0;regs.EVENTS_EP0DATADONE=1;s_Usbd.Flags|=USBD_FLAG_SUSPENDED;
 interrupt();assert(!dmaBusy && !ep0OutStarts && regs.EVENTS_EP0DATADONE==1);
 resumeOnBusEvent=true;regs.EVENTS_USBEVENT=1;
 interrupt();assert(activeBit==16 && ep0OutStarts==1 && busEventCalls==1);
 puts("PASS: EP0 IN status clears independently; pending IN and suspended OUT retain their handshake");

 // Foreground close waits for DMA END, including EP0 IN before its host
 // handshake, and discards only regular queue entries without starting work.
 for(unsigned ep=0;ep<=8;++ep)for(unsigned out=0;out<2;++out)
 for(unsigned masked=0;masked<2;++masked){
  init();irqMask=masked;dmaBusy=0x82;regs.EPSTATUS.bits=1U<<(ep+out*16);
  assert(CFifoPut(s_Usbd.hQue));
  if(ep==8){if(out)regs.EVENTS_ENDISOOUT=1;else regs.EVENTS_ENDISOIN=1;}
  else{if(out)regs.EVENTS_ENDEPOUT[ep]=1;else regs.EVENTS_ENDEPIN[ep]=1;}
  regs.EVENTS_EP0DATADONE=0;
  const uint32_t bit=1U<<(ep+out*16);
  // A wait for another direction leaves this DMA running.
  nRFUsbdDmaWait(~bit);
  assert(dmaBusy && regs.EPSTATUS.bits==bit && CFifoUsed(s_Usbd.hQue)==1);
  nRFUsbdDmaWait(bit);
  assert(!dmaBusy && !regs.EPSTATUS.bits && irqMask==masked);
  assert(CFifoUsed(s_Usbd.hQue)==(ep>0 && ep<8?0:1));
  assert(!regularStarts && !ep0InStarts && !isoStarts[0] && !isoStarts[1]);
 }
 puts("PASS: close wait retires only the selected DMA direction, for all 18, without starting another DMA");
 // OUT: the SOF offer starts a DMA into the controller's own buffer, sized
 // by the received packet; its END completes it with the byte count.
 init();frame(17);
 assert(dmaBusy && activeBit==24 && isoStarts[0]==1);
 assert(regs.ISOOUT.PTR==uint32_t(uintptr_t(outSlot)) && regs.ISOOUT.MAXCNT==17);
 finish();
 assert(callbacks[0]==1 && lengths[0]==17 && !memcmp(lastOut,hostOut,17));
 assert(!dmaBusy && s_Usbd.IsoDataFlag==0);
 assert(drdyCnt==1 && s_Usbd.pIsoBuffer[0]==nullptr);
 puts("PASS: ISO OUT lands in the submitted destination and completes at ENDISOOUT");

 // No destination submitted: the owner is asked once (DRDY) with a packet
 // waiting. If it submits one the frame is read into it; if not the
 // frame is dropped and the channel released.
 init();s_Usbd.pIsoBuffer[0]=nullptr;drdyGives=true;frame(17);
 assert(drdyCnt==1 && activeBit==24 && regs.ISOOUT.PTR==uint32_t(uintptr_t(outSlot)));
 finish();
 assert(callbacks[0]==1 && lengths[0]==17 && !memcmp(lastOut,hostOut,17) && !dmaBusy);
 init();drdyGives=false;frame(17);
 assert(drdyCnt==1 && !dmaBusy && isoStarts[0]==0 && s_Usbd.IsoDataFlag==0 &&
  dmaLocks==dmaUnlocks);
 // No packet: the owner is not asked.
 init();s_Usbd.pIsoBuffer[0]=nullptr;frame(-1);frame(0);
 assert(drdyCnt==0 && isoStarts[0]==0);
 puts("PASS: DRDY requests an OUT destination; without a submission the frame is dropped");

 // No OUT packet this frame: nothing starts, the request is dropped, the
 // channel is released.
 init();frame(-1);
 assert(!dmaBusy && isoStarts[0]==0 && s_Usbd.IsoDataFlag==0 && dmaLocks==dmaUnlocks);
 // A packet larger than the endpoint is not read.
 init();s_Usbd.IsoMaxPacketSize[0]=9;frame(17);
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
 assert(s_Usbd.pIsoBuffer[1]==inBuffer && s_Usbd.IsoInDmaLen==9);
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
 interrupt();assert(regs.EPSTATUS.bits==(1U<<8) && dmaBusy && !callbacks[1]);
 finish();assert(callbacks[1]==1);
 // EP0 IN retires at its own END and reports the drained queue.
 init();dmaBusy=0x82;pkt=(nRFEPPkt_t*)CFifoPut(s_Usbd.hEp0Que);pkt->Len=8;
 nRFUsbdEp0InStart(pkt);assert(activeBit==0);
 interrupt();assert(regs.EPSTATUS.bits==1 && dmaBusy && !ep0Callbacks[1]);
 regs.EVENTS_ENDEPIN[0]=1;
 interrupt();assert(regs.EPSTATUS.bits==0 && !dmaBusy && ep0Callbacks[1]==1);
 assert(CFifoUsed(s_Usbd.hEp0Que)==0);
 puts("PASS: a DMA retires only at its END event, EP0 IN included");

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
 productionEpClose(0,8,false);
 assert(callbacks[0]==0 && !dmaBusy && s_Usbd.IsoDataFlag==0 && !s_Usbd.IsoOpen);
 assert(productionIsoOpen(0,8,false,9) && s_Usbd.IsoOpen);
 activeBit=-1;frame(9);finish();
 assert(callbacks[0]==1 && lengths[0]==9);
 puts("PASS: close retires an active ISO DMA silently; reopen starts clean");

 // Open refuses a packet size the hardware cannot hold.
 init();assert(!productionIsoOpen(0,8,true,513) && productionIsoOpen(0,8,true,512));
 assert(regs.ISOSPLIT==USBD_ISOSPLIT_SPLIT_HalfIN && regs.ISOINCONFIG==USBD_ISOINCONFIG_RESPONSE_ZeroData);
 puts("PASS: ISO open is bounded by the 512-byte half buffer");

 // Receive captures a pointer in the shared queue. Readiness belongs to the
 // ISR, which consumed it when it queued DRDY, so EPDATASTATUS is left alone.
 // A blocked channel retains the buffer until DMA actually runs; completion
 // reports the actual byte count.
 for(unsigned masked=0;masked<2;++masked)for(uint8_t ep=1;ep<8;++ep)
 for(uint16_t length:{0U,1U,17U,64U}){
  init();irqMask=masked;s_Usbd.Flags|=USBD_FLAG_SUSPENDED;
  const uint32_t bit=1U<<(ep+16),otherBit=1U<<23;
  s_Usbd.EpReg[ep-1][0]={regularCallback,outSlot};
  regs.SIZE.EPOUT[ep]=length;regs.EPDATASTATUS.bits=bit|otherBit;
  regularCompletions=0;regularLength=999;
  assert(productionEpReceive(0,ep,outSlot,64));
  assert(irqMask==masked && CFifoUsed(s_Usbd.hQue)==1 && !dmaBusy);
  assert(regs.EPDATASTATUS.bits==(bit|otherBit));
  auto *entry=(nRFUsbdQue_t*)CFifoPeek(s_Usbd.hQue);
  assert(entry->EpNum==ep && entry->Dir==NRFX_USBD_QUE_OUT);
  assert(entry->pBuffer==outSlot && entry->Len==64);
  assert(s_Usbd.EpReg[ep-1][0].Handler==regularCallback);
  assert(s_Usbd.EpReg[ep-1][0].pContext==outSlot);
  s_Usbd.Flags&=(uint8_t)~USBD_FLAG_SUSPENDED;
  nRFUsbdResumeQueuedDmaLocked();assert(activeBit==ep+16 && !regularCompletions);
  assert(regs.EPOUT[ep].PTR==uint32_t(uintptr_t(outSlot)) && regs.EPOUT[ep].MAXCNT==length);
  finish();assert(!regularCompletions && !dmaBusy);
  AppEvtHandlerDispatch();assert(regularCompletions==1 && regularLength==length);
  assert(!memcmp(outSlot,hostOut,length));
 }
 // A full request queue refuses the destination; the owner can submit it
 // again once a slot frees.
 init();s_Usbd.Flags|=USBD_FLAG_SUSPENDED;
 while(CFifoPut(s_Usbd.hQue)!=nullptr){}
 assert(!productionEpReceive(0,1,outSlot,64));
 (void)CFifoGet(s_Usbd.hQue);
 assert(productionEpReceive(0,1,outSlot,64));
 puts("PASS: OUT queues one destination, leaves readiness to the ISR and completes after DMA");

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

code = (preamble.replace('@@DRIVER_TYPES@@', types)
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
    subprocess.run([str(Path(temp) / 'iso_test')] +
                   (['--regular-out'] if args.regular_out else ['--regular'] if args.regular else []), check=True)
