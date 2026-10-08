#!/usr/bin/env python3
"""Compile the complete USBHS port against a DWC2 register model.

Only the MMIO lvalue and register-pointer types are replaced. Production
functions, the Nordic controller API and generic interface code are compiled.
The model implements W1C interrupts, reset completion, NAK/disable handshakes
and DMA completion; it does not model USB signalling or analog PHY timing.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/Nordic/nRF54/src/usb_ctrlr_nrf54.cpp').read_text()
source, count = re.subn(r'#define NRF54_USBD_REG\(ofs\) \\\n[^\n]+',
                       '#define NRF54_USBD_REG(ofs) reg(ofs)', source)
assert count == 1
source = source.replace('volatile uint32_t *pReg', 'Register *pReg')
source = source.replace('volatile uint32_t *pCtl', 'Register *pCtl')

NRF = r'''
#pragma once
#include <cstdint>
#include <cstddef>
#define USBHS_PRESENT 1
#define USBHSCORE_PRESENT 1
#define USBHS_HAS_CORE_EVENT 0
#define NRF_USBHSCORE_S nullptr
#define NRF_USBHSCORE_NS nullptr
// LM20A/B have no wrapper CORE event or INTEN registers. The DWC2 core
// drives USBHS_IRQn directly; do not invent wrapper registers for the model.
struct Wrapper {
 uint32_t ENABLE, TASKS_START;
 struct {uint32_t CONFIG,OVERRIDEVALUES,INPUTOVERRIDE;} PHY;
};
extern Wrapper wrapper;
#define NRF_USBHS (&wrapper)
extern uint32_t powerRegs[300];
#define NRF_VREGUSB powerRegs
#define NRF_CLOCK nullptr
#define NRF_FICR nullptr
#define USBHS_ENABLE_CORE_Msk 1U
#define USBHS_ENABLE_PHY_Msk 2U
#define USBHS_PHY_OVERRIDEVALUES_ID_Device 1U
#define USBHS_PHY_OVERRIDEVALUES_ID_Pos 31U
#define USBHS_PHY_INPUTOVERRIDE_ID_Msk (1U<<31)
#define USBHS_PHY_INPUTOVERRIDE_VBUSVALID_Msk (1U<<30)
#define USBHS_PHY_INPUTOVERRIDE_SUSPENDM0_Msk (1U<<25)
#define USBHS_PHY_CONFIG_ResetValue 0x5533D6F0U
#define USBHS_TASKS_START_TASKS_START_Trigger 1U
#define NRF_CLOCK_DOMAIN_HFCLK24M 0
#define NRF_CLOCK_EVENT_HFCLK24MSTARTED 0
#define NRF_CLOCK_TASK_HFCLK24MSTART 0
#define NRF_VREGUSB_EVENT_VBUS_DETECTED 0
#define NRF_VREGUSB_EVENT_VBUS_REMOVED 1
#define NRF_VREGUSB_TASK_START 0
#define NRF_VREGUSB_INT_VBUS_DETECTED_MASK 1U
#define NRF_VREGUSB_INT_VBUS_REMOVED_MASK 2U
#define USBHS_IRQn 0
#define VREGUSB_IRQn 1
extern bool irqEnabled[2];
inline bool nrf_clock_is_running(void*,int,void*){return true;}
inline void nrf_clock_event_clear(void*,int){}
inline bool nrf_clock_event_check(void*,int){return true;}
inline void nrf_clock_task_trigger(void*,int){}
inline bool nrf_vregusb_event_check(uint32_t *p,int ev){return p[ev]!=0;}
inline void nrf_vregusb_event_clear(uint32_t *p,int ev){p[ev]=0;}
inline void nrf_vregusb_task_trigger(uint32_t*,int){}
inline void nrf_vregusb_int_enable(uint32_t*,uint32_t){}
inline uint32_t nrf_ficr_deviceid_get(void*,unsigned i){return i ? 0x89ABCDEFU:0x01234567U;}
inline void NVIC_SetPriority(int,int){}
inline void NVIC_ClearPendingIRQ(int){}
inline void NVIC_DisableIRQ(int irq){irqEnabled[irq]=false;}
inline void NVIC_EnableIRQ(int irq){irqEnabled[irq]=true;}
inline void __DSB(){}
inline void __ISB(){}
inline void nrfx_coredep_delay_us(uint32_t){}
'''
MODEL = r'''
#include <cassert>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>
#include <initializer_list>
#include "nrf.h"
Wrapper wrapper;
uint32_t powerRegs[300];
bool irqEnabled[2];
static bool legacyReset;
struct Register {
 unsigned offset;
 uint32_t value;
 operator uint32_t() const {return value;}
 Register &operator=(uint32_t v);
 Register &operator|=(unsigned long v){return *this=value|v;}
 Register &operator&=(unsigned long v){return *this=value&v;}
};
static Register registers[1024];
static Register &reg(unsigned ofs) {
 assert(wrapper.ENABLE==3U); // Core MMIO while unpowered is a hardware fault.
 auto &r=registers[ofs/4];r.offset=ofs;return r;
}
static void set(unsigned ofs,uint32_t value){registers[ofs/4].value=value;}
static uint32_t get(unsigned ofs){return registers[ofs/4].value;}
static void add(unsigned ofs,uint32_t bits){set(ofs,get(ofs)|bits);}
Register &Register::operator=(uint32_t v){
 if(offset==0x010){ // Flush self-clears; new reset completes with DONE set.
  value=(1U<<31)|((v&1U)&&!legacyReset ? (1U<<29)|1U:0U);return *this;
 }
 if(offset==0x014 || ((offset>=0x900&&offset<0xD00)&&(offset%0x20)==8)){
  value&=~v;return *this; // W1C
 }
 if(offset==0x804){
  value=v&~((1U<<9)|(1U<<10));
  if(v&(1U<<9))add(0x014,1U<<7);
  if(v&(1U<<10))set(0x014,get(0x014)&~(1U<<7));
  return *this;
 }
 if(offset>=0x900&&offset<0xD00&&(offset%0x20)==0){
  value=v&~((1U<<26)|(1U<<27)|(1U<<28)|(1U<<29)|(1U<<30));
  if(v&(1U<<28))value&=~(1U<<16);
  if(v&(1U<<29))value|=1U<<16;
  if(v&(1U<<27)){value|=1U<<17;if(offset<0xB00)add(offset+8,1U<<6);}
  if(v&(1U<<26))value&=~(1U<<17);
  if(v&(1U<<30)){value&=~(1U<<31);add(offset+8,1U<<1);}
  return *this;
 }
 value=v;return *this;
}
'''
TEST = r'''
#include "usb/usb_iso.h"
struct Event {uint32_t id;void *context;UsbEvtQueHandler_t handler;};
static std::deque<Event> queue;
static unsigned queueLimit=4,processRequests,resetEvents,sofEvents;
static bool processOwed;
static UsbSetupData_t lastSetup;
static unsigned setupEvents,ep0Completions;
static uint16_t lastEp0Length,sofFrame;
static uint8_t ep0Reply[80];
static unsigned setupAction;
extern "C" bool UsbEvtQue(uint32_t id,void *context,UsbEvtQueHandler_t handler){
 if(queue.size()>=queueLimit)return false;
 queue.push_back({id,context,handler});return true;
}
extern "C" void UsbProcessQue(int){++processRequests;processOwed=true;}
extern "C" void UsbDevProcessEvent(int,const UsbCtrlrEvt_t *event){
 switch(event->Type){
 case USB_CTRLR_EVT_RESET:++resetEvents;UsbCtrlrEpCloseAll(0);break;
 case USB_CTRLR_EVT_SOF:++sofEvents;sofFrame=event->FrameNo;break;
 case USB_CTRLR_EVT_SETUP:
  ++setupEvents;lastSetup=event->Setup;
  if(setupAction==1)assert(UsbCtrlrEp0Send(0,ep0Reply,sizeof(ep0Reply))==64);
  if(setupAction==2)UsbCtrlrEpStall(0,0,false);
  break;
 case USB_CTRLR_EVT_XFER_CMPL:++ep0Completions;lastEp0Length=event->Xfer.Length;break;
 default:break;
 }
}
static void drain(){
 unsigned bound=1000;
 while(!queue.empty() || processOwed){
  assert(bound--);
  if(!queue.empty()){auto event=queue.front();queue.pop_front();event.handler(event.id,event.context);}
  else {processOwed=false;UsbCtrlrProcess(0);}
 }
}
static void irq(){
 uint32_t pending=0;
 for(unsigned ep=0;ep<16;++ep){
  if(get(0x908+ep*32)&get(0x810))pending|=1U<<ep;
  if(get(0xB08+ep*32)&get(0x814))pending|=1U<<(ep+16);
 }
 set(0x818,pending);pending&=get(0x81C);
 set(0x014,(get(0x014)&~((1U<<18)|(1U<<19)))|
   (pending&0xFFFF?1U<<18:0U)|(pending>>16?1U<<19:0U));
 if(irqEnabled[USBHS_IRQn] && (get(0x008)&1U) && (get(0x014)&get(0x018)))
  USBHS_IRQHandler();
}
static void complete(uint8_t ep,bool in,unsigned amount){
 const unsigned base=(in?0x900:0xB00)+ep*32;
 auto &x=s_Ctrlr.Xfer[ep][in];
 set(base,get(base)&~(1U<<31));
 set(base+16,x.TotalLen-amount);
 add(base+8,1);irq();
}
struct Owner {
 alignas(4) uint8_t buffer[2048];
 uint8_t ep;bool in,hold,closeOnComplete;
 unsigned ready,completed,failed;uint16_t length;
};
static Owner owners[16][2];
static void callback(UsbCtrlrEvtType_t event,uint16_t length,void *ctx){
 auto &o=*static_cast<Owner*>(ctx);
 if(event==USB_CTRLR_EVT_DRDY){
  ++o.ready;
  if(!o.hold)assert(UsbCtrlrEpReceive(0,o.ep,o.buffer,sizeof(o.buffer)));
 }else if(event==USB_CTRLR_EVT_XFER_CMPL){
  ++o.completed;o.length=length;
  if(o.closeOnComplete)UsbCtrlrEpClose(0,o.ep,o.in);
 }else if(event==USB_CTRLR_EVT_XFER_FAILED){++o.failed;}
}
static Owner &open(unsigned ep,bool in,unsigned type,unsigned mps){
 auto &o=owners[ep][in];o={};o.ep=ep;o.in=in;
 UsbCtrlrEpBind(0,ep,in,true,callback,&o);
 assert(UsbCtrlrEpOpenData(0,ep,in,type,mps));return o;
}
static void init(bool hs=false,bool lowPower=false){
 memset(registers,0,sizeof(registers));memset(powerRegs,0,sizeof(powerRegs));
 queue.clear();queueLimit=4;processRequests=0;processOwed=false;
 resetEvents=setupEvents=ep0Completions=sofEvents=setupAction=0;
 wrapper={};powerRegs[0x400/4]=1U<<2;
 memset(irqEnabled,0,sizeof(irqEnabled));
 UsbCtrlrCfg_t cfg={};cfg.IntPrio=6;cfg.bLowPowerSuspend=lowPower;assert(UsbCtrlrInit(0,&cfg));
 set(0x010,1U<<31);set(0x048,2U<<3);set(0x04C,3040U<<16);
 assert(UsbCtrlrStart(0));assert(s_Ctrlr.Started);
 assert(get(0x05C)==0x0BE00C00U);
 assert((wrapper.PHY.INPUTOVERRIDE&(1U<<30))!=0);
 UsbCtrlrIntEnable(0);UsbCtrlrConnect(0);
 assert((wrapper.PHY.INPUTOVERRIDE&(1U<<30))==0);
 set(0x808,hs?0U:2U);add(0x014,1U<<13);irq();
 assert(UsbCtrlrHighSpeed(0)==hs);
}
static void setup(unsigned offset,const UsbSetupData_t &packet){
 assert(offset+8<=sizeof(s_Ep0Bounce));
 memcpy(reinterpret_cast<uint8_t*>(s_Ep0Bounce)+offset,&packet,8);
 set(0xB14,uint32_t(uintptr_t(s_Ep0Bounce)+offset+8));
 set(0xB00,get(0xB00)&~(1U<<31));
 add(0xB08,(1U<<15)|1U);irq(); // SETUP DMA before phase-done.
 add(0xB08,1U<<3);irq();
}
int main(){
 // Disabling both interrupt gates retains a core event for re-enable.
 init();const auto interruptMask=get(0x018);
 UsbCtrlrIntDisable(0);
 assert(!irqEnabled[USBHS_IRQn] && !(get(0x008)&1U));
 set(0x808,0U);add(0x014,1U<<13);irq();
 assert(!UsbCtrlrHighSpeed(0) && (get(0x014)&(1U<<13)));
 UsbCtrlrIntEnable(0);
 assert(irqEnabled[USBHS_IRQn] && (get(0x008)&1U));
 assert(get(0x018)==interruptMask);
 irq();assert(UsbCtrlrHighSpeed(0) && !(get(0x014)&(1U<<13)));
 for(bool legacy:{false,true}){legacyReset=legacy;init();}
 for(bool hs:{false,true})for(unsigned ep=1;ep<16;++ep){
  init(hs);unsigned mps=hs?512:64;
  auto &out=open(ep,false,USB_ENDPATT_TRANS_BULK,mps);
  assert(s_Ctrlr.Xfer[ep][0].State==NRF54_XFER_ACTIVE);
  assert(!UsbCtrlrEpReceive(0,ep,out.buffer,mps));
  complete(ep,false,mps-1); // Completion and the next DRDY run in the interrupt.
  assert(out.completed==1 && out.length==mps-1 && out.ready==2 && queue.empty());
  assert(s_Ctrlr.Xfer[ep][0].State==NRF54_XFER_ACTIVE);
  out.hold=true;complete(ep,false,0);drain();assert(out.completed==2);
  auto ready=out.ready;
  for(unsigned i=0;i<5;++i)UsbCtrlrProcess(0);
  assert(out.ready==ready && !processOwed); // Full RX FIFO does not self-poll.
  out.hold=false;assert(UsbCtrlrEpReceive(0,ep,out.buffer,mps));
  out.closeOnComplete=true;complete(ep,false,1);drain();
  assert(out.completed==3 && out.ready==ready && !s_Ctrlr.Xfer[ep][0].Open);
  auto &in=open(ep,true,USB_ENDPATT_TRANS_BULK,mps);
  assert(UsbCtrlrEpSend(0,ep,in.buffer,mps));
  const auto ptr=get(0x914+ep*32);
  assert(!UsbCtrlrEpSend(0,ep,in.buffer+4,4));assert(get(0x914+ep*32)==ptr);
  complete(ep,true,mps);assert(in.completed==1 && in.length==mps && queue.empty());
  for(unsigned offset=1;offset<4;++offset){
   assert(UsbCtrlrEpSend(0,ep,in.buffer+offset,7));
   assert((get(0x914+ep*32)&3U)==0);
   assert(s_Ctrlr.Xfer[ep][1].TotalLen==4-offset);
   complete(ep,true,4-offset);drain();assert(in.length==4-offset);
  }
  assert(UsbCtrlrEpSend(0,ep,nullptr,0));complete(ep,true,0);drain();assert(in.length==0);
 }
 // Endpoint completions do not use the work queue: a full queue delays nothing.
 init();auto &out=open(1,false,USB_ENDPATT_TRANS_BULK,64);auto &in=open(1,true,USB_ENDPATT_TRANS_BULK,64);
 queueLimit=0;assert(UsbCtrlrEpSend(0,1,in.buffer,64));
 complete(1,true,64);complete(1,false,23);
 assert(out.completed==1 && in.completed==1 && out.length==23 && queue.empty());
 // A reopened endpoint starts clean; a bus reset cancels its active transfer
 // without a callback.
 out.hold=true;complete(1,false,7);assert(out.completed==2);UsbCtrlrEpClose(0,1,false);
 auto &newOut=open(1,false,USB_ENDPATT_TRANS_BULK,64);assert(newOut.completed==0);
 add(0x014,1U<<12);irq();drain();assert(newOut.completed==0 && resetEvents==1);
 assert(s_Ctrlr.TxFifoWords[1]==0 && s_Ctrlr.FifoTop==3024);
 // FIFO extents are reclaimed repeatedly; a live FIFO never moves.
 init(true);open(2,true,USB_ENDPATT_TRANS_BULK,512);auto fixed=get(0x108);
 for(unsigned i=0;i<40;++i){open(1,true,USB_ENDPATT_TRANS_ISO,i%2?1024:17);UsbCtrlrEpClose(0,1,true);assert(get(0x108)==fixed);}
 assert(!UsbCtrlrEpOpenData(0,3,true,USB_ENDPATT_TRANS_ISO,0x1400));
 // Periodic IN needs MC=1. ISO uses next-frame parity and immediate callbacks.
 init(true);auto &isoIn=open(1,true,USB_ENDPATT_TRANS_ISO,1024);auto &isoOut=open(1,false,USB_ENDPATT_TRANS_ISO,1024);
 for(unsigned frame:{0U,1U,0x3FFFU}){
  set(0x808,frame<<8);assert(UsbCtrlrIsoSend(0,1,isoIn.buffer,19));
  assert((get(0x910+32)>>29)==1 && ((get(0x900+32)>>16)&1U)==((frame+1)&1U));
  complete(1,true,19);complete(1,false,17);
  assert(queue.empty());
 }
 assert(isoIn.completed==3 && isoOut.completed==3);
 isoOut.hold=true;assert(UsbCtrlrIsoSend(0,1,nullptr,0));assert(s_Ctrlr.Xfer[1][0].State==NRF54_XFER_IDLE);
 isoOut.hold=false;assert(UsbCtrlrIsoSend(0,1,isoIn.buffer,0));
 complete(1,true,0);complete(1,false,0);assert(isoIn.length==0 && isoOut.length==0);
 assert(UsbCtrlrIsoSend(0,1,isoIn.buffer,10));
 add(0x014,1U<<20);irq();irq();irq();assert(isoIn.failed==1);
 // OUT ISO expiry uses global OUT NAK and endpoint-disabled, then can receive again.
 set(0x808,((get(0xB20)>>16)&1U)<<8);add(0x014,1U<<21);irq();irq();irq();
 assert(isoOut.failed==1 && s_Ctrlr.Xfer[1][0].State==NRF54_XFER_IDLE);
 assert(UsbCtrlrIsoSend(0,1,isoIn.buffer,10));
 UsbCtrlrSofEnable(0,true);set(0x808,7U<<8);add(0x014,1U<<3);irq();assert(sofFrame==8);

 // Interrupt endpoints use the same completion path, with one periodic transaction.
 for(bool hs:{false,true}){
  init(hs);unsigned mps=hs?1024:64;
  auto &intr=open(1,true,USB_ENDPATT_TRANS_INT,mps);
  assert(UsbCtrlrEpSend(0,1,intr.buffer,mps));assert(get(0x930)>>29==1U);
  complete(1,true,mps);assert(intr.completed==1 && intr.length==mps);
  auto &rx=open(1,false,USB_ENDPATT_TRANS_INT,mps);
  UsbCtrlrEpStall(0,1,false);assert(get(0xB20)&(1U<<21));
  UsbCtrlrEpClearStall(0,1,false);assert(s_Ctrlr.Xfer[1][0].State==NRF54_XFER_ACTIVE);
  complete(1,false,1);drain();assert(rx.completed==1);
 }
 // A bad ISO OUT packet is dropped, including errors reported before XFRC.
 init(true);open(1,true,USB_ENDPATT_TRANS_ISO,17);auto &badOut=open(1,false,USB_ENDPATT_TRANS_ISO,17);
 assert(UsbCtrlrIsoSend(0,1,nullptr,0));add(0xB28,1U<<8);irq();
 complete(1,false,17);assert(badOut.completed==0 && badOut.failed==1);
 assert(UsbCtrlrIsoSend(0,1,nullptr,0));complete(1,false,9);assert(badOut.completed==1);
 // EP0 SETUP DMA may occupy any word in the same OUT data buffer.
 init();memset(ep0Reply,0xA5,sizeof(ep0Reply));setupAction=1;
 UsbSetupData_t packet={};packet.bmRequestType=0x80;packet.bRequest=6;packet.wLength=80;
 setup(8,packet);assert(setupEvents==1 && lastSetup.bRequest==6);
 assert(memcmp(s_Ep0In,ep0Reply,64)==0 && get(0x914)==uint32_t(uintptr_t(s_Ep0In)));
 packet.bRequest=8;setup(64,packet);assert(setupEvents==2 && lastSetup.bRequest==8);
 assert(memcmp(s_Ep0In,ep0Reply,64)==0); // SETUP cannot overwrite the IN bounce.
 complete(0,true,64);assert(ep0Completions==1);
 assert(UsbCtrlrEp0Status(0,0));add(0xB08,1U<<5);irq();complete(0,false,0);
 assert(ep0Completions==2);
 setupAction=0;packet.bmRequestType=0;packet.bRequest=0x40;packet.wLength=65;
 setup(16,packet);assert((get(0xB10)&0x7F)==64 && (get(0xB10)>>29)==3);
 set(0xB10,0);set(0xB00,get(0xB00)&~(1U<<31));add(0xB08,1);irq();
 assert(lastEp0Length==64 && (get(0xB10)&0x7F)==64);
 set(0xB10,63);set(0xB00,get(0xB00)&~(1U<<31));add(0xB08,1);irq();assert(lastEp0Length==1);
 setupAction=2;setup(0,packet);assert(s_Ctrlr.Xfer[0][0].State==NRF54_XFER_IDLE && (get(0xB00)&(1U<<21)));
 // DWC2 programs the new address before status IN is armed, not at completion.
 setupAction=0;packet.bRequest=5;packet.wValue=42;packet.wLength=0;setup(0,packet);
 const uint32_t addressConfig=get(0x800)&~0x7F0U;
 UsbCtrlrSetAddress(0,42);assert(get(0x800)==(addressConfig|(42U<<4)));
 assert(UsbCtrlrEp0Status(0,0x80));complete(0,true,0);
 assert(get(0x800)==(addressConfig|(42U<<4)) && (get(0xB00)&(1U<<31)));
 add(0x014,1U<<12);irq();assert((get(0x800)&0x7F0)==0);
 // Suspend preserves active requests. Wake ungates the PHY before use.
 for(bool low:{false,true}){
  init(false,low);auto &o=open(1,false,USB_ENDPATT_TRANS_BULK,64);
  add(0x014,1U<<11);irq();assert(s_Ctrlr.Suspended && bool(get(0xE00)&1U)==low);
  UsbCtrlrRemoteWakeup(0);assert((get(0xE00)&1U)==0 && s_Ctrlr.Suspended);
  add(0x014,1U<<31);irq();assert(!s_Ctrlr.Suspended);
  complete(1,false,20);drain();assert(o.length==20);
 }
 // Unplug stops DMA without reading the dead core; the active transfer gets no callback.
 init();auto &o=open(1,false,USB_ENDPATT_TRANS_BULK,64);complete(1,false,12);
 assert(o.completed==1 && s_Ctrlr.Xfer[1][0].State==NRF54_XFER_ACTIVE);
 powerRegs[1]=1;powerRegs[0x400/4]=0;VREGUSB_IRQHandler();assert(!s_Ctrlr.Started && wrapper.ENABLE==0);
 assert(!irqEnabled[USBHS_IRQn]);
 drain();assert(o.completed==1 && resetEvents==1);UsbCtrlrEpCloseAll(0);
 powerRegs[0]=1;powerRegs[0x400/4]=4;VREGUSB_IRQHandler();drain();assert(s_Ctrlr.Started && !s_UsbdRestart);
 powerRegs[0]=powerRegs[1]=1;powerRegs[0x400/4]=4;VREGUSB_IRQHandler();drain();
 assert(s_Ctrlr.Started && resetEvents==2);
 UsbCtrlrStop(0);UsbCtrlrIntEnable(0);UsbCtrlrDisconnect(0);UsbCtrlrEpClearStall(0,1,false);
 assert(!irqEnabled[USBHS_IRQn] && !(get(0x008)&1U));
 assert(!UsbCtrlrEpSend(0,1,o.buffer,1));assert(!UsbCtrlrEpReceive(0,16,o.buffer,64));
 char serial[17];assert(UsbCtrlrGetSerial(0,serial,sizeof(serial))==16 && !strcmp(serial,"0123456789ABCDEF"));

 // Exercise the actual generic byte FIFO implementation against this port.
 init();
 alignas(4) static uint8_t rxMem[USB_INTRF_RXMEM_SIZE(2,64)];
 alignas(4) static uint8_t txMem[CFIFO_MEMSIZE(256)];
 UsbDevIntrf_t data={};UsbIntrfCfg_t cfg={};
 cfg.DevNo=0;cfg.EpNo=3;cfg.bBlocking=true;cfg.Mode=USB_INTRF_MODE_BYTE;
 cfg.RxFifoMemSize=sizeof(rxMem);cfg.pRxFifoMem=rxMem;
 cfg.TxFifoBlkSize=1;cfg.TxFifoMemSize=sizeof(txMem);cfg.pTxFifoMem=txMem;cfg.BufferSize=64;
 assert(UsbIntrfInit(&data,&cfg));assert(UsbIntrfConfigure(&data,64));
 assert(UsbCtrlrEpOpenData(0,3,true,USB_ENDPATT_TRANS_BULK,64));
 assert(UsbCtrlrEpOpenData(0,3,false,USB_ENDPATT_TRANS_BULK,64));
 auto &rx=s_Ctrlr.Xfer[3][0];
 memset(rx.pBuffer,0x31,64);complete(3,false,64);drain();
 memset(rx.pBuffer,0x32,64);complete(3,false,64);drain();
 assert(rx.State==NRF54_XFER_IDLE && data.RxPending);
 uint8_t received[64];
 assert(DeviceIntrfRxData(&data.DevIntrf,received,64)==64 && received[0]==0x31);
 assert(rx.State==NRF54_XFER_ACTIVE && !data.RxPending);
 queueLimit=0;memset(rx.pBuffer,0x33,17);complete(3,false,17);
 assert(CFifoUsed(data.hRxFifo)==2);queueLimit=1;drain();
 assert(DeviceIntrfRxData(&data.DevIntrf,received,64)==64 && received[0]==0x32);
 assert(DeviceIntrfRxData(&data.DevIntrf,received,64)==17 && received[0]==0x33);
 static uint8_t tx[67];memset(tx,0x77,sizeof(tx));
 assert(DeviceIntrfTxData(&data.DevIntrf,tx,67)==67);
 assert(s_Ctrlr.Xfer[3][1].TotalLen==64);complete(3,true,64);drain();
 assert(s_Ctrlr.Xfer[3][1].TotalLen==3);complete(3,true,3);drain();assert(CFifoUsed(data.hTxFifo)==0);
 UsbCtrlrEpCloseAll(0);UsbIntrfUnconfigure(&data);
 // Actual ISO FIFO callbacks run once per interval, including missed IN frames.
 init(true);UsbIsoIntrf_t iso={};UsbDevIntrf_t isoData={};
 alignas(4) static uint8_t isoRx[USB_ISO_INTRF_FIFO_MEMSIZE(1024)];
 alignas(4) static uint8_t isoTx[USB_ISO_INTRF_FIFO_MEMSIZE(1024)];
 UsbIsoIntrfCfg_t isoCfg={};isoCfg.DevNo=0;isoCfg.EpNo=4;isoCfg.BufferSize=1024;
 isoCfg.pRxFifoMem=isoRx;isoCfg.pTxFifoMem=isoTx;
 assert(UsbIsoIntrfInit(&iso,&isoData,&isoCfg));assert(UsbIsoIntrfOpen(&iso,1024,4));
 assert(UsbIsoIntrfSendFrame(&iso,tx,67));
 UsbCtrlrEpProcessEvent(0,4,true,USB_CTRLR_EVT_SOF,7);
 assert(s_Ctrlr.Xfer[4][1].State==NRF54_XFER_IDLE);
 UsbCtrlrEpProcessEvent(0,4,true,USB_CTRLR_EVT_SOF,8);
 assert(s_Ctrlr.Xfer[4][1].State==NRF54_XFER_ACTIVE);
 memset(s_Ctrlr.Xfer[4][0].pBuffer,0x55,31);complete(4,false,31);complete(4,true,67);
 assert(DeviceIntrfRxData(&isoData.DevIntrf,received,64)==31 && received[0]==0x55);
 assert(CFifoUsed(isoData.hTxFifo)==0 && queue.empty());
 assert(UsbIsoIntrfSendFrame(&iso,tx,10));
 UsbCtrlrEpProcessEvent(0,4,true,USB_CTRLR_EVT_SOF,16);
 add(0x014,1U<<20);irq();irq();irq();
 assert(iso.TxMissCnt==1 && CFifoUsed(isoData.hTxFifo)==0);
 UsbIsoIntrfClose(&iso);
 puts("PASS: complete nRF54 USBHS port: FS/HS, DMA ownership, completions in the interrupt, FIFO reuse, EP0, ISO, power lifecycle");
}
'''

with tempfile.TemporaryDirectory(prefix='iosonata-nrf54-usb-') as tmp:
    tmp = Path(tmp)
    for name in ('nrf.h', 'nrf_peripherals.h', 'hal/nrf_clock.h', 'hal/nrf_vregusb.h',
                 'hal/nrf_ficr.h', 'lib/nrfx_coredep.h'):
        path = tmp / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(NRF if name == 'nrf.h' else '#include "nrf.h"\n')
    path = tmp / 'test.cpp'
    binary = tmp / 'test'
    path.write_text(MODEL + '\n' + source + '\n' + TEST)
    # DeviceIntrf uses GNU C++ variable-length arrays; keep that extension enabled.
    # Non-PIE keeps static DMA pointers representable in the MCU's 32-bit registers.
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=gnu++17', '-O1', '-g',
                    '-x', 'c++', '-Wall', '-Wextra', '-Werror', '-Wno-missing-field-initializers', '-Wno-vla',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all', '-no-pie',
                    '-I' + str(tmp), '-I' + str(ROOT / 'include'),
                    '-I' + str(ROOT / 'ARM/Nordic/include'),
                    str(path), str(ROOT / 'src/usb/usb_intrf.cpp'),
                    str(ROOT / 'src/usb/usb_iso.cpp'),
                    str(ROOT / 'src/device_intrf.cpp'), str(ROOT / 'src/cfifo.c'),
                    '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
