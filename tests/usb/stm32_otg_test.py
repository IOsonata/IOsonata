#!/usr/bin/env python3
"""Exercise STM32 OTG buffer ownership with ST's actual register structures.

The CPU intrinsics are mocked. Register storage has no hardware side effects:
this tests the software transfer boundaries, not enumeration or USB timing.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CPU = r'''
#pragma once
#include <stdint.h>
#define __IO volatile
#define __I volatile const
#define __IM volatile const
#define __O volatile
inline void NVIC_DisableIRQ(int) {}
inline void NVIC_EnableIRQ(int) {}
inline void NVIC_SetPriority(int, int) {}
inline void NVIC_ClearPendingIRQ(int) {}
inline void NVIC_SetPendingIRQ(int) {}
'''
TEST = r'''
#include <cassert>
#include <cstdio>
#include <cstring>
#include "ARM/ST/src/usb_otg_stm32.cpp"

alignas(4) static uint8_t registers[0x12000];
static unsigned completed, cancelled, ready, failed, resets;
static uint16_t lastLength;
static bool rebind;
static uint8_t receive[1024];
static UsbCtrlrEvt_t control;
bool Stm32UsbOtgPowerInit(const UsbCtrlrCfg_t *) { return true; }
void Stm32UsbOtgReset() { resets++; }
USB_OTG_GlobalTypeDef *Stm32UsbOtgRegs()
{
	return reinterpret_cast<USB_OTG_GlobalTypeDef *>(registers);
}
IRQn_Type Stm32UsbOtgIrq() { return USB_OTG_HS_IRQn; }
uint16_t Stm32UsbOtgFifoWords() { return 1024; }
void UsbProcessQue(int) {}
void UsbDevProcessEvent(int, const UsbCtrlrEvt_t *pEvent) { control = *pEvent; }
extern "C" uint32_t SystemCoreClockGet() { return 96000000; }

static void Handler(UsbCtrlrEvtType_t Event, uint16_t Length, void *)
{
	lastLength = Length;
	if (Event == USB_CTRLR_EVT_XFER_CMPL)
	{
		completed++;
		assert(!s_Usb.Ep[1][0].Busy);
		assert(s_Usb.Ep[1][0].pBuffer == nullptr);
		if (rebind)
		{
			UsbCtrlrEpBind(0, 1, false, false, nullptr, nullptr);
		}
	}
	if (Event == USB_CTRLR_EVT_CANCEL) { cancelled++; }
	if (Event == USB_CTRLR_EVT_DRDY) { ready++; }
	if (Event == USB_CTRLR_EVT_XFER_FAILED) { failed++; }
}

int main()
{
	UsbCtrlrCfg_t cfg = {};
	assert(UsbCtrlrInit(0, &cfg));
	s_Usb.Started = true;
	s_Usb.HighSpeed = true;
	s_Usb.pReg->GINTMSK = USB_OTG_GINTSTS_ENUMDNE;
	s_Usb.pReg->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
	Stm32UsbOtgInterrupt();
	assert((Device()->DCTL & USB_OTG_DCTL_CGINAK) != 0);
	assert((s_Usb.pReg->GUSBCFG & USB_OTG_GUSBCFG_TRDT) ==
		(9U << USB_OTG_GUSBCFG_TRDT_Pos));
	assert(GrowRx(1024));
	assert(AllocateTx(0, 64));
	assert(UsbCtrlrEpOpenData(0, 1, true, BULK, 512));
	assert(UsbCtrlrEpOpenData(0, 1, false, BULK, 512));
	assert(UsbCtrlrEpOpenData(0, 2, true, INT, 16));
	assert(!UsbCtrlrEpOpenData(0, 3, true, ISO, 1024));
	assert(!UsbCtrlrEpOpenData(0, 3, true, BULK, 64));
	for (unsigned ep = 0; ep < USB_EP_COUNT; ep++)
	{
		if (s_Usb.TxWords[ep] != 0)
		{
			assert(s_Usb.TxBase[ep] >= s_Usb.RxWords);
			assert(s_Usb.TxBase[ep] + s_Usb.TxWords[ep] <= 1024);
		}
	}
	// OUT accepts a full-packet buffer and rejects a second owner.
	UsbCtrlrEpBind(0, 1, false, false, Handler, nullptr);
	assert(!UsbCtrlrEpReceive(0, 1, receive, 511));
	assert(UsbCtrlrEpReceive(0, 1, receive, sizeof(receive)));
	assert(!UsbCtrlrEpReceive(0, 1, receive, sizeof(receive)));
	*Fifo(0) = 0x00CCBBAA;
	s_Usb.pReg->GRXSTSP = 1 | (3 << 4) | (2 << 17);
	ReceiveFifo();
	assert(receive[0] == 0xAA && receive[1] == 0xBB && receive[2] == 0xCC);
	Complete(1, false, false);
	assert(completed == 1 && ready == 1);
	// A callback may unbind itself; stale DRDY must not reach its replacement.
	assert(UsbCtrlrEpReceive(0, 1, receive, sizeof(receive)));
	rebind = true;
	Complete(1, false, false);
	assert(completed == 2 && ready == 1);
	UsbCtrlrEpBind(0, 1, false, false, Handler, nullptr);
	assert(UsbCtrlrEpReceive(0, 1, receive, sizeof(receive)));
	Forget(1, false);
	Forget(1, false);
	assert(cancelled == 1);
	// Unaligned IN sources are copied without reading outside their length.
	uint8_t bytes[] = {0, 1, 2, 3};
	assert(UsbCtrlrEpSend(0, 1, bytes + 1, 3));
	assert(*Fifo(1) == 0x00030201);
	assert(!UsbCtrlrEpSend(0, 1, bytes, 4));
	Complete(1, true, false);
	assert(UsbCtrlrEpSend(0, 1, nullptr, 0));
	assert(In(1)->DIEPTSIZ == (1U << USB_OTG_DIEPTSIZ_PKTCNT_Pos));
	Complete(1, true, false);
	// EP0 copies stack replies before returning and rejects nonzero status OUT.
	s_Usb.Ep[0][1].Mps = 64;
	assert(UsbCtrlrEp0Send(0, bytes + 1, 3) == 3);
	assert(s_Usb.Ep[0][1].pBuffer == nullptr && *Fifo(0) == 0x00030201);
	s_Usb.Ep[0][0].Busy = true;
	s_Usb.Ep[0][0].Actual = 1;
	s_Usb.ControlRemaining = 0;
	Complete(0, false, false);
	assert(control.Xfer.Result == USB_CTRLR_XFER_FAILED);
	// A peripheral that never completes a flush must fail, not spin forever.
	assert(!FlushTx(1));
	Fault();
	assert(!s_Usb.Started && s_Usb.Recover && resets == 1);
	assert(!UsbCtrlrIsoInit(0));
	puts("STM32 OTG software ownership tests PASS");
}
'''

with tempfile.TemporaryDirectory() as tmp:
    path = Path(tmp)
    (path / 'core_cm33.h').write_text(CPU)
    (path / 'idelay.h').write_text('#pragma once\ninline void usDelay(unsigned) {}\n')
    (path / 'test.cpp').write_text(TEST)
    cmd = [os.environ.get('CXX', 'g++'), '-std=c++17', '-g',
           '-fsanitize=address,undefined', '-DSTM32WBA65xx',
           '-I' + tmp, '-I' + str(ROOT), '-I' + str(ROOT / 'include'),
           '-I' + str(ROOT / 'ARM/ST/include'),
           '-I' + str(ROOT / 'ARM/ST/STM32WBAxx/include'),
           str(path / 'test.cpp'), '-o', str(path / 'test')]
    subprocess.run(cmd, check=True)
    subprocess.run([str(path / 'test')], check=True)
