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
#include "usb_ctrlr_stm32_test.cpp"

alignas(4) static uint8_t registers[0x12000];
static unsigned completed, cancelled, ready, failed, resets;
static uint16_t lastLength;
static bool rebind;
static uint8_t receive[1024];
static UsbCtrlrEvt_t control;
#if defined(STM32H753xx)
static SysLog_t logState;
SysLog_t *SysLogGet() { return &logState; }
int SysLogPrintf(SysLog_t *, const char *, ...) { return 0; }
bool AppEvtHandlerQue(uint32_t, void *, AppEvtHandler_t) { return false; }
#endif
bool Stm32UsbOtgPowerInit(const UsbCtrlrCfg_t *) { return true; }
void Stm32UsbOtgReset() { resets++; }
USB_OTG_GlobalTypeDef *Stm32UsbOtgRegs()
{
	return reinterpret_cast<USB_OTG_GlobalTypeDef *>(registers);
}
IRQn_Type Stm32UsbOtgIrq() {
#if defined(STM32H753xx)
	return OTG_FS_IRQn;
#else
	return USB_OTG_HS_IRQn;
#endif
}
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
	constexpr bool highSpeed = USB_HIGHSPEED_CAPABLE_0;
	constexpr unsigned bulkMps = USB_CTRLR0_BULK_PKT_LEN_MAX;
	s_Usb.HighSpeed = highSpeed;
	Device()->DSTS = highSpeed ? 0 : USB_OTG_DSTS_ENUMSPD;
	s_Usb.pReg->GINTMSK = USB_OTG_GINTSTS_ENUMDNE;
	s_Usb.pReg->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
	Stm32UsbOtgInterrupt();
	assert((Device()->DCTL & USB_OTG_DCTL_CGINAK) != 0);
	assert((s_Usb.pReg->GUSBCFG & USB_OTG_GUSBCFG_TRDT) ==
		((highSpeed ? 9U : 6U) << USB_OTG_GUSBCFG_TRDT_Pos));
	assert(GrowRx(1024));
	assert(AllocateTx(0, 64));
	assert(UsbCtrlrEpOpenData(0, 1, true, BULK, bulkMps));
	assert(UsbCtrlrEpOpenData(0, 1, false, BULK, bulkMps));
	assert(UsbCtrlrEpOpenData(0, 2, true, INT, 16));
	assert(!UsbCtrlrEpOpenData(0, 3, true, ISO, 1024));
	if (highSpeed)
		assert(!UsbCtrlrEpOpenData(0, 3, true, BULK, 64));
	else
	{
		assert(!UsbCtrlrEpOpenData(0, 3, true, BULK, 512));
		assert(UsbCtrlrEpOpenData(0, 8, true, BULK, 64));
		assert(!UsbCtrlrEpOpenData(0, 9, true, BULK, 64));
	}
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
	assert(!UsbCtrlrEpReceive(0, 1, receive, bulkMps - 1));
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
	// Exercise word copies, every tail and unaligned buffers, with canaries.
	for (unsigned offset = 0; offset < 4; offset++)
	{
		for (unsigned length = 0; length <= bulkMps; length++)
		{
			uint8_t *buffer = new uint8_t[length + offset];
			for (unsigned i = 0; i < length; i++) buffer[offset + i] = i + 1;
			assert(UsbCtrlrEpSend(0, 1, buffer + offset, length));
			if (length != 0)
			{
				unsigned start = (length - 1) & ~3U;
				uint32_t expected = 0;
				for (unsigned i = start; i < length; i++)
					expected |= uint32_t(buffer[offset + i]) << (8 * (i - start));
				assert(*Fifo(1) == expected);
			}
			Complete(1, true, false);
			delete[] buffer;
		}
		for (unsigned length = 0; length <= 69; length++)
		{
			for (unsigned capacity = 0; capacity <= 69; capacity++)
			{
				uint8_t buffer[80];
				memset(buffer, 0xEE, sizeof(buffer));
				*Fifo(0) = 0x44332211;
				ReadFifo(buffer + offset, capacity, length);
				unsigned copied = capacity < length ? capacity : length;
				for (unsigned i = 0; i < sizeof(buffer); i++)
					assert(buffer[i] == (i >= offset && i < offset + copied ?
						0x11 * (1 + (i - offset) % 4) : 0xEE));
			}
		}
	}
	ReadFifo(nullptr, 64, 65);
	// Sparse dispatch includes the highest endpoint and software-only DRDY.
	s_Usb.pReg->GINTMSK = 0;
	s_Usb.pReg->GINTSTS = 0;
	const unsigned lastEp = USB_EP_COUNT - 1;
	s_Usb.Ep[lastEp][1].Busy = true;
	s_Usb.Ep[lastEp][1].Length = 1;
	Device()->DAINT = 1U << lastEp;
	Device()->DAINTMSK = 0;
	Device()->DIEPMSK = USB_OTG_DIEPINT_XFRC;
	In(lastEp)->DIEPINT = USB_OTG_DIEPINT_XFRC;
	s_Usb.RxReady = 0;
	Stm32UsbOtgInterrupt();
	assert(s_Usb.Ep[lastEp][1].Busy);
	Device()->DAINTMSK = 1U << lastEp;
	Stm32UsbOtgInterrupt();
	assert(!s_Usb.Ep[lastEp][1].Busy);
	Device()->DAINT = 0;
	s_Usb.RxReady = 1U << 1;
	const unsigned previousReady = ready;
	Stm32UsbOtgInterrupt();
	assert(ready == previousReady + 1 && s_Usb.RxReady == 0);
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

for device, family, cpu in (('STM32WBA65xx', 'STM32WBAxx', 'core_cm33.h'),
                            ('STM32H753xx', 'STM32H7xx', 'core_cm7.h')):
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp)
        (path / cpu).write_text(CPU)
        (path / 'idelay.h').write_text('#pragma once\ninline void usDelay(unsigned) {}\n')
        # Isolate FIFO ownership from physical clocks/pins in this host test.
        # The complete merged source is compiled by stm32_otg_build_test.py;
        # MCU initialization is exercised by the family register tests.
        source = (ROOT / 'ARM/ST/src/usb_ctrlr_stm32.cpp').read_text()
        begin = source.index('// MCU clock, power, PHY and interrupt setup.')
        end = source.index('// Endpoint state and FIFO transfers.')
        hooks = """extern bool Stm32UsbOtgPowerInit(const UsbCtrlrCfg_t *pCfg);
extern void Stm32UsbOtgReset(void);
extern USB_OTG_GlobalTypeDef *Stm32UsbOtgRegs(void);
extern IRQn_Type Stm32UsbOtgIrq(void);
extern uint16_t Stm32UsbOtgFifoWords(void);

"""
        (path / 'usb_ctrlr_stm32_test.cpp').write_text(source[:begin] + hooks + source[end:])
        (path / 'test.cpp').write_text(TEST)
        cmd = [os.environ.get('CXX', 'g++'), '-std=c++17', '-g',
               '-fsanitize=address,undefined', '-D' + device,
               '-I' + tmp, '-I' + str(ROOT), '-I' + str(ROOT / 'include'),
               '-I' + str(ROOT / 'ARM/ST/include'),
               '-I' + str(ROOT / 'ARM/ST' / family / 'include'),
               str(path / 'test.cpp'), '-o', str(path / 'test')]
        subprocess.run(cmd, check=True)
        subprocess.run([str(path / 'test')], check=True)


