#!/usr/bin/env python3
"""Exercise production nRF52 FIFO queueing and UsbIntrf with simulated DMA.

Runs the actual queue, DMA retirement, completion and generic TX functions.
The register model separates ENDEP from host consumption and rejects reloads
of an occupied endpoint. It does not model USB electrical timing.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / "ARM/Nordic/nRF52/src/usb_ctrlr_nrf52.cpp").read_text()
header = (ROOT / "ARM/Nordic/include/usb_ctrlr.h").read_text()


def function(name):
    match = re.search(r"(?:void|bool|uint8_t|nRFUsbEpReg_t \*)\s*" + name +
                      r"\([^;{}]*\)\s*\{", source)
    assert match, name
    end = source.index("{", match.start()) + 1
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


queue_types = source[source.index("#define NRFUSBD_QUE_DEPTH"):
                     source.index("#pragma pack(pop)") + len("#pragma pack(pop)")]
flag_enum = re.search(r"enum\s*\{[^}]*USBD_FLAG_ISO_IN_CMPL[^}]*\};", header).group(0)
queue_mem = source[source.index("alignas(4) static uint8_t s_QueMem"):
                   source.index("nRFUsbdState_t s_Usbd;")]
code = r'''
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "usb/usb_intrf.h"
#include "coredev/interrupt.h"
#include "app_evt_handler.h"
constexpr unsigned NRFX_USBD_MAX_PACKET_SIZE = 64;
constexpr unsigned NRFX_USBD_EASYDMA_BUSY_REG_BUSY = 0x82;
constexpr unsigned NRFX_USBD_EASYDMA_BUSY_REG_CLEAR = 0;
unsigned dmaBusy;
#define NRFX_USBD_EASYDMA_BUSY_REG dmaBusy
struct W1C {
    uint32_t bits = 0;
    operator uint32_t() const { return bits; }
    void operator=(uint32_t value) { bits &= ~value; }
};
struct Endpoint { uint32_t PTR = 0, MAXCNT = 0, AMOUNT = 0; };
using USBD_EPIN_Type = Endpoint;
using USBD_EPOUT_Type = Endpoint;
struct Registers {
    Endpoint EPIN[8], EPOUT[8];
    uint32_t EVENTS_ENDEPIN[8] = {}, EVENTS_ENDEPOUT[8] = {};
    uint32_t TASKS_STARTEPIN[8] = {}, TASKS_STARTEPOUT[8] = {};
    struct { uint32_t EPOUT[8] = {}; } SIZE;
    W1C EPSTATUS, EPDATASTATUS;
} regs;
auto *NRF_USBD = &regs;
struct nRFUsbEpReg_t {
    uint8_t *pBuffer;
    UsbCtrlrEpHandler_t Handler;
    void *pContext;
    uint16_t MaxPacketSize;
    bool bBlocking;
};
struct {
    volatile uint32_t Flags;
    hCFifo_t hQue, hEp0Que;
    nRFUsbEpReg_t EpReg[9][2];
    uint8_t Ep0Bounce[64];
} s_Usbd;
bool (*nRFUsbdIsoStart)() = nullptr;
bool occupied[8];
uint8_t packets[8][64];
unsigned starts[8], completions, callbacks;
unsigned activeEp;
std::vector<uint8_t> wire[8];
void __DSB() {
    for (unsigned ep = 1; ep < 8; ++ep) {
        if (!regs.TASKS_STARTEPIN[ep]) continue;
        assert(dmaBusy && regs.EPSTATUS.bits == 0 && !occupied[ep]);
        const auto &dma = regs.EPIN[ep];
        assert((dma.PTR & 3U) == 0 && dma.MAXCNT <= 64);
        memcpy(packets[ep], reinterpret_cast<const void *>(uintptr_t(dma.PTR)), dma.MAXCNT);
        occupied[ep] = true;
        activeEp = ep;
        ++starts[ep];
        regs.EPSTATUS.bits = 1U << ep;
        regs.TASKS_STARTEPIN[ep] = 0;
    }
}
extern "C" bool UsbCtrlrHighSpeed(int) { return false; }
extern "C" bool UsbCtrlrEpXfer(int, uint8_t, uint16_t) { return false; }
'''
code += queue_types + "\n" + flag_enum + "\n" + queue_mem
names = ["nRFUsbEpDir", "nRFUsbGetEpReg", "nRFUsbEpRegisteredEvent",
         "nRFUsbdDmaActive", "nRFUsbdDmaUnlock", "nRFUsbdRetireDma",
         "nRFUsbdStartDmaNow", "nRFUsbdStartQueuedDma", "nRFUsbdResumeQueuedDmaLocked",
         "nRFUsbdQueInFifo", "nRFUsbdProcessInComplete", "nRFUsbdQueueInComplete",
         "UsbCtrlrEpInXfer", "UsbCtrlrEpAlloc"]
code += "\n".join(function(name) for name in names)
code += r'''
alignas(4) static uint8_t txMem[CFIFO_MEMSIZE(256)];
alignas(4) static uint8_t rxMem[USB_INTRF_RXMEM_SIZE(4, 64)];
alignas(4) static uint8_t rxBuffer[64], txBuffer[64];
UsbDevIntrf_t intrf;
UsbCtrlrEpHandler_t genericIn;
void complete(uint8_t ep, UsbCtrlrEvtType_t event, uint16_t length,
              UsbCtrlrXferResult_t result, void *context) {
    ++callbacks;
    genericIn(ep, event, length, result, context);
}
void init(unsigned offset, unsigned capacity = 256, unsigned mps = 64) {
    regs = {};
    s_Usbd = {};
    dmaBusy = completions = callbacks = 0;
    memset(occupied, 0, sizeof(occupied));
    memset(starts, 0, sizeof(starts));
    for (auto &v : wire) v.clear();
    s_Usbd.hQue = CFifoInit(s_QueMem, sizeof(s_QueMem), sizeof(nRFUsbdQue_t), false);
    s_Usbd.hEp0Que = CFifoInit(s_Ep0QueMem, sizeof(s_Ep0QueMem), sizeof(nRFEPPkt_t), true);
    assert(AppEvtHandlerInit(nullptr, 0));
    UsbIntrfCfg_t cfg = {};
    cfg.EpNo = 1;
    cfg.bBlocking = true;
    cfg.pTxFifoMem = txMem;
    cfg.TxFifoMemSize = CFIFO_MEMSIZE(capacity);
    cfg.TxFifoBlkSize = 1;
    cfg.pRxFifoMem = rxMem;
    cfg.RxFifoMemSize = sizeof(rxMem);
    cfg.BufferSize = 64;
    cfg.pRxBuffer = rxBuffer;
    cfg.pTxBuffer = txBuffer;
    assert(UsbIntrfInit(&intrf, &cfg) && UsbIntrfConfigure(&intrf, mps));
    s_Usbd.EpReg[1][1].MaxPacketSize = mps;
    genericIn = s_Usbd.EpReg[1][1].Handler;
    s_Usbd.EpReg[1][1].Handler = complete;
    for (unsigned i = 0; i < offset; ++i) {
        auto *p = CFifoPut(intrf.hTxFifo);
        assert(p); *p = 0;
        assert(CFifoGet(intrf.hTxFifo) == p);
    }
}
void endDma() {
    assert(dmaBusy);
    const auto ep = activeEp;
    regs.EPIN[ep].AMOUNT = regs.EPIN[ep].MAXCNT;
    regs.EVENTS_ENDEPIN[ep] = 1;
    assert(nRFUsbdRetireDma(ep));
    assert(!dmaBusy && regs.EPSTATUS.bits == 0);
    nRFUsbdStartQueuedDma();
}
void ack(unsigned ep = 1) {
    assert(occupied[ep]);
    const auto len = regs.EPIN[ep].AMOUNT;
    wire[ep].insert(wire[ep].end(), packets[ep], packets[ep] + len);
    occupied[ep] = false;
    regs.EPDATASTATUS.bits |= 1U << ep;
    nRFUsbdQueueInComplete(ep, len);
    regs.EPDATASTATUS = 1U << ep;
    nRFUsbdResumeQueuedDmaLocked();
    ++completions;
}
void drain() {
    for (unsigned n = 0; n < 1024; ++n) {
        if (dmaBusy) endDma();
        if (occupied[1]) ack();
        AppEvtHandlerExec();
        if (atomic_load(&intrf.DevIntrf.bTxReady)) return;
    }
    assert(false && "TX did not drain");
}
void submit(const std::vector<uint8_t> &data) {
    assert(DeviceIntrfTxData(&intrf.DevIntrf, data.data(), data.size()) == int(data.size()));
}
int main() {
    std::vector<uint8_t> data(100);
    for (unsigned i = 0; i < data.size(); ++i) data[i] = uint8_t(i + 1);
    for (unsigned offset = 0; offset < 4; ++offset) {
        init(offset);
        submit(data);
        const unsigned repair = offset ? 4 - offset : 0;
        assert(CFifoUsed(s_Usbd.hQue) == (offset ? 2 : 1));
        assert(CFifoUsed(intrf.hTxFifo) == int(data.size() - repair));
        auto *first = reinterpret_cast<nRFUsbdQue_t *>(CFifoPeek(s_Usbd.hQue));
        assert(first && first->Dir == (offset ? NRFX_USBD_QUE_IN_SCRATCH : NRFX_USBD_QUE_IN_FIFO));
        endDma();
        assert(!dmaBusy && starts[1] == 1);
        assert(CFifoUsed(s_Usbd.hQue) == (offset ? 1 : 0));
        if (offset) {
            auto *second = reinterpret_cast<nRFUsbdQue_t *>(CFifoPeek(s_Usbd.hQue));
            assert(second && second->Dir == NRFX_USBD_QUE_IN_FIFO && second->Len == 64);
            assert((uintptr_t(CFifoPeek(second->hFifo)) & 3) == 0);
            ack();
            assert(dmaBusy && starts[1] == 2 && callbacks == 0);
            AppEvtHandlerExec();
            assert(callbacks == 0 && CFifoUsed(intrf.hTxFifo) == int(data.size() - repair));
        }
        drain();
        assert(wire[1] == data && CFifoUsed(intrf.hTxFifo) == 0);
    }
    puts("PASS: aligned and all unaligned heads; immediate FIFO continuation without AppEvt");

    for (unsigned capacity : {255U, 256U})
    for (unsigned offset : {1U, 2U, 3U, 253U, 254U})
    for (unsigned length : {1U, 2U, 3U, 4U, 5U, 63U, 64U, 100U}) {
        init(offset, capacity);
        std::vector<uint8_t> bytes(data.begin(), data.begin() + length);
        submit(bytes);
        drain();
        assert(wire[1] == bytes && CFifoUsed(intrf.hTxFifo) == 0);
    }
    init(1, 256, 1);
    submit(data);
    drain();
    assert(wire[1] == data);
    puts("PASS: short prefixes, physical wrap, odd FIFO capacity and one-byte MPS");

    init(1);
    // Capacity one forces only the optional second insertion to fail.
    s_Usbd.hQue = CFifoInit(s_QueMem, CFIFO_TOTAL_MEMSIZE(1, sizeof(nRFUsbdQue_t)),
                            sizeof(nRFUsbdQue_t), false);
    submit(data);
    assert(CFifoUsed(s_Usbd.hQue) == 1 && CFifoUsed(intrf.hTxFifo) == 97);
    assert(s_Usbd.hQue->DropCnt == 0);
    endDma();
    ack();
    assert(!dmaBusy && CFifoUsed(intrf.hTxFifo) == 97);
    AppEvtHandlerExec();
    assert(callbacks == 1 && dmaBusy && regs.EPIN[1].MAXCNT == 64);
    drain();
    assert(wire[1] == data && CFifoUsed(intrf.hTxFifo) == 0);
    puts("PASS: full second slot leaves FIFO data queued and retries on the next turn");
}
'''
# The existing host shim discards IN pointers. Use the production directional
# declaration so the real controller implementation can receive the TX FIFO.
port = (ROOT / "tests/usb/hostport/usb_ctrlr.h").read_text()
start = port.index("static inline bool UsbCtrlrEpInXfer(")
end = port.index("static inline bool UsbCtrlrEpOutXfer(", start)
port = port[:start] + "bool UsbCtrlrEpInXfer(int, uint8_t, uint8_t *, uint16_t);\n\n" + port[end:]
with tempfile.TemporaryDirectory(prefix="iosonata-fifo-queue-") as temp:
    temp = Path(temp)
    (temp / "usb_ctrlr.h").write_text(port)
    (temp / "queue_test.cpp").write_text(code)
    target = temp / "queue_test"
    subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-O1",
                    "-Wall", "-Wextra", "-Wno-missing-field-initializers",
                    "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
                    "-fno-pie", "-no-pie", "-DNRF52_SERIES", "-I" + str(temp),
                    "-I" + str(ROOT / "include"), "-x", "c++",
                    str(temp / "queue_test.cpp"), str(ROOT / "src/usb/usb_intrf.cpp"),
                    str(ROOT / "src/cfifo.c"), str(ROOT / "src/app_evt_handler.cpp"),
                    "-o", str(target)], check=True)
    env = dict(os.environ, ASAN_OPTIONS="detect_leaks=0", UBSAN_OPTIONS="halt_on_error=1")
    subprocess.run([str(target)], check=True, env=env)
