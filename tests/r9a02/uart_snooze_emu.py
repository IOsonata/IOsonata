#!/usr/bin/env python3
# R9A02G021 UART reception in Snooze on an emulator (uart_r9a02.cpp, UART0 =
# SAU0 channels 0 and 1), with uart_snooze_fixture.cpp built by
# build_uart_snooze_emu.py.
#
# The script sends the characters. It models what the manual describes:
#   - SAU UART transmit channel: SDR goes to the shift register when it is
#     empty, with the buffer empty event when SMR.MD0 = 1. With MD0 = 0,
#     transfer end raises the event instead. BFF while SDR waits, TSF while
#     a character is sent. Stopping the channel loses both, starting it clears
#     BFF. A character ends each time the firmware polls SSR outside an
#     interrupt, or when the harness waits; interrupts pending are taken at
#     a poll when the firmware has them enabled.
#   - SAU UART receive channel: a character sets BFF, reading SDR clears it.
#     In Snooze (SSCm.SWC = 1, 23.6.3) the character is in SDR at the end of
#     the reception, its end goes to SELSR0; the error flags are not set
#     with SSEC = 1. BFF is unchanged with SWC = 1 as for the simplified SPI
#     (23.5.7), or set (SNZ_BFF): the driver relies on neither.
#   - Snooze settings of table 23.95 checked when a character comes in
#     Snooze: fMCK = PCLKB / 2^5 (4800) or 2^4 (9600), STCLK 106 (HOCO
#     32 MHz) or 79 (24 MHz), SSEC = 1.
#   - ICU IELSR and the Software Standby machinery as in spi_slave_emu.py.
#     SNZ_CANCEL_NORMAL: the SELSR0 event also reaches ICU_SNZCANCEL in
#     Normal mode (12.2.10 does not say).
#   - TX idle level: POSR/PORR update the PFS output latch (16.2.3), ST
#     retains SO (23.3.15), and SO writes are ignored while SOE is set
#     (23.3.19). A character starts low and ends high; individual data
#     bits and electrical behavior are not modeled.
#
# Needs unicorn 2.1 and pyelftools, firmware without Zba, Zbb and Zbs.
#
# Copyright (c) 2026, I-SYST inc., all rights reserved. MIT License.
import sys, struct, random
from unicorn import *
from unicorn.riscv_const import *
from elftools.elf.elffile import ELFFile

ELF = sys.argv[1]
STOP = 0x000FFF00
EVT_PORT_IRQ2 = 0x03
IR = 1 << 16
DTCE = 1 << 24

sym = {}
symsize = {}
with open(ELF, "rb") as f:
    elf = ELFFile(f)
    entry = elf.header["e_entry"]
    for s in elf.get_section_by_name(".symtab").iter_symbols():
        sym.setdefault(s.name, s["st_value"])
        symsize.setdefault(s.name, s["st_size"])
    segs = [(seg["p_paddr"], seg.data()) for seg in elf.iter_segments()
            if seg["p_type"] == "PT_LOAD" and seg["p_filesz"]]

def S(name):
    for k, v in sym.items():
        if k == name or (name.startswith("~") and name[1:] in k):
            return v
    raise KeyError(name)

uc = Uc(UC_ARCH_RISCV, UC_MODE_RISCV32)
uc.mem_map(0, 0x20000, UC_PROT_ALL)
uc.mem_map(STOP & ~0xFFF, 0x1000, UC_PROT_ALL)
uc.mem_map(0x01000000, 0x20000, UC_PROT_ALL)
uc.mem_map(0x20000000, 0x40000, UC_PROT_ALL)
for a, d in segs:
    uc.mem_write(a, d)
uc.mem_map(0xE0000000, 0x100000, UC_PROT_ALL)
uc.mem_map(0xE2000000, 0x10000, UC_PROT_ALL)

ICU, PORT, SAU, UARTA = 0x40006000, 0x40040000, 0x40094000, 0x40096000
MM = (ICU, PORT, SAU, UARTA)
plain = [(0x40000000, ICU), (ICU + 0x1000, PORT), (PORT + 0x1000, SAU),
         (SAU + 0x1000, UARTA), (UARTA + 0x1000, 0x41000000)]
for lo, hi in plain:
    uc.mem_map(lo, hi - lo, UC_PROT_ALL)

back = {b: bytearray(0x1000) for b in MM}
anomalies = []

def bget(b, off, size):
    return int.from_bytes(back[b][off:off + size], "little")

def bset(b, off, size, v):
    back[b][off:off + size] = (v & ((1 << (8 * size)) - 1)).to_bytes(size, "little")

# ---- ICU ----
class Icu:
    def __init__(s):
        s.ir = [False] * 32
        s.dtce = [False] * 32
        s.iels = [0] * 32
        s.lost = 0
icu = Icu()

def icu_read(off, size):
    if 0x300 <= off < 0x380:
        assert size == 4 and off % 4 == 0
        n = (off - 0x300) // 4
        return icu.iels[n] | (IR if icu.ir[n] else 0) | (DTCE if icu.dtce[n] else 0)
    return bget(ICU, off, size)

def icu_write(off, size, v):
    if 0x300 <= off < 0x380:
        assert size == 4
        n = (off - 0x300) // 4
        icu.iels[n] = v & 0x1F
        icu.ir[n] = icu.ir[n] and bool(v & IR)
        icu.dtce[n] = bool(v & DTCE)
        if icu.dtce[n] and icu.ir[n]:
            anomalies.append("DTCE set with IR at 1 on slot %d" % n)
        return
    bset(ICU, off, size, v)

def event(slot):
    if slot is None:
        anomalies.append("event without route")
        return
    if icu.ir[slot]:
        icu.lost += 1
        return
    if icu.dtce[slot]:
        dtc_run(slot)
    else:
        icu.ir[slot] = True

def uart_event(evt):
    # In polling mode UART events have no ICU route.
    slot = slot_of(evt)
    if slot is not None:
        event(slot)

# ---- PORT: P202 is the chip select input ----
cs = [1]
tx_gpio_low = []
def port_read(off, size):
    if off < 0xA0 and off % 0x20 == 4 and size == 4:
        port = off // 0x20
        v = 0xFFFF
        if port == 2 and not cs[0]:
            v &= ~(1 << 2)
        return v
    return bget(PORT, off, size)

def port_write(off, size, v):
    bset(PORT, off, size, v)
    if off < 0xA0 and off % 0x20 == 8 and size == 4:
        # PCNTR3: POSR in bits 15:0, PORR in bits 31:16.
        for pin in range(16):
            pfs = 0x800 + (off // 0x20) * 0x40 + pin * 4
            latch = bget(PORT, pfs, 4)
            if v & (1 << pin):
                latch |= 1
            if v & (1 << (pin + 16)):
                latch &= ~1
            bset(PORT, pfs, 4, latch)
    if off == 0x8C8 and size == 4 and (v & 0x10005) == 4:
        # P302: PMR=0, PDR=1, PODR=0 actively drives low.
        tx_gpio_low.append(v)

# ---- SAU0 channels 0 (TX) and 1 (RX), UART0 ----
EVT_TX, EVT_RX, EVT_SNZCANCEL = 0x3B, 0x3C, 0x0B
SNZ_ROUTE_IR = [True]   # the Snooze reception end also sets the route IR
SNZ_CANCEL_NORMAL = [False]
SNZ_BFF = [False]
class Uart:
    def __init__(s):
        s.running = [False, False]
        s.sdr = 0
        s.bff = False
        s.ovf = False
        s.err = 0
        s.tx = []
        s.shifting = False
        s.shift = 0
        s.txsdr = 0
        s.txbff = False
        s.cut = 0
        s.snz = 0
        s.snz_set = []
ua = Uart()
uarta = [Uart(), Uart()]
HOLD_TX = [False]       # force the bounded TX wait to expire
TX_STATUS_READS = [0]

def tx_level(high):
    if bget(SAU, 0xAA, 2) & 1:
        so = bget(SAU, 0xA8, 2)
        bset(SAU, 0xA8, 2, (so | 1) if high else (so & ~1))

RX_ON_TX_STEP = []      # full duplex peer: one RX byte per TX character

def tx_step():
    if HOLD_TX[0]:
        return
    for ch, a in enumerate(uarta):
        if a.shifting:
            a.tx.append(a.shift)
            if a.txbff:
                a.shift = a.txsdr
                a.txbff = False
                uart_event(0x5D + ch * 3)
            else:
                a.shifting = False
    if ua.shifting:
        ua.tx.append(ua.shift)
        if RX_ON_TX_STEP:
            rx_char(RX_ON_TX_STEP.pop(0))
        if ua.txbff:
            ua.shift = ua.txsdr
            ua.txbff = False
            if bget(SAU, 0x090, 2) & 1:
                uart_event(EVT_TX)
        else:
            ua.shifting = False
            tx_level(True)
        if not bget(SAU, 0x090, 2) & 1:
            uart_event(EVT_TX)

DEPTH = [0]
IRQ_REQ = [False]
INJECT_RX = [None]      # character received at the next SSR poll of the receive channel
SKIP_STEP = [False]     # the poll after interrupts taken there is the same one

def poll_step():
    if DEPTH[0] == 0:
        if SKIP_STEP[0]:
            SKIP_STEP[0] = False
        else:
            tx_step()
def poll_point():
    # Interrupts pending and enabled are taken while the firmware polls
    if DEPTH[0] == 0 and (uc.reg_read(UC_RISCV_REG_MSTATUS) & 8) and pending():
        IRQ_REQ[0] = True
        uc.emu_stop()
SNZ_EXPECT = [None]     # (prs, stclk) the Snooze must use

def swc():
    return bget(SAU, 0xB8, 2) & 1

def sau_read(off, size):
    if off == 0x002:
        if not ua.running[1]:
            return bget(SAU, off, 2)
        v = ua.sdr
        if ua.bff and not swc():
            ua.bff = False
        return v
    if off == 0x080:
        TX_STATUS_READS[0] += 1
        poll_step()
        poll_point()
        return (0x20 if ua.txbff else 0) | (0x40 if ua.shifting else 0)
    if off == 0x082:
        poll_step()
        poll_point()
        if INJECT_RX[0] is not None and not IRQ_REQ[0]:
            # Arrives after this poll, its interrupt not taken before the
            # hook sets the channel for Snooze
            rx_char(INJECT_RX[0])
            INJECT_RX[0] = None
        return (0x20 if ua.bff else 0) | (1 if ua.ovf else 0) | ua.err
    if off == 0x0A0:
        return (1 if ua.running[0] else 0) | (2 if ua.running[1] else 0)
    return bget(SAU, off, size)

def sau_write(off, size, v):
    if off == 0x0A8:
        protected = bget(SAU, 0xAA, 2) & 0xF
        bset(SAU, off, size, (v & ~protected) | (bget(SAU, off, size) & protected))
        return
    if off == 0x000:
        assert size == 2, "SDR transmit writes must be halfwords"
        if not ua.running[0]:
            bset(SAU, off, 2, v)
        elif ua.txbff:
            anomalies.append("TX SDR written with BFF set")
        elif not ua.shifting:
            ua.shifting = True
            ua.shift = v & 0xFF
            tx_level(False)
            if bget(SAU, 0x090, 2) & 1:
                uart_event(EVT_TX)
        else:
            ua.txsdr = v & 0xFF
            ua.txbff = True
        return
    if off == 0x002:
        if not ua.running[1]:
            bset(SAU, off, 2, v)
        return
    if off == 0x08A:
        if v & 1:
            ua.ovf = False
        ua.err &= ~v
        return
    if off in (0x0A2, 0x0A4):
        for ch in (0, 1):
            if v & (1 << ch):
                if ch == 0 and off == 0x0A4 and ua.running[0] and (ua.shifting or ua.txbff):
                    ua.cut += 1
                if ch == 0:
                    ua.shifting = False
                    ua.txbff = False
                ua.running[ch] = off == 0x0A2
                if ch == 1:
                    ua.bff = False
        return
    bset(SAU, off, size, v)

def rx_char(c):
    if not ua.running[1]:
        return False
    if swc():
        smr = bget(SAU, 0x092, 2)
        cks = (smr >> 15) & 1
        prs = (bget(SAU, 0xA6, 2) >> (4 * cks)) & 0xF
        stclk = bget(SAU, 0x002, 2) >> 9
        ua.snz_set.append((prs, stclk, bget(SAU, 0xB8, 2), bool(bget(SAU, 0x09A, 2) & 0x400)))
        ua.snz += 1
        ua.sdr = c
        if SNZ_BFF[0]:
            ua.bff = True
        if SNZ_ROUTE_IR[0]:
            event(slot_of(EVT_RX))
        if bget(ICU, 0x200, 2) == EVT_RX:
            event(slot_of(EVT_SNZCANCEL))
        return True
    if ua.bff:
        ua.ovf = True
    ua.sdr = c
    ua.bff = True
    uart_event(EVT_RX)
    if SNZ_CANCEL_NORMAL[0] and bget(ICU, 0x200, 2) == EVT_RX:
        event(slot_of(EVT_SNZCANCEL))
    return True

# UARTA: TXBA and shift register, buffer-empty TX_ENDI, RX_ENDI even on
# receive errors (ISRMA = 1). There is no RX-ready status bit.
def uarta_read(off, size):
    assert size == 1
    if off < 16:
        ch, reg = divmod(off, 8)
        a = uarta[ch]
        if reg == 1:
            a.bff = False
            return a.sdr
        if reg == 5:
            poll_step()
            poll_point()
            return (0x20 if a.txbff else 0) | (0x10 if a.shifting else 0) | a.err
    return bget(UARTA, off, size)

def uarta_write(off, size, v):
    assert size == 1
    if off < 16:
        ch, reg = divmod(off, 8)
        a = uarta[ch]
        if reg == 0:
            assert a.running[0], "TXBA written while disabled"
            assert not a.txbff, "TXBA written while full"
            if not a.shifting:
                a.shifting = True
                a.shift = v
                uart_event(0x5D + ch * 3)
            else:
                a.txsdr = v
                a.txbff = True
            return
        if reg == 2:
            if not v & 0x40:
                a.shifting = a.txbff = False
            a.running = [bool(v & 0x40), bool(v & 0x20)]
        if reg == 6:
            a.err &= ~v
    bset(UARTA, off, size, v)

def uarta_rx(ch, c, err=0):
    a = uarta[ch]
    assert a.running[1]
    a.err |= err
    if a.bff:
        a.err |= 1
    else:
        a.sdr = c
        a.bff = True
    uart_event(0x5E + ch * 3)

MODEL = {ICU: (icu_read, icu_write), PORT: (port_read, port_write), SAU: (sau_read, sau_write),
         UARTA: (uarta_read, uarta_write)}
for b, (r, w) in MODEL.items():
    uc.mmio_map(b, 0x1000, lambda u, o, sz, d, r=r: r(o, sz), None,
                lambda u, o, sz, v, d, w=w: w(o, sz, v), None)

def bus_read(a, size):
    for b in MM:
        if b <= a < b + 0x1000:
            return MODEL[b][0](a - b, size)
    return int.from_bytes(uc.mem_read(a, size), "little")

def bus_write(a, size, v):
    for b in MM:
        if b <= a < b + 0x1000:
            return MODEL[b][1](a - b, size, v)
    uc.mem_write(a, (v & ((1 << (8 * size)) - 1)).to_bytes(size, "little"))

# ---- DTC ----
dtc_runs = [0]
def dtc_run(slot):
    assert bus_read(0x4000540C, 1) & 1, "DTC activation while DTCST is stopped"
    assert clic_ie(slot), "DTC activation source must be enabled in CLIC (14.5)"
    vbr = bus_read(0x40005404, 4)
    rec = bus_read(vbr + 4 * slot, 4)
    if rec == 0:
        anomalies.append("DTC start without records")
        return
    cras = []
    while True:
        mrb, mra = bus_read(rec + 2, 1), bus_read(rec + 3, 1)
        sar, dar, cra = bus_read(rec + 4, 4), bus_read(rec + 8, 4), bus_read(rec + 14, 2)
        assert (mra >> 6) & 3 == 0, "only normal mode expected"
        sz = (1, 2, 4)[(mra >> 4) & 3]
        v = bus_read(sar, sz)
        bus_write(dar, sz, v)
        sm, dm = (mra >> 2) & 3, (mrb >> 2) & 3
        sar += sz if sm == 2 else (-sz if sm == 3 else 0)
        dar += sz if dm == 2 else (-sz if dm == 3 else 0)
        before = cra if cra else 0x10000
        cras.append(before)
        bus_write(rec + 4, 4, sar)
        bus_write(rec + 8, 4, dar)
        bus_write(rec + 14, 2, (before - 1) & 0xFFFF)
        if (mrb & 0x80) and not (mrb & 0x40):
            rec += 16
            continue
        break
    if len(set(cras)) != 1:
        anomalies.append("chain counters differ %s" % cras)
    dtc_runs[0] += 1
    if before == 1:
        icu.dtce[slot] = False
        icu.ir[slot] = True
    else:
        icu.ir[slot] = False

# ---- target calls ----
def hook_pclk(u, addr, size, ud):
    u.reg_write(UC_RISCV_REG_A0, PCLK[0])
    u.reg_write(UC_RISCV_REG_PC, u.reg_read(UC_RISCV_REG_RA))
PCLK = [32000000]
HOCO = [32000000]
uc.hook_add(UC_HOOK_CODE, hook_pclk, begin=S("SystemPeriphClockGet"), end=S("SystemPeriphClockGet"))
def hook_hoco(u, addr, size, ud):
    u.reg_write(UC_RISCV_REG_A0, HOCO[0])
    u.reg_write(UC_RISCV_REG_PC, u.reg_read(UC_RISCV_REG_RA))
uc.hook_add(UC_HOOK_CODE, hook_hoco, begin=S("R9a02HocoClockGet"), end=S("R9a02HocoClockGet"))
def hook_sysinit(u, addr, size, ud):
    u.reg_write(UC_RISCV_REG_PC, u.reg_read(UC_RISCV_REG_RA))
uc.hook_add(UC_HOOK_CODE, hook_sysinit, begin=S("SystemInit"), end=S("SystemInit"))

# mcycle: unicorn returns host time, the firmware bounds its waits with it.
# Each read here advances it by 100 cycles.
MCYCLE = [0]
def hook_mcycle(u, addr, size, ud):
    insn = struct.unpack("<I", u.mem_read(addr, 4))[0]
    rd = (insn >> 7) & 31
    if rd:
        u.reg_write(UC_RISCV_REG_X0 + rd, MCYCLE[0] & 0xFFFFFFFF)
    MCYCLE[0] += 100
    u.reg_write(UC_RISCV_REG_PC, addr + 4)
for a, d in segs:
    for off in range(0, len(d) - 3, 2):
        w = struct.unpack_from("<I", d, off)[0]
        if (w & 0xFFFFF07F) == 0xB0002073:      # csrrs rd, mcycle, x0
            uc.hook_add(UC_HOOK_CODE, hook_mcycle, begin=a + off, end=a + off)

uc.reg_write(UC_RISCV_REG_MSTATUS, 0)
uc.emu_start(entry, S("main"), timeout=2_000_000_000)
assert uc.reg_read(UC_RISCV_REG_PC) == S("main"), "did not reach main"
uc.mem_write(S("SystemCoreClock"), struct.pack("<I", 32000000))
uc.mem_write(0x4001E03C, b"\x01")           # OSCSF.HOCOSF, the HOCO runs

CALL_SP = [None]      # stack of a call made while the target waits in WFI

XREGS = [UC_RISCV_REG_X1 + i for i in range(31)]

# Runs from pc up to STOP, or up to WFI with bWfi. The interrupts requested
# at a poll point run on a stack below the one in use, then the code goes on.
def run(pc, bWfi=False):
    while True:
        IRQ_REQ[0] = False
        uc.emu_start(pc, STOP, timeout=2_000_000_000, count=5_000_000)
        pc = uc.reg_read(UC_RISCV_REG_PC)
        if pc == STOP or (bWfi and wfi_at[0] is not None):
            return
        assert IRQ_REQ[0], "stopped at pc %x" % pc
        ctx = [uc.reg_read(r) for r in XREGS]
        mst = uc.reg_read(UC_RISCV_REG_MSTATUS)
        sp = CALL_SP[0]
        CALL_SP[0] = (ctx[1] - 0x100) & ~15
        deliver(idle=False)
        SKIP_STEP[0] = True
        CALL_SP[0] = sp
        for r, v in zip(XREGS, ctx):
            uc.reg_write(r, v)
        uc.reg_write(UC_RISCV_REG_MSTATUS, mst)

def call(name, *args):
    for r, v in zip((UC_RISCV_REG_A0, UC_RISCV_REG_A1, UC_RISCV_REG_A2), args):
        uc.reg_write(r, v & 0xFFFFFFFF)
    uc.reg_write(UC_RISCV_REG_RA, STOP)
    uc.reg_write(UC_RISCV_REG_SP, CALL_SP[0] if CALL_SP[0] else S("__StackTop") - 0x200)
    uc.reg_write(UC_RISCV_REG_MSTATUS, 8)
    try:
        run(S(name))
    except UcError as e:
        print("crash in", name, "pc %x ra %x" % (uc.reg_read(UC_RISCV_REG_PC), uc.reg_read(UC_RISCV_REG_RA)), e)
        raise
    assert uc.reg_read(UC_RISCV_REG_PC) == STOP, "%s did not return, pc %x" % (name, uc.reg_read(UC_RISCV_REG_PC))
    v = uc.reg_read(UC_RISCV_REG_A0)
    return v - (1 << 32) if v & 0x80000000 else v

def gi(name):
    v = struct.unpack("<i", uc.mem_read(S(name), 4))[0]
    return v

def si(name, v):
    uc.mem_write(S(name), struct.pack("<i", v))

# Software Standby: R9a02LowPowerEnter runs up to its WFI, the harness plays
# what happens in Software Standby and Snooze, delivers the wake-up
# interrupts as the CPU takes them after WFI, then the call goes on.
WFI = 0x10500073
wfi_at = [None]
AT_WFI = [None]         # SSC, SNZCANCEL CLIC IE and IR at WFI
def hook_wfi(u, addr, size, ud):
    if size == 4 and struct.unpack("<I", u.mem_read(addr, 4))[0] == WFI:
        wfi_at[0] = addr
        u.emu_stop()
_lpe = S("R9a02LowPowerEnter")
uc.hook_add(UC_HOOK_CODE, hook_wfi, begin=_lpe, end=_lpe + symsize["R9a02LowPowerEnter"] - 1)

def standby(during, first=None):
    uc.mem_write(0x4001E03C, b"\x01")       # OSCSF.HOCOSF, the HOCO runs
    wfi_at[0] = None
    uc.reg_write(UC_RISCV_REG_RA, STOP)
    uc.reg_write(UC_RISCV_REG_SP, S("__StackTop") - 0x200)
    uc.reg_write(UC_RISCV_REG_MSTATUS, 8)
    run(S("EmuStandby"), True)
    pc = uc.reg_read(UC_RISCV_REG_PC)
    assert wfi_at[0] is not None and pc in (wfi_at[0], wfi_at[0] + 4), "no WFI, pc %x" % pc
    ctx_pc = wfi_at[0]
    AT_WFI[0] = (bget(SAU, 0xB8, 2), clic_ie(slot_of(EVT_SNZCANCEL)) if slot_of(EVT_SNZCANCEL) is not None else None,
                 icu.ir[slot_of(EVT_SNZCANCEL)] if slot_of(EVT_SNZCANCEL) is not None else None)
    ctx = [uc.reg_read(r) for r in XREGS]
    mst = uc.reg_read(UC_RISCV_REG_MSTATUS)
    CALL_SP[0] = (ctx[1] - 0x100) & ~15
    during()
    deliver(first)
    CALL_SP[0] = None
    for r, v in zip(XREGS, ctx):
        uc.reg_write(r, v)
    uc.reg_write(UC_RISCV_REG_MSTATUS, mst)
    wfi_at[0] = None
    run(ctx_pc + 4)
    assert uc.reg_read(UC_RISCV_REG_PC) == STOP, "standby did not return"
    return uc.reg_read(UC_RISCV_REG_A0)

HOOKS = S("~s_IcuHook")
def slot_of(evt):
    for n in range(32):
        e = uc.mem_read(HOOKS + 16 * n + 12, 1)[0]
        if e == evt:
            return n
    return None

def clic_ie(n):
    return uc.mem_read(0xE2001000 + 4 * (19 + n) + 1, 1)[0] != 0

def pending():
    return [n for n in range(32) if icu.ir[n] and not icu.dtce[n] and clic_ie(n)]

# Interrupts taken until none is pending. idle: the CPU waits meanwhile, the
# transmission goes on.
def deliver(first=None, limit=2000, idle=True):
    DEPTH[0] += 1
    try:
        for _ in range(limit):
            p = pending()
            if not p:
                if idle and (ua.shifting or any(a.shifting for a in uarta)):
                    tx_step()
                    continue
                return
            if first is not None:
                pf = slot_of(first)
                if pf in p:
                    p = [pf]
            call("R9a02IrqDispatch", 19 + p[0])
        raise AssertionError("interrupt storm")
    finally:
        DEPTH[0] -= 1

# ---- checks ----
# Interrupt on the first data write of a bulk copy, before it is complete.
# -1 delivers an already pending TX interrupt; a byte injects RX as well.
COPY_IRQ = [None]
copy_irq_seen = []
def hook_copy_irq(u, access, addr, size, value, ud):
    if COPY_IRQ[0] is None:
        return
    c = COPY_IRQ[0]
    # RV32 CFifo header is 32 bytes; the fixture holds 32 data bytes.
    target = S("~s_TxMem") + 32 if c < 0 else S("g_RxBuf")
    if not target <= addr < target + 32:
        return
    COPY_IRQ[0] = None
    enabled = bool(u.reg_read(UC_RISCV_REG_MSTATUS) & 8)
    copy_irq_seen.append(enabled)
    if enabled:
        if c >= 0:
            rx_char(c)
        IRQ_REQ[0] = True
        u.emu_stop()
uc.hook_add(UC_HOOK_MEM_WRITE, hook_copy_irq, begin=0x20000000, end=0x2003FFFF)

fails = []
def check(name, cond, info=""):
    if not cond:
        fails.append(name + " " + info)
        print("FAIL", name, info)

SNZCR, SNZEDCR1 = 0x4001E092, 0x4001E095

def rx_all():
    n = call("EmuUartRx")
    return list(uc.mem_read(S("g_RxBuf"), n)) if n > 0 else []

def uart_clock():
    cks = (bget(SAU, 0x092, 2) >> 15) & 1
    return ((bget(SAU, 0xA6, 2) >> (4 * cks)) & 0xF, bget(SAU, 0x002, 2) >> 9, bget(SAU, 0x000, 2) >> 9)

def normal(prs, stclk):
    return (bget(SAU, 0xB8, 2) == 0 and ua.running == [True, True] and uart_clock() == (prs, stclk, stclk))

random.seed(2)
TXPAT = lambda n: [0x30 + i for i in range(n)]
SNZC = lambda: slot_of(EVT_SNZCANCEL)

# Initialization must not drive a low pulse while P302 passes through GPIO.
check("idle init", call("EmuUartInit", 9600) == 1)
check("no GPIO low during init", not tx_gpio_low, str([hex(v) for v in tx_gpio_low]))
check("idle high after init", bget(SAU, 0xA8, 2) & 1)
# One character, stopped during its start bit by a rate change. No queued
# character can hide a low output by completing after the restart.
check("rate test tx", call("EmuUartTx", 1) == 1 and ua.shifting)
check("start bit low", not (bget(SAU, 0xA8, 2) & 1))
check("rate test change", call("EmuUartRate", 9600) > 0)
check("idle high after rate change", bget(SAU, 0xA8, 2) & 1)
deliver()
# Snooze clock changes and entry also restore idle if the TX wait expires.
call("EmuUartTx", 1)
HOLD_TX[0] = True
check("timeout Snooze enable", call("EmuUartSnooze", 1) == 1)
check("idle high after Snooze clock timeout", bget(SAU, 0xA8, 2) & 1)
HOLD_TX[0] = False
deliver()
call("EmuUartTx", 1)
HOLD_TX[0] = True
seen = []
standby(lambda: seen.append(bget(SAU, 0xA8, 2) & 1))
check("idle high during Snooze after timeout", seen == [1], str(seen))
check("idle high after Snooze timeout", bget(SAU, 0xA8, 2) & 1)
HOLD_TX[0] = False
deliver()
call("EmuUartPowerOff")

for hoco, rate, prs, stclk, snzstclk in ((32000000, 9600, 4, 103, 106), (32000000, 4800, 5, 103, 106),
                                         (24000000, 9600, 4, 77, 79), (24000000, 4800, 5, 77, 79)):
    tag = "%d MHz %d bps" % (hoco // 1000000, rate)
    PCLK[0] = hoco
    HOCO[0] = hoco
    check(tag + " init", call("EmuUartInit", rate) == 1)
    # Bytes queued just before: sent before the channels stop, none cut
    ua.tx.clear()
    cut0 = ua.cut
    check(tag + " tx queued", call("EmuUartTx", 24) == 24)
    check(tag + " enable", call("EmuUartSnooze", 1) == 1)
    check(tag + " enable: tx sent first", ua.tx == TXPAT(24) and ua.cut == cut0, "%d sent, %d cut" % (len(ua.tx), ua.cut - cut0))
    check(tag + " clock", uart_clock() == (prs, stclk, stclk), str(uart_clock()))
    check(tag + " SELSR0 SAU0_ENDI1", bget(ICU, 0x200, 2) == EVT_RX)
    check(tag + " SNZCR RXD0REQEN", uc.mem_read(SNZCR, 1)[0] & 0x04 != 0)
    check(tag + " SNZEDCR1 SAU0NCRED", uc.mem_read(SNZEDCR1, 1)[0] == 0x10)
    check(tag + " SNZCANCEL off in Normal mode", not clic_ie(SNZC()))
    exp = (hoco >> prs) // (2 * (stclk + 1))
    check(tag + " rate", call("EmuUartRate", rate) == exp and uart_clock() == (prs, stclk, stclk), "%d" % exp)
    # Transmission after the clock set up
    ua.tx.clear()
    call("EmuUartTx", 5)
    deliver()
    check(tag + " tx after enable", ua.tx == TXPAT(5), str(ua.tx))
    for both, cn, sbff in [(a, b, c) for a in (True, False) for b in (False, True) for c in (False, True)]:
        SNZ_ROUTE_IR[0] = both
        SNZ_CANCEL_NORMAL[0] = cn
        SNZ_BFF[0] = sbff
        vtag = tag + (" route+cancel" if both else " cancel only") + (" cancel in Normal" if cn else "") + \
            (" BFF" if sbff else "")
        c = random.randrange(256)
        evt0 = gi("g_RxEvt")
        seen = []
        def during():
            seen.append((ua.running[:], bget(SAU, 0xB8, 2)))
            rx_char(c)
        # Bytes queued, not waited for: sent before the standby
        ua.tx.clear()
        call("EmuUartTx", 12)
        standby(during)
        check(vtag + " tx before standby", ua.tx == TXPAT(12) and ua.cut == cut0, "%d sent" % len(ua.tx))
        check(vtag + " set for Snooze", seen == [([False, True], 3)], str(seen))
        check(vtag + " SNZCANCEL on at WFI, no request left", AT_WFI[0][1:] == (True, False), str(AT_WFI[0]))
        check(vtag + " Snooze settings", ua.snz_set[-1] == (prs, snzstclk, 3, True), str(ua.snz_set[-1]))
        check(vtag + " back to normal", normal(prs, stclk), str(uart_clock()))
        check(vtag + " SNZCANCEL off after", not clic_ie(SNZC()))
        check(vtag + " character", rx_all() == [c])
        check(vtag + " one RXDATA event", gi("g_RxEvt") == evt0 + 1, str(gi("g_RxEvt") - evt0))
        # The next characters in normal operation
        nxt = [random.randrange(256) for _ in range(3)]
        for b in nxt:
            rx_char(b)
            deliver()
        check(vtag + " normal after", rx_all() == nxt)
    # A character in Normal mode leaves an ICU_SNZCANCEL request: dropped
    # at the next standby, the Snooze character is taken once
    SNZ_CANCEL_NORMAL[0] = True
    rx_char(0x55)
    deliver()
    check(tag + " request left in Normal mode", icu.ir[SNZC()] and rx_all() == [0x55])
    c = random.randrange(256)
    standby(lambda: rx_char(c))
    check(tag + " request left: dropped", AT_WFI[0][2] is False and rx_all() == [c] and normal(prs, stclk))
    SNZ_CANCEL_NORMAL[0] = False
    SNZ_ROUTE_IR[0] = True
    SNZ_BFF[0] = False
    # The answer sent from UART_EVT_RXDATA leaves in normal operation
    si("g_CbMode", 1)
    ua.tx.clear()
    c = random.randrange(256)
    standby(lambda: rx_char(c))
    deliver()
    check(tag + " echo from the event", ua.tx == [c] and normal(prs, stclk), str(ua.tx))
    # Echo of a character received before the standby, its interrupt taken
    # while the hook waits: sent before the standby
    ua.tx.clear()
    rx_char(0x42)
    seen = []
    standby(lambda: seen.append(ua.tx[:]))
    check(tag + " echo before standby", seen == [[0x42]] and ua.tx == [0x42] and normal(prs, stclk), "%s %s" % (seen, ua.tx))
    # Echo of a character the hook takes: waits in the TX CFIFO until the
    # wake-up
    ua.tx.clear()
    INJECT_RX[0] = 0x45
    seen = []
    standby(lambda: seen.append((ua.tx[:], bget(SAU, 0xB8, 2))))
    deliver()
    check(tag + " echo of the hook", seen == [([], 3)] and ua.tx == [0x45] and normal(prs, stclk), "%s %s" % (seen, ua.tx))
    # Disable from the event of the hook, while set for Snooze: stays disabled
    si("g_CbMode", 3)
    INJECT_RX[0] = 0x46
    seen = []
    standby(lambda: seen.append(bget(SAU, 0xB8, 2)))
    si("g_CbMode", 0)
    check(tag + " disabled while set, hook", seen == [3] and ua.running == [False, False] and
          bget(SAU, 0xB8, 2) == 0 and uart_clock()[1] == stclk and rx_all() == [0x46],
          "%s %s %s" % (seen, ua.running, uart_clock()))
    call("EmuUartEnable")
    check(tag + " enable after, hook", normal(prs, stclk))
    # Snooze off from the event of the hook, while set for Snooze: back to
    # normal operation at once
    si("g_CbMode", 2)
    INJECT_RX[0] = 0x47
    seen = []
    standby(lambda: seen.append((bget(SAU, 0xB8, 2), ua.running[:])))
    si("g_CbMode", 0)
    check(tag + " off while set, hook", seen == [(0, [True, True])] and normal(prs, stclk) and
          uc.mem_read(SNZCR, 1)[0] == 0 and rx_all() == [0x47], "%s %s" % (seen, uart_clock()))
    check(tag + " enable again 0", call("EmuUartSnooze", 1) == 1)
    si("g_CbMode", 0)
    # Woken without a character
    standby(lambda: None)
    check(tag + " no character", normal(prs, stclk) and rx_all() == [])
    # A character received before the standby, its interrupt not taken yet
    rx_char(0x41)
    seen = []
    standby(lambda: seen.append(bget(SAU, 0xB8, 2)))
    check(tag + " character before", seen == [3] and rx_all() == [0x41] and normal(prs, stclk))
    # Snooze turned off from the event of that character, while set for Snooze
    si("g_CbMode", 2)
    rx_char(0x43)
    seen = []
    standby(lambda: seen.append(bget(SAU, 0xB8, 2)))
    si("g_CbMode", 0)
    check(tag + " off while set: back to normal", seen == [0] and rx_all() == [0x43] and normal(prs, stclk) and
          uc.mem_read(SNZCR, 1)[0] == 0, "%s %s" % (seen, uart_clock()))
    check(tag + " off while set: 115200", call("EmuUartRate", 115200) > 0)
    call("EmuUartRate", rate)
    check(tag + " enable again", call("EmuUartSnooze", 1) == 1)
    # Disable from the event while set for Snooze: stays disabled
    si("g_CbMode", 3)
    rx_char(0x44)
    standby(lambda: None)
    si("g_CbMode", 0)
    check(tag + " disabled while set", ua.running == [False, False] and bget(SAU, 0xB8, 2) == 0 and
          rx_all() == [0x44], str(ua.running))
    call("EmuUartEnable")
    check(tag + " enable after", normal(prs, stclk))
    # Another Snooze set up replaces it: not set for Snooze any more, and
    # turning the UART's off leaves the other one
    check(tag + " other set up", call("EmuOtherSnooze", 1) == 1)
    seen = []
    standby(lambda: seen.append(bget(SAU, 0xB8, 2)))
    check(tag + " other: not set for Snooze", seen == [0] and normal(prs, stclk))
    check(tag + " other: kept by off", call("EmuUartSnooze", 0) == 1 and
          uc.mem_read(0x4001E098, 4) == b"\x08\x00\x00\x00", uc.mem_read(0x4001E098, 4).hex())
    call("EmuOtherSnooze", 0)
    check(tag + " enable again 2", call("EmuUartSnooze", 1) == 1)
    # Rates outside table 23.95 refused while Snooze is set up
    check(tag + " 115200 refused", call("EmuUartRate", 115200) == 0 and uart_clock() == (prs, stclk, stclk))
    # Off
    check(tag + " off", call("EmuUartSnooze", 0) == 1 and uc.mem_read(SNZCR, 1)[0] == 0)
    seen = []
    standby(lambda: seen.append(bget(SAU, 0xB8, 2)))
    check(tag + " off: not set for Snooze", seen == [0])
    # Rate change while sending: the transmission goes on
    ua.tx.clear()
    call("EmuUartTx", 8)
    check(tag + " 115200 after off", call("EmuUartRate", 115200) > 0)
    deliver()
    check(tag + " tx after a rate change", len(ua.tx) >= 7 and ua.tx[-6:] == TXPAT(8)[-6:], str(ua.tx))
    call("EmuUartPowerOff")

# Clocks that table 23.95 does not cover, and rates
for hoco, pclk, rate in ((48000000, 48000000, 9600), (32000000, 16000000, 9600), (32000000, 32000000, 115200)):
    PCLK[0] = pclk
    HOCO[0] = hoco
    tag = "HOCO %d PCLKB %d %d bps" % (hoco // 1000000, pclk // 1000000, rate)
    check(tag + " init", call("EmuUartInit", rate) == 1)
    check(tag + " refused", call("EmuUartSnooze", 1) == 0 and uc.mem_read(SNZCR, 1)[0] == 0)
    call("EmuUartPowerOff")

# Power off removes the Snooze set up
PCLK[0] = HOCO[0] = 32000000
check("power off init", call("EmuUartInit", 9600) == 1 and call("EmuUartSnooze", 1) == 1)
call("EmuUartPowerOff")
seen = []
standby(lambda: seen.append(bget(SAU, 0xB8, 2)))
check("power off: no Snooze", seen == [0] and uc.mem_read(SNZCR, 1)[0] == 0)

# Blocking FIFOs use reserve/publish and peek/release while interrupts run.
si("g_FifoBlocking", 1)
PCLK[0] = HOCO[0] = 48000000
check("blocking init", call("EmuUartInit", 1000000) == 1)
for n in (1, 4, 8, 16, 24, 32, 7, 31):
    data = [(i + n) & 0xFF for i in range(n)]
    for c in data:
        rx_char(c)
        deliver(idle=False)
    check("blocking RX wrap %d" % n, rx_all() == data)
    ua.tx.clear()
    check("blocking TX queue %d" % n, call("EmuUartTx", n) == n)
    deliver()
    check("blocking TX wrap %d" % n, ua.tx == TXPAT(n), str(ua.tx))

# TX ISR runs during the copy. It must not see the reserved, incomplete span.
ua.tx.clear()
call("EmuUartTx", 1)
copy_irq_seen.clear()
COPY_IRQ[0] = -1
check("blocking TX interrupted copy accepted", call("EmuUartTx", 16) == 16)
check("blocking TX copy interruptible", copy_irq_seen == [True], str(copy_irq_seen))
deliver()
check("blocking TX publishes after copy", ua.tx == TXPAT(1) + TXPAT(16), str(ua.tx))

# RX ISR appends while the application copies a span; the old data remains
# owned by the reader until CFifoRelease, including at a wrap boundary.
for n in (13, 24, 32):
    data = [(0x80 + i) & 0xFF for i in range(n)]
    for c in data:
        rx_char(c)
        deliver(idle=False)
    copy_irq_seen.clear()
    COPY_IRQ[0] = 0xE5
    got = rx_all()
    check("blocking RX copy interruptible %d" % n, copy_irq_seen == [True], str(copy_irq_seen))
    # A full blocking FIFO refuses the arriving byte, preserving all old data.
    expected = data if n == 32 else data + [0xE5]
    check("blocking RX keeps copied data %d" % n, got == expected, str(got))

# A full blocking TX FIFO returns a partial count; nothing accepted is lost.
HOLD_TX[0] = True
ua.tx.clear()
check("blocking fill TX", call("EmuUartTx", 32) == 32)
deliver(idle=False)
n = call("EmuUartTx", 16)
check("blocking TX partial", n == 2, str(n))
check("blocking TX full", call("EmuUartTx", 8) == 0)
HOLD_TX[0] = False
deliver()
check("blocking TX backpressure data", ua.tx == TXPAT(32) + TXPAT(2), str(ua.tx))
call("EmuUartPowerOff")
si("g_FifoBlocking", 0)

# Polling keeps both directions progressing without waiting for TX on wire.
si("g_PollMode", 1)
si("g_FifoBlocking", 1)
check("polling init", call("EmuUartInit", 1000000) == 1)
HOLD_TX[0] = True
ua.tx.clear()
poll_start = TX_STATUS_READS[0]
check("polling queues burst", call("EmuUartTx", 24) == 24)
check("polling does not wait for TX", TX_STATUS_READS[0] - poll_start <= 10,
      str(TX_STATUS_READS[0] - poll_start))
check("polling holds hardware", ua.shifting and ua.txbff)
HOLD_TX[0] = False
received = []
for c in range(24):
    rx_char(0x80 + c)
    received.extend(rx_all())
check("polling full duplex RX", received == list(range(0x80, 0x98)), str(received))
check("polling RX calls drain TX", ua.tx == TXPAT(24), str(ua.tx))
check("polling duplex init", call("EmuUartInit", 1000000) == 1)
ua.tx.clear()
RX_ON_TX_STEP[:] = range(0xA0, 0xB8)
check("polling duplex queues burst", call("EmuUartTx", 24) == 24)
received = []
for i in range(24):
    received.extend(rx_all())
check("polling duplex wire TX", ua.tx == TXPAT(24), str(ua.tx))
check("polling services RX during TX", received == list(range(0xA0, 0xB8)), str(received))
check("polling peer exhausted", not RX_ON_TX_STEP)
call("EmuUartPowerOff")
si("g_PollMode", 0)
si("g_FifoBlocking", 0)

# DTC transmit uses the same CFIFO contract, with RX still serviced by the
# CPU. Exercise both event delivery and polling, including UARTA byte access.
PCLK[0] = HOCO[0] = 48000000
si("g_DmaMode", 1)
for devno in (0, 3, 4):
    si("g_DevNo", devno)
    a = ua if devno == 0 else uarta[devno - 3]
    txevt = EVT_TX if devno == 0 else 0x5D + (devno - 3) * 3
    rate = 1000000 if devno == 0 else 115200
    def receive(c, err=0):
        if devno == 0:
            ua.err = err
            rx_char(c)
        else:
            uarta_rx(devno - 3, c, err)
    for poll in (0, 1):
        si("g_PollMode", poll)
        for blocking in (0, 1):
            si("g_FifoBlocking", blocking)
            tag = "DTC DevNo %d poll %d blocking %d" % (devno, poll, blocking)
            check(tag + " init", call("EmuUartInit", rate) == 1)
            txevents = gi("g_TxEvt")
            txslot = slot_of(txevt)
            check(tag + " route", txslot is not None and clic_ie(txslot))
            # The byte producer only enqueues while TX is busy. Its first
            # byte bypasses descriptor setup; the accumulated queue still
            # becomes a real autonomous DTC burst on the next TX event.
            dma_calls = [0]
            def count_dma(u, addr, size, ud):
                dma_calls[0] += 1
            dma_name = "~R9a02UartTxDmaP" if devno == 0 else "~R9a02UartaTxDmaP"
            h = uc.hook_add(UC_HOOK_CODE, count_dma, begin=S(dma_name), end=S(dma_name))
            HOLD_TX[0] = True
            a.tx.clear()
            before = dtc_runs[0]
            for _ in range(9):
                check(tag + " byte accepted", call("EmuUartTx", 1) == 1)
                if devno == 0:
                    # An immediate buffer-empty IRQ used to retire each
                    # isolated byte before the producer could queue more.
                    # A single CPU byte now notifies at transfer end.
                    deliver(idle=False)
                    check(tag + " no premature byte event", not icu.ir[txslot] and gi("g_TxEvt") == txevents)
                    check(tag + " single byte uses transfer end", bget(SAU, 0x090, 2) & 1 == 0)
            check(tag + " byte enqueue avoids DMA setup", dma_calls[0] == 0)
            HOLD_TX[0] = False
            for _ in range(24):
                tx_step()
                if poll:
                    rx_all()
                else:
                    deliver(idle=False)
            check(tag + " byte producer wire", a.tx == [0x30] * 9, str(a.tx))
            check(tag + " queued bytes use DTC", dtc_runs[0] > before)
            if devno == 0:
                check(tag + " queued block restores buffer empty", bget(SAU, 0x090, 2) & 1 == 1)
            # New data must not re-enter DMA service or alter the active
            # descriptor. The existing transfer drains it in FIFO order.
            HOLD_TX[0] = True
            a.tx.clear()
            call("EmuUartTx", 16)
            rec = bus_read(bus_read(0x40005404, 4) + 4 * txslot, 4)
            record = bytes(uc.mem_read(rec, 16))
            calls = dma_calls[0]
            for _ in range(8):
                call("EmuUartTx", 1)
            check(tag + " active descriptor unchanged", bytes(uc.mem_read(rec, 16)) == record)
            check(tag + " busy enqueue avoids DMA service", dma_calls[0] == calls)
            uc.hook_del(h)
            HOLD_TX[0] = False
            callbacks = gi("g_TxEvt")
            for _ in range(48):
                tx_step()
                if poll:
                    rx_all()
                else:
                    deliver(idle=False)
            check(tag + " queued block order", a.tx == TXPAT(16) + [0x30] * 8, str(a.tx))
            check(tag + " callback after queue drains", gi("g_TxEvt") == callbacks + (0 if poll else 1))
            # DTC completion leaves its last byte in the hardware buffer.
            # Copy the next block now, before that buffer empties, so the
            # next CPU event only arms it. Accepted FIFO space is reusable.
            HOLD_TX[0] = True
            a.tx.clear()
            initial = 8 if poll else 16
            call("EmuUartTx", initial)
            call("EmuUartTx", 16)
            HOLD_TX[0] = False
            while icu.dtce[txslot]:
                tx_step()
            check(tag + " last DMA byte still buffered", a.txbff)
            HOLD_TX[0] = True
            deliver(idle=False)
            check(tag + " next block staged before empty", bus_read(rec + 14, 2) == 16 and not icu.dtce[txslot])
            staged_sar = bus_read(rec + 4, 4)
            check(tag + " staging releases FIFO", call("EmuUartTx", 32) == 32)
            HOLD_TX[0] = False
            tx_step()
            deliver(idle=False)
            check(tag + " buffer-empty arms staged block", icu.dtce[txslot] and bus_read(rec + 14, 2) == 15
                  and bus_read(rec + 4, 4) == staged_sar + (2 if devno == 0 else 1))
            deliver()
            check(tag + " staged block survives FIFO reuse", a.tx == TXPAT(initial) + TXPAT(16) + TXPAT(32), str(a.tx))
            for n in (1, 2, 16, 32, 7, 31):
                a.tx.clear()
                before = dtc_runs[0]
                check(tag + " accepted %d" % n, call("EmuUartTx", n) == n)
                for _ in range(80):
                    tx_step()
                    if poll:
                        rx_all()
                    else:
                        deliver(idle=False)
                check(tag + " wire %d" % n, a.tx == TXPAT(n), str(a.tx))
                check(tag + " DTC used %d" % n, n == 1 or dtc_runs[0] > before)
                check(tag + " completed %d" % n, not icu.dtce[txslot])
            # No CPU handler runs during a block. This distinguishes DTC
            # movement from the old byte interrupt path.
            check(tag + " autonomous init", call("EmuUartInit", rate) == 1)
            HOLD_TX[0] = True
            a.tx.clear()
            before = dtc_runs[0]
            block = 8 if poll else 16
            call("EmuUartTx", block)
            HOLD_TX[0] = False
            for _ in range(18):
                tx_step()
            check(tag + " autonomous block", a.tx == TXPAT(block) and dtc_runs[0] - before == block - 1,
                  "%s transfers %d" % (a.tx, dtc_runs[0] - before))
            deliver(idle=False)
            # One-character RX burst must be available immediately; line
            # errors remain visible while DTC transmits.
            evt0 = gi("g_RxEvt")
            receive(0xA5)
            deliver(idle=False)
            check(tag + " short RX", rx_all() == [0xA5])
            check(tag + " RX callback", gi("g_RxEvt") == evt0 + (0 if poll else 1))
            err0 = gi("g_LineEvt")
            receive(0xF3, 4)  # parity error
            deliver(idle=False)
            check(tag + " parity discarded", rx_all() == [])
            check(tag + " error callback", gi("g_LineEvt") == err0 + (0 if poll else 1))
            a.tx.clear()
            call("EmuUartTx", 32)
            received = []
            for c in range(32):
                receive(0x80 + c)
                tx_step()
                deliver(idle=False)
                received.extend(rx_all())
            deliver()
            check(tag + " full duplex TX", a.tx == TXPAT(32), str(a.tx))
            check(tag + " full duplex RX", received == list(range(0x80, 0xA0)), str(received))
            # Rate changes retain staged bytes not yet written to hardware.
            # SAU stops its current character; UARTA drains its two registers.
            a.tx.clear()
            HOLD_TX[0] = True
            call("EmuUartTx", 24)
            HOLD_TX[0] = False
            check(tag + " active rate change", call("EmuUartRate", rate) > 0)
            for _ in range(50):
                tx_step()
                deliver(idle=False)
                if poll:
                    rx_all()
            check(tag + " rate retains staged tail", a.tx[-22:] == TXPAT(24)[-22:], str(a.tx))
            # Hold the line, overwrite/reject queued data, then let DMA
            # finish. Its already copied block must remain intact.
            HOLD_TX[0] = True
            a.tx.clear()
            check(tag + " first block", call("EmuUartTx", block) == block)
            check(tag + " queued block", call("EmuUartTx", 32) == 32)
            n = call("EmuUartTx", 8)
            check(tag + " full FIFO policy", n == (0 if blocking else 8), str(n))
            HOLD_TX[0] = False
            for _ in range(100):
                tx_step()
                if poll:
                    rx_all()
                else:
                    deliver(idle=False)
            expected = TXPAT(block) + (TXPAT(32) if blocking else TXPAT(32)[8:] + TXPAT(8))
            check(tag + " private block protected", a.tx == expected, str(a.tx))
            # Stop while the DTC owns the route; no later write may reach
            # a stopped UART. Enable discards the old transmit queue.
            HOLD_TX[0] = True
            call("EmuUartTx", 24)
            check(tag + " active before disable", icu.dtce[txslot])
            call("EmuUartDisable")
            check(tag + " disabled DTC", not icu.dtce[txslot] and not a.running[0])
            HOLD_TX[0] = False
            call("EmuUartEnable")
            a.tx.clear()
            call("EmuUartTx", 3)
            for _ in range(20):
                tx_step()
                if poll:
                    rx_all()
                else:
                    deliver(idle=False)
            check(tag + " enable flushes old DMA", a.tx == TXPAT(3), str(a.tx))
            # Reinitializing an active instance also relinquishes the old
            # DTC record before rebuilding the route and CFIFOs.
            HOLD_TX[0] = True
            call("EmuUartTx", 16)
            check(tag + " reinit", call("EmuUartInit", rate) == 1)
            check(tag + " reinit disabled DTC", not icu.dtce[slot_of(txevt)])
            HOLD_TX[0] = False
            call("EmuUartPowerOff")
            check(tag + " route released", slot_of(txevt) is None and not any(icu.dtce))
            check(tag + " TX callbacks", gi("g_TxEvt") == txevents if poll else gi("g_TxEvt") > txevents)
    # The default 16-byte CFIFO also works; format masking remains in RX.
    si("g_DefaultFifo", 1)
    si("g_FifoBlocking", 1)
    si("g_PollMode", 0)
    si("g_DataBits", 7)
    check("DTC default FIFO init %d" % devno, call("EmuUartInit", rate) == 1)
    a.tx.clear()
    check("DTC default FIFO accept %d" % devno, call("EmuUartTx", 16) == 16)
    deliver()
    check("DTC default FIFO wire %d" % devno, a.tx == TXPAT(16), str(a.tx))
    receive(0xE5)
    deliver()
    check("DTC RX mask %d" % devno, rx_all() == [0x65])
    call("EmuUartPowerOff")
    si("g_DefaultFifo", 0)
    si("g_DataBits", 0)
    # A DTC setup failure must unwind every route and allow a clean retry.
    def fail_dtc_setup(u, addr, size, ud):
        u.reg_write(UC_RISCV_REG_A0, 0)
        u.reg_write(UC_RISCV_REG_PC, u.reg_read(UC_RISCV_REG_RA))
    h = uc.hook_add(UC_HOOK_CODE, fail_dtc_setup, begin=S("R9a02DtcSet"), end=S("R9a02DtcSet"))
    check("DTC setup failure %d" % devno, call("EmuUartInit", rate) == 0)
    check("DTC failure releases route %d" % devno, slot_of(txevt) is None and not any(icu.dtce))
    uc.hook_del(h)
    check("DTC retry %d" % devno, call("EmuUartInit", rate) == 1)
    call("EmuUartPowerOff")

# DMA drains before Snooze entry and resumes after the receive wake-up.
si("g_DevNo", 0)
PCLK[0] = HOCO[0] = 32000000
check("DTC Snooze init", call("EmuUartInit", 9600) == 1)
ua.tx.clear()
cut0 = ua.cut
call("EmuUartTx", 24)
check("DTC Snooze enable", call("EmuUartSnooze", 1) == 1)
check("DTC Snooze drains before clock", ua.tx == TXPAT(24) and ua.cut == cut0, str(ua.tx))
ua.tx.clear()
call("EmuUartTx", 24)
standby(lambda: rx_char(0xA5))
check("DTC Snooze drains before standby", ua.tx == TXPAT(24) and ua.cut == cut0, str(ua.tx))
check("DTC Snooze RX", rx_all() == [0xA5])
ua.tx.clear()
call("EmuUartTx", 24)
deliver()
check("DTC after Snooze", ua.tx == TXPAT(24), str(ua.tx))
call("EmuUartPowerOff")
si("g_DmaMode", 0)
si("g_FifoBlocking", 0)

# UARTA without DMA preserves byte interrupt and polling operation.
PCLK[0] = HOCO[0] = 48000000
for devno in (3, 4):
    si("g_DevNo", devno)
    a = uarta[devno - 3]
    for poll in (0, 1):
        si("g_PollMode", poll)
        tag = "UARTA no DMA %d poll %d" % (devno, poll)
        check(tag + " init", call("EmuUartInit", 115200) == 1)
        a.tx.clear()
        check(tag + " TX accepted", call("EmuUartTx", 24) == 24)
        call("EmuUartRate", 115200)
        deliver()
        check(tag + " TX rate change", a.tx == TXPAT(24), str(a.tx))
        uarta_rx(devno - 3, 0xA5)
        deliver()
        check(tag + " RX", rx_all() == [0xA5])
        call("EmuUartPowerOff")
si("g_DevNo", 0)
si("g_PollMode", 0)

# Byte interrupt mode also reports ready once the entire queue has drained.
for devno in (0, 3, 4):
    si("g_DevNo", devno)
    a = ua if devno == 0 else uarta[devno - 3]
    check("IRQ callback init %d" % devno, call("EmuUartInit", 115200) == 1)
    HOLD_TX[0] = True
    a.tx.clear()
    callbacks = gi("g_TxEvt")
    call("EmuUartTx", 24)
    deliver(idle=False)
    check("IRQ no per-byte callback %d" % devno, gi("g_TxEvt") == callbacks)
    HOLD_TX[0] = False
    deliver()
    check("IRQ queue order %d" % devno, a.tx == TXPAT(24), str(a.tx))
    check("IRQ drained callback %d" % devno, gi("g_TxEvt") == callbacks + 1)
    call("EmuUartPowerOff")

# The pulse dispatch shortcut must preserve acknowledgement order, DTC
# ownership and the generation check when a level handler replaces itself.
REQUEUE = [False]
def hook_requeue(u, access, addr, size, value, ud):
    if REQUEUE[0] and value < 32:
        icu.ir[value] = True
        REQUEUE[0] = False
h = uc.hook_add(UC_HOOK_MEM_WRITE, hook_requeue, begin=S("g_IrqSlot"), end=S("g_IrqSlot") + 3)
for level in (0, 1):
    slot = call("EmuIrqInit", level)
    check("IRQ route", 0 <= slot < 32)
    calls = gi("g_IrqCalls")
    icu.ir[slot] = True
    icu.dtce[slot] = True
    call("R9a02IrqDispatch", 19 + slot)
    check("IRQ leaves DTC owner", gi("g_IrqCalls") == calls and icu.ir[slot] and icu.dtce[slot])
    icu.dtce[slot] = False
    call("R9a02IrqDispatch", 19 + slot)
    check("IRQ ack order %d" % level, gi("g_IrqSeen") == level and not icu.ir[slot])
    check("IRQ called once", gi("g_IrqCalls") == calls + 1)
    si("g_IrqMode", 1)
    REQUEUE[0] = True
    event(slot)
    call("R9a02IrqDispatch", 19 + slot)
    check("IRQ replaced same slot", gi("g_IrqSlot") == slot)
    check("IRQ new event preserved %d" % level, not REQUEUE[0] and icu.ir[slot] and clic_ie(slot))
    call("R9a02IrqDispatch", 19 + slot)
    check("IRQ new owner delivered", gi("g_IrqCalls") == calls + 3 and not icu.ir[slot])
    si("g_IrqMode", 2)
    event(slot)
    call("R9a02IrqDispatch", 19 + slot)
    check("IRQ unregister stays released", icu.iels[slot] == 0 and not clic_ie(slot))
    call("EmuIrqRelease")
uc.hook_del(h)

check("no anomalies", not anomalies, str(anomalies[:4]))
print("Snooze characters", ua.snz, "lost", icu.lost, "TX bytes cut", ua.cut)
print("PASS" if not fails else "FAIL %d" % len(fails))
sys.exit(1 if fails else 0)
