#!/usr/bin/env python3
# R9A02G021 UART reception in Snooze on an emulator (uart_r9a02.cpp, UART0 =
# SAU0 channels 0 and 1), with uart_snooze_fixture.cpp built by
# build_uart_snooze_emu.py.
#
# The script sends the characters. It models what the manual describes:
#   - SAU UART transmit channel: SDR goes to the shift register when it is
#     empty, with the buffer empty event; BFF while SDR waits, TSF while a
#     character is sent. Stopping the channel loses both, starting it clears
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

ICU, PORT, SAU = 0x40006000, 0x40040000, 0x40094000
MM = (ICU, PORT, SAU)
plain = [(0x40000000, ICU), (ICU + 0x1000, PORT), (PORT + 0x1000, SAU), (SAU + 0x1000, 0x41000000)]
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

# ---- PORT: P202 is the chip select input ----
cs = [1]
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
        s.tx = []
        s.shifting = False
        s.shift = 0
        s.txsdr = 0
        s.txbff = False
        s.cut = 0
        s.snz = 0
        s.snz_set = []
ua = Uart()

def tx_step():
    if ua.shifting:
        ua.tx.append(ua.shift)
        if ua.txbff:
            ua.shift = ua.txsdr
            ua.txbff = False
            event(slot_of(EVT_TX))
        else:
            ua.shifting = False

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
        return (0x20 if ua.bff else 0) | (1 if ua.ovf else 0)
    if off == 0x0A0:
        return (1 if ua.running[0] else 0) | (2 if ua.running[1] else 0)
    return bget(SAU, off, size)

def sau_write(off, size, v):
    if off == 0x000:
        if not ua.running[0]:
            bset(SAU, off, 2, v)
        elif ua.txbff:
            anomalies.append("TX SDR written with BFF set")
        elif not ua.shifting:
            ua.shifting = True
            ua.shift = v & 0xFF
            event(slot_of(EVT_TX))
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
    event(slot_of(EVT_RX))
    if SNZ_CANCEL_NORMAL[0] and bget(ICU, 0x200, 2) == EVT_RX:
        event(slot_of(EVT_SNZCANCEL))
    return True

MODEL = {ICU: (icu_read, icu_write), PORT: (port_read, port_write), SAU: (sau_read, sau_write)}
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
                if idle and ua.shifting:
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

check("no anomalies", not anomalies, str(anomalies[:4]))
print("Snooze characters", ua.snz, "lost", icu.lost, "TX bytes cut", ua.cut)
print("PASS" if not fails else "FAIL %d" % len(fails))
sys.exit(1 if fails else 0)
