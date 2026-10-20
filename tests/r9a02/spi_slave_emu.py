#!/usr/bin/env python3
# R9A02G021 SPI slave check on an emulator (spi_r9a02.cpp, SPI21 = SAU1
# channel 1), with spi_slave_fixture.cpp built by build_spi_slave_emu.py.
#
# The script is the external master. It models what the manual describes:
#   - SAU slave channel in continuous mode (23.5.6, figures 23.25 and 23.26):
#     after SS, CPHA=0 loads the first SDR write into the shift register;
#     CPHA=1 waits for the first SCK edge. Both raise an initial buffer-empty
#     event without received data in continuous mode. CPHA=1 continuous
#     transfers exchange SDR and the shifter at the NEXT byte's first edge,
#     so the final received byte stays in the shifter when SCK stops.
#     In single mode (23.5.6, figure 23.23), reception reaches SDR and raises
#     the event at the final sampling edge, without another SCK edge.
#     Reading received data clears BFF, reading transmit data does
#     not. OVF on a write with BFF at 1, on received data overwritten, on a
#     byte clocked without transmit data.
#   - ICU IELSR: an event sets IR, only software clears it; an event with IR
#     at 1 is lost (table 12.8 note 2). DTCE at 1 gives the events to the DTC.
#   - DTC: normal mode chains (table 14.3) with write back; when the counter
#     of the last record ends, DTCE goes to 0 and IR stays at 1.
# Interrupts are delivered through R9a02IrqDispatch at the first clock edge
# and byte boundaries. This is a register-order model, not a cycle model.
# What the model assumes is to confirm on the board (SpiMasterSlave).
#
# Needs unicorn 2.1 and pyelftools. unicorn does not decode Zba, Zbb and
# Zbs: the firmware is built with -march=rv32imac_zicsr_zifencei.
#
# Copyright (c) 2026, I-SYST inc., all rights reserved. MIT License.
import sys, struct, random
from unicorn import *
from unicorn.riscv_const import *
from elftools.elf.elffile import ELFFile

ELF = sys.argv[1]
STOP = 0x000FFF00
EVT_SAU1_ENDI1 = 0x40
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

CAPACITY = symsize["g_Rx"]

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

# ---- SAU1 channels 0 (SPI20) and 1 (SPI21) as slaves ----
class Slave:
    def __init__(s):
        s.running = False
        s.shift = None
        s.rxshift = None
        s.sdr = 0
        s.bff = False
        s.isrx = False
        s.ovf = False
        s.dat_hi = []
        s.events = 0
        s.snz = 0
SL = {0: Slave(), 1: Slave()}
sl = SL[1]
CH = 1                  # channel the master clocks
U1 = 0x100
EVT_ENDI = {0: 0x3F, 1: 0x40}
EVT_SNZCANCEL = 0x0B
SNZ_ROUTE_IR = [True]   # the Snooze reception end also sets the route IR
SNZ_CANCEL_NORMAL = [False]     # the SELSR0 event also reaches ICU_SNZCANCEL in Normal mode
snzc_normal = [0]

def sl_event(ch):
    SL[ch].events += 1
    event(slot_of(EVT_ENDI[ch]))
    if SNZ_CANCEL_NORMAL[0] and bget(ICU, 0x200, 2) == EVT_ENDI[ch]:
        snzc_normal[0] += 1
        event(slot_of(EVT_SNZCANCEL))

def sau_ch(off, base):
    for ch in (0, 1):
        if off == U1 + base + 2 * ch:
            return ch
    return None

def sau_read(off, size):
    ch = sau_ch(off, 0)
    if ch is not None:
        m = SL[ch]
        v = m.sdr if m.running else bget(SAU, off, 2)
        if m.running and m.bff and m.isrx:
            m.bff = False
        return v
    ch = sau_ch(off, 0x80)
    if ch is not None:
        m = SL[ch]
        return (0x20 if m.bff else 0) | (1 if m.ovf else 0)
    if off == U1 + 0xA0:
        return sum((1 << ch) for ch in (0, 1) if SL[ch].running)
    return bget(SAU, off, size)

def sau_write(off, size, v):
    ch = sau_ch(off, 0)
    if ch is not None:
        assert size == 2, "SDR written by %d bytes" % size
        m = SL[ch]
        if not m.running:
            bset(SAU, off, 2, v)
            return
        if v & 0xFF00:
            m.dat_hi.append(v)
        if m.bff:
            m.ovf = True
        m.sdr = v & 0xFF
        m.isrx = False
        m.bff = True
        if m.shift is None and not second_edge(ch):
            m.shift = m.sdr
            m.bff = False
            if continuous(ch):
                sl_event(ch)
        return
    ch = sau_ch(off, 0x88)
    if ch is not None:
        if v & 1:
            SL[ch].ovf = False
        return
    if off in (U1 + 0xA2, U1 + 0xA4):
        for ch in (0, 1):
            if v & (1 << ch):
                m = SL[ch]
                m.running = off == U1 + 0xA2
                m.bff = False
                m.shift = None
                m.rxshift = None
        return
    bset(SAU, off, size, v)

def snooze_on(ch):
    return ch == 0 and bget(SAU, U1 + 0xB8, 2) & 1

def second_edge(ch):
    return not (bget(SAU, U1 + 0x98 + 2 * ch, 2) & 0x2000)

def continuous(ch):
    return bool(bget(SAU, U1 + 0x90 + 2 * ch, 2) & 1)

def first_edge(ch, service=True):
    m = SL[ch]
    if m.shift is None and second_edge(ch) and m.bff and not m.isrx:
        m.shift = m.sdr
        m.bff = False
        if m.rxshift is not None:
            m.sdr = m.rxshift
            m.rxshift = None
            m.isrx = True
            m.bff = True
        if continuous(ch):
            sl_event(ch)
        if service:
            deliver()

def clock(mosi, ch=None, service_start=True):
    ch = CH if ch is None else ch
    m = SL[ch]
    if not m.running:
        return None
    if snooze_on(ch):
        # Snooze reception (23.5.7): slave reception, single transfer mode,
        # BFF and OVF unchanged with SWC = 1. The end goes to SELSR0.
        scr = bget(SAU, U1 + 0x98 + 2 * ch, 2)
        smr = bget(SAU, U1 + 0x90 + 2 * ch, 2)
        if (scr >> 14) & 3 != 1 or smr & 1:
            anomalies.append("Snooze channel set %04x %04x" % (scr, smr))
        m.sdr = mosi
        m.isrx = True
        m.snz += 1
        m.events += 1
        if SNZ_ROUTE_IR[0]:
            event(slot_of(EVT_ENDI[ch]))
        if bget(ICU, 0x200, 2) == EVT_ENDI[ch]:
            event(slot_of(EVT_SNZCANCEL))
        return 0xFF
    first_edge(ch, service_start)
    if m.shift is None:
        m.ovf = True
        out = 0xFF
    else:
        out = m.shift & 0xFF
    if continuous(ch) and second_edge(ch):
        # No next leading edge: the last received byte cannot be read
        # from SDR. CS alone does not transfer it out of the shifter.
        m.rxshift = mosi
        m.shift = None
        return out
    if continuous(ch):
        if m.bff and not m.isrx:
            m.shift = m.sdr
        else:
            m.ovf = True
            m.shift = None
    else:
        m.shift = None
        if m.bff:
            m.ovf = True
    m.sdr = mosi
    m.isrx = True
    m.bff = True
    sl_event(ch)
    return out

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
PCLK = [48000000]
uc.hook_add(UC_HOOK_CODE, hook_pclk, begin=S("SystemPeriphClockGet"), end=S("SystemPeriphClockGet"))
def hook_sysinit(u, addr, size, ud):
    u.reg_write(UC_RISCV_REG_PC, u.reg_read(UC_RISCV_REG_RA))
uc.hook_add(UC_HOOK_CODE, hook_sysinit, begin=S("SystemInit"), end=S("SystemInit"))

uc.reg_write(UC_RISCV_REG_MSTATUS, 0)
uc.emu_start(entry, S("main"), timeout=2_000_000_000)
assert uc.reg_read(UC_RISCV_REG_PC) == S("main"), "did not reach main"
uc.mem_write(S("SystemCoreClock"), struct.pack("<I", 48000000))
uc.mem_write(0x4001E03C, b"\x01")           # OSCSF.HOCOSF, the HOCO runs

CALL_SP = [None]      # stack of a call made while the target waits in WFI

def call(name, *args):
    for r, v in zip((UC_RISCV_REG_A0, UC_RISCV_REG_A1, UC_RISCV_REG_A2), args):
        uc.reg_write(r, v & 0xFFFFFFFF)
    uc.reg_write(UC_RISCV_REG_RA, STOP)
    uc.reg_write(UC_RISCV_REG_SP, CALL_SP[0] if CALL_SP[0] else S("__StackTop") - 0x200)
    uc.reg_write(UC_RISCV_REG_MSTATUS, 8)
    try:
        uc.emu_start(S(name), STOP, timeout=2_000_000_000, count=5_000_000)
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
def hook_wfi(u, addr, size, ud):
    if size == 4 and struct.unpack("<I", u.mem_read(addr, 4))[0] == WFI:
        wfi_at[0] = addr
        u.emu_stop()
_lpe = S("R9a02LowPowerEnter")
uc.hook_add(UC_HOOK_CODE, hook_wfi, begin=_lpe, end=_lpe + symsize["R9a02LowPowerEnter"] - 1)
XREGS = [UC_RISCV_REG_X1 + i for i in range(31)]
AT_WFI = [None]         # ICU_SNZCANCEL CLIC IE and IR at WFI

def standby(during, first=None):
    uc.mem_write(0x4001E03C, b"\x01")       # OSCSF.HOCOSF, the HOCO runs
    wfi_at[0] = None
    uc.reg_write(UC_RISCV_REG_RA, STOP)
    uc.reg_write(UC_RISCV_REG_SP, S("__StackTop") - 0x200)
    uc.reg_write(UC_RISCV_REG_MSTATUS, 8)
    uc.emu_start(S("EmuStandby"), STOP, timeout=2_000_000_000, count=5_000_000)
    pc = uc.reg_read(UC_RISCV_REG_PC)
    assert wfi_at[0] is not None and pc in (wfi_at[0], wfi_at[0] + 4), "no WFI, pc %x" % pc
    sc = slot_of(EVT_SNZCANCEL)
    AT_WFI[0] = (clic_ie(sc), icu.ir[sc]) if sc is not None else None
    ctx = [uc.reg_read(r) for r in XREGS]
    mst = uc.reg_read(UC_RISCV_REG_MSTATUS)
    CALL_SP[0] = (ctx[1] - 0x100) & ~15
    during()
    deliver(first)
    CALL_SP[0] = None
    for r, v in zip(XREGS, ctx):
        uc.reg_write(r, v)
    uc.reg_write(UC_RISCV_REG_MSTATUS, mst)
    uc.emu_start(wfi_at[0] + 4, STOP, timeout=2_000_000_000, count=5_000_000)
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

def deliver(first=None, limit=200):
    for _ in range(limit):
        p = pending()
        if not p:
            return
        if first is not None:
            pf = slot_of(first)
            if pf in p:
                p = [pf]
        call("R9a02IrqDispatch", 19 + p[0])
    raise AssertionError("interrupt storm")

# ---- frames ----
def frame(mosi, first_at_end=None, late=None, cs_low_before_isr=False):
    # late: set of byte indexes after which interrupts are not delivered
    cs[0] = 0
    miso = []
    for i, b in enumerate(mosi):
        o = clock(b)
        miso.append(0xFF if o is None else o)
        if late is None or i not in late:
            deliver()
    cs[0] = 1
    event(slot_of(EVT_PORT_IRQ2))
    deliver(first_at_end)
    return miso

fails = []
def check(name, cond, info=""):
    if not cond:
        fails.append(name + " " + info)
        print("FAIL", name, info)

def tx_expect(n, txlen, tx):
    return [tx[i] if i < txlen else 0xFF for i in range(n)]

def setbufs(rxlen, txlen, seed):
    assert max(rxlen, txlen) <= CAPACITY
    si("g_RxLen", rxlen)
    si("g_TxLen", txlen)
    tx = bytes((seed * 7 + i * 13 + 1) & 0xFF for i in range(CAPACITY))
    uc.mem_write(S("g_Tx"), tx)
    uc.mem_write(S("g_Rx"), b"\x00" * CAPACITY)
    return tx

def run_frames(label, rxlen, txlen, lengths, first=None):
    for k, n in enumerate(lengths):
        tx = setbufs(rxlen, txlen, k + n)
        # The buffers are taken at arm time: reset applies the new lengths
        call("EmuReset")
        done = gi("g_Done")
        mosi = [random.randrange(256) for _ in range(n)]
        uc.mem_write(S("g_Rx"), b"\x00" * CAPACITY)
        miso = frame(mosi, first, late={n - 1} if first is not None else None)
        cnt = gi("g_Count")
        rx = bytes(uc.mem_read(S("g_Rx"), CAPACITY))
        exp_cnt = -1 if (rxlen > 0 and n > rxlen) else n
        tag = "%s rx%d tx%d n%d" % (label, rxlen, txlen, n)
        check(tag + " event", gi("g_Done") == done + 1, "done %d->%d" % (done, gi("g_Done")))
        check(tag + " count", cnt == exp_cnt, "count %d expected %d" % (cnt, exp_cnt))
        m = min(n, rxlen)
        check(tag + " rx", list(rx[:m]) == mosi[:m], "rx %s" % rx[:m].hex())
        check(tag + " rx untouched", all(x == 0 for x in rx[m:]), "")
        check(tag + " miso", miso == tx_expect(n, txlen, tx), "miso %s" % bytes(miso).hex())

random.seed(1)

# Init checks
check("init 7-bit DMA refused", call("EmuInit", 1 | 2, 0, 1000000) == 0)
check("init no interrupt refused", call("EmuInit", 4, 0, 1000000) == 0)
check("init five pins refused", call("EmuInit", 8, 0, 1000000) == 0)
check("init CS without IRQ refused", call("EmuInit", 0x20, 0, 1000000) == 0)
check("init MAN refused", call("EmuInit", 0x40, 0, 1000000) == 0)
check("init rate above fMCK/8 refused", call("EmuInit", 1, 0, 3000001) == 0)
check("init DMA ok", call("EmuInit", 1, 0, 3000000) == 1)
smr = bget(SAU, U1 + 0x90 + 2 * CH, 2)
check("SMR CCS MD0 bit5", smr & 0x4021 == 0x4021, "SMR %04x" % smr)
check("SDR STCLK 0", bget(SAU, U1 + 2 * CH, 2) & 0xFE00 == 0)
check("SPS PRS 1 (24 MHz)", bget(SAU, U1 + 0xA6, 2) & 0xF == 1 or (bget(SAU, U1 + 0xA6, 2) >> 4) & 0xF == 1,
      "SPS %04x" % bget(SAU, U1 + 0xA6, 2))
check("rate set 3 MHz", call("EmuRate", 3000000) == 3000000)
check("rate above refused", call("EmuRate", 3100000) == 0)
check("master Tx refused", call("EmuMasterTx") == 0)
check("arms after init", gi("g_Arms") >= 1)

lengths = [0, 1, 2, 3, 4, 16, 61, 62, 63, 64, 65, 70, 128]
for dma in (1, 0):
    for mode in range(4):
        check("init dma%d mode%d" % (dma, mode), call("EmuInit", dma, mode, 1000000) == 1)
        label = "dma%d m%d" % (dma, mode)
        check(label + " event selection", continuous(CH) == (mode & 1 == 0))
        if mode & 1:
            for service in (True, False):
                tx = setbufs(64, 64, 1)
                call("EmuReset")
                endi = slot_of(EVT_SAU1_ENDI1)
                check(label + " first TX waits for SCK", sl.shift is None and sl.bff and sl.sdr == tx[0])
                check(label + " DTC ready for transfer end", not icu.ir[endi] and icu.dtce[endi] == bool(dma))
                events_before = sl.events
                dtc_before = dtc_runs[0]
                cs[0] = 0
                first_edge(CH, service)
                check(label + " first byte preserved", sl.shift == tx[0])
                check(label + " first edge has no event", sl.events == events_before and dtc_runs[0] == dtc_before)
                check(label + " first edge does not store RX", bytes(uc.mem_read(S("g_Rx"), 64)) == bytes(64))
                # A partial byte is not a transfer end and is not counted.
                cs[0] = 1
                event(slot_of(EVT_PORT_IRQ2))
                deliver(EVT_PORT_IRQ2)
                check(label + " partial first byte count", gi("g_Count") == 0)
        run_frames(label, 64, 64, lengths)
        run_frames(label + " cs-first", 64, 64, [1, 16, 63, 64], first=EVT_PORT_IRQ2)
        run_frames(label + " sau-first", 64, 64, [1, 16, 63, 64], first=EVT_SAU1_ENDI1)
        if CAPACITY >= 129:
            run_frames(label + " demo", 129, 129, [1, 16, 129, 1, 129])
        for rl, tl in ((64, 10), (10, 64), (10, 10), (0, 20), (20, 0), (0, 0), (1, 1), (2, 3), (3, 2)):
            run_frames(label + " uneven", rl, tl, [1, 2, 3, 5, 9, 10, 11, 12, 13, 20])

# Back to back frames without reset: the event handler gives the same buffers
check("init dma1", call("EmuInit", 1, 0, 1000000) == 1)
tx = setbufs(64, 64, 3)
call("EmuReset")
for n in (5, 1, 64, 2, 64):
    mosi = [random.randrange(256) for _ in range(n)]
    miso = frame(mosi)
    check("b2b n%d" % n, gi("g_Count") == n and list(uc.mem_read(S("g_Rx"), n)) == mosi and
          miso == tx_expect(n, 64, tx), "count %d" % gi("g_Count"))

# Late CPU in interrupt mode: the byte interrupt of byte 3 waits two bytes
check("init dma0 late", call("EmuInit", 0, 0, 1000000) == 1)
tx = setbufs(64, 64, 4)
call("EmuReset")
miso = frame([1, 2, 3, 4, 5, 6, 7, 8], late={3, 4})
check("late int: error", gi("g_Count") == -1, "count %d" % gi("g_Count"))
mosi = list(range(10))
miso = frame(mosi)
check("after late: recovers", gi("g_Count") == 10 and list(uc.mem_read(S("g_Rx"), 10)) == mosi)

# DTC run end interrupt taken late, after the last byte, CS first: the byte
# whose event was lost is read from SDR. Run of 15 (TX 17), frame 16.
check("init dma1 runend", call("EmuInit", 1, 0, 1000000) == 1)
tx = setbufs(64, 17, 5)
call("EmuReset")
mosi = [random.randrange(256) for _ in range(16)]
lost0 = icu.lost
miso = frame(mosi, first_at_end=EVT_PORT_IRQ2, late={14, 15})
check("runend late: event lost", icu.lost == lost0 + 1, "lost %d" % (icu.lost - lost0))
check("runend late: count 16", gi("g_Count") == 16, "count %d" % gi("g_Count"))
check("runend late: rx", list(uc.mem_read(S("g_Rx"), 16)) == mosi)
# Frame that ends with the run, CS first: no extra byte
tx = setbufs(64, 17, 6)
call("EmuReset")
mosi = [random.randrange(256) for _ in range(15)]
miso = frame(mosi, first_at_end=EVT_PORT_IRQ2, late={14})
check("runend at frame end: count 15", gi("g_Count") == 15, "count %d" % gi("g_Count"))
check("runend at frame end: rx", list(uc.mem_read(S("g_Rx"), 15)) == mosi)
check("runend at frame end: miso", miso == tx_expect(15, 17, tx))

# CS already low when the next frame is prepared: that frame reports -1
tx = setbufs(64, 64, 7)
call("EmuReset")
cs[0] = 0
for b in (1, 2, 3):
    clock(b)
    deliver()
cs[0] = 1
event(slot_of(EVT_PORT_IRQ2))
cs[0] = 0          # the master starts again before the slave handler runs
deliver()
for b in (4, 5):
    clock(b)
    deliver()
cs[0] = 1
event(slot_of(EVT_PORT_IRQ2))
deliver()
check("cs low at arm: -1", gi("g_Count") == -1, "count %d" % gi("g_Count"))

# Disable: no event, no channel. Enable: armed again.
done = gi("g_Done")
call("EmuDisable")
check("disabled: channel stopped", not sl.running)
cs[0] = 0
clock(9)
cs[0] = 1
event(slot_of(EVT_PORT_IRQ2))
deliver()
check("disabled: no event", gi("g_Done") == done)
call("EmuEnable")
mosi = [7, 8, 9]
frame(mosi)
check("enable again", gi("g_Count") == 3 and list(uc.mem_read(S("g_Rx"), 3)) == mosi)

# Power off releases the routes, a new Init works
call("EmuPowerOff")
check("power off: routes gone", slot_of(EVT_SAU1_ENDI1) is None and slot_of(EVT_PORT_IRQ2) is None)
check("init after power off", call("EmuInit", 1, 0, 1000000) == 1)
frame([1, 2])
check("frame after power off", gi("g_Count") == 2)

# Reception in Snooze on SPI20 (SAU1 channel 0)
SNZCR = 0x4001E092
def ssc():
    return bget(SAU, U1 + 0xB8, 2)
def ch0_normal():
    return (bget(SAU, U1 + 0x98, 2) >> 14) & 3 == 3 and continuous(0) == (not second_edge(0)) and ssc() == 0

call("EmuPowerOff")
CH = 0
for dma, mode in ((d, m) for d in (1, 0) for m in range(4)):
    tag = "snooze dma%d m%d" % (dma, mode)
    check(tag + " init SPI20", call("EmuInit20", dma, 500000, mode) == 1)
    check(tag + " enable", call("EmuSnooze", 1) == 1)
    check(tag + " SELSR0 SAU1_ENDI0", bget(ICU, 0x200, 2) == EVT_ENDI[0], "%x" % bget(ICU, 0x200, 2))
    snzcr = uc.mem_read(SNZCR, 1)[0]
    check(tag + " SNZCR", (snzcr & 0x08) != 0 and bool(snzcr & 0x02) == bool(dma), "%02x" % snzcr)
    check(tag + " rate above 0.5 Mbps refused while enabled", call("EmuRate", 600000) == 0)
    check(tag + " off for the rate", call("EmuSnooze", 0) == 1 and call("EmuRate", 600000) == 600000)
    check(tag + " enable above 0.5 Mbps refused", call("EmuSnooze", 1) == 0)
    call("EmuRate", 500000)
    check(tag + " enable again", call("EmuSnooze", 1) == 1)
    check(tag + " SNZCANCEL off in Normal mode", not clic_ie(slot_of(EVT_SNZCANCEL)))
    for both, cn in ((True, False), (False, False), (True, True), (False, True)):
        SNZ_ROUTE_IR[0] = both
        SNZ_CANCEL_NORMAL[0] = cn
        vtag = tag + (" route+cancel" if both else " cancel only") + (" cancel in Normal" if cn else "")
        for n in (1, 2, 16, 64):
            tx = setbufs(64, 64, n + dma)
            call("EmuReset")
            mosi = [random.randrange(256) for _ in range(n)]
            miso = []
            snz_before = SL[0].snz
            armed = []
            def during():
                armed.append(ssc() == 1)
                cs[0] = 0
                miso.append(clock(mosi[0]))
            standby(during)
            check(vtag + " n%d set for Snooze" % n, armed == [True] and SL[0].snz == snz_before + 1)
            check(vtag + " n%d SNZCANCEL on at WFI, no request left" % n, AT_WFI[0] == (True, False), str(AT_WFI[0]))
            check(vtag + " n%d back to normal" % n, ch0_normal())
            check(vtag + " n%d SNZCANCEL off after" % n, not clic_ie(slot_of(EVT_SNZCANCEL)))
            for b in mosi[1:]:
                miso.append(clock(b))
                deliver()
            cs[0] = 1
            event(slot_of(EVT_PORT_IRQ2))
            deliver()
            rx = list(uc.mem_read(S("g_Rx"), n))
            check(vtag + " n%d count" % n, gi("g_Count") == n, "count %d" % gi("g_Count"))
            check(vtag + " n%d rx" % n, rx == mosi)
            check(vtag + " n%d miso" % n, miso == [0xFF] + tx_expect(n, 64, tx)[1:], bytes(miso).hex())
    SNZ_CANCEL_NORMAL[0] = False
    SNZ_ROUTE_IR[0] = True
    # Woken without a frame: back to normal, the next frame starts at byte 0
    tx = setbufs(64, 64, 3)
    call("EmuReset")
    standby(lambda: None)
    check(tag + " no frame: back to normal", ch0_normal())
    mosi = [random.randrange(256) for _ in range(10)]
    miso = frame(mosi)
    check(tag + " no frame: next frame", gi("g_Count") == 10 and miso == tx_expect(10, 64, tx) and
          list(uc.mem_read(S("g_Rx"), 10)) == mosi)
    # Chip select low at the standby entry: not set for Snooze
    seen = []
    cs[0] = 0
    standby(lambda: seen.append(ssc()))
    cs[0] = 1
    event(slot_of(EVT_PORT_IRQ2))
    deliver()
    check(tag + " CS low: not set for Snooze", seen == [0] and ch0_normal())
    # One-byte frame, chip select interrupt before the Snooze end
    tx = setbufs(64, 64, 4)
    call("EmuReset")
    b0 = random.randrange(256)
    def during1():
        cs[0] = 0
        clock(b0)
        cs[0] = 1
        event(slot_of(EVT_PORT_IRQ2))
    standby(during1, first=EVT_PORT_IRQ2)
    check(tag + " one byte, CS first", gi("g_Count") == 1 and uc.mem_read(S("g_Rx"), 1)[0] == b0 and ch0_normal(),
          "count %d" % gi("g_Count"))
    # Another Snooze set up replaces it: not set for Snooze any more, and
    # turning the slave's off leaves the other one
    check(tag + " other set up", call("EmuOtherSnooze", 1) == 1)
    tx = setbufs(64, 64, 5)
    call("EmuReset")
    seen = []
    standby(lambda: seen.append(ssc()))
    check(tag + " other: not set for Snooze", seen == [0] and ch0_normal())
    check(tag + " other: kept by off", call("EmuSnooze", 0) == 1 and
          uc.mem_read(0x4001E098, 4) == b"\x08\x00\x00\x00", uc.mem_read(0x4001E098, 4).hex())
    mosi = [random.randrange(256) for _ in range(8)]
    miso = frame(mosi)
    check(tag + " other: frame", gi("g_Count") == 8 and miso == tx_expect(8, 64, tx) and
          list(uc.mem_read(S("g_Rx"), 8)) == mosi)
    call("EmuOtherSnooze", 0)
    check(tag + " enable again 2", call("EmuSnooze", 1) == 1)
    # Off: no Snooze set up left
    check(tag + " off", call("EmuSnooze", 0) == 1 and uc.mem_read(SNZCR, 1)[0] == 0)
    seen = []
    standby(lambda: seen.append(ssc()))
    check(tag + " off: not set for Snooze", seen == [0])
    call("EmuPowerOff")
CH = 1

check("no DAT[8]/STCLK bits written", not SL[0].dat_hi and not SL[1].dat_hi, str(SL[1].dat_hi[:4]))
check("no anomalies", not anomalies, str(anomalies[:4]))
print("DTC runs", dtc_runs[0], "events", SL[0].events + SL[1].events, "lost", icu.lost,
      "SNZCANCEL in Normal mode", snzc_normal[0])
print("PASS" if not fails else "FAIL %d" % len(fails))
sys.exit(1 if fails else 0)
