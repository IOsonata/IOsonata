#!/usr/bin/env python3
# R9A02G021 I2C master on an emulator (i2c_r9a02.cpp IICA1, i2c_sau_r9a02.cpp
# IIC20), with i2c_fixture.cpp built by build_i2c_emu.py.
#
# The two buses are modeled at the bit level: SDA and SCL are wired AND of
# every driver (GPIO, IICA, SAU, slaves) with pull-ups. Slave devices decode
# START, STOP, address, data and ACK from the line changes. The controllers
# follow the manual:
#   - IICA (24.2 to 24.4): STT, address and data written to IICA, events at
#     the 9th clock for the address and with WTIM = 1, at the 8th with
#     WTIM = 0, ACK from ACKE, WREL, SPT, IICBSY, STCF, LREL, pins low in
#     peripheral mode while IICE = 0 (24.2.9). Checked: IICE set with both
#     lines high, ACKE, WTIM and SPIE changed only with IICE = 0 or in a clock
#     stretch, IICF and IICCTLn1 written with IICE = 0, IICA written only in a
#     transmission stretch, SPT and STT not after an acknowledged received byte.
#     PDR_REQUIRED: the IICA drives a pin only in output mode (PDR = 1), the
#     reading of 24.2.9 and figure 24.16; run with and without.
#   - SAU simplified I2C (23.8): SO and CKO drive SDA and SCL while SOE = 0
#     and the channel is stopped, an SDR write makes 8 clocks and the ACK
#     clock, PEF on a NACK in transmission, the ACK driven in reception while
#     SOE = 1, the transfer end event (MD0 = 0). SCL needs PDR = 1 (16.5.1).
#     Checked: SCR and SMR written with the channel stopped, SO and CKO
#     written only while they are under software control.
# Slaves: 0x68 (registers, WHO_AM_I 0x75 = 0x71) and 0x50, both with an
# auto incremented register pointer. A slave can NACK a data byte, or hold
# SDA low as when the master stopped in the middle of a read.
#
# Needs unicorn 2.1 and pyelftools, firmware without Zba, Zbb and Zbs.
#
# Copyright (c) 2026, I-SYST inc., all rights reserved. MIT License.
import sys, struct
from unicorn import *
from unicorn.riscv_const import *
from elftools.elf.elffile import ELFFile

ELF = sys.argv[1]
STOP = 0x000FFF00
IR = 1 << 16
DTCE = 1 << 24

sym = {}
with open(ELF, "rb") as f:
    elf = ELFFile(f)
    entry = elf.header["e_entry"]
    for s in elf.get_section_by_name(".symtab").iter_symbols():
        sym.setdefault(s.name, s["st_value"])
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

ICU, PORT, MSTP, IICA, SAU = 0x40006000, 0x40040000, 0x40047000, 0x40093000, 0x40094000
MM = (ICU, PORT, MSTP, IICA, SAU)
plain = [(0x40000000, ICU), (ICU + 0x1000, PORT), (PORT + 0x1000, MSTP), (MSTP + 0x1000, IICA),
         (SAU + 0x1000, 0x41000000)]
for lo, hi in plain:
    uc.mem_map(lo, hi - lo, UC_PROT_ALL)

back = {b: bytearray(0x1000) for b in MM}
anom = []

def bget(b, off, size):
    return int.from_bytes(back[b][off:off + size], "little")

def bset(b, off, size, v):
    back[b][off:off + size] = (v & ((1 << (8 * size)) - 1)).to_bytes(size, "little")

def note(msg):
    anom.append(msg)

# ---- ICU ----
icu_ir = [False] * 32
icu_iels = [0] * 32

def icu_read(off, size):
    if 0x300 <= off < 0x380:
        n = (off - 0x300) // 4
        return icu_iels[n] | (IR if icu_ir[n] else 0)
    return bget(ICU, off, size)

def icu_write(off, size, v):
    if 0x300 <= off < 0x380:
        n = (off - 0x300) // 4
        icu_iels[n] = v & 0x1F
        icu_ir[n] = icu_ir[n] and bool(v & IR)
        return
    bset(ICU, off, size, v)

HOOKS = S("~s_IcuHook")
def slot_of(evt):
    for n in range(32):
        if uc.mem_read(HOOKS + 16 * n + 12, 1)[0] == evt:
            return n
    return None

def event(evt):
    n = slot_of(evt)
    if n is None or icu_iels[n] == 0:
        note("event 0x%02X without route" % evt)
        return
    icu_ir[n] = True

def clic_ie(n):
    return uc.mem_read(0xE2001000 + 4 * (19 + n) + 1, 1)[0] != 0

def pending():
    return [n for n in range(32) if icu_ir[n] and clic_ie(n)]

# ---- MSTP ----
for o in (0, 4, 8):
    bset(MSTP, o, 4, 0xFFFFFFFF)
def mstp_read(off, size):
    return bget(MSTP, off, size)
def mstp_write(off, size, v):
    bset(MSTP, off, size, v)
def stopped(bit):
    return bool(bget(MSTP, 0, 4) & (1 << bit))

# ---- Pins ----
PODR, PIDR, PDR, PCR, NCODR, PMR = 1, 2, 4, 0x10, 0x40, 1 << 16
PSEL_IICA, PSEL_SAU = 0x07, 0x11
pfs = {}
for p in range(5):
    for n in range(16):
        pfs[(p, n)] = 0
pfs[(2, 3)] = 0x10
for n, v in ((0, 0x10010), (1, 0x10010), (2, 0x10000), (3, 0x10010)):
    pfs[(3, n)] = v
pwpr = [0x80]
PDR_REQUIRED = [True]

class Bus:
    def __init__(s, name, sda, scl):
        s.name, s.sda_pin, s.scl_pin = name, sda, scl
        s.sda, s.scl = 1, 1
        s.slaves = []
        s.starts = s.stops = 0
        s.cmos_high = 0

IICA_BUS = Bus("IICA1", (0, 10), (0, 11))
SAU_BUS = Bus("IIC20", (1, 2), (1, 1))
BUSES = (IICA_BUS, SAU_BUS)

def pin_low(pin):
    v = pfs[pin]
    if not (v & PMR):
        if v & PDR:
            if not (v & PODR):
                return True
            if not (v & NCODR):
                return None         # CMOS high on an I2C line
        return False
    psel = (v >> 24) & 0x1F
    if psel == PSEL_IICA and pin in (IICA_BUS.sda_pin, IICA_BUS.scl_pin):
        return iica.drives_low(pin == IICA_BUS.sda_pin, v)
    if psel == PSEL_SAU and pin in (SAU_BUS.sda_pin, SAU_BUS.scl_pin):
        return sau.drives_low(pin == SAU_BUS.sda_pin, v)
    return False

def level(bus, sda):
    pin = bus.sda_pin if sda else bus.scl_pin
    low = pin_low(pin)
    if low is None:
        bus.cmos_high += 1
        low = False
    for sl in bus.slaves:
        if (sl.sda_low if sda else sl.scl_low):
            if pin_low(pin) is None:
                note("%s %s driven high by the MCU against a slave" % (bus.name, "SDA" if sda else "SCL"))
            low = True
    return 0 if low else 1

def bus_update(bus):
    for _ in range(64):
        sda = level(bus, True)
        scl = level(bus, False)
        if scl != bus.scl:
            bus.scl = scl
            for sl in bus.slaves:
                if scl:
                    sl.rise(bus.sda)
                else:
                    sl.fall()
            continue
        if sda != bus.sda:
            bus.sda = sda
            if bus.scl:
                if sda:
                    bus.stops += 1
                    for sl in bus.slaves:
                        sl.stop()
                    if bus is IICA_BUS:
                        iica.on_stop()
                else:
                    bus.starts += 1
                    for sl in bus.slaves:
                        sl.start()
                    if bus is IICA_BUS:
                        iica.on_start()
            continue
        return
    note("%s does not settle" % bus.name)

def update_all():
    for b in BUSES:
        bus_update(b)

def port_read(off, size):
    if off < 0xA0 and off % 0x20 == 4 and size == 4:
        port = off // 0x20
        v = 0
        for n in range(16):
            lvl = 1
            for b in BUSES:
                if (port, n) == b.sda_pin:
                    lvl = b.sda
                elif (port, n) == b.scl_pin:
                    lvl = b.scl
            v |= lvl << n
        return v
    if off < 0xA0 and off % 0x20 == 0:
        port = off // 0x20
        pdr = sum(1 << n for n in range(16) if pfs[(port, n)] & PDR)
        podr = sum(1 << n for n in range(16) if pfs[(port, n)] & PODR)
        return (pdr | (podr << 16)) & ((1 << (8 * size)) - 1)
    if 0x800 <= off < 0xC00:
        p, n = (off - 0x800) // 0x40, ((off - 0x800) % 0x40) // 4
        v = pfs[(p, n)] & ~PIDR
        for b in BUSES:
            if (p, n) == b.sda_pin and b.sda:
                v |= PIDR
            if (p, n) == b.scl_pin and b.scl:
                v |= PIDR
        return (v >> (8 * (off & 3))) & ((1 << (8 * size)) - 1)
    if off == 0xD03:
        return pwpr[0]
    return bget(PORT, off, size)

def port_write(off, size, v):
    if off == 0xD03:
        nv = (v & 0x80) | ((v & 0x40) if not (pwpr[0] & 0x80) else (pwpr[0] & 0x40))
        pwpr[0] = nv
        return
    if 0x800 <= off < 0xC00:
        if not (pwpr[0] & 0x40):
            note("PFS %x written with PFSWE 0" % off)
            return
        assert size == 4 and off % 4 == 0
        p, n = (off - 0x800) // 0x40, ((off - 0x800) % 0x40) // 4
        old = pfs[(p, n)]
        if (old & PMR) and (v & PMR) and ((old ^ v) & 0x1F000000):
            note("PFS %x PSEL changed with PMR 1" % off)
        pfs[(p, n)] = v & ~PIDR
        update_all()
        return
    if off < 0xA0 and off % 0x20 == 8 and size == 4:
        port = off // 0x20
        for n in range(16):
            if v & (1 << n):
                pfs[(port, n)] |= PODR
            if v & (1 << (n + 16)):
                pfs[(port, n)] &= ~PODR
        update_all()
        return
    if off < 0xA0 and off % 0x20 == 0:
        note("PCNTR1 written")
    bset(PORT, off, size, v)

# ---- Slave device, bit level ----
class Slave:
    def __init__(s, addr, regs=None):
        s.addr = addr
        s.regs = bytearray(256) if regs is None else regs
        s.ptr = 0
        s.state = "idle"
        s.sda_low = False
        s.scl_low = False
        s.nack_after = None         # NACK the data byte after this many
        s.log = []
        s.reset_counts()

    def reset_counts(s):
        s.wr = []
        s.rd = []
        s.nacked_by_master = 0
        s.acked_last_read = False

    def start(s):
        if s.sda_low:
            # Its own pull of SDA, not a START
            return
        s.log.append("S")
        s.state = "addr"
        s.bits = 0
        s.shift = 0
        s.sda_low = False

    def stop(s):
        if s.state in ("tx",) and s.acked_last_read:
            note("slave 0x%02X: STOP after an acknowledged read byte" % s.addr)
        s.log.append("P")
        s.state = "idle"
        s.sda_low = False

    def rise(s, sda):
        if s.state in ("addr", "rx"):
            s.shift = ((s.shift << 1) | sda) & 0x1FF
            s.bits += 1
        elif s.state == "tx":
            s.bits += 1
        elif s.state == "ack_tx":
            s.master_ack = sda == 0

    def fall(s):
        st = s.state
        if st == "addr" and s.bits == 8:
            a = s.shift & 0xFF
            if (a >> 1) == s.addr:
                s.rw = a & 1
                s.log.append("A%02X%s" % (s.addr, "R" if s.rw else "W"))
                s.sda_low = True
                s.state = "ack_addr"
                s.first = True
                s.ndata = 0
            else:
                s.state = "ignore"
        elif st == "ack_addr":
            s.sda_low = False
            if s.rw:
                s.state = "tx"
                s.load()
            else:
                s.state = "rx"
                s.bits = 0
                s.shift = 0
        elif st == "rx" and s.bits == 8:
            b = s.shift & 0xFF
            if s.nack_after is not None and s.ndata >= s.nack_after:
                s.state = "nack_rx"
                s.log.append("N")
                return
            if s.first:
                s.ptr = b
                s.first = False
            else:
                s.regs[s.ptr] = b
                s.wr.append(b)
                s.ptr = (s.ptr + 1) & 0xFF
            s.ndata += 1
            s.sda_low = True
            s.state = "ack_rx"
        elif st == "ack_rx":
            s.sda_low = False
            s.state = "rx"
            s.bits = 0
            s.shift = 0
        elif st == "nack_rx":
            s.state = "ignore"
        elif st == "tx":
            if s.bits < 8:
                s.sda_low = not ((s.byte >> (7 - s.bits)) & 1)
            else:
                s.sda_low = False
                s.state = "ack_tx"
        elif st == "ack_tx":
            s.rd.append(s.byte)
            if s.master_ack:
                s.acked_last_read = True
                s.ptr = (s.ptr + 1) & 0xFF
                s.state = "tx"
                s.load()
            else:
                s.acked_last_read = False
                s.nacked_by_master += 1
                s.ptr = (s.ptr + 1) & 0xFF
                s.state = "ignore"
                s.sda_low = False

    def load(s):
        s.byte = s.regs[s.ptr]
        s.bits = 0
        s.master_ack = False
        s.sda_low = not ((s.byte >> 7) & 1)

    # Holds SDA low as in the middle of a read of 0x00, until 8 clocks pass
    def stuck(s):
        s.state = "tx"
        s.byte = 0
        s.bits = 0
        s.sda_low = True
        s.master_ack = False
        s.acked_last_read = False

def make_slaves(bus):
    a = Slave(0x68)
    for i in range(256):
        a.regs[i] = (i * 7 + 3) & 0xFF
    a.regs[0x75] = 0x71
    b = Slave(0x50)
    for i in range(256):
        b.regs[i] = (0xA5 ^ i) & 0xFF
    bus.slaves = [a, b]
    return a, b

# ---- IICA1, 24.2 to 24.4 ----
SPT, STT, ACKE, WTIM, SPIE, WREL, LREL, IICE = 1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80
MSTS, ALD, TRC, ACKD, STD, SPD = 0x80, 0x40, 0x08, 0x04, 0x02, 0x01
EVT_IIC1 = 0x2C

class Iica:
    def __init__(s):
        s.ctl0 = 0
        s.ctl1 = 0
        s.iicf = 0
        s.wl = s.wh = s.sva = 0
        s.reset()
        s.events = 0

    def reset(s):
        s.iicbsy = 0
        s.stcf = 0
        s.msts = s.trc = s.ackd = s.std = s.spd = s.ald = 0
        s.wait = None
        s.phase = "idle"
        s.sda_out = s.scl_out = 1
        s.shift = 0
        s.master_acked = False
        s.rx_bytes = 0

    def iice(s):
        return bool(s.ctl0 & IICE)

    def drives_low(s, sda, v):
        if not s.iice():
            return True                 # 24.2.9
        out = s.sda_out if sda else s.scl_out
        if out:
            return False
        if PDR_REQUIRED[0] and not (v & PDR):
            s.no_pdr = True
            return False
        return True

    def drive(s, sda=None, scl=None):
        if sda is not None:
            s.sda_out = sda
        if scl is not None:
            s.scl_out = scl
        bus_update(IICA_BUS)

    def ev(s):
        s.events += 1
        event(EVT_IIC1)

    def on_start(s):
        if s.iice():
            s.iicbsy = 1

    def on_stop(s):
        if s.iice():
            s.iicbsy = 0
            s.spd = 1
            s.msts = 0
            if s.ctl0 & SPIE:
                s.ev()

    def write_ctl0(s, v):
        was = s.iice()
        if not was and (v & IICE):
            if not (IICA_BUS.sda and IICA_BUS.scl):
                note("IICE set with SDA or SCL low")
            s.ctl0 = v & 0x9C
            s.reset()
            s.iicbsy = 0 if (s.iicf & 2) else 1
            bus_update(IICA_BUS)
            if v & (STT | SPT | WREL | LREL):
                note("trigger written with IICE 0 to 1")
            return
        if was and not (v & IICE):
            s.ctl0 = v & 0x9C
            s.reset()
            bus_update(IICA_BUS)
            return
        if not was:
            s.ctl0 = v & 0x9C
            return
        if ((v ^ s.ctl0) & (ACKE | WTIM | SPIE)) and s.wait is None:
            note("ACKE, WTIM or SPIE changed outside a clock stretch (%02X to %02X)" % (s.ctl0, v))
        s.ctl0 = v & 0x9C
        trig = v & (STT | SPT | WREL | LREL)
        if bin(trig).count("1") > 1:
            note("more than one trigger in one write %02X" % v)
        if v & LREL:
            s.msts = s.trc = s.ackd = s.std = 0
            s.wait = None
            s.phase = "idle"
            s.drive(1, 1)
        elif v & STT:
            s.start()
        elif v & SPT:
            s.stop()
        elif v & WREL:
            s.wrel()

    def start(s):
        if s.msts and s.wait is not None:
            if s.trc == 0 and s.master_acked:
                note("restart after an acknowledged received byte")
            if s.wait == 8:
                note("restart at the 8th clock stretch")
            if s.phase == "rx" and s.rx_bytes == 0:
                note("restart in master reception before a byte was received")
            s.drive(sda=1)
            s.drive(scl=1)
            s.drive(sda=0)
            s.drive(scl=0)
            s.std = 1
            s.trc = 1
            s.wait = None
            s.phase = "start"
            return
        if s.msts:
            note("STT while master outside a clock stretch")
            return
        if s.iicbsy:
            if s.iicf & 1:
                s.stcf = 1
            else:
                note("start reserved, not modeled")
            return
        if not (IICA_BUS.sda and IICA_BUS.scl):
            note("start with a line low")
        s.drive(sda=0)
        s.drive(scl=0)
        s.msts = s.trc = s.std = 1
        s.stcf = 0
        s.phase = "start"
        s.wait = None

    def stop(s):
        if not s.msts or s.wait is None:
            note("SPT outside a master clock stretch")
            return
        if s.wait == 8:
            note("SPT at the 8th clock stretch")
        if s.trc == 0 and s.master_acked:
            note("stop after an acknowledged received byte")
        if s.phase == "rx" and s.rx_bytes == 0:
            note("stop in master reception before a byte was received")
        s.wait = None
        s.phase = "idle"
        s.drive(sda=0)
        s.drive(scl=1)
        s.drive(sda=1)
        s.trc = 0

    def clock(s):
        s.drive(scl=1)
        b = IICA_BUS.sda
        s.drive(scl=0)
        return b

    def write_iica(s, v):
        if not s.iice():
            note("IICA written with IICE 0")
            return
        if s.msts and s.phase == "start":
            s.send(v, True)
        elif s.msts and s.wait is not None and s.trc:
            s.send(v, False)
        else:
            note("IICA written outside a transmission stretch (phase %s wait %s trc %d)" % (s.phase, s.wait, s.trc))

    def send(s, v, addr):
        s.wait = None
        if not addr and not (s.ctl0 & WTIM):
            note("transmission with WTIM 0")
        for i in range(7, -1, -1):
            bit = (v >> i) & 1
            s.drive(sda=bit)
            s.drive(scl=1)
            if bit and not IICA_BUS.sda:
                s.ald = 1
                s.msts = 0
                s.phase = "idle"
                s.drive(1, 1)
                s.ev()
                return
            s.drive(scl=0)
        s.drive(sda=1)
        s.ackd = 1 if s.clock() == 0 else 0
        s.shift = v
        if addr:
            s.trc = 0 if (v & 1) else 1
            s.phase = "rx" if (v & 1) else "tx"
            s.rx_bytes = 0
        s.master_acked = False
        s.wait = 9
        s.ev()

    def wrel(s):
        if s.wait is None:
            note("WREL outside a clock stretch")
            return
        if s.trc:
            note("WREL in transmission")
            return
        if s.wait == 8:
            s.ack_clock()
            if s.ctl0 & WTIM:
                s.wait = 9
                s.ev()
                return
        s.receive()

    def ack_clock(s):
        ack = bool(s.ctl0 & ACKE)
        s.drive(sda=0 if ack else 1)
        s.clock()
        s.drive(sda=1)
        s.master_acked = ack

    def receive(s):
        s.wait = None
        s.drive(sda=1)
        b = 0
        for _ in range(8):
            b = (b << 1) | s.clock()
        s.shift = b
        s.rx_bytes += 1
        if not (s.ctl0 & WTIM):
            s.wait = 8
            s.ev()
            return
        s.ack_clock()
        s.wait = 9
        s.ev()

    def read(s, off):
        if off == 0x00:
            return s.shift
        if off == 0x01:
            v = (MSTS if s.msts else 0) | (ALD if s.ald else 0) | (TRC if s.trc else 0) | \
                (ACKD if s.ackd else 0) | (STD if s.std else 0) | (SPD if s.spd else 0)
            s.ald = 0
            return v
        if off == 0x02:
            return (s.stcf << 7) | (s.iicbsy << 6) | (s.iicf & 3)
        if off == 0x80:
            return s.ctl0 & 0x9C
        if off == 0x81:
            v = s.ctl1 & 0xCD
            if s.iice():
                v |= (0x20 if IICA_BUS.scl else 0) | (0x10 if IICA_BUS.sda else 0)
            return v
        return {0x82: s.wl, 0x83: s.wh, 0x84: s.sva}.get(off, 0)

    def write(s, off, v):
        if off == 0x00:
            s.write_iica(v)
        elif off == 0x02:
            if s.iice():
                note("IICF written with IICE 1")
            else:
                s.iicf = v & 3
        elif off == 0x80:
            s.write_ctl0(v)
        elif off == 0x81:
            if s.iice() and ((v ^ s.ctl1) & 0x4D):
                note("IICCTL1 written with IICE 1")
            s.ctl1 = v
        elif off in (0x82, 0x83, 0x84):
            if s.iice():
                note("IICWL, IICWH or SVA written with IICE 1")
            if off == 0x82:
                s.wl = v
            elif off == 0x83:
                s.wh = v
            else:
                s.sva = v

iica = Iica()
iica.no_pdr = False

def iica_read(off, size):
    if off >= 0x100:
        if stopped(8):
            note("IICA1 read in module stop")
        return iica.read(off - 0x100)
    return bget(IICA, off, size)

def iica_write(off, size, v):
    if off >= 0x100:
        if stopped(8):
            note("IICA1 written in module stop")
            return
        iica.write(off - 0x100, v & 0xFF)
        return
    bset(IICA, off, size, v)

# ---- SAU1 channel 0, simplified I2C IIC20, 23.8 ----
EVT_SAU1_0 = 0x3F
SAU_SO_LSB = [False]    # SO keeps the last data bit after the ACK slot (manual silent)
PEF, OVF, BFF, TSF = 2, 1, 0x20, 0x40

class SauI2c:
    def __init__(s):
        s.se = 0
        s.so = 0x0F0F
        s.soe = 0
        s.scr = 0x0087
        s.smr = 0x0020
        s.sdr = 0
        s.pef = s.ovf = s.bff = 0
        s.data = 0xFF
        s.events = 0

    def sda_bit(s):
        return s.so & 1

    def scl_bit(s):
        return (s.so >> 8) & 1

    def drives_low(s, sda, v):
        if sda:
            if s.sda_bit() and not (v & NCODR):
                return None             # CMOS high on SDA
            return not s.sda_bit()
        if not (v & PDR):
            return False                # SCL needs PDR = 1 (16.5.1)
        return not s.scl_bit()

    def set_sda(s, b):
        s.so = (s.so & ~1) | b
        bus_update(SAU_BUS)

    def set_scl(s, b):
        s.so = (s.so & ~0x100) | (b << 8)
        bus_update(SAU_BUS)

    def clock(s):
        s.set_scl(1)
        b = SAU_BUS.sda
        s.set_scl(0)
        return b

    def transfer(s, v):
        trxe = (s.scr >> 14) & 3
        if s.scl_bit():
            note("SAU transfer with SCL high")
        if trxe == 2:
            if not s.soe & 1:
                note("SAU transmission with SOE 0")
            for i in range(7, -1, -1):
                s.set_sda((v >> i) & 1)
                s.clock()
            s.set_sda(1)
            if s.clock():
                s.pef = 1
            if SAU_SO_LSB[0]:
                s.so = (s.so & ~1) | (v & 1)
                bus_update(SAU_BUS)
            s.data = v
        elif trxe == 1:
            if v & 0xFF != 0xFF:
                note("SAU reception started with %02X, not FF" % v)
            if s.bff:
                s.ovf = 1
            s.set_sda(1)
            b = 0
            for _ in range(8):
                b = (b << 1) | s.clock()
            ack = bool(s.soe & 1)
            if ack:
                s.set_sda(0)
            s.clock()
            s.data = b
            s.bff = 1
        else:
            note("SAU SDR written with TRXE %d" % trxe)
            return
        s.events += 1
        event(EVT_SAU1_0)

    def read(s, off):
        if off == 0x00:
            if s.se & 1:
                s.bff = 0
                return s.data
            return s.sdr
        if off == 0x80:
            return (PEF if s.pef else 0) | (OVF if s.ovf else 0) | (BFF if s.bff else 0)
        if off == 0x90:
            return s.smr
        if off == 0x98:
            return s.scr
        if off == 0xA0:
            return s.se | 0
        if off == 0xA8:
            return s.so
        if off == 0xAA:
            return s.soe
        return 0

    def write(s, off, v):
        if off == 0x00:
            if s.se & 1:
                s.transfer(v)
            else:
                s.sdr = v
        elif off == 0x88:
            if v & PEF:
                s.pef = 0
            if v & OVF:
                s.ovf = 0
        elif off == 0x90:
            if s.se & 1:
                note("SMR10 written with the channel running")
            s.smr = v
        elif off == 0x98:
            if s.se & 1:
                note("SCR10 written with the channel running")
            s.scr = v
        elif off == 0xA2:
            if v & 1:
                s.se |= 1
                s.bff = 0
        elif off == 0xA4:
            if v & 1:
                s.se &= ~1
        elif off == 0xA8:
            nv = (v & ~0x0C0C) | 0x0C0C
            if (s.soe & 1) and ((nv ^ s.so) & 1):
                note("SO bit 0 written with SOE 1")
                nv = (nv & ~1) | (s.so & 1)
            if (s.se & 1) and ((nv ^ s.so) & 0x100):
                note("CKO bit 0 written with the channel running")
                nv = (nv & ~0x100) | (s.so & 0x100)
            s.so = nv
            bus_update(SAU_BUS)
        elif off == 0xAA:
            s.soe = v & 3
            bus_update(SAU_BUS)

sau = SauI2c()

def sau_read(off, size):
    if off >= 0x100:
        if stopped(20):
            note("SAU1 read in module stop")
        return sau.read(off - 0x100)
    return bget(SAU, off, size)

def sau_write(off, size, v):
    if off >= 0x100:
        if stopped(20):
            note("SAU1 written in module stop")
            return
        sau.write(off - 0x100, v)
        return
    bset(SAU, off, size, v)

MODEL = {ICU: (icu_read, icu_write), PORT: (port_read, port_write), MSTP: (mstp_read, mstp_write),
         IICA: (iica_read, iica_write), SAU: (sau_read, sau_write)}
for b, (r, w) in MODEL.items():
    uc.mmio_map(b, 0x1000, lambda u, o, sz, d, r=r: r(o, sz), None,
                lambda u, o, sz, v, d, w=w: w(o, sz, v), None)

# ---- target ----
PCLK = [32000000]
def hook_pclk(u, addr, size, ud):
    u.reg_write(UC_RISCV_REG_A0, PCLK[0])
    u.reg_write(UC_RISCV_REG_PC, u.reg_read(UC_RISCV_REG_RA))
uc.hook_add(UC_HOOK_CODE, hook_pclk, begin=S("SystemPeriphClockGet"), end=S("SystemPeriphClockGet"))
def hook_ret(u, addr, size, ud):
    u.reg_write(UC_RISCV_REG_PC, u.reg_read(UC_RISCV_REG_RA))
uc.hook_add(UC_HOOK_CODE, hook_ret, begin=S("SystemInit"), end=S("SystemInit"))

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
        if (w & 0xFFFFF07F) == 0xB0002073:
            uc.hook_add(UC_HOOK_CODE, hook_mcycle, begin=a + off, end=a + off)

uc.reg_write(UC_RISCV_REG_MSTATUS, 0)
uc.emu_start(entry, S("main"), count=5_000_000)
assert uc.reg_read(UC_RISCV_REG_PC) == S("main"), "did not reach main"
uc.mem_write(S("SystemCoreClock"), struct.pack("<I", 32000000))

DEPTH = [0]
def call(name, *args, sp=None):
    for r, v in zip((UC_RISCV_REG_A0, UC_RISCV_REG_A1, UC_RISCV_REG_A2), args):
        uc.reg_write(r, v & 0xFFFFFFFF)
    uc.reg_write(UC_RISCV_REG_RA, STOP)
    uc.reg_write(UC_RISCV_REG_SP, sp if sp else S("__StackTop") - 0x400)
    uc.reg_write(UC_RISCV_REG_MSTATUS, 8 if DEPTH[0] == 0 else 0)
    uc.emu_start(S(name), STOP, count=50_000_000)
    assert uc.reg_read(UC_RISCV_REG_PC) == STOP, "%s did not return, pc %x" % (name, uc.reg_read(UC_RISCV_REG_PC))
    v = uc.reg_read(UC_RISCV_REG_A0)
    return v - (1 << 32) if v & 0x80000000 else v

# Interrupts pending with their CLIC input on, taken until none is left
def deliver(limit=2000):
    DEPTH[0] += 1
    try:
        for _ in range(limit):
            p = pending()
            if not p:
                return
            call("R9a02IrqDispatch", 19 + p[0], sp=S("__StackTop") - 0x200)
        raise AssertionError("interrupt storm")
    finally:
        DEPTH[0] -= 1

def gi(name):
    return struct.unpack("<i", uc.mem_read(S(name), 4))[0]

def si(name, v):
    uc.mem_write(S(name), struct.pack("<i", v))

def op(name, *args):
    si("g_EvtCnt", 0)
    si("g_EvtLen", -99)
    n0 = len(anom)
    r = call(name, *args)
    deliver()
    return r

fails = []
def check(name, cond, info=""):
    if not cond:
        fails.append(name + " " + info)
        print("FAIL", name, info)

def idle(bus):
    return bus.sda == 1 and bus.scl == 1

# ---- tests ----
def run_suite(devno, bus, intr, label):
    a, b = make_slaves(bus)
    ctrl = iica if devno == 1 else sau
    t = "%s %s" % (label, "int" if intr else "poll")
    rate = op("EmuI2cInit", devno, 100000, 1 if intr else 0)
    check(t + " init", rate > 0, "rate %d" % rate)
    if rate <= 0:
        return
    check(t + " rate", 90000 <= rate <= 100000, "%d" % rate)
    check(t + " idle after init", idle(bus))

    # Scan as I2CScan
    found = []
    for addr in range(0x08, 0x78):
        if op("EmuI2cProbe", addr):
            found.append(addr)
        if not idle(bus):
            check(t + " idle after probe 0x%02X" % addr, False, "sda %d scl %d" % (bus.sda, bus.scl))
            break
    check(t + " scan", found == [0x50, 0x68], str([hex(x) for x in found]))
    check(t + " probe ends", gi("g_EvtCnt") == 0, "events %d" % gi("g_EvtCnt"))

    # Start and stop in the read direction, no data: the slave transmits a
    # byte with bit 7 at 0, the master reads it without ACK before the stop
    a.ptr = 0x02
    a.regs[0x02] = 0x15
    a.reset_counts()
    r = op("EmuI2cProbeRx", 0x68)
    check(t + " probe rx", r == 1 and idle(bus), "ret %d sda %d scl %d" % (r, bus.sda, bus.scl))
    check(t + " probe rx one byte not acknowledged", a.rd == [0x15] and a.nacked_by_master == 1,
          "%s %d" % (a.rd, a.nacked_by_master))
    a.regs[0x02] = (0x02 * 7 + 3) & 0xFF

    # Register read with restart, 1 and 4 bytes
    for (reg, n) in ((0x75, 1), (0x10, 4), (0x20, 2)):
        a.reset_counts()
        r = op("EmuI2cRead", 0x68, reg, n)
        data = list(uc.mem_read(S("g_RxBuf"), n))
        exp = list(a.regs[reg:reg + n])
        evts = gi("g_EvtCnt")
        cnt = gi("g_EvtLen") if intr else r
        check(t + " read %02X x%d data" % (reg, n), data == exp, "%s %s" % (data, exp))
        check(t + " read %02X x%d count" % (reg, n), cnt == n, "ret %d evt %d len %d" % (r, evts, gi("g_EvtLen")))
        if intr:
            check(t + " read %02X x%d one event" % (reg, n), evts == 1 and r == -1, "ret %d evt %d" % (r, evts))
        check(t + " read %02X x%d last NACK" % (reg, n), a.nacked_by_master == 1, "%d" % a.nacked_by_master)
        check(t + " read %02X x%d idle" % (reg, n), idle(bus))

    # Register write
    a.reset_counts()
    vals = [0x11, 0x22, 0x33, 0x44]
    uc.mem_write(S("g_TxBuf"), bytes(vals))
    r = op("EmuI2cWrite", 0x68, 0x40, 4)
    check(t + " write data", list(a.regs[0x40:0x44]) == vals, str(list(a.regs[0x40:0x44])))
    if intr:
        check(t + " write event", gi("g_EvtCnt") == 1 and gi("g_EvtLen") == 5,
              "evt %d len %d ret %d" % (gi("g_EvtCnt"), gi("g_EvtLen"), r))
    else:
        check(t + " write count", r == 4, "%d" % r)
    check(t + " write idle", idle(bus))

    # Receive without command, continues at the pointer
    a.reset_counts()
    a.ptr = 0x40
    r = op("EmuI2cRx", 0x68, 3)
    data = list(uc.mem_read(S("g_RxBuf"), 3))
    check(t + " rx data", data == [0x11, 0x22, 0x33], str(data))
    check(t + " rx count", (gi("g_EvtLen") if intr else r) == 3, "ret %d len %d" % (r, gi("g_EvtLen")))
    check(t + " rx idle", idle(bus))

    # Data NACK after 2 bytes
    a.reset_counts()
    a.nack_after = 3                    # pointer byte then 2 data bytes
    uc.mem_write(S("g_TxBuf"), bytes([0x55, 0x66, 0x77, 0x88]))
    r = op("EmuI2cTx", 0x68, 4)
    a.nack_after = None
    acked = gi("g_EvtLen") if intr else r
    check(t + " tx nack count", acked == 3, "ret %d len %d" % (r, gi("g_EvtLen")))
    check(t + " tx nack idle", idle(bus))

    # Address NACK in a read
    r = op("EmuI2cRead", 0x33, 0x00, 2)
    check(t + " read absent", (r <= 0) and gi("g_EvtCnt") <= 1, "ret %d evt %d" % (r, gi("g_EvtCnt")))
    check(t + " read absent idle", idle(bus))

    # A slave left in the middle of a read holds SDA low
    a.stuck()
    bus_update(bus)
    check(t + " stuck setup", bus.sda == 0)
    op("EmuI2cReset")
    check(t + " reset recovers", idle(bus), "sda %d scl %d" % (bus.sda, bus.scl))
    r = op("EmuI2cRead", 0x68, 0x75, 1)
    check(t + " read after recovery", uc.mem_read(S("g_RxBuf"), 1)[0] == 0x71, "ret %d" % r)

    # Rate change and fast mode
    r = op("EmuI2cRate", 400000)
    check(t + " rate 400k", 300000 <= r <= 400000, "%d" % r)
    r = op("EmuI2cRead", 0x50, 0x07, 2)
    data = list(uc.mem_read(S("g_RxBuf"), 2))
    check(t + " read at 400k", data == list(b.regs[7:9]), str(data))
    check(t + " idle at end", idle(bus))
    op("EmuI2cPowerOff")

def run_all(pdr_req):
    PDR_REQUIRED[0] = pdr_req
    for intr in (False, True):
        n0 = len(anom)
        run_suite(1, IICA_BUS, intr, "IICA1%s" % (" PDR" if pdr_req else ""))
        if iica.no_pdr:
            note("IICA output on a pin with PDR 0")
            iica.no_pdr = False
        check("IICA1 %s %s anomalies" % ("PDR" if pdr_req else "", "int" if intr else "poll"),
              len(anom) == n0, "; ".join(sorted(set(anom[n0:]))[:6]))

# Init checks of the IICA settings at 100 kHz and PCLKB 32 MHz (24.3.2)
def iica_settings():
    make_slaves(IICA_BUS)
    r = op("EmuI2cInit", 1, 100000, 0)
    fmck = 16000000
    wl = -(-fmck * 47 // (100 * 100000))
    wh = -(-fmck * 53 // (100 * 100000))
    check("IICA1 IICCTL1", iica.ctl1 == 0x01, "%02X" % iica.ctl1)
    check("IICA1 IICWL IICWH", (iica.wl, iica.wh) == (wl, wh), "%d %d, %d %d" % (iica.wl, iica.wh, wl, wh))
    check("IICA1 IICF", iica.iicf == 3, "%02X" % iica.iicf)
    check("IICA1 SDA output", (pfs[(0, 10)] & (PDR | NCODR | PMR)) == (PDR | NCODR | PMR), "%08X" % pfs[(0, 10)])
    check("IICA1 SCL output", (pfs[(0, 11)] & (PDR | NCODR | PMR)) == (PDR | NCODR | PMR), "%08X" % pfs[(0, 11)])
    op("EmuI2cPowerOff")

iica_settings()
for req in (False, True):
    run_all(req)

for lsb in (False, True):
    SAU_SO_LSB[0] = lsb
    for intr in (False, True):
        n0 = len(anom)
        run_suite(6, SAU_BUS, intr, "IIC20%s" % (" SO-LSB" if lsb else ""))
        check("IIC20 %s %s anomalies" % ("SO-LSB" if lsb else "", "int" if intr else "poll"), len(anom) == n0,
              "; ".join(sorted(set(anom[n0:]))[:6]))

print("IICA events %d, SAU events %d, IICA bus starts %d stops %d, SAU bus starts %d stops %d" %
      (iica.events, sau.events, IICA_BUS.starts, IICA_BUS.stops, SAU_BUS.starts, SAU_BUS.stops))
if fails:
    print("%d checks failed" % len(fails))
    sys.exit(1)
print("PASS")
