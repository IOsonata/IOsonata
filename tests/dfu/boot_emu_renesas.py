#!/usr/bin/env python3
# Run the cross built stage 0 boot of the Renesas targets in unicorn, with a
# model of their flash sequencer:
#
#   re01    RE01 1500KB, Cortex-M0+, FACI commands (dfu_re01.cpp)
#   r9a02   R9A02G021, RISC-V, low power flash sequencer (dfu_r9a02.cpp)
#
# boot_emu_renesas.py re01|r9a02 <boot.elf> <signed app.bin> [scenario]
#
# Neither part has a slot 1, so the flash is prepared here, or by calling the
# boot's own target layer:
#
#   empty   nothing in slot 0: the boot does not start anything, it goes to
#           recovery (the wire protocol server)
#   start   the payload at slot 0 and its record, built here as dfu_layout.cpp
#           lays it out: the boot verifies it and jumps to the application
#   tamper  as start with one payload byte changed: refused, recovery
#   write   slot 0 and the record hold junk; DfuTgtErase and DfuTgtWrite of
#           the boot, called in the emulator, erase them and write the
#           payload (source in flash) and the record (source in RAM,
#           unaligned); then a reset: the boot starts the application
#   write8k re01 only: as write, with 8 KB blocks everywhere
#   fail    the sequencer reports a program error on the second command:
#           DfuTgtWrite returns false, P/E mode is left, and a later write
#           works
#
# The model checks that every command is issued from RAM with interrupts off,
# in P/E mode, that nothing reads or runs from the code flash while it is in
# P/E mode, that a program lands on erased memory, and the mode entry keys.
#
# SystemInit is skipped (clock setup), everything else runs as built.
# Needs unicorn and pyelftools.
import sys, struct
from unicorn import *
from unicorn.arm_const import *
from unicorn.riscv_const import *
from elftools.elf.elffile import ELFFile

T, ELF, APP = sys.argv[1], sys.argv[2], open(sys.argv[3], "rb").read()
SCEN = sys.argv[4] if len(sys.argv) > 4 else "start"

RV = T == "r9a02"
if T == "re01":
    FLASH, RAM, RAMSZ, WU = 0x180000, 0x20000000, 0x40000, 128
elif RV:
    FLASH, RAM, RAMSZ, WU = 0x40000, 0x20000000, 0x8000, 4
else:
    sys.exit("target: re01 or r9a02")
UNIT = max(16, WU)                  # DfuStateUnit
STOP = 0x000FFF00 if RV else 0x0017FF00   # return address of the calls here

sym = {}
with open(ELF, "rb") as f:
    elf = ELFFile(f)
    for s in elf.get_section_by_name(".symtab").iter_symbols():
        sym.setdefault(s.name, s["st_value"])
    mem = bytearray(b"\xff" * FLASH)
    for seg in elf.iter_segments():
        if seg["p_type"] == "PT_LOAD" and seg["p_filesz"] and seg["p_paddr"] < FLASH:
            d = seg.data()
            mem[seg["p_paddr"]:seg["p_paddr"] + len(d)] = d

SLOT0, SLOT0_END = sym["__dfu_slot0_start"], sym["__dfu_slot0_end"]
REC, REC_END = sym["__dfu_rec_start"], sym["__dfu_rec_end"]
assert "__dfu_slot1_start" not in sym, "wired only: no slot 1 expected"

hdr_size = struct.unpack_from("<H", APP, 8)[0]
img_size = struct.unpack_from("<I", APP, 12)[0]
payload = APP[hdr_size:hdr_size + img_size]
tlvs = APP[hdr_size + img_size:]
plen = (img_size + WU - 1) // WU * WU

def record_body():
    b = struct.pack("<HH", hdr_size, len(tlvs)) + APP[:hdr_size] + tlvs
    return b + b"\xff" * (-len(b) % WU)

def magic_unit():
    return struct.pack("<I", 0x43455244) + b"\xff" * (UNIT - 4)

def pad(b):
    return b + b"\xff" * (plen - len(b))

if SCEN in ("start", "tamper"):
    p = bytearray(pad(payload))
    if SCEN == "tamper":
        p[500] ^= 1
    mem[SLOT0:SLOT0 + plen] = p
    body = record_body()
    mem[REC:REC + UNIT] = magic_unit()
    mem[REC + UNIT:REC + UNIT + len(body)] = body
elif SCEN.startswith("write") or SCEN == "fail":
    # Junk where the image and the record go, so every erase has work to do,
    # and a copy of the payload high in flash as the program source.
    mem[SLOT0:SLOT0 + plen + 0x1000] = b"\x00" * (plen + 0x1000)
    mem[REC:REC + 0x1000] = b"\x5a" * 0x1000
SRC = SLOT0_END - 0x4000 - plen
mem[SRC:SRC + plen] = pad(payload)

if RV:
    uc = Uc(UC_ARCH_RISCV, UC_MODE_RISCV32)
else:
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.mem_map(0, FLASH, UC_PROT_ALL)
uc.mem_write(0, bytes(mem))
if STOP >= FLASH:
    uc.mem_map(STOP & ~0xFFF, 0x1000, UC_PROT_ALL)
uc.mem_map(0x01000000, 0x20000, UC_PROT_ALL)     # option setting area
uc.mem_map(RAM, 0x40000, UC_PROT_ALL)
uc.mem_map(0x40000000, 0x01000000, UC_PROT_ALL)  # peripherals
uc.mem_map(0xE0000000, 0x100000, UC_PROT_ALL)    # SCS / CLIC

errors = []
ev = {"erase": 0, "program": 0, "pe": 0}
st = {"pe": 0, "err": 0, "fail_at": 2 if SCEN == "fail" else 0, "fetch": None}
GEOM = "8k" if SCEN == "write8k" else "ra6"

def pc():
    return uc.reg_read(UC_RISCV_REG_PC if RV else UC_ARM_REG_PC)

def irq_off():
    if RV:
        return (uc.reg_read(UC_RISCV_REG_MSTATUS) & 8) == 0
    return (uc.reg_read(UC_ARM_REG_PRIMASK) & 1) == 1

def check_cmd(what):
    if not (RAM <= pc() < RAM + RAMSZ):
        errors.append("%s issued from 0x%x, not RAM" % (what, pc()))
    if not irq_off():
        errors.append("%s with interrupts on" % what)
    if st["pe"] != 1:
        errors.append("%s outside code flash P/E mode" % what)

def hook_fetch(uc, addr, size, ud):
    errors.append("code flash run at 0x%x in P/E mode" % addr)
    uc.emu_stop()

def hook_flash_r(uc, access, addr, size, value, ud):
    if st["pe"]:
        errors.append("code flash read 0x%x in P/E mode" % addr)

def hook_flash_w(uc, access, addr, size, value, ud):
    errors.append("store to code flash 0x%x" % addr)

def pe(on):
    if on and not st["pe"]:
        st["fetch"] = uc.hook_add(UC_HOOK_CODE, hook_fetch, begin=0, end=FLASH - 1)
        ev["pe"] += 1
    elif not on and st["pe"]:
        uc.hook_del(st["fetch"])
    st["pe"] = 1 if on else 0

def program(addr, data, errbit):
    ev["program"] += 1
    if st["fail_at"] and ev["program"] == st["fail_at"]:
        st["err"] |= errbit
        return
    old = bytes(uc.mem_read(addr, len(data)))
    if old != b"\xff" * len(data):
        errors.append("program 0x%x over non erased memory" % addr)
        st["err"] |= errbit
        return
    uc.mem_write(addr, bytes(data))

def w32(addr, v):
    uc.mem_write(addr, struct.pack("<I", v & 0xFFFFFFFF))

# ---------------------------------------------------------------------------
# RE01: FACI registers of RE01_1500KB.h, commands at 0x407E0000
# ---------------------------------------------------------------------------
FACI_CMD, FASTAT, FSADDR, FSTATR = 0x407E0000, 0x407FE010, 0x407FE030, 0x407FE080
FENTRYR, FPCKAR, FWEPROR = 0x407FE084, 0x407FE0E4, 0x4001E416
fc = {"state": "idle", "n": 0, "data": b"", "fwep": 0, "pcka": 0}

def re01_block(addr):
    b = 0x2000 if (GEOM == "8k" or addr < 0x10000) else 0x8000
    return addr & ~(b - 1), b

def re01_w(uc, access, addr, size, value, ud):
    if addr == FENTRYR:
        if value == 0xAA01:
            pe(True)
        elif value == 0xAA00:
            pe(False)
        else:
            errors.append("FENTRYR 0x%x" % value)
    elif addr == FWEPROR:
        fc["fwep"] = value & 3
    elif addr == FPCKAR:
        if value >> 8 != 0x1E:
            errors.append("FPCKAR key 0x%x" % value)
        fc["pcka"] = value & 0xFF
    elif addr == FASTAT:
        pass
    elif addr == FACI_CMD:
        s = fc["state"]
        if s == "data":
            if size != 2:
                errors.append("program data written with size %d" % size)
            fc["data"] += struct.pack("<H", value & 0xFFFF)
            if len(fc["data"]) == 2 * fc["n"]:
                fc["state"] = "final_p"
            return
        if size != 1:
            errors.append("command written with size %d" % size)
        v = value & 0xFF
        if v in (0x50, 0xB3):
            if v == 0x50:
                st["err"] = 0
            fc["state"] = "idle"
            return
        check_cmd("FACI 0x%02x" % v)
        if fc["fwep"] != 1:
            errors.append("command with FWEPROR %d" % fc["fwep"])
        if fc["pcka"] == 0:
            errors.append("command with FPCKAR unset")
        if s == "idle" and v == 0xE8:
            fc["state"] = "count"
        elif s == "count":
            fc["n"], fc["data"], fc["state"] = v, b"", "data"
            if v != WU // 2:
                errors.append("program count %d" % v)
        elif s == "idle" and v == 0x20:
            fc["state"] = "final_e"
        elif s == "final_p" and v == 0xD0:
            a = struct.unpack("<I", uc.mem_read(FSADDR, 4))[0]
            if a % WU:
                errors.append("program at 0x%x" % a)
            program(a, fc["data"], 1 << 12)
            fc["state"] = "idle"
        elif s == "final_e" and v == 0xD0:
            a = struct.unpack("<I", uc.mem_read(FSADDR, 4))[0]
            base, b = re01_block(a)
            uc.mem_write(base, b"\xff" * b)
            ev["erase"] += 1
            fc["state"] = "idle"
        else:
            errors.append("command 0x%02x in state %s" % (v, s))
            st["err"] |= 1 << 14
            fc["state"] = "idle"

def re01_r(uc, access, addr, size, value, ud):
    if addr == FSTATR:
        w32(FSTATR, 0x8000 | st["err"])
    elif addr == FENTRYR:
        uc.mem_write(FENTRYR, struct.pack("<H", 1 if st["pe"] else 0))
    elif addr == FASTAT:
        uc.mem_write(FASTAT, bytes([0x10 if st["err"] & (1 << 14) else 0]))

# ---------------------------------------------------------------------------
# R9A02: low power flash sequencer at 0x407EC000
# ---------------------------------------------------------------------------
FLP = 0x407EC000
lp = {"fpr": 0, "fpmcr": 0x08, "seq": [], "frdy": 0, "fisr": 0}

def rd16(off):
    return struct.unpack("<H", uc.mem_read(FLP + off, 2))[0]

def r9_w(uc, access, addr, size, value, ud):
    off = addr - FLP
    if off == 0x3FB0:
        if value == 0xAA01:
            pe(True)
        elif value == 0xAA00:
            if lp["fpmcr"] != 0x08:
                errors.append("read mode entered with FPMCR 0x%x" % lp["fpmcr"])
            pe(False)
        else:
            errors.append("FENTRYR 0x%x" % value)
    elif off == 0x180:
        lp["seq"] = [] if value == 0xA5 else None
    elif off == 0x100:
        if lp["seq"] is None:
            errors.append("FPMCR written without FPR unlock")
            return
        lp["seq"].append(value & 0xFF)
        if len(lp["seq"]) == 3:
            a, b, c = lp["seq"]
            if a == c and b == (~a & 0xFF):
                lp["fpmcr"] = a
            else:
                errors.append("FPMCR sequence %s" % lp["seq"])
            lp["seq"] = None
    elif off == 0x1D8:
        lp["fisr"] = value
    elif off == 0x124:
        if value & 1:
            st["err"] = 0
            lp["frdy"] = 0
    elif off == 0x114:
        if value & 0x80:
            cmd = value & 0x7F
            check_cmd("FCR 0x%02x" % value)
            if lp["fpmcr"] != 0x02:
                errors.append("command with FPMCR 0x%x" % lp["fpmcr"])
            if lp["fisr"] == 0:
                errors.append("command with FISR unset")
            if uc.mem_read(FLP + 0x104, 1)[0] != 0:
                errors.append("command with FASR not user area")
            s = rd16(0x110) << 16 | rd16(0x108)
            if cmd == 0x01:
                if s % 4:
                    errors.append("program at 0x%x" % s)
                data = struct.pack("<HH", rd16(0x130), rd16(0x138))
                program(s, data, 0x02)
            elif cmd == 0x04:
                e = rd16(0x120) << 16 | rd16(0x118)
                if s % 0x800 or (e + 1) % 0x800 or e < s:
                    errors.append("erase 0x%x - 0x%x" % (s, e))
                    st["err"] |= 0x10
                else:
                    uc.mem_write(s, b"\xff" * (e + 1 - s))
                    ev["erase"] += 1
            else:
                errors.append("FCR command 0x%x" % cmd)
                st["err"] |= 0x10
            lp["frdy"] = 1
        elif value == 0:
            lp["frdy"] = 0

def r9_r(uc, access, addr, size, value, ud):
    off = addr - FLP
    if off == 0x12C:
        uc.mem_write(addr, bytes([0x40 if lp["frdy"] else 0]))
    elif off == 0x1F0:
        uc.mem_write(addr, struct.pack("<H", st["err"]))
    elif off == 0x3FB0:
        uc.mem_write(addr, struct.pack("<H", 1 if st["pe"] else 0))

# ---------------------------------------------------------------------------

class Reset(Exception):
    pass

class Recovery(Exception):
    pass

def hook_scs_w(uc, access, addr, size, value, ud):
    if addr == 0xE000ED0C and (value >> 16) == 0x05FA:
        raise Reset()

uc.hook_add(UC_HOOK_MEM_READ, hook_flash_r, begin=0, end=FLASH - 1)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_flash_w, begin=0, end=FLASH - 1)
if RV:
    uc.hook_add(UC_HOOK_MEM_WRITE, r9_w, begin=FLP, end=FLP + 0x3FFF)
    uc.hook_add(UC_HOOK_MEM_READ, r9_r, begin=FLP, end=FLP + 0x3FFF)
else:
    uc.hook_add(UC_HOOK_MEM_WRITE, re01_w, begin=0x40000000, end=0x40FFFFFF)
    uc.hook_add(UC_HOOK_MEM_READ, re01_r, begin=0x40000000, end=0x40FFFFFF)
    uc.hook_add(UC_HOOK_MEM_WRITE, hook_scs_w, begin=0xE000E000, end=0xE000EFFF)

def addr_of(name):
    return sym[name] & ~1

# SystemInit: clock setup, returns at once here.
def hook_sysinit(uc, addr, size, ud):
    if RV:
        uc.reg_write(UC_RISCV_REG_PC, uc.reg_read(UC_RISCV_REG_RA))
    else:
        uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))
uc.hook_add(UC_HOOK_CODE, hook_sysinit, begin=addr_of("SystemInit"),
            end=addr_of("SystemInit"))

# Recovery: main sets up the slot 0 store only when it serves SMP.
store_tgt = addr_of("_Z11DfuStoreTgtP11__Dfu_Storejm")
at_main = [False]
def hook_marks(uc, addr, size, ud):
    if addr == store_tgt:
        raise Recovery()
    if addr == addr_of("main") and stop_at_main[0]:
        at_main[0] = True
        uc.emu_stop()
uc.hook_add(UC_HOOK_CODE, hook_marks, begin=store_tgt, end=store_tgt)
uc.hook_add(UC_HOOK_CODE, hook_marks, begin=addr_of("main"), end=addr_of("main"))
stop_at_main = [False]

if RV:
    app_entry = SLOT0
else:
    app_sp, app_pc = struct.unpack_from("<II", payload, 0)
    app_entry = app_pc & ~1

def run_from_reset():
    at_main[0] = False
    if RV:
        # ResetEntry pushes before it sets sp, so give it one.
        uc.reg_write(UC_RISCV_REG_SP, sym["__StackTop"])
        uc.reg_write(UC_RISCV_REG_MSTATUS, 0)
        uc.emu_start(sym["__dfu_boot_start"], app_entry, timeout=1_800_000_000)
    else:
        sp, rpc = struct.unpack("<II", uc.mem_read(sym["__dfu_boot_start"], 8))
        uc.reg_write(UC_ARM_REG_SP, sp)
        uc.reg_write(UC_ARM_REG_PRIMASK, 0)
        uc.emu_start(rpc | 1, app_entry, timeout=1_800_000_000)

def call(name, *args):
    fn = addr_of(name)
    if RV:
        for r, v in zip((UC_RISCV_REG_A0, UC_RISCV_REG_A1, UC_RISCV_REG_A2), args):
            uc.reg_write(r, v)
        uc.reg_write(UC_RISCV_REG_RA, STOP)
        uc.reg_write(UC_RISCV_REG_SP, sym["__StackTop"] - 0x100)
        uc.reg_write(UC_RISCV_REG_MSTATUS, 8)       # MIE on, the target turns it off
        uc.emu_start(fn, STOP, timeout=600_000_000)
        return uc.reg_read(UC_RISCV_REG_A0) & 0xFF
    for r, v in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2), args):
        uc.reg_write(r, v)
    uc.reg_write(UC_ARM_REG_LR, STOP | 1)
    uc.reg_write(UC_ARM_REG_SP, sym["__StackTop"] - 0x100)
    uc.reg_write(UC_ARM_REG_PRIMASK, 0)
    uc.emu_start(fn | 1, STOP, timeout=600_000_000)
    return uc.reg_read(UC_ARM_REG_R0) & 0xFF

# The boot's own target layer, called with the RAM set up by a reset.
calls = []
def prepare_by_target():
    stop_at_main[0] = True
    run_from_reset()
    stop_at_main[0] = False
    assert at_main[0], "main not reached"
    SCR = RAM + 0x6000                 # scratch source buffers, unaligned
    for a in range(SLOT0, SLOT0 + plen + 0x1000, 0x800 if RV else 0x8000):
        calls.append(("erase 0x%x" % a, call("DfuTgtErase", a)))
    for a in range(REC, REC + 0x1000, 0x800 if RV else 0x8000):
        calls.append(("erase rec 0x%x" % a, call("DfuTgtErase", a)))
    if SCEN == "fail":
        r = call("DfuTgtWrite", SLOT0, SRC, 4 * WU)
        calls.append(("write, error injected (want 0)", r))
        st["fail_at"] = 0
        return r == 0 and st["pe"] == 0
    calls.append(("payload from flash 0x%x" % SRC, call("DfuTgtWrite", SLOT0, SRC, plen)))
    body = record_body()
    uc.mem_write(SCR + 1, body)
    calls.append(("record body from RAM+1", call("DfuTgtWrite", REC + UNIT, SCR + 1, len(body))))
    uc.mem_write(SCR + 0x1001, magic_unit())
    calls.append(("record magic", call("DfuTgtWrite", REC, SCR + 0x1001, UNIT)))
    # Refused: not a unit boundary, and past the end of the code flash.
    calls.append(("unaligned erase (want 0)", call("DfuTgtErase", SLOT0 + 4)))
    calls.append(("write past the end (want 0)", call("DfuTgtWrite", FLASH - WU, SCR, 2 * WU)))
    return True

ok = True
started = recovery = False
if SCEN.startswith("write") or SCEN == "fail":
    ok = prepare_by_target()
    for n, r in calls:
        print("  %-40s -> %d" % (n, r))
    want = [0 if "want 0" in n else 1 for n, _ in calls]
    ok = ok and [r for _, r in calls] == want
    if SCEN == "fail":
        # After the failed command the sequencer takes the next write.
        a = SLOT0 + 0x800 if RV else SLOT0 + 0x8000
        r = call("DfuTgtErase", a) and call("DfuTgtWrite", a, SRC, 2 * WU)
        print("  %-40s -> %d" % ("erase + write after the error", r))
        ok = ok and r == 1

if SCEN != "fail":
    for attempt in range(2):
        try:
            run_from_reset()
            started = pc() == app_entry
            break
        except Recovery:
            recovery = True
            break
        except Reset:
            continue

print("target %s, scenario %s" % (T, SCEN))
print("  slot 0 0x%x - 0x%x, record 0x%x, boot 0x%x - 0x%x" %
      (SLOT0, SLOT0_END, REC, sym["__dfu_boot_start"], sym["__dfu_boot_end"]))
print("  commands: %d erase, %d program, %d P/E entries" %
      (ev["erase"], ev["program"], ev["pe"]))
if SCEN != "fail":
    print("  started application: %s, recovery: %s, pc 0x%x" % (started, recovery, pc()))
if SCEN in ("start", "write", "write8k"):
    ok = ok and started and not recovery
    if started and not RV:
        vtor = struct.unpack("<I", uc.mem_read(0xE000ED08, 4))[0]
        sp = uc.reg_read(UC_ARM_REG_SP)
        print("  vtor 0x%x, sp 0x%x (vector table 0x%x)" % (vtor, sp, app_sp))
        ok = ok and vtor == SLOT0 and sp == app_sp
    if started and RV:
        mie = uc.reg_read(UC_RISCV_REG_MSTATUS) & 8
        print("  mstatus.MIE %d at the entry" % (mie >> 3))
        ok = ok and mie == 0
    if SCEN.startswith("write"):
        ok = ok and bytes(uc.mem_read(SLOT0, img_size)) == payload
elif SCEN in ("empty", "tamper"):
    ok = ok and recovery and not started
for e in errors[:8]:
    print("  error:", e)
ok = ok and not errors
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
