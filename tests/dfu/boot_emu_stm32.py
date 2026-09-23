#!/usr/bin/env python3
# Run the cross built STM32 stage 0 boot (ARM machine code) in unicorn, with
# an emulated flash controller, for STM32L476 (L4, double word ECC program,
# 2 KB pages, VTOR start), STM32L4S9 (L4+, DBANK set: two 1 MB banks of
# 4 KB pages, hard float) and STM32F030x8 (F0, half word program, 1 KB
# pages, vector table copied to SRAM and SRAM mapped at 0).
#
# boot_emu_stm32.py l476|l4s9|f030 <boot.elf> <signed app.bin> [scenario [timeout us]]
#
# These layouts have no slot 1: the boot updates slot 0 over the UART. The
# scenarios prepare the memory the way that upload leaves it, then reset:
#
#   empty    nothing in slot 0, no record: the boot does not start anything
#   signed   record and payload put in by the host: the boot verifies the
#            image and starts it
#   tamper   as signed, one payload byte changed: refused, nothing started
#   flash    the boot's own DfuTgtErase and DfuTgtWrite, called in the
#            emulator, erase the record and slot 0 and write the payload from
#            an unaligned source in flash, then the record, magic last; then
#            a reset, and the boot must verify and start the image. The
#            controller model checks the key sequence, the PG/PER/STRT use,
#            alignment, program once per unit between erases (ECC on L4)
#            and that only erased bits are programmed.
#
# The app is signed with the key the boot holds, imgtool --header-size 0x20
# --pad-header --align 4. Needs unicorn and pyelftools.
import sys, struct
from unicorn import *
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile

T = sys.argv[1]
ELF = sys.argv[2]
APP = open(sys.argv[3], "rb").read()
SCEN = sys.argv[4] if len(sys.argv) > 4 else "signed"
TMO = int(sys.argv[5]) if len(sys.argv) > 5 else 600_000_000

FLASH_BASE, FREG, RCC = 0x08000000, 0x40022000, 0x40021000
L4 = T in ("l476", "l4s9")
if L4:
    FLASH, PAGE, UNIT, RAM = 0x100000, 0x800, 8, 0x18000
    if T == "l4s9":
        FLASH, PAGE, RAM = 0x200000, 0x1000, 0xA0000
    FSIZE_REG, CPU = 0x1FFF75E0, UC_CPU_ARM_CORTEX_M4
    KEYR, SR, CR, AR = FREG + 0x08, FREG + 0x10, FREG + 0x14, None
    CR_PG, CR_PER, CR_STRT, CR_LOCK = 1 << 0, 1 << 1, 1 << 16, 1 << 31
    SR_BSY, SR_EOP, SR_PROGERR, SR_PGSERR = 1 << 16, 1 << 0, 1 << 3, 1 << 7
    CFGR, BDCR = RCC + 0x08, RCC + 0x90
elif T == "f030":
    FLASH, PAGE, UNIT, RAM = 0x10000, 0x400, 2, 0x2000
    FSIZE_REG, CPU = 0x1FFFF7CC, UC_CPU_ARM_CORTEX_M0
    KEYR, SR, CR, AR = FREG + 0x04, FREG + 0x0C, FREG + 0x10, FREG + 0x14
    CR_PG, CR_PER, CR_STRT, CR_LOCK = 1 << 0, 1 << 1, 1 << 6, 1 << 7
    SR_BSY, SR_EOP, SR_PROGERR, SR_PGSERR = 1 << 0, 1 << 5, 1 << 2, 0
    CFGR, BDCR = RCC + 0x04, RCC + 0x20
else:
    sys.exit("target: l476, l4s9 or f030")

with open(ELF, "rb") as f:
    elf = ELFFile(f)
    sym = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
    segs = [(seg["p_paddr"], seg.data()) for seg in elf.iter_segments()
            if seg["p_type"] == "PT_LOAD" and seg["p_filesz"]]
SLOT0, SLOT0_END = sym["__dfu_slot0_start"], sym["__dfu_slot0_end"]
REC, REC_END = sym["__dfu_rec_start"], sym["__dfu_rec_end"]
BOOT, BOOT_END, FLAG = sym["__dfu_boot_start"], sym["__dfu_boot_end"], sym["__dfu_flag"]
BSS_END = sym["__bss_end__"]
STATE = max(16, UNIT)

mem = bytearray(b"\xff" * FLASH)
for pa, d in segs:
    if FLASH_BASE <= pa < FLASH_BASE + FLASH:
        mem[pa - FLASH_BASE:pa - FLASH_BASE + len(d)] = d

# Image pieces: header, payload, TLV areas (protected, if any, then the
# unprotected area), as dfu_layout.cpp keeps them in the record.
hdr_size = struct.unpack_from("<H", APP, 8)[0]
prot = struct.unpack_from("<H", APP, 10)[0]
img_size = struct.unpack_from("<I", APP, 12)[0]
toff = hdr_size + img_size
tlv_len = prot + struct.unpack_from("<H", APP, toff + prot + 2)[0]
payload = APP[hdr_size:toff]
body = struct.pack("<HH", hdr_size, tlv_len) + APP[:hdr_size] + APP[toff:toff + tlv_len]
body += b"\xff" * (-len(body) % UNIT)
magic = struct.pack("<I", 0x43455244) + b"\xff" * (STATE - 4)
app_vec = struct.unpack_from("<II", payload, 0)

def put(addr, data):
    mem[addr - FLASH_BASE:addr - FLASH_BASE + len(data)] = data

if SCEN in ("signed", "tamper"):
    put(SLOT0, payload)
    put(REC, magic + body)
    if SCEN == "tamper":
        put(SLOT0 + 300, bytes([mem[SLOT0 + 300 - FLASH_BASE] ^ 1]))

# Where the flash scenario takes its source from: the top of slot 0, one
# byte off word alignment, well past the payload.
SRC = SLOT0_END - ((len(payload) + len(body) + 0x10FF) & ~0xFFF) + 1
if SCEN == "flash":
    assert SRC > SLOT0 + len(payload)
    put(SRC, payload)
    put(SRC + len(payload), body)

uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.ctl_set_cpu_model(CPU)
uc.mem_map(0, 0x10000, UC_PROT_ALL)                # alias of what MEM_MODE maps
uc.mem_map(FLASH_BASE, FLASH, UC_PROT_ALL)
uc.mem_write(FLASH_BASE, bytes(mem))
uc.mem_map(0x1FFF0000, 0x10000, UC_PROT_ALL)       # system memory, size register
uc.mem_write(FSIZE_REG, struct.pack("<H", FLASH >> 10))
uc.mem_map(0x10000000, 0x10000, UC_PROT_ALL)       # L4 SRAM2
uc.mem_map(0x20000000, 0x100000, UC_PROT_ALL)
uc.mem_map(0x40000000, 0x20000000, UC_PROT_ALL)    # peripherals
uc.mem_map(0xE0000000, 0x100000, UC_PROT_ALL)      # system control space
uc.mem_write(CR, struct.pack("<I", CR_LOCK))
if T == "l4s9":
    uc.mem_write(FREG + 0x20, struct.pack("<I", 1 << 22))   # OPTR.DBANK

stats = {"writes": 0, "erases": 0, "resets": 0}
errors = []
ctl = {"key": 0, "prog": {}, "pend": None, "sr": 0}

def rd32(a):
    return struct.unpack("<I", uc.mem_read(a, 4))[0]

# The flash as the controller sees it: a store lands only with PG set and
# the controller unlocked, into erased bits, in program units written once
# since their erase. On L4 a double word is two word stores to its two
# halves, lower first; the unit is programmed when the second one lands.
def hook_flash_w(uc, access, addr, size, value, ud):
    cr = rd32(CR)
    old = int.from_bytes(uc.mem_read(addr, size), "little")
    value &= (1 << (8 * size)) - 1
    stats["writes"] += 1
    if cr & CR_LOCK or not cr & CR_PG:
        errors.append("store 0x%x with CR 0x%x" % (addr, cr))
        return
    if size * 8 != UNIT * 8 and not (UNIT == 8 and size == 4):
        errors.append("store of %d bytes at 0x%x" % (size, addr))
    if old & value != value:
        errors.append("0x%x not erased (0x%x <- 0x%x)" % (addr, old, value))
    u = addr & ~(UNIT - 1)
    if UNIT == 8:
        if addr == u:
            ctl["pend"] = u
            return
        if ctl["pend"] != u:
            errors.append("second word 0x%x without its first" % addr)
        ctl["pend"] = None
    if ctl["prog"].get(u):
        errors.append("unit 0x%x programmed twice" % u)
    ctl["prog"][u] = True

def erase(addr):
    base = addr & ~(PAGE - 1)
    uc.mem_write(base, b"\xff" * PAGE)
    for a in range(base, base + PAGE, UNIT):
        ctl["prog"].pop(a, None)
    stats["erases"] += 1

def hook_periph_w(uc, access, addr, size, value, ud):
    if addr == KEYR:
        seq = {0: 0x45670123, 1: 0xCDEF89AB}
        if ctl["key"] in seq and value == seq[ctl["key"]]:
            ctl["key"] += 1
            if ctl["key"] == 2:
                uc.mem_write(CR, struct.pack("<I", rd32(CR) & ~CR_LOCK))
        else:
            errors.append("bad key 0x%x" % value)
    elif addr == CR:
        old = rd32(CR)
        if old & CR_LOCK and value != old | CR_LOCK and not value & CR_LOCK:
            errors.append("CR write 0x%x while locked" % value)
        if value & CR_LOCK:
            ctl["key"] = 0
        if value & CR_STRT:
            if not value & CR_PER or value & CR_PG:
                errors.append("STRT with CR 0x%x" % value)
            elif L4:
                page = (value >> 3) & 0xFF
                bank = (value >> 11) & 1
                erase(FLASH_BASE + bank * (FLASH // 2) + page * PAGE)
            else:
                erase(rd32(AR))
            # Done at once: STRT reads back clear, EOP is up.
            ctl["sr"] |= SR_EOP
    elif addr == SR:
        # Write one to clear, on the model's copy: the store itself lands
        # after this hook.
        ctl["sr"] &= ~value

# Oscillator ready bits follow their enable bits, the clock switch status
# follows the switch.
RDY = [(0, 1), (8, 10), (16, 17), (24, 25), (26, 27)] if L4 else \
      [(0, 1), (16, 17), (24, 25)]
def hook_periph_r(uc, access, addr, size, value, ud):
    if addr == RCC:
        v = rd32(RCC)
        for on, rdy in RDY:
            v = v | (1 << rdy) if v & (1 << on) else v & ~(1 << rdy)
        uc.mem_write(RCC, struct.pack("<I", v))
    elif addr == CFGR:
        v = rd32(CFGR)
        uc.mem_write(CFGR, struct.pack("<I", (v & ~0xC) | ((v & 3) << 2)))
    elif addr == BDCR:
        v = rd32(BDCR)
        uc.mem_write(BDCR, struct.pack("<I", v | 2 if v & 1 else v & ~2))
    elif addr == SR:
        # Never busy: every operation completes as it starts.
        uc.mem_write(SR, struct.pack("<I", ctl["sr"]))
    elif addr == CR:
        uc.mem_write(CR, struct.pack("<I", rd32(CR) & ~CR_STRT))

class Reset(Exception):
    pass

def hook_scs_w(uc, access, addr, size, value, ud):
    if addr == 0xE000ED0C and (value >> 16) == 0x05FA:   # AIRCR SYSRESETREQ
        raise Reset()

def hook_intr(uc, intno, ud):
    raise Exception("exception %d at 0x%x" % (intno, uc.reg_read(UC_ARM_REG_PC)))

uc.hook_add(UC_HOOK_MEM_WRITE, hook_flash_w, begin=FLASH_BASE, end=FLASH_BASE + FLASH - 1)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_periph_w, begin=0x40000000, end=0x5FFFFFFF)
uc.hook_add(UC_HOOK_MEM_READ, hook_periph_r, begin=0x40000000, end=0x5FFFFFFF)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_scs_w, begin=0xE000E000, end=0xE000EFFF)
uc.hook_add(UC_HOOK_INTR, hook_intr)

stop = app_vec[1] & ~1

# A place that is never code: the end of the boot region, erased.
STOP_CALL = BOOT_END - 0x10

def call(name, *args):
    uc.reg_write(UC_ARM_REG_SP, FLAG)
    for r, a in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
        uc.reg_write(r, a)
    uc.reg_write(UC_ARM_REG_LR, STOP_CALL | 1)
    uc.emu_start(sym[name] | 1, STOP_CALL, timeout=TMO)
    return uc.reg_read(UC_ARM_REG_R0)

if SCEN == "flash":
    # Before the boot runs, as its recovery upload would: a startup first,
    # for its .data and clocks, stopped at main.
    sp, pc = struct.unpack("<II", uc.mem_read(BOOT, 8))
    uc.reg_write(UC_ARM_REG_SP, sp)
    uc.emu_start(pc | 1, sym["main"] & ~1, timeout=TMO)
    ok_ops = True
    for a in range(REC, SLOT0 + ((len(payload) + PAGE - 1) & ~(PAGE - 1)), PAGE):
        ok_ops &= call("DfuTgtErase", a) & 0xFF == 1
    plen = (len(payload) + UNIT - 1) & ~(UNIT - 1)
    ok_ops &= call("DfuTgtWrite", SLOT0, SRC, plen) & 0xFF == 1
    ok_ops &= call("DfuTgtWrite", REC + STATE, SRC + len(payload), len(body)) & 0xFF == 1
    uc.mem_write(0x20000000 + RAM - 0x100, magic)
    ok_ops &= call("DfuTgtWrite", REC, 0x20000000 + RAM - 0x100, STATE) & 0xFF == 1
    # A page in the second bank: the page number counts in its bank, BKER
    # says which. Its twin in the first bank must stay.
    if L4:
        b2 = FLASH_BASE + FLASH // 2 + 5 * PAGE
        b1 = FLASH_BASE + 5 * PAGE
        uc.mem_write(b2, b"\x5a" * PAGE)
        ok_ops &= call("DfuTgtErase", b2) & 0xFF == 1
        ok_ops &= bytes(uc.mem_read(b2, PAGE)) == b"\xff" * PAGE
        ok_ops &= bytes(uc.mem_read(b1, 16)) != b"\xff" * 16
    # A misaligned write is refused.
    ok_ops &= call("DfuTgtWrite", SLOT0 + UNIT // 2, SRC, UNIT) & 0xFF == 0
    print("  target calls: %s, flash writes %d, erases %d" %
          ("ok" if ok_ops else "FAILED", stats["writes"], stats["erases"]))
    if not ok_ops:
        errors.append("a target call failed")

# The stack: RAM above the static data is filled with a pattern, what the
# boot leaves of it is the room it never used.
FILL = 0xA5
def run():
    uc.mem_write(BSS_END, bytes([FILL]) * (FLAG - BSS_END))
    sp, pc = struct.unpack("<II", uc.mem_read(BOOT, 8))
    uc.reg_write(UC_ARM_REG_SP, sp)
    uc.emu_start(pc | 1, stop, timeout=TMO)

for attempt in range(3):
    try:
        run()
        break
    except Reset:
        stats["resets"] += 1
    except UcError as e:
        pc = uc.reg_read(UC_ARM_REG_PC) & ~1
        errors.append("emulation stopped at 0x%x: %s" % (pc, e))
        break

ram = bytes(uc.mem_read(BSS_END, FLAG - BSS_END))
room = next((i for i, b in enumerate(ram) if b != FILL), len(ram))
pc = uc.reg_read(UC_ARM_REG_PC)
sp = uc.reg_read(UC_ARM_REG_SP)
vtor = rd32(0xE000ED08)
started = pc == stop
print("target %s, scenario %s" % (T, SCEN))
print("  stopped at 0x%x (app reset 0x%x), sp 0x%x (app sp 0x%x), vtor 0x%x" %
      (pc, stop, sp, app_vec[0], vtor))
print("  static data ends 0x%x, stack room never used %d of %d bytes" %
      (BSS_END, room, FLAG - BSS_END))
ok = not errors
if SCEN in ("signed", "flash"):
    ok = ok and started and sp == app_vec[0]
    if L4:
        ok = ok and vtor == SLOT0
    else:
        remap = rd32(0x40010000) & 3
        vec = bytes(uc.mem_read(0x20000000, 0xC0)) == payload[:0xC0]
        print("  SYSCFG MEM_MODE %d, vectors in SRAM %s" % (remap, vec))
        ok = ok and remap == 3 and vec
    installed = bytes(uc.mem_read(SLOT0, len(payload))) == payload
    print("  payload in slot 0: %s, record magic 0x%x" % (installed, rd32(REC)))
    ok = ok and installed
else:
    ok = ok and not started
if room == 0:
    errors.append("the stack used all RAM above the static data, and likely"
                  " ran into it")
    ok = False
for e in errors[:6]:
    print("  error:", e)
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
