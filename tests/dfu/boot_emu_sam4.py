#!/usr/bin/env python3
# Run the cross built SAM4 stage 0 boot (ARM machine code) in unicorn, with a
# model of the flash controller: EEFC on SAM4E16E, FLASHCALW on SAM4L.
#
# boot_emu_sam4.py e|l <boot.elf> <scenario> [timeout s]
#
# e is the SAM4E16E, l the SAM4LC8C / SAM4LS8C. No slot 1 on these parts, so
# the scenarios prepare slot 0 on the host, the way the recovery upload leaves
# it: payload at slot 0, record (magic unit, DfuRecInfo_t, header, TLVs).
#
#   empty   nothing in slot 0: the boot must not start anything, it goes to
#           recovery (DfuMgr::Init reached).
#   good    a signed image in slot 0 with its record: the boot verifies it and
#           starts it (application reset handler reached, SP and VTOR set).
#   tamper  the same with a payload byte changed: refused, recovery.
#   flash   empty, then DfuTgtEraseUnit, DfuTgtErase and DfuTgtWrite called
#           directly on the boot code, source in the flash and unaligned,
#           against the controller model: command sequence, key, page range,
#           busy flash never read or fetched from, cache off and invalidated
#           around each command, NOR programming (a bit never goes 0 -> 1).
#
# The image is made here: a random payload with a vector table for slot 0,
# signed with exemples/dfu/dfu_dev_key.pem, the key DfuBoot holds. Needs
# unicorn, pyelftools and imgtool.
import os, random, struct, subprocess, sys, tempfile
from unicorn import *
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile

T, ELF, SCEN = sys.argv[1], sys.argv[2], sys.argv[3]
TMO = int(sys.argv[4]) if len(sys.argv) > 4 else 900
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

if T == "e":
    NAME = "SAM4E16E"
    FBASE, FSIZE, RAM, RAMSZ = 0x400000, 0x100000, 0x20000000, 0x20000
    CTRL = 0x400E0A00      # EEFC: FMR 0, FCR 4, FSR 8, FRR C
    CACHE = 0x400C4000     # CMCC: CTRL 8, SR C, MAINT0 20
    PAGE, EUNIT = 512, 8192
elif T == "l":
    NAME = "SAM4LC8C"
    FBASE, FSIZE, RAM, RAMSZ = 0, 0x80000, 0x20000000, 0x10000
    CTRL = 0x400A0000      # FLASHCALW: FCR 0, FCMD 4, FSR 8
    CACHE = 0x400A0400     # HCACHE: CTRL 8, SR C, MAINT0 20
    PM_PBBMASK = 0x400E002C
    CHIPID_CIDR = 0x400E0740
    PAGE, EUNIT = 512, 512
else:
    sys.exit("target: e or l")

with open(ELF, "rb") as f:
    elf = ELFFile(f)
    sym = {s.name: s["st_value"]
           for s in elf.get_section_by_name(".symtab").iter_symbols()}
    mem = bytearray(b"\xff" * FSIZE)
    for seg in elf.iter_segments():
        if seg["p_type"] != "PT_LOAD" or seg["p_filesz"] == 0:
            continue
        pa = seg["p_paddr"]
        if FBASE <= pa < FBASE + FSIZE:
            d = seg.data()
            mem[pa - FBASE:pa - FBASE + len(d)] = d

SLOT0, SLOT0_END = sym["__dfu_slot0_start"], sym["__dfu_slot0_end"]
REC, REC_END = sym["__dfu_rec_start"], sym["__dfu_rec_end"]
BOOT, BOOT_END = sym["__dfu_boot_start"], sym["__dfu_boot_end"]
FLAG = sym["__dfu_flag"]
assert "__dfu_slot1_start" not in sym, "wired only: no slot 1"
RECOVERY = sym["_ZN6DfuMgr4InitERK12__DfuMgr_Cfg"] & ~1

# The application image, signed. Payload not a page multiple, so the last
# page holds bytes past it.
tmp = tempfile.mkdtemp()
rnd = random.Random(7)
payload = bytearray(rnd.getrandbits(8) for _ in range(12 * 1024 + 5))
APP_SP, APP_RESET = FLAG, SLOT0 + 0x101
struct.pack_into("<II", payload, 0, APP_SP, APP_RESET)
open(os.path.join(tmp, "app.bin"), "wb").write(payload)
subprocess.run(["imgtool", "sign", "-k", os.path.join(ROOT, "exemples/dfu/dfu_dev_key.pem"),
                "--header-size", "0x20", "--pad-header", "--align", "4",
                "-v", "1.0.0", "-S", hex(SLOT0_END - SLOT0),
                os.path.join(tmp, "app.bin"), os.path.join(tmp, "app_s.bin")],
               check=True, stdout=subprocess.DEVNULL)
IMG = open(os.path.join(tmp, "app_s.bin"), "rb").read()
HDR = struct.unpack_from("<H", IMG, 8)[0]
ISZ = struct.unpack_from("<I", IMG, 12)[0]

def put(addr, data):
    mem[addr - FBASE:addr - FBASE + len(data)] = data

if SCEN in ("good", "tamper"):
    body = IMG[HDR:HDR + ISZ]
    if SCEN == "tamper":
        body = bytearray(body); body[700] ^= 0x10
    put(SLOT0, body)
    tlv = IMG[HDR + ISZ:]
    unit = max(16, PAGE)
    put(REC, struct.pack("<I", 0x43455244) + b"\xff" * (unit - 4))
    put(REC + unit, struct.pack("<HH", HDR, len(tlv)) + IMG[:HDR] + tlv)

uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.mem_map(FBASE, FSIZE, UC_PROT_ALL)
uc.mem_write(FBASE, bytes(mem))
uc.mem_map(RAM, RAMSZ, UC_PROT_ALL)
uc.mem_map(0x40000000, 0x20000000, UC_PROT_ALL)    # peripherals
uc.mem_map(0xE0000000, 0x100000, UC_PROT_ALL)      # system control space

errors = []
st = {"cmd": 0, "erase": 0, "write": 0, "nor_bad": 0, "busy": 0,
      "fmr": 0, "cache_en": 0, "stale": False, "pbbmask": 0}
latch = {}          # address -> (flash word before the store, word stored)
toggle = {}

def rd32(a):
    return struct.unpack("<I", uc.mem_read(a, 4))[0]

def wr32(a, v):
    uc.mem_write(a, struct.pack("<I", v & 0xFFFFFFFF))

def in_flash(a):
    return FBASE <= a < FBASE + FSIZE

# Stores to the flash go to the latch (EEFC) or page buffer (FLASHCALW), not
# to the array: keep the word the array held, put it back at the command.
def hook_flash_w(uc, access, addr, size, value, ud):
    if st["busy"]:
        errors.append("store to flash 0x%x while busy" % addr)
    if size != 4 or addr & 3:
        errors.append("latch store of %d bytes at 0x%x" % (size, addr))
    if st["cache_en"]:
        errors.append("latch store at 0x%x with the cache on" % addr)
    if addr not in latch:
        latch[addr] = (rd32(addr), value & 0xFFFFFFFF)
    else:
        latch[addr] = (latch[addr][0], value & 0xFFFFFFFF)

def restore_latch():
    for a, (old, _) in latch.items():
        wr32(a, old)
    latch.clear()

def program(page_addr):
    for a, (old, new) in list(latch.items()):
        if not (page_addr <= a < page_addr + PAGE):
            errors.append("latch word 0x%x outside page 0x%x" % (a, page_addr))
            continue
        if (old & new) != new:
            st["nor_bad"] += 1
        wr32(a, old & new)
        del latch[a]
    restore_latch()
    st["write"] += 1

def erase(addr, n):
    if addr < BOOT_END and addr + n > BOOT:
        errors.append("erase in the boot at 0x%x" % addr)
    restore_latch()
    uc.mem_write(addr, b"\xff" * n)
    st["erase"] += 1

def command(value):
    if st["busy"]:
        errors.append("command 0x%x while busy" % value)
    # Only the commands that change the array; SystemInit also gives the
    # high speed mode commands.
    page_cmd = (value & 0xFF) in ((0x01, 0x07) if T == "e" else (0x01, 0x02, 0x03))
    if page_cmd and st["cache_en"]:
        errors.append("command 0x%x with the cache on" % value)
    st["cmd"] += 1
    if page_cmd:
        st["stale"] = True
    if T == "e":
        if value >> 24 != 0x5A:
            errors.append("EEFC key 0x%x" % value); return
        cmd, arg = value & 0xFF, (value >> 8) & 0xFFFF
        if (st["fmr"] >> 8) & 0xF != 6:
            errors.append("EEFC command with FWS %d" % ((st["fmr"] >> 8) & 0xF))
        if cmd == 0x01:                          # WP
            program(FBASE + arg * PAGE)
        elif cmd == 0x07:                        # EPA
            code = arg & 3
            pages = 4 << code
            first = arg & ~3
            if first % pages:
                errors.append("EPA first page %d not a multiple of %d" % (first, pages))
            erase(FBASE + first * PAGE, pages * PAGE)
        else:
            errors.append("EEFC command %d" % cmd)
    else:
        if value >> 24 != 0xA5:
            errors.append("FLASHCALW key 0x%x" % value); return
        cmd, page = value & 0x3F, (value >> 8) & 0xFFFF
        if cmd == 0x01:                          # WP
            program(FBASE + page * PAGE)
        elif cmd == 0x02:                        # EP
            erase(FBASE + page * PAGE, PAGE)
        elif cmd == 0x03:                        # CPB
            restore_latch()
        elif cmd in (0x10, 0x11):                # HSEN, HSDIS
            pass
        else:
            errors.append("FLASHCALW command %d" % cmd)
    st["busy"] = 3            # FSR reads before FRDY is back

def hook_periph_w(uc, access, addr, size, value, ud):
    if T == "e" and addr == CTRL + 0:
        st["fmr"] = value
    elif addr == CTRL + 4:
        command(value)
    elif addr == CACHE + 0x08:
        en = value & 1
        if en and st["stale"]:
            errors.append("cache on again without an invalidate")
        st["cache_en"] = en
    elif addr == CACHE + 0x20 and value & 1:
        if st["cache_en"]:
            errors.append("invalidate with the cache on")
        st["stale"] = False
    elif T == "l" and addr == PM_PBBMASK:
        st["pbbmask"] = value

def hook_periph_r(uc, access, addr, size, value, ud):
    if addr == CTRL + 8:                         # FSR
        if st["busy"]:
            st["busy"] -= 1
            wr32(addr, 0)
        else:
            wr32(addr, 1)                        # FRDY, no error
    elif T == "e" and addr == CTRL + 0:
        wr32(addr, st["fmr"])
    elif addr == CACHE + 0x0C:
        wr32(addr, st["cache_en"])
    elif T == "l" and addr == PM_PBBMASK:
        wr32(addr, st["pbbmask"])
    elif T == "l" and addr == CHIPID_CIDR:
        wr32(addr, (0xAB0A09E0 & ~0xF00) | 0xA00)   # NVPSIZ 512 KB
    elif addr not in (CTRL, CTRL + 4, CACHE + 8, CACHE + 0x20):
        # Anything else SystemInit or the UART polls: alternately all set
        # and all clear, so a wait for a bit either way ends.
        t = toggle.get(addr, 0) ^ 1
        toggle[addr] = t
        wr32(addr & ~3, 0xFFFFFFFF if t else 0)

def hook_flash_r(uc, access, addr, size, value, ud):
    if st["busy"]:
        errors.append("flash read at 0x%x while busy" % addr)

def hook_flash_x(uc, addr, size, ud):
    if st["busy"]:
        errors.append("fetch from flash at 0x%x while busy" % addr)
        uc.emu_stop()

def hook_recovery(uc, addr, size, ud):
    uc.emu_stop()

uc.hook_add(UC_HOOK_MEM_WRITE, hook_flash_w, begin=FBASE, end=FBASE + FSIZE - 1)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_periph_w, begin=0x40000000, end=0x5FFFFFFF)
uc.hook_add(UC_HOOK_MEM_READ, hook_periph_r, begin=0x40000000, end=0x5FFFFFFF)
uc.hook_add(UC_HOOK_CODE, hook_recovery, begin=RECOVERY, end=RECOVERY)

sp, pc = struct.unpack("<II", uc.mem_read(BOOT, 8))
uc.reg_write(UC_ARM_REG_SP, sp)
stop = APP_RESET & ~1
uc.emu_start(pc | 1, stop, timeout=TMO * 1000000)

pc = uc.reg_read(UC_ARM_REG_PC) & ~1
started = pc == stop
recovery = pc == RECOVERY
vtor = rd32(0xE000ED08)
print("target %s, scenario %s" % (NAME, SCEN))
print("  boot 0x%x, record 0x%x, slot 0 0x%x - 0x%x, flag 0x%x" %
      (BOOT, REC, SLOT0, SLOT0_END, FLAG))
print("  stopped at 0x%x: %s, sp 0x%x, vtor 0x%x" %
      (pc, "application reset" if started else
       "recovery (DfuMgr::Init)" if recovery else "elsewhere",
       uc.reg_read(UC_ARM_REG_SP), vtor))
print("  commands %d (erases %d, page writes %d)" %
      (st["cmd"], st["erase"], st["write"]))

ok = False
if SCEN == "good":
    ok = started and uc.reg_read(UC_ARM_REG_SP) == APP_SP and vtor == SLOT0
elif SCEN in ("empty", "tamper"):
    ok = recovery
elif SCEN == "flash":
    ok = recovery
    # Direct calls, the fetch and read checks on for the flash.
    uc.hook_add(UC_HOOK_MEM_READ, hook_flash_r, begin=FBASE, end=FBASE + FSIZE - 1)
    uc.hook_add(UC_HOOK_CODE, hook_flash_x, begin=FBASE, end=FBASE + FSIZE - 1)
    RET = FLAG - 0x100

    def call(name, *args):
        for i, a in enumerate(args):
            uc.reg_write(UC_ARM_REG_R0 + i, a & 0xFFFFFFFF)
        uc.reg_write(UC_ARM_REG_SP, FLAG - 0x200)
        uc.reg_write(UC_ARM_REG_LR, RET | 1)
        uc.emu_start(sym[name] | 1, RET, timeout=60 * 1000000)
        if uc.reg_read(UC_ARM_REG_PC) & ~1 != RET:
            errors.append("%s did not return" % name)
        return uc.reg_read(UC_ARM_REG_R0)

    def check(what, cond):
        global ok
        print("  %-4s %s" % ("ok" if cond else "FAIL", what))
        ok = ok and cond

    # The application may have the code cache on: the target turns it off
    # around each command and back on, invalidated.
    st["cache_en"] = 1
    if T == "l":
        st["pbbmask"] |= 2

    check("write unit %d" % PAGE, call("DfuTgtWriteUnit") == PAGE)
    check("erase unit %d in slot 0" % EUNIT, call("DfuTgtEraseUnit", SLOT0) == EUNIT)
    check("erase unit 0 past the flash", call("DfuTgtEraseUnit", FBASE + FSIZE) == 0)
    check("erase unit 0 below the flash", FBASE == 0 or call("DfuTgtEraseUnit", FBASE - 4) == 0)

    # Something programmed in two units of slot 0, then erase the first.
    uc.mem_write(SLOT0, b"\x00" * (2 * EUNIT))
    e0 = st["erase"]
    check("erase of a unit", call("DfuTgtErase", SLOT0) == 1)
    check("  unit reads as ones", bytes(uc.mem_read(SLOT0, EUNIT)) == b"\xff" * EUNIT)
    check("  next unit untouched", bytes(uc.mem_read(SLOT0 + EUNIT, EUNIT)) == b"\x00" * EUNIT)
    check("  one erase command", st["erase"] == e0 + 1)
    check("erase off a unit start refused",
          EUNIT == PAGE or call("DfuTgtErase", SLOT0 + PAGE) == 0)
    check("erase past the flash refused", call("DfuTgtErase", FBASE + FSIZE) == 0)

    # Program 3 pages from the boot code itself, unaligned source, into
    # erased units.
    for a in range(SLOT0, SLOT0 + 2 * EUNIT, EUNIT):
        call("DfuTgtErase", a)
    src = BOOT + 0x101
    n = 3 * PAGE
    w0 = st["write"]
    check("write, source in the flash, unaligned", call("DfuTgtWrite", SLOT0, src, n) == 1)
    check("  data programmed", bytes(uc.mem_read(SLOT0, n)) == bytes(uc.mem_read(src, n)))
    check("  one page command per page", st["write"] == w0 + 3)
    check("  page after untouched", bytes(uc.mem_read(SLOT0 + n, PAGE)) == b"\xff" * PAGE)

    # Program over what is there with other data: NOR keeps the zeros, the
    # read back after the command sees it and the write fails.
    check("write over programmed data refused",
          call("DfuTgtWrite", SLOT0, BOOT + 0x2000, PAGE) == 0)
    st["nor_bad"] = 0
    check("write off a page refused", call("DfuTgtWrite", SLOT0 + 4, BOOT, PAGE) == 0)
    check("write of a part page refused", call("DfuTgtWrite", SLOT0 + n, BOOT, 16) == 0)
    check("write past the flash refused",
          call("DfuTgtWrite", FBASE + FSIZE - PAGE, BOOT, 2 * PAGE) == 0)

if SCEN == "flash":
    print("  %-4s cache on again after the calls" % ("ok" if st["cache_en"] else "FAIL"))
    ok = ok and st["cache_en"] == 1 and not st["stale"]

if st["nor_bad"]:
    errors.append("%d words programmed over zeros" % st["nor_bad"])
for e in errors[:8]:
    print("  error:", e)
ok = ok and not errors
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
