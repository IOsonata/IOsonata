#!/usr/bin/env python3
# Run the cross built stage 0 boot (ARM machine code) of an NXP LPC target in
# unicorn, with the IAP ROM modelled in Python: LPC11U35 (Cortex-M0, 4 KB
# sectors, SYSMEMREMAP start) and LPC1769 (Cortex-M3, 4 KB then 32 KB
# sectors, VTOR start).
#
# boot_emu_lpc.py 11u35|1769 <boot.elf> [scenario]
#
# These layouts have no slot 1: an image reaches slot 0 over the UART in
# recovery. The flash is prepared here the way a finished upload leaves it,
# payload in slot 0 and the record (magic unit, DfuRecInfo_t, header, TLVs)
# built in Python, with an application signed by imgtool with
# exemples/dfu/dfu_dev_key.pem, the key of exemples/dfu/dfu_boot_key.c.
#
# scenario:
#   good    signed image and record: the boot verifies it and starts it
#   tamper  a payload byte changed: refused, the boot goes to recovery
#   empty   nothing in slot 0: no start, the boot goes to recovery
#   iap     from main, the target functions called directly: erase, write
#           from an unaligned flash and RAM source, argument checks
#
# Recovery shows as the first UART register write. The IAP model checks
# that interrupts are off (PRIMASK) at each call, that a sector is prepared
# before an erase or a copy, the copy arguments (256 byte destination, word
# aligned RAM source, count), that the clock argument is SystemCoreClock in
# kHz, and that nothing but the ROM would touch the top 32 bytes of RAM.
# On LPC11U35 the USB SRAM counts as unclocked until SYSAHBCLKCTRL enables it,
# and no code may run in the first 512 bytes once SYSMEMREMAP maps RAM there.
# The unused RAM is filled with a pattern to find the deepest stack use.
#
# Needs unicorn, pyelftools and imgtool.
import os, sys, struct, random, subprocess, tempfile
from unicorn import *
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile

T = sys.argv[1]
ELF = sys.argv[2]
SCEN = sys.argv[3] if len(sys.argv) > 3 else "good"
KEY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "../../exemples/dfu/dfu_dev_key.pem")

if T == "11u35":
    FLASH = 0x10000
    SECTORS = [(i * 0x1000, 0x1000) for i in range(16)]
    RAM, RAMSZ = 0x10000000, 0x2000
    UART = 0x40008000
    CORE = "LPC11U35"
elif T == "1769":
    FLASH = 0x80000
    SECTORS = [(i * 0x1000, 0x1000) for i in range(16)] + \
              [(0x10000 + i * 0x8000, 0x8000) for i in range(14)]
    RAM, RAMSZ = 0x10000000, 0x8000
    UART = 0x4000C000
    CORE = "LPC1769"
else:
    sys.exit("target: 11u35 or 1769")

IAP_TOP = RAM + RAMSZ - 32          # ROM work area, 32 bytes
IAP_ENTRY = 0x1FFF1FF0
RET = 0x1FFF0000                    # where called functions return to
UNIT = 256                          # write unit, also the state unit

sym = {}
with open(ELF, "rb") as f:
    elf = ELFFile(f)
    for s in elf.get_section_by_name(".symtab").iter_symbols():
        sym[s.name] = s["st_value"]
    mem = bytearray(b"\xff" * FLASH)
    for seg in elf.iter_segments():
        if seg["p_type"] == "PT_LOAD" and seg["p_filesz"] and seg["p_paddr"] < FLASH:
            d = seg.data()
            mem[seg["p_paddr"]:seg["p_paddr"] + len(d)] = d

SLOT0, SLOT0_END = sym["__dfu_slot0_start"], sym["__dfu_slot0_end"]
REC, REC_END = sym["__dfu_rec_start"], sym["__dfu_rec_end"]
BOOT = sym["__dfu_boot_start"]
FLAG = sym["__dfu_flag"]
STACK_TOP = sym["__StackTop"]
BSS_END = sym["__bss_end__"]

# Application: a vector table for slot 0 and filler, signed.
app_sp = IAP_TOP                    # the application stack top, as its script sets it
app_reset = SLOT0 + 0x101
rnd = random.Random(7)
body = bytearray(rnd.getrandbits(8) for _ in range(6000))
struct.pack_into("<II", body, 0, app_sp, app_reset)
with tempfile.TemporaryDirectory() as td:
    open(td + "/app.bin", "wb").write(body)
    subprocess.run(["imgtool", "sign", "-k", KEY, "--header-size", "0x20",
                    "--pad-header", "--align", "4", "-v", "1.0.0", "-S",
                    hex(SLOT0_END - SLOT0 + 0x1000), td + "/app.bin",
                    td + "/app_signed.bin"], check=True, stdout=subprocess.DEVNULL)
    IMG = open(td + "/app_signed.bin", "rb").read()

hdr_size = struct.unpack_from("<H", IMG, 8)[0]
img_size = struct.unpack_from("<I", IMG, 12)[0]
payload = IMG[hdr_size:hdr_size + img_size]
tlvs = IMG[hdr_size + img_size:]

def put_image(pl):
    mem[SLOT0:SLOT0 + len(pl)] = pl
    rec = bytearray(b"\xff" * (REC_END - REC))
    struct.pack_into("<I", rec, 0, 0x43455244)              # "DREC"
    body = struct.pack("<HH", hdr_size, len(tlvs)) + IMG[:hdr_size] + tlvs
    rec[UNIT:UNIT + len(body)] = body
    mem[REC:REC_END] = rec

if SCEN == "good":
    put_image(payload)
elif SCEN == "tamper":
    pl = bytearray(payload); pl[300] ^= 1; put_image(bytes(pl))

uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.mem_map(0, FLASH, UC_PROT_ALL)
uc.mem_write(0, bytes(mem))
uc.mem_map(RAM, 0x10000, UC_PROT_ALL)
uc.mem_map(0x1FFF0000, 0x10000, UC_PROT_ALL)            # boot ROM
uc.mem_write(IAP_ENTRY, struct.pack("<H", 0x4770))      # bx lr
uc.mem_write(RET, struct.pack("<H", 0xBE00))            # bkpt, never run
uc.mem_map(0x20000000, 0x100000, UC_PROT_ALL)           # USB and AHB SRAM
uc.mem_map(0x40000000, 0x20000000, UC_PROT_ALL)         # peripherals
uc.mem_map(0xE0000000, 0x100000, UC_PROT_ALL)           # system control space

# RAM pattern between the end of .bss and the stack top: the lowest word
# changed at the end is the deepest the stack went.
PAT = 0xA5A5A5A5
for a in range(BSS_END, STACK_TOP, 4):
    uc.mem_write(a, struct.pack("<I", PAT))
uc.mem_write(FLAG, b"\0" * 16)

errors = []
stats = {"prep": 0, "erase": 0, "copy": 0, "bad_prog": 0, "khz": 0}
prepared = set()

def sect_of(addr):
    for i, (a, s) in enumerate(SECTORS):
        if a <= addr < a + s:
            return i
    return -1

def rd32(a):
    return struct.unpack("<I", uc.mem_read(a, 4))[0]

def hook_iap(uc, addr, size, ud):
    cmd = struct.unpack("<5I", uc.mem_read(uc.reg_read(UC_ARM_REG_R0), 20))
    res = uc.reg_read(UC_ARM_REG_R1)
    st = 0
    if uc.reg_read(UC_ARM_REG_PRIMASK) != 1:
        errors.append("IAP %d with interrupts on" % cmd[0])
    khz = rd32(sym["SystemCoreClock"]) // 1000
    stats["khz"] = khz
    if cmd[0] == 50:                                    # prepare
        stats["prep"] += 1
        if not (0 <= cmd[1] <= cmd[2] < len(SECTORS)):
            st = 7
        else:
            prepared.update(range(cmd[1], cmd[2] + 1))
    elif cmd[0] == 52:                                  # erase sectors
        stats["erase"] += 1
        if cmd[3] != khz or khz == 0:
            errors.append("erase clock %d kHz, SystemCoreClock %d kHz" % (cmd[3], khz))
        if not (0 <= cmd[1] <= cmd[2] < len(SECTORS)):
            st = 7
        elif not set(range(cmd[1], cmd[2] + 1)) <= prepared:
            st = 9
        else:
            for s in range(cmd[1], cmd[2] + 1):
                a, n = SECTORS[s]
                uc.mem_write(a, b"\xff" * n)
        prepared.clear()
    elif cmd[0] == 51:                                  # copy RAM to flash
        stats["copy"] += 1
        dst, src, cnt = cmd[1], cmd[2], cmd[3]
        if cmd[4] != khz or khz == 0:
            errors.append("copy clock %d kHz, SystemCoreClock %d kHz" % (cmd[4], khz))
        if dst % 256:
            st = 3
        elif src % 4:
            st = 2
        elif not (RAM <= src < RAM + RAMSZ or 0x20000000 <= src < 0x20100000):
            st = 4
        elif cnt not in (256, 512, 1024, 4096):
            st = 6
        elif sect_of(dst) not in prepared or sect_of(dst + cnt - 1) not in prepared:
            st = 9
        else:
            old = bytes(uc.mem_read(dst, cnt))
            new = bytes(uc.mem_read(src, cnt))
            if any(o != 0xFF for o in old):
                stats["bad_prog"] += 1
            uc.mem_write(dst, bytes(o & n for o, n in zip(old, new)))
        prepared.clear()
    else:
        st = 1
    uc.mem_write(res, struct.pack("<I", st))

def hook_ram_w(uc, access, addr, size, value, ud):
    if IAP_TOP <= addr < RAM + RAMSZ:
        errors.append("write 0x%x in the IAP area, pc 0x%x" %
                      (addr, uc.reg_read(UC_ARM_REG_PC)))

# LPC11U USB SRAM: clocked by SYSAHBCLKCTRL bit 27, off at reset.
def hook_usbram(uc, access, addr, size, value, ud):
    if T == "11u35" and not rd32(0x40048080) & (1 << 27):
        errors.append("USB SRAM 0x%x with its clock off, pc 0x%x" %
                      (addr, uc.reg_read(UC_ARM_REG_PC)))

# LPC11U SYSMEMREMAP user RAM mode maps SRAM0 over the first 512 bytes:
# boot code there must not run once it is set.
def hook_remap(uc, addr, size, ud):
    if T == "11u35" and rd32(0x40048000) == 1:
        errors.append("code at 0x%x run with RAM mapped at 0" % addr)
        uc.emu_stop()

class Recovery(Exception):
    pass

def hook_uart_w(uc, access, addr, size, value, ud):
    raise Recovery()

def hook_periph_r(uc, access, addr, size, value, ud):
    # PLL status registers polled by SystemInit read as locked.
    if T == "11u35" and addr in (0x4004800C, 0x40048014):
        uc.mem_write(addr, struct.pack("<I", 1))
    if T == "1769" and addr == 0x400FC088:
        con = rd32(0x400FC080)
        uc.mem_write(addr, struct.pack("<I", ((con & 3) << 24) |
                                        ((con & 1) << 26)))

class Reset(Exception):
    pass

def hook_scs_w(uc, access, addr, size, value, ud):
    if addr == 0xE000ED0C and (value >> 16) == 0x05FA:
        raise Reset()

def hook_intr(uc, intno, ud):
    raise Exception("exception %d at 0x%x" % (intno, uc.reg_read(UC_ARM_REG_PC)))

uc.hook_add(UC_HOOK_CODE, hook_iap, begin=IAP_ENTRY, end=IAP_ENTRY)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_ram_w, begin=IAP_TOP, end=RAM + RAMSZ - 1)
uc.hook_add(UC_HOOK_MEM_READ | UC_HOOK_MEM_WRITE, hook_usbram,
            begin=0x20004000, end=0x200047FF)
uc.hook_add(UC_HOOK_CODE, hook_remap, begin=0, end=0x1FF)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_uart_w, begin=UART, end=UART + 0xFFF)
uc.hook_add(UC_HOOK_MEM_READ, hook_periph_r, begin=0x40000000, end=0x5FFFFFFF)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_scs_w, begin=0xE000E000, end=0xE000EFFF)
uc.hook_add(UC_HOOK_INTR, hook_intr)

def call(fn, *args):
    for r, v in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
        uc.reg_write(r, v)
    uc.reg_write(UC_ARM_REG_LR, RET | 1)
    uc.emu_start(sym[fn] | 1, RET, timeout=60_000_000)
    return uc.reg_read(UC_ARM_REG_R0)

sp, pc = struct.unpack("<II", uc.mem_read(BOOT, 8))
uc.reg_write(UC_ARM_REG_SP, sp)
outcome = "?"
try:
    stop = (sym["main"] if SCEN == "iap" else app_reset) & ~1
    uc.emu_start(pc | 1, stop, timeout=900_000_000)
    outcome = "main" if SCEN == "iap" else "start"
except Recovery:
    outcome = "recovery"
except Reset:
    outcome = "reset"

print("target %s, scenario %s" % (CORE, SCEN))
ok = True
if SCEN == "iap":
    ok = outcome == "main"
    base = SLOT0 + 0x1000 if T == "11u35" else 0x10000
    checks = []
    checks.append(("erase unit at slot 0", call("DfuTgtEraseUnit", SLOT0),
                   0x1000 if SLOT0 < 0x10000 else 0x8000))
    checks.append(("erase unit past the flash", call("DfuTgtEraseUnit", FLASH), 0))
    # Source in flash, odd address: the boot's own code.
    uc.mem_write(base, b"\0" * 16)                      # not erased
    checks.append(("erase", call("DfuTgtErase", base), 1))
    checks.append(("erased", bytes(uc.mem_read(base, 16)) == b"\xff" * 16, True))
    checks.append(("erase off a unit", call("DfuTgtErase", base + 256), 0))
    checks.append(("write from flash+1", call("DfuTgtWrite", base, 1, 512), 1))
    checks.append(("flash data", bytes(uc.mem_read(base, 512)) == bytes(mem[1:513]), True))
    src = RAM + 0x800 + 3
    pat = bytes((i * 7 + 1) & 0xFF for i in range(256))
    uc.mem_write(src, pat)
    checks.append(("write from RAM+3", call("DfuTgtWrite", base + 512, src, 256), 1))
    checks.append(("RAM data", bytes(uc.mem_read(base + 512, 256)) == pat, True))
    checks.append(("write unaligned address", call("DfuTgtWrite", base + 4, src, 256), 0))
    checks.append(("write partial unit", call("DfuTgtWrite", base + 1024, src, 100), 0))
    checks.append(("write past the flash", call("DfuTgtWrite", FLASH - 256, src, 512), 0))
    if T == "1769":
        last = SECTORS[-1][0]
        mem_last = bytes(uc.mem_read(last, 16))
        uc.mem_write(last + 0x7F00, b"\0" * 16)
        checks.append(("erase last 32 KB sector", call("DfuTgtErase", last), 1))
        checks.append(("erased to its end", bytes(uc.mem_read(last + 0x7F00, 16)) == b"\xff" * 16, True))
        checks.append(("erase inside a 32 KB sector", call("DfuTgtErase", last + 0x1000), 0))
    for name, got, want in checks:
        good = got == want
        ok = ok and good
        print("  %-28s %s" % (name, "ok" if good else "got %r, want %r" % (got, want)))
else:
    pc = uc.reg_read(UC_ARM_REG_PC)
    sp = uc.reg_read(UC_ARM_REG_SP)
    print("  outcome %s, pc 0x%x, sp 0x%x" % (outcome, pc, sp))
    if T == "11u35":
        remap = rd32(0x40048000)
        vec = bytes(uc.mem_read(RAM, 192))
        print("  SYSMEMREMAP %d, RAM vectors %s, MAINCLKSEL %d" %
              (remap, "= slot 0" if vec == payload[:192] else "differ",
               rd32(0x40048070)))
        started = outcome == "start" and sp == app_sp and remap == 1 and \
                  vec == payload[:192] and rd32(0x40048070) == 0
    else:
        vtor = rd32(0xE000ED08)
        print("  VTOR 0x%x, PLL0CON %d, CLKSRCSEL %d" %
              (vtor, rd32(0x400FC080), rd32(0x400FC10C)))
        started = outcome == "start" and sp == app_sp and vtor == SLOT0 and \
                  rd32(0x400FC080) == 0
    ok = started if SCEN == "good" else outcome == "recovery"

low = STACK_TOP
for a in range(BSS_END, STACK_TOP, 4):
    if rd32(a) != PAT:
        low = a
        break
print("  RAM: .bss end 0x%x, stack top 0x%x, deepest stack %d of %d bytes" %
      (BSS_END, STACK_TOP, STACK_TOP - low, STACK_TOP - BSS_END))
if low <= BSS_END:
    errors.append("stack reached .bss")
print("  IAP prepare %d, erase %d, copy %d, copies over programmed bytes %d, "
      "clock %d kHz" % (stats["prep"], stats["erase"], stats["copy"],
                        stats["bad_prog"], stats["khz"]))
for e in errors[:5]:
    print("  error:", e)
ok = ok and not errors and stats["bad_prog"] == 0
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
