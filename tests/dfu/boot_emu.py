#!/usr/bin/env python3
# Run the cross built stage 0 boot (ARM machine code) in unicorn, with an
# emulated memory controller, for nRF52840 and nRF52832 (NVMC, MBR,
# SoftDevice start) and nRF54L15 (RRAMC, VTOR start). Checks the install from
# slot 1 and the jump.
#
# boot_emu.py 52|52832|54 <boot.elf> <signed app.bin> [scenario [timeout us]]
#
# 52 is the nRF52840.
#
# scenario: install (default), tamper (a payload byte changed: refused, no
# start), cut:N (power lost at the N-th flash word write, then a reset).
#
# The ELF is the DfuBoot project build, the app the BleOta build signed with
# the key the boot holds. Needs unicorn and pyelftools.
import sys, struct
from unicorn import *
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile

T = sys.argv[1]
ELF = sys.argv[2]
APP = open(sys.argv[3], "rb").read()
SCEN = sys.argv[4] if len(sys.argv) > 4 else "install"

N52 = T in ("52", "52832")

# The memory and its controller per target. The layout itself comes from the
# ELF: the __dfu_* symbols the linker took from the target's dfu_layout_*.ld.
if T == "52":
    FLASH, NAME, NVMC, PAGE = 0x100000, "52840", 0x4001E000, 0x1000
elif T == "52832":
    FLASH, NAME, NVMC, PAGE = 0x80000, "52832", 0x4001E000, 0x1000
elif T == "54":
    FLASH, NAME, RRAMC, PAGE = 0x17D000, "54L15", 0x5004B000, 0x1000
else:
    sys.exit("target: 52, 52832 or 54")

with open(ELF, "rb") as f:
    sym = {}
    for s in ELFFile(f).get_section_by_name(".symtab").iter_symbols():
        if s.name.startswith("__dfu_"):
            sym[s.name] = s["st_value"]
SLOT0, SLOT0_END = sym["__dfu_slot0_start"], sym["__dfu_slot0_end"]
SLOT1, SLOT1_END = sym["__dfu_slot1_start"], sym["__dfu_slot1_end"]
REC, BOOT = sym["__dfu_rec_start"], sym["__dfu_boot_start"]
SDEND = SLOT0

mem = bytearray(b"\xff" * FLASH)

# Boot image
with open(ELF, "rb") as f:
    elf = ELFFile(f)
    for seg in elf.iter_segments():
        if seg["p_type"] != "PT_LOAD" or seg["p_filesz"] == 0:
            continue
        pa = seg["p_paddr"]
        if pa < FLASH:
            d = seg.data()
            mem[pa:pa + len(d)] = d
    entry_vec = BOOT

# Fake SoftDevice info on nRF52: magic and end address, and a vector table at
# the SoftDevice start whose reset handler is where the emulation stops.
SD_RESET = 0x1100
if N52:
    struct.pack_into("<II", mem, 0x1000, 0x20001000, SD_RESET | 1)
    struct.pack_into("<II", mem, 0x3004, 0x51B1E5DB, SDEND)

def put_slot1(img, pending=True):
    mem[SLOT1:SLOT1_END] = b"\xff" * (SLOT1_END - SLOT1)
    mem[SLOT1:SLOT1 + len(img)] = img
    if pending:
        struct.pack_into("<I", mem, SLOT1_END - 32, 0x444E4550)

put_slot1(APP)
if SCEN == "tamper":
    img = bytearray(APP); img[0x20 + 500] ^= 1; put_slot1(bytes(img))

uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.mem_map(0, (FLASH + 0xFFFF) & ~0xFFFF, UC_PROT_ALL)
uc.mem_write(0, bytes(mem))
uc.mem_map(0x10000000, 0x10000, UC_PROT_ALL)      # FICR, UICR
uc.mem_write(0x10001000, b"\xff" * 0x1000)
uc.mem_map(0x20000000, 0x40000, UC_PROT_ALL)
uc.mem_map(0x40000000, 0x20000000, UC_PROT_ALL)    # peripherals
uc.mem_map(0xE0000000, 0x100000, UC_PROT_ALL)      # system control space
uc.mem_map(0xF0000000, 0x10000, UC_PROT_ALL)
if N52:
    uc.mem_write(0x10000010, struct.pack("<II", PAGE, FLASH // PAGE))
else:
    uc.mem_map(0x00FF0000, 0x10000, UC_PROT_ALL)   # FICR, UICR on nRF54L
    uc.mem_write(0x00FFD000, b"\xff" * 0x1000)

stats = {"writes": 0, "erases": 0, "nor_bad": 0, "resets": 0}
flash_mode = [0]   # nRF52 NVMC CONFIG, nRF54 RRAMC write enable

errors = []
# Power loss: "cut:N" stops the core at the N-th flash word write, then the
# next run starts from reset on what the memory holds.
cut = [int(SCEN.split(":")[1]) if SCEN.startswith("cut:") else 0, False]

# Before the store lands: check the controller mode and, on NOR, that the
# bits written are still erased. The store itself then goes through.
def hook_flash_w(uc, access, addr, size, value, ud):
    old = int.from_bytes(uc.mem_read(addr, size), "little")
    value &= (1 << (8 * size)) - 1
    if flash_mode[0] != 1:
        errors.append("write 0x%x in mode %d" % (addr, flash_mode[0]))
    if N52 and (old & value) != value:
        stats["nor_bad"] += 1
    stats["writes"] += 1
    if cut[0] > 0 and stats["writes"] >= cut[0]:
        cut[0] = 0
        cut[1] = True
        uc.emu_stop()

def hook_periph_w(uc, access, addr, size, value, ud):
    if N52:
        if addr == NVMC + 0x504:       # CONFIG
            flash_mode[0] = value
        elif addr == NVMC + 0x508:     # ERASEPAGE
            if flash_mode[0] != 2:
                errors.append("erase in mode %d" % flash_mode[0])
            uc.mem_write(value, b"\xff" * PAGE)
            stats["erases"] += 1
    else:
        if addr == RRAMC + 0x500:      # CONFIG, WEN bit 0
            flash_mode[0] = value & 1

def hook_periph_r(uc, access, addr, size, value, ud):
    if N52:
        if addr in (NVMC + 0x400, NVMC + 0x408):   # READY, READYNEXT
            uc.mem_write(addr, struct.pack("<I", 1))
    else:
        if RRAMC <= addr < RRAMC + 0x1000:
            # READY, READYNEXT, BUFSTATUS.WRITEBUFEMPTY: always ready.
            uc.mem_write(addr, struct.pack("<I", 1))
    # Clock and power status registers that SystemInit polls read as done.
    if (addr & 0xFFF) in (0x40C, 0x418, 0x100, 0x104, 0x108) and addr < 0x60000000:
        uc.mem_write(addr, struct.pack("<I", 1))

mbr = []

class Reset(Exception):
    pass

def hook_scs_w(uc, access, addr, size, value, ud):
    if addr == 0xE000ED0C and (value >> 16) == 0x05FA:   # AIRCR SYSRESETREQ
        raise Reset()

def hook_intr(uc, intno, ud):
    # SVC to the MBR: IRQ forward address set. Answer success.
    if intno == 2:
        blk = struct.unpack("<II", uc.mem_read(uc.reg_read(UC_ARM_REG_R0), 8))
        mbr.append(blk)
        uc.reg_write(UC_ARM_REG_R0, 0)
        return
    raise Exception("exception %d at 0x%x" % (intno, uc.reg_read(UC_ARM_REG_PC)))

uc.hook_add(UC_HOOK_MEM_WRITE, hook_flash_w, begin=0, end=FLASH - 1)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_periph_w, begin=0x40000000, end=0x5FFFFFFF)
uc.hook_add(UC_HOOK_MEM_READ, hook_periph_r, begin=0x40000000, end=0x5FFFFFFF)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_scs_w, begin=0xE000E000, end=0xE000EFFF)
uc.hook_add(UC_HOOK_INTR, hook_intr)

app_vec = struct.unpack_from("<II", APP, 0x20)
stop = SD_RESET if N52 else (app_vec[1] & ~1)

def run():
    sp, pc = struct.unpack("<II", uc.mem_read(entry_vec, 8))
    uc.reg_write(UC_ARM_REG_SP, sp)
    uc.emu_start(pc | 1, stop, timeout=int(sys.argv[5]) if len(sys.argv) > 5 else 900_000_000)

for attempt in range(4):
    try:
        run()
        if cut[1]:
            cut[1] = False
            stats["resets"] += 1
            continue
        break
    except Reset:
        stats["resets"] += 1
        continue
    except UcError:
        # WFE: the boot found nothing to start and waits there.
        pc = uc.reg_read(UC_ARM_REG_PC) & ~1
        if {0xBF20, 0xBF30} & {struct.unpack("<H", uc.mem_read(pc, 2))[0],
                                  struct.unpack("<H", uc.mem_read(pc - 2, 2))[0]}:
            break
        raise

pc = uc.reg_read(UC_ARM_REG_PC)
sp = uc.reg_read(UC_ARM_REG_SP)
vtor = struct.unpack("<I", uc.mem_read(0xE000ED08, 4))[0]
slot0 = bytes(uc.mem_read(SLOT0, len(APP) - 0x20))
hdr_size = struct.unpack_from("<H", APP, 8)[0]
img_size = struct.unpack_from("<I", APP, 12)[0]
payload = APP[hdr_size:hdr_size + img_size]
installed = bytes(uc.mem_read(SLOT0, img_size)) == payload
rec_magic = struct.unpack("<I", uc.mem_read(REC, 4))[0]
done = struct.unpack("<I", uc.mem_read(SLOT1_END - 16, 4))[0]

print("target nRF%s, scenario %s" % (NAME, SCEN))
print("  stopped at 0x%x (expected 0x%x), sp 0x%x, vtor 0x%x" % (pc, stop, sp, vtor))
print("  payload in slot 0: %s, record magic 0x%x, trailer done 0x%x" %
      (installed, rec_magic, done))
print("  flash writes %d, erases %d, NOR violations %d, resets %d" %
      (stats["writes"], stats["erases"], stats["nor_bad"], stats["resets"]))
ok = pc == stop and installed and rec_magic == 0x43455244 and done == 0x454E4F44 \
     and stats["nor_bad"] == 0
if T == "54":
    ok = ok and vtor == SLOT0 and sp == app_vec[0]
else:
    print("  MBR calls:", ["cmd %d addr 0x%x" % b for b in mbr])
    ok = ok and (SCEN == "tamper" or mbr[-1:] == [(6, 0x1000)])
if SCEN.startswith("cut:"):
    ok = ok and stats["resets"] >= 1
if SCEN == "tamper":
    ok = pc != stop and not installed and done == 0x454E4F44
for e in errors[:5]:
    print("  error:", e)
ok = ok and not errors
if SCEN.startswith("cut:"):
    ok = ok and stats["resets"] >= 1
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
