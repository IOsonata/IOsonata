#!/usr/bin/env python3
# Run the cross built stage 0 boot (DfuBoot Release ELF) in unicorn for the
# other Nordic targets, with an emulated memory controller:
#
#   52805, 52810   NVMC with ERASEPAGE, MBR and S112 start (boot at the top,
#                  wired only)
#   5340app        nRF5340 application core, secure NVMC without ERASEPAGE
#                  (a word written in erase mode erases its page), wired only
#   5340net        nRF5340 network core, non secure NVMC, flash at 0x01000000,
#                  2 KB pages, slot 1
#   9160, 91x1     secure NVMC without ERASEPAGE, slot 1
#   lm20           nRF54LM20 RRAMC, slot 1
#
# boot_emu_nordic.py <target> <boot.elf> [scenario [timeout us]]
#
# scenario:
#   empty     nothing in slot 0: the boot must not start anything
#   rec       signed payload in slot 0 and its record, built here the way
#             DfuRecWrite lays it out: the boot verifies and starts it
#   rectamper the same with a payload byte changed: refused, no start
#   install   (slot 1 targets) signed image in slot 1 marked pending, an
#             older image in slot 0 and the record
#   tamper    (slot 1 targets) the same with a payload byte changed
#   cut:N     (slot 1 targets) install with power lost at the N-th flash
#             word write, then a reset
#
# The application is a small vector table and a loop, made and signed here
# with imgtool and exemples/dfu/dfu_dev_key.pem, the key the boot holds.
# The layout comes from the __dfu_* symbols of the ELF. Needs unicorn,
# pyelftools and imgtool.
import os, sys, struct, subprocess, tempfile
from unicorn import *
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile

T = sys.argv[1]
ELF = sys.argv[2]
SCEN = sys.argv[3] if len(sys.argv) > 3 else "rec"
TIMEOUT = int(sys.argv[4]) if len(sys.argv) > 4 else 900_000_000
ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

# Memory and controller per target, from the device headers:
#   flash (base, size), controller kind and base, FICR code page size word,
#   page, RAM blocks, SoftDevice start.
TGT = {
	"52805": dict(flash=(0, 0x30000), kind="nvmc52", ctl=0x4001E000,
				  ficr=0x10000010, page=0x1000, ram=[(0x20000000, 0x10000)], sd=True),
	"5340app": dict(flash=(0, 0x100000), kind="nvmc53", ctl=0x50039000,
					ficr=0x00FF0220, page=0x1000, ram=[(0x20000000, 0x80000)], sd=False),
	"5340net": dict(flash=(0x01000000, 0x40000), kind="nvmc53", ctl=0x41080000,
					ficr=0x01FF0220, page=0x800,
					ram=[(0x21000000, 0x10000), (0x20000000, 0x80000)], sd=False),
	"9160": dict(flash=(0, 0x100000), kind="nvmc53", ctl=0x50039000,
				 ficr=0x00FF0220, page=0x1000, ram=[(0x20000000, 0x40000)], sd=False),
	"lm20": dict(flash=(0, 0x200000), kind="rramc", ctl=0x5004E000,
				 ficr=None, page=0x1000, ram=[(0x20000000, 0x80000)], sd=False),
}
TGT["52810"] = TGT["52805"]
TGT["91x1"] = TGT["9160"]
if T not in TGT:
	sys.exit("target: " + ", ".join(TGT))
t = TGT[T]
FBASE, FSIZE = t["flash"]
PAGE = t["page"]

with open(ELF, "rb") as f:
	elf = ELFFile(f)
	sym = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()
		   if s.name.startswith("__dfu_")}
	segs = [(seg["p_paddr"], seg.data()) for seg in elf.iter_segments()
			if seg["p_type"] == "PT_LOAD" and seg["p_filesz"] > 0]
SLOT0, SLOT0_END = sym["__dfu_slot0_start"], sym["__dfu_slot0_end"]
SLOT1, SLOT1_END = sym.get("__dfu_slot1_start", 0), sym.get("__dfu_slot1_end", 0)
REC, BOOT = sym["__dfu_rec_start"], sym["__dfu_boot_start"]
HAS_SLOT1 = SLOT1_END > SLOT1
UNIT = 16		# state unit: max(16, program unit 4)

if SCEN in ("install", "tamper") or SCEN.startswith("cut:"):
	if not HAS_SLOT1:
		sys.exit("%s has no slot 1" % T)

# The application: vector table, the reset handler a branch to itself.
RAM0 = t["ram"][0][0]
app_sp = RAM0 + 0x2000
app_reset = SLOT0 + 0x200
payload = bytearray(b"\x00" * 0x400)
struct.pack_into("<II", payload, 0, app_sp, app_reset | 1)
for i in range(2, 16):
	struct.pack_into("<I", payload, 4 * i, app_reset | 1)
struct.pack_into("<H", payload, 0x200, 0xE7FE)			# b .
for i in range(0x204, 0x400, 4):
	struct.pack_into("<I", payload, i, (i * 2654435761) & 0xFFFFFFFF)

tmp = tempfile.mkdtemp()
with open(os.path.join(tmp, "app.bin"), "wb") as f:
	f.write(payload)
max_size = (SLOT1_END - SLOT1 - 2 * UNIT) if HAS_SLOT1 else (SLOT0_END - SLOT0 + 0x1000)
subprocess.run(["imgtool", "sign", "-k", os.path.join(ROOT, "exemples/dfu/dfu_dev_key.pem"),
				"--header-size", "0x20", "--pad-header", "--align", "4", "-v", "1.2.3",
				"-S", str(max_size), os.path.join(tmp, "app.bin"),
				os.path.join(tmp, "app_signed.bin")], check=True, capture_output=True)
APP = open(os.path.join(tmp, "app_signed.bin"), "rb").read()
HDR = struct.unpack_from("<H", APP, 8)[0]
IMG = struct.unpack_from("<I", APP, 12)[0]
TLVOFF = HDR + IMG
TLVLEN = len(APP) - TLVOFF
assert APP[HDR:HDR + IMG] == bytes(payload) and HDR == 0x20

mem = bytearray(b"\xff" * FSIZE)
def put(addr, data):
	mem[addr - FBASE:addr - FBASE + len(data)] = data
def get(addr, n):
	return bytes(mem[addr - FBASE:addr - FBASE + n])

for pa, d in segs:
	if FBASE <= pa < FBASE + FSIZE:
		put(pa, d)

# nRF52: a SoftDevice information structure and a vector table at the
# SoftDevice start whose reset handler is where the emulation stops.
SD_RESET = 0x1100
if t["sd"]:
	put(0x1000, struct.pack("<II", 0x20001000, SD_RESET | 1))
	put(0x3004, struct.pack("<II", 0x51B1E5DB, SLOT0))

def put_rec(pl):
	put(SLOT0, pl)
	body = struct.pack("<HH", HDR, TLVLEN) + APP[:HDR] + APP[TLVOFF:]
	put(REC + UNIT, body)
	put(REC, struct.pack("<I", 0x43455244) + b"\xff" * (UNIT - 4))

def put_slot1(img):
	put(SLOT1, b"\xff" * (SLOT1_END - SLOT1))
	put(SLOT1, img)
	put(SLOT1_END - 2 * UNIT, struct.pack("<I", 0x444E4550))

if SCEN == "rec":
	put_rec(payload)
elif SCEN == "rectamper":
	p = bytearray(payload); p[0x300] ^= 1; put_rec(bytes(p))
elif SCEN == "install" or SCEN.startswith("cut:"):
	# An older image where the new one goes, so the install has to erase.
	put(SLOT0, b"\x00" * 0x2000)
	put(REC, b"\x5a" * 0x100)
	put_slot1(APP)
elif SCEN == "tamper":
	img = bytearray(APP); img[HDR + 0x300] ^= 1; put_slot1(bytes(img))
elif SCEN != "empty":
	sys.exit("scenario?")

uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.mem_map(FBASE, (FSIZE + 0xFFFF) & ~0xFFFF, UC_PROT_ALL)
uc.mem_write(FBASE, bytes(mem))
for a, n in t["ram"]:
	uc.mem_map(a, n, UC_PROT_ALL)
if t["ficr"] is not None:
	# FICR, UICR, and below them the RAM copy of FICR that nRF53 and nRF91
	# SystemInit makes for the non secure side.
	fb = t["ficr"] & ~0xFFFF
	uc.mem_map(fb - 0x10000, 0x20000, UC_PROT_ALL)
	# FICR reads as unprogrammed, trim entries included (their address all
	# ones, so SystemInit applies none), but for the code page size.
	uc.mem_write(fb, b"\xff" * 0x2000)				# FICR, UICR
	uc.mem_write(t["ficr"], struct.pack("<I", PAGE))
else:
	uc.mem_map(0x00FF0000, 0x10000, UC_PROT_ALL)		# FICR, UICR on nRF54L
	uc.mem_write(0x00FFD000, b"\xff" * 0x1000)
uc.mem_map(0x40000000, 0x20000000, UC_PROT_ALL)		# peripherals
uc.mem_map(0xE0000000, 0x100000, UC_PROT_ALL)		# system control space
uc.mem_map(0xF0000000, 0x10000, UC_PROT_ALL)

# RRAM has no erase: the target layer writes ones, counted as writes.
stats = {"writes": 0, "erases": 0, "nor_bad": 0, "resets": 0}
mode = [0]		# NVMC CONFIG, RRAMC write enable
errors = []
cut = [int(SCEN.split(":")[1]) if SCEN.startswith("cut:") else 0, False]
CTL = t["ctl"]
KIND = t["kind"]

def hook_flash_w(uc, access, addr, size, value, ud):
	value &= (1 << (8 * size)) - 1
	if KIND == "nvmc53" and mode[0] == 2:
		# Erase mode: a word written anywhere in a page erases the page.
		if value != 0xFFFFFFFF or size != 4:
			errors.append("erase write 0x%x of 0x%x" % (addr, value))
		pg = addr - (addr % PAGE)
		uc.mem_write(pg, b"\xff" * PAGE)
		stats["erases"] += 1
		return
	if mode[0] != 1:
		errors.append("write 0x%x in mode %d at pc 0x%x" % (addr, mode[0], uc.reg_read(UC_ARM_REG_PC)))
	old = int.from_bytes(uc.mem_read(addr, size), "little")
	if KIND != "rramc" and (old & value) != value:
		stats["nor_bad"] += 1
	if KIND != "rramc" and (size != 4 or addr & 3):
		errors.append("NVMC write of %d bytes at 0x%x" % (size, addr))
	stats["writes"] += 1
	if cut[0] > 0 and stats["writes"] >= cut[0]:
		cut[0] = 0
		cut[1] = True
		uc.emu_stop()

def hook_periph_w(uc, access, addr, size, value, ud):
	if KIND.startswith("nvmc"):
		if addr == CTL + 0x504:			# CONFIG
			mode[0] = value
		elif addr == CTL + 0x508 and KIND == "nvmc52":	# ERASEPAGE
			if mode[0] != 2:
				errors.append("erase in mode %d" % mode[0])
			uc.mem_write(value, b"\xff" * PAGE)
			stats["erases"] += 1
	elif addr == CTL + 0x500:			# RRAMC CONFIG, WEN bit 0
		mode[0] = value & 1

def hook_periph_r(uc, access, addr, size, value, ud):
	if KIND.startswith("nvmc") and addr in (CTL + 0x400, CTL + 0x408):	# READY, READYNEXT
		uc.mem_write(addr, struct.pack("<I", 1))
	elif KIND == "rramc" and CTL <= addr < CTL + 0x1000 and addr != CTL + 0x500:
		uc.mem_write(addr, struct.pack("<I", 1))		# READY, BUFSTATUS
	# Clock and power status registers that SystemInit polls read as done.
	elif (addr & 0xFFF) in (0x40C, 0x418, 0x100, 0x104, 0x108) and addr < 0x60000000:
		uc.mem_write(addr, struct.pack("<I", 1))

mbr = []

class Reset(Exception):
	pass

def hook_scs_w(uc, access, addr, size, value, ud):
	if addr == 0xE000ED0C and (value >> 16) == 0x05FA:	# AIRCR SYSRESETREQ
		raise Reset()

def hook_intr(uc, intno, ud):
	# SVC to the MBR: IRQ forward address set. Answer success.
	if intno == 2 and t["sd"]:
		mbr.append(struct.unpack("<II", uc.mem_read(uc.reg_read(UC_ARM_REG_R0), 8)))
		uc.reg_write(UC_ARM_REG_R0, 0)
		return
	raise Exception("exception %d at 0x%x" % (intno, uc.reg_read(UC_ARM_REG_PC)))

uc.hook_add(UC_HOOK_MEM_WRITE, hook_flash_w, begin=FBASE, end=FBASE + FSIZE - 1)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_periph_w, begin=0x40000000, end=0x5FFFFFFF)
uc.hook_add(UC_HOOK_MEM_READ, hook_periph_r, begin=0x40000000, end=0x5FFFFFFF)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_scs_w, begin=0xE000E000, end=0xE000EFFF)
uc.hook_add(UC_HOOK_INTR, hook_intr)

stop = SD_RESET if t["sd"] else app_reset
short = SCEN in ("empty", "rectamper", "tamper")

def run():
	sp, pc = struct.unpack("<II", uc.mem_read(BOOT, 8))
	uc.reg_write(UC_ARM_REG_SP, sp)
	uc.emu_start(pc | 1, stop, timeout=min(TIMEOUT, 20_000_000) if short else TIMEOUT)

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
		# WFE or WFI: the boot found nothing to start and waits there.
		pc = uc.reg_read(UC_ARM_REG_PC) & ~1
		if {0xBF20, 0xBF30} & {struct.unpack("<H", uc.mem_read(pc, 2))[0],
								  struct.unpack("<H", uc.mem_read(pc - 2, 2))[0]}:
			break
		raise

pc = uc.reg_read(UC_ARM_REG_PC)
sp = uc.reg_read(UC_ARM_REG_SP)
vtor = struct.unpack("<I", uc.mem_read(0xE000ED08, 4))[0]
installed = bytes(uc.mem_read(SLOT0, IMG)) == bytes(payload)
rec_magic = struct.unpack("<I", uc.mem_read(REC, 4))[0]
done = struct.unpack("<I", uc.mem_read(SLOT1_END - UNIT, 4))[0] if HAS_SLOT1 else 0

print("target %s, scenario %s, slot 0 0x%x, slot 1 %s, record 0x%x, boot 0x%x" %
	  (T, SCEN, SLOT0, "0x%x" % SLOT1 if HAS_SLOT1 else "none", REC, BOOT))
print("  stopped at 0x%x (start is 0x%x), sp 0x%x, vtor 0x%x" % (pc, stop, sp, vtor))
print("  payload in slot 0: %s, record magic 0x%x%s" %
	  (installed, rec_magic, ", trailer done 0x%x" % done if HAS_SLOT1 else ""))
print("  flash writes %d, erases %d, NOR violations %d, resets %d" %
	  (stats["writes"], stats["erases"], stats["nor_bad"], stats["resets"]))
if t["sd"]:
	print("  MBR calls:", ["cmd %d addr 0x%x" % b for b in mbr])

started = pc == stop
if t["sd"]:
	started_ok = started and mbr[-1:] == [(6, 0x1000)]
else:
	started_ok = started and vtor == SLOT0 and sp == app_sp

if SCEN == "empty":
	ok = not started and stats["writes"] == 0 and stats["erases"] == 0
elif SCEN == "rec":
	ok = started_ok and installed and stats["writes"] == 0
elif SCEN == "rectamper":
	ok = not started and stats["writes"] == 0 and stats["erases"] == 0
elif SCEN == "tamper":
	ok = not started and not installed and done == 0x454E4F44
else:
	ok = started_ok and installed and rec_magic == 0x43455244 and done == 0x454E4F44 \
		 and (stats["erases"] > 0 or KIND == "rramc")
	if SCEN.startswith("cut:"):
		ok = ok and stats["resets"] >= 1
ok = ok and stats["nor_bad"] == 0
for e in errors[:5]:
	print("  error:", e)
ok = ok and not errors
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
