#!/usr/bin/env python3
# Run the cross built nRF52832 stage 0 boot (DfuBoot Release ELF) in unicorn
# and drive its recovery with Python/dfu_wire.py over a pty: the machine code
# of the boot, the UART driver and its interrupts, the wire protocol, the
# signature check and the NVMC writes, against the real host tool.
#
# boot_emu_wire.py <boot.elf> <app.bin> <key.pem>
#
# app.bin is the application linked for slot 0, not signed: it is signed here
# in each variant the steps need. <key.pem> is the key the boot holds. Needs
# unicorn, pyelftools, pyserial and imgtool.
#
# Modelled: NVMC, legacy UART0 (RXD, TXD, events, INTENSET, one interrupt),
# GPIO input, AIRCR reset, retained RAM. Not modelled: the MBR, whose start of
# the boot and interrupt forwarding are Nordic's code, UART timing beyond the
# byte rate, the erase and write times.
import fcntl, os, re, select, struct, subprocess, sys, tempfile, time, tty
from unicorn import *
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile

ELF, APPBIN, KEY = sys.argv[1:4]
HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "..", "..", "Python", "dfu_wire.py")

FLASH, PAGE = 0x80000, 0x1000
NVMC = 0x4001E000
UART0 = 0x40002000
U_RXDRDY, U_TXDRDY, U_INTENSET, U_INTENCLR = 0x108, 0x11C, 0x304, 0x308
U_ENABLE, U_RXD, U_TXD = 0x500, 0x518, 0x51C
UART_IRQN = 2
GPIO_IN = 0x50000510
NVIC_ISER0 = 0xE000E100
TRAMP = FLASH - 0x10		# handler return, in the unused end of the boot
SLICE_US = 500				# a slice lets through about 6 bytes at 115200 baud
DFU_FLAG_RECOVERY = 0x56434552
REC_MAGIC = 0x43455244

with open(ELF, "rb") as f:
	elf = ELFFile(f)
	sym = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()
		   if s.name.startswith("__dfu_") or s.name == "__Vectors"}
	segs = [(s["p_paddr"], s.data()) for s in elf.iter_segments()
			if s["p_type"] == "PT_LOAD" and s["p_filesz"] > 0]
SLOT0, SLOT0_END = sym["__dfu_slot0_start"], sym["__dfu_slot0_end"]
SLOT1, SLOT1_END = sym["__dfu_slot1_start"], sym["__dfu_slot1_end"]
REC, BOOT, FLAG = sym["__dfu_rec_start"], sym["__dfu_boot_start"], sym["__dfu_flag"]

uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.mem_map(0, FLASH, UC_PROT_ALL)
uc.mem_write(0, b"\xff" * FLASH)
for pa, d in segs:
	if pa < FLASH:
		uc.mem_write(pa, d)
uc.mem_write(TRAMP, struct.pack("<H", 0xE7FE))
uc.mem_map(0x10000000, 0x10000, UC_PROT_ALL)		# FICR, UICR
uc.mem_write(0x10001000, b"\xff" * 0x1000)
uc.mem_write(0x10000010, struct.pack("<II", PAGE, FLASH // PAGE))
for pa, d in segs:
	if 0x10001000 <= pa < 0x10002000:
		uc.mem_write(pa, d)
uc.mem_map(0x20000000, 0x10000, UC_PROT_ALL)
uc.mem_map(0x40000000, 0x20000000, UC_PROT_ALL)
uc.mem_map(0xE0000000, 0x100000, UC_PROT_ALL)
uc.mem_map(0xF0000000, 0x10000, UC_PROT_ALL)

stats = dict(writes=0, erases=0, nor_bad=0, resets=0)
errors = []
mode = [0]
intmask = [0]
rxq = bytearray()
reset_req = [False]
cut = [0, False]
at_tramp = [False]
UNTIL = [0xFFFFFFFE]

master, slave = os.openpty()
tty.setraw(slave)
fcntl.fcntl(master, fcntl.F_SETFL, fcntl.fcntl(master, fcntl.F_GETFL) | os.O_NONBLOCK)
PORT = os.ttyname(slave)


def rd32(a):
	return struct.unpack("<I", uc.mem_read(a, 4))[0]


def wr32(a, v):
	uc.mem_write(a, struct.pack("<I", v))


def hook_flash_w(uc, access, addr, size, value, ud):
	if addr >= TRAMP:
		return
	old = int.from_bytes(uc.mem_read(addr, size), "little")
	value &= (1 << (8 * size)) - 1
	if mode[0] != 1:
		errors.append("write 0x%x in mode %d" % (addr, mode[0]))
	if (old & value) != value:
		stats["nor_bad"] += 1
	stats["writes"] += 1
	if cut[0] and stats["writes"] >= cut[0]:
		cut[0] = 0
		cut[1] = True
		uc.emu_stop()


def hook_periph_w(uc, access, addr, size, value, ud):
	if addr == NVMC + 0x504:
		mode[0] = value
	elif addr == NVMC + 0x508:
		if mode[0] != 2:
			errors.append("erase in mode %d" % mode[0])
		uc.mem_write(value, b"\xff" * PAGE)
		stats["erases"] += 1
	elif addr == UART0 + U_INTENSET:
		intmask[0] |= value
	elif addr == UART0 + U_INTENCLR:
		intmask[0] &= ~value
	elif addr == UART0 + U_TXD:
		os.write(master, bytes([value & 0xFF]))
		wr32(UART0 + U_TXDRDY, 1)


def hook_periph_r(uc, access, addr, size, value, ud):
	if addr in (NVMC + 0x400, NVMC + 0x408):
		wr32(addr, 1)
	elif addr in (0x40000100, 0x40000104, 0x4000040C, 0x40000418):	# clock status
		wr32(addr, 1)
	elif addr == UART0 + U_RXD:
		if rxq:
			wr32(addr, rxq.pop(0))
		if rxq:
			wr32(UART0 + U_RXDRDY, 1)


def hook_tramp(uc, addr, size, ud):
	at_tramp[0] = True
	uc.emu_stop()


def hook_scs_w(uc, access, addr, size, value, ud):
	if addr == 0xE000ED0C and (value >> 16) == 0x05FA:
		reset_req[0] = True
		uc.emu_stop()


uc.hook_add(UC_HOOK_MEM_WRITE, hook_flash_w, begin=0, end=FLASH - 1)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_periph_w, begin=0x40000000, end=0x5FFFFFFF)
uc.hook_add(UC_HOOK_MEM_READ, hook_periph_r, begin=0x40000000, end=0x5FFFFFFF)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_scs_w, begin=0xE000E000, end=0xE000EFFF)
uc.hook_add(UC_HOOK_CODE, hook_tramp, begin=TRAMP, end=TRAMP + 1)


class Core:
	def __init__(self):
		self.isr = False
		self.frame = None
		self.pc = 0
		self.ran = True

	def reset(self):
		# RAM keeps its content over a reset, the peripherals do not
		uc.mem_write(0x40000000 + 0x2000, b"\0" * 0x1000)
		uc.mem_write(NVIC_ISER0, b"\0" * 32)
		wr32(0xE000ED08, 0)
		intmask[0] = 0
		mode[0] = 0
		rxq.clear()
		reset_req[0] = False
		sp, pc = struct.unpack("<II", uc.mem_read(BOOT, 8))
		uc.reg_write(UC_ARM_REG_SP, sp)
		uc.reg_write(UC_ARM_REG_LR, 0xFFFFFFFF)
		uc.reg_write(UC_ARM_REG_XPSR, 1 << 24)
		uc.reg_write(UC_ARM_REG_PRIMASK, 0)
		self.pc = pc & ~1
		self.isr = False
		stats["resets"] += 1

	def pending(self):
		if self.isr or uc.reg_read(UC_ARM_REG_PRIMASK) & 1:
			return False
		if not (rd32(NVIC_ISER0) >> UART_IRQN) & 1:
			return False
		m = intmask[0]
		return bool((rd32(UART0 + U_RXDRDY) and m & (1 << 2)) or
					(rd32(UART0 + U_TXDRDY) and m & (1 << 7)))

	def enter(self):
		xpsr = uc.reg_read(UC_ARM_REG_XPSR)
		if xpsr & 0x0600FC00:
			# Inside an IT block: the core takes the exception after it here
			self.step(1)
			return
		sp = uc.reg_read(UC_ARM_REG_SP)
		regs = [uc.reg_read(r) for r in (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3,
										 UC_ARM_REG_R12, UC_ARM_REG_LR)]
		self.frame = (sp, regs, self.pc, xpsr)
		uc.mem_write(sp - 32, b"".join(struct.pack("<I", v) for v in regs + [self.pc | 1, xpsr]))
		uc.reg_write(UC_ARM_REG_SP, sp - 32)
		uc.reg_write(UC_ARM_REG_LR, TRAMP | 1)
		self.pc = rd32(BOOT + (16 + UART_IRQN) * 4) & ~1
		self.isr = True

	def leave(self):
		sp, regs, pc, xpsr = self.frame
		for r, v in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3,
						 UC_ARM_REG_R12, UC_ARM_REG_LR), regs):
			uc.reg_write(r, v)
		uc.reg_write(UC_ARM_REG_SP, sp)
		uc.reg_write(UC_ARM_REG_XPSR, xpsr)
		self.pc = pc
		self.isr = False

	def drain(self):
		# The boot's delay before a reset, 20 ms for the response to leave the
		# FIFO, is a counted loop that takes well under a slice here. What the
		# transmit interrupt would send in that time goes out now.
		uc.reg_write(UC_ARM_REG_XPSR, 1 << 24)
		self.pc = uc.reg_read(UC_ARM_REG_PC) & ~1
		while self.pending():
			self.enter()
			while self.isr:
				self.step()

	def step(self, count=0):
		# Returns "idle" when the core stopped in WFI or WFE. The stop address
		# stays the same on every call: unicorn flushes its translated code
		# when it changes.
		at_tramp[0] = False
		try:
			if count:
				uc.emu_start(self.pc | 1, UNTIL[0], count=count)
			else:
				uc.emu_start(self.pc | 1, UNTIL[0], timeout=SLICE_US)
		except UcError:
			pc = uc.reg_read(UC_ARM_REG_PC) & ~1
			for a in (pc, pc - 2):
				if struct.unpack("<H", uc.mem_read(a, 2))[0] in (0xBF20, 0xBF30):
					self.pc = a + 2
					return "idle"
			raise
		self.pc = uc.reg_read(UC_ARM_REG_PC) & ~1
		if self.isr and at_tramp[0]:
			self.leave()
		return "run"


core = Core()


def run(proc=None, app=None, settle=0.0, limit=120.0, power_cut=None):
	# Run until the application entry, or until proc has ended and settle
	# seconds have passed. Returns "app" or "recovery".
	t0 = time.monotonic()
	tend = None
	while True:
		if time.monotonic() - t0 > limit:
			raise SystemExit("timeout, pc 0x%x" % core.pc)
		if proc is not None and proc.poll() is not None and tend is None:
			tend = time.monotonic() + settle
		if tend is None and proc is None:
			tend = t0 + settle
		if app is not None and core.pc == app and not core.isr:
			return "app"
		if tend is not None and time.monotonic() > tend:
			return "recovery"
		try:
			while len(rxq) < 4096:
				d = os.read(master, 4096)
				if not d:
					break
				rxq.extend(d)
		except (BlockingIOError, OSError):
			pass
		# The line paces the bytes: the main loop runs between interrupts
		if rxq and core.ran and not rd32(UART0 + U_RXDRDY) and rd32(UART0 + U_ENABLE):
			wr32(UART0 + U_RXDRDY, 1)
			core.ran = False
		if core.pending():
			core.enter()
			if not core.isr:
				continue
		if not core.isr:
			core.ran = True
		st = core.step()
		if reset_req[0] or cut[1]:
			if cut[1] and power_cut is not None:
				power_cut()
			elif reset_req[0]:
				core.drain()
			cut[1] = False
			core.reset()
		elif st == "idle" and not rxq and not core.pending():
			select.select([master], [], [], 0.002)


def wire(*args):
	return subprocess.Popen([sys.executable, "-I", TOOL, "--port", PORT, *args],
							stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


fails = 0


def check(ok, what):
	global fails
	print("  %s  %s" % ("ok  " if ok else "FAIL", what))
	if not ok:
		fails += 1


def sign(tmp, name, ver, key=KEY, tamper=False):
	out = os.path.join(tmp, name)
	subprocess.run(["imgtool", "sign", "-k", key, "--header-size", "0x20", "--pad-header", "--align", "4",
					"-v", ver, "-S", hex(SLOT1_END - SLOT1 - 32), APPBIN, out],
				   check=True, stdout=subprocess.DEVNULL)
	if tamper:
		d = bytearray(open(out, "rb").read())
		d[0x20 + 100] ^= 1
		open(out, "wb").write(d)
	return out


def tool_run(*args, app=None, settle=0.3, power_cut=None):
	p = wire(*args)
	res = run(proc=p, app=app, settle=settle, power_cut=power_cut)
	if res == "app" and p.poll() is None:
		p.wait(timeout=10)
	out = p.stdout.read() if p.poll() is not None else ""
	return p.returncode, out, res


def boot(app, settle=0.5):
	core.reset()
	return run(app=app, settle=settle)


def in_place(payload):
	return bytes(uc.mem_read(SLOT0, len(payload))) == payload


with tempfile.TemporaryDirectory() as tmp:
	payload = open(APPBIN, "rb").read()
	sp0, entry = struct.unpack_from("<II", payload, 0)
	entry &= ~1
	UNTIL[0] = entry
	other = os.path.join(tmp, "other.pem")
	subprocess.run(["imgtool", "keygen", "-k", other, "-t", "ecdsa-p256"], check=True)
	v100 = sign(tmp, "v100.bin", "1.0.0")
	v110 = sign(tmp, "v110.bin", "1.1.0")
	v090 = sign(tmp, "v090.bin", "0.9.0")
	badkey = sign(tmp, "badkey.bin", "1.0.0", key=other)
	bad = sign(tmp, "tamper.bin", "1.0.0", tamper=True)

	wr32(GPIO_IN, 0xFFFFFFFF)
	print("slot 0 0x%x, record 0x%x, boot 0x%x, app entry 0x%x, port %s" % (SLOT0, REC, BOOT, entry, PORT))

	print("empty memory: the boot serves recovery")
	core.reset()
	rc, out, res = tool_run("--info")
	check(rc == 0 and res == "recovery", "--info answers, rc %s" % rc)
	print("\n".join("    " + l for l in out.splitlines()))
	check(stats["writes"] == 0 and stats["erases"] == 0, "nothing written or erased")

	print("image signed with another key")
	rc, out, res = tool_run(badkey)
	check(rc != 0 and stats["writes"] == 0 and stats["erases"] == 0,
		  "refused, nothing written or erased: %s" % out.strip().splitlines()[-1][:60])

	print("payload changed after signing")
	rc, out, res = tool_run(bad)
	check(rc != 0, "refused at the end: %s" % out.strip().splitlines()[-1][:60])
	check(rd32(REC) != REC_MAGIC, "no record")
	check(boot(entry) == "recovery", "after a reset the boot stays in recovery")

	print("signed image")
	rc, out, res = tool_run(v100, app=entry)
	check(rc == 0, "upload rc %s: %s" % (rc, out.strip().splitlines()[-1][:60] if out.strip() else ""))
	check(res == "app", "the application is started")
	check(in_place(payload), "payload in slot 0")
	check(rd32(REC) == REC_MAGIC, "record written")
	check(uc.reg_read(UC_ARM_REG_SP) == sp0 and rd32(0xE000ED08) == SLOT0, "SP and VTOR are the application's")
	check(stats["nor_bad"] == 0, "no write over bits already programmed")

	print("power cycle")
	check(boot(entry) == "app", "the application starts again, no host")

	print("button held at reset")
	wr32(GPIO_IN, 0)
	core.reset()
	rc, out, res = tool_run("--info", app=entry)
	check(rc == 0 and res == "recovery", "recovery with a valid application")
	cur = re.search(r"(?m)^\s*\S*ver\S*\s+.*1\.0\.0", out, re.I)
	check(cur is not None, "version of slot 0 reported as 1.0.0")
	w0, e0 = stats["writes"], stats["erases"]
	print("older image, in recovery")
	rc, out, res = tool_run(v090, app=entry)
	check(rc != 0 and stats["writes"] == w0 and stats["erases"] == e0,
		  "refused, nothing written or erased: %s" % out.strip().splitlines()[-1][:60])
	wr32(GPIO_IN, 0xFFFFFFFF)
	check(boot(entry) == "app", "button released: the application still starts")

	print("application asks for recovery")
	wr32(FLAG, DFU_FLAG_RECOVERY)
	check(boot(entry) == "recovery", "recovery after the request")
	rc, out, res = tool_run(v110, app=entry)
	check(rc == 0 and res == "app", "newer image taken and started, rc %s" % rc)
	check(boot(entry) == "app", "the request does not last: the next reset starts the application")

	print("power lost in the middle of an upload")
	wr32(GPIO_IN, 0)
	core.reset()
	run(settle=0.3)
	cut[0] = stats["writes"] + 500
	p = wire(v110, "--no-resume")
	run(proc=p, settle=0.2, power_cut=lambda: p.kill())
	p.wait()
	wr32(GPIO_IN, 0xFFFFFFFF)
	check(rd32(REC) != REC_MAGIC, "no record after the cut")
	check(boot(entry) == "recovery", "the boot stays in recovery")
	rc, out, res = tool_run(v110, app=entry)
	check(rc == 0 and res == "app" and in_place(payload), "uploaded again and started, rc %s" % rc)

	check(stats["nor_bad"] == 0, "no write over bits already programmed, all steps")
	for e in errors[:5]:
		check(False, e)
	print("flash writes %d, erases %d, resets %d" % (stats["writes"], stats["erases"], stats["resets"]))

print("RESULT:", "PASS" if fails == 0 else "FAIL %d" % fails)
sys.exit(0 if fails == 0 else 1)
