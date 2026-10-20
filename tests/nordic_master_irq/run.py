#!/usr/bin/env python3
"""Execute the real Arm SPIM and TWIM master drivers with bIntEn on nRF9160.

Usage: run.py fixture.elf
Requires unicorn and pyelftools. The register model starts nothing by itself:
each START or STARTTX/STARTRX task is recorded, and the test then raises the
completion events and runs the shared interrupt handler, which is what the
hardware and NVIC do. Pin configuration is stubbed; this does not model
electrical signalling, bus timing or hardware errata.
"""
import struct
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_WRITE, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0

u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
for base, size in [(0, 0x100000), (0x20000000, 0x100000), (0x50000000, 0x200000), (0x50800000, 0x100000), (0xE0000000, 0x100000)]:
    u.mem_map(base, size)
with open(sys.argv[1], 'rb') as f:
    elf = ELFFile(f)
    symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    for seg in elf.iter_segments():
        if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
            u.mem_write(seg['p_vaddr'], seg.data())

def get(addr):
    return struct.unpack('<I', u.mem_read(addr, 4))[0]

def put(addr, value):
    u.mem_write(addr, struct.pack('<I', value))

def signed(v):
    return v - (1 << 32) if v & 0x80000000 else v

def setup(name, args, sp):
    for i, arg in enumerate(args):
        u.reg_write(UC_ARM_REG_R0 + i, arg)
    u.reg_write(UC_ARM_REG_SP, sp)
    u.reg_write(UC_ARM_REG_LR, 0xF0001)

def call(name, *args, sp=0x200FF000):
    setup(name, args, sp)
    u.emu_start(symbols[name] | 1, 0xF0000, count=10000000)
    assert u.reg_read(UC_ARM_REG_PC) == 0xF0000, (name, 'did not return')
    return signed(u.reg_read(UC_ARM_REG_R0))

def call_break(name, args, brk, action):
    """Run name until the function brk is entered, run action, then resume."""
    hit = []
    def code(uc, address, size, data):
        if not hit:
            hit.append(address)
            uc.emu_stop()
    where = [v for k, v in symbols.items() if k.startswith('_Z%d%s' % (len(brk), brk))][0] & ~1
    h = u.hook_add(UC_HOOK_CODE, code, begin=where, end=where)
    u.ctl_flush_tb()    # blocks translated earlier do not run a new hook
    setup(name, args, 0x200FF000)
    u.emu_start(symbols[name] | 1, 0xF0000, count=10000000)
    u.hook_del(h)
    assert hit, (name, 'never reached', brk)
    ctx = u.context_save()
    action()
    u.context_restore(ctx)
    u.emu_start(hit[0] | 1, 0xF0000, count=10000000)
    assert u.reg_read(UC_ARM_REG_PC) == 0xF0000, (name, 'did not return after', brk)
    return signed(u.reg_read(UC_ARM_REG_R0))

o = [get(symbols['offsets'] + i * 4) for i in range(17)]
DMA_MAX = get(symbols['dma_max'])
BUF = call('buffer_address')
BASE = {0: 0x50008000, 1: 0x50008000}   # SPIM0 and TWIM0 share the instance
pending = []
actions = []
active = None
polled = False      # model hardware that finishes by itself, for bIntEn false

def write(uc, access, addr, size, value, data):
    if active is None or value != 1:
        return
    base = BASE[active]
    offset = addr - base
    if active == 1 and offset == o[0]:
        rx, tx = base + o[4], base + o[5]
        kind, reg = ('rx', rx) if get(rx + 4) else ('tx', tx)
        pending.append((kind, get(reg), get(reg + 4)))
        if polled:
            pending.pop()
            put(reg + 8, get(reg + 4))
            put(base + o[1], 1)
            put(base + o[2 if kind == 'rx' else 3], 1)
    elif active == 0 and offset in (o[6], o[7]):
        kind, reg = ('rx', base + o[12]) if offset == o[6] else ('tx', base + o[13])
        pending.append((kind, get(reg), get(reg + 4)))
        if polled:
            pending.pop()
            put(base + (o[10] if kind == 'rx' else o[11]), 1)
    elif active == 0 and offset == o[8]:
        actions.append('stop')
        if polled:
            put(base + o[9], 1)

u.hook_add(UC_HOOK_MEM_WRITE, write)

def isr(bus):
    call('isr', bus, 0, sp=0x200F0000)

def spi_finish():
    """Hardware completes the oldest SPIM block, then the interrupt runs."""
    kind, ptr, length = pending.pop(0)
    base = BASE[1]
    put(base + o[4 if kind == 'rx' else 5] + 8, length)
    put(base + o[1], 1)
    put(base + o[2], 1)
    put(base + o[3], 1)
    isr(1)
    assert get(base + o[1]) == 0 and get(base + o[2]) == 0 and get(base + o[3]) == 0

def check(cond, *info):
    assert cond, info

def sequence(kind, total):
    blocks = []
    left = total
    while left:
        blocks.append(min(left, DMA_MAX))
        left -= blocks[-1]
    return blocks

# ---- SPI master, interrupt and DMA
check(call('init', 1, 0, 1, 1) == 1, 'spi init')
active = 1
for kind, length in [('tx', 10), ('rx', 20), ('tx', 2 * DMA_MAX + 5), ('rx', DMA_MAX + 7), ('tx', DMA_MAX)]:
    pending.clear()
    call('init', 1, 0, 1, 1)
    r = call(kind, 1, length)
    check(r == -1, kind, length, 'returns', r)
    check(call('busy', 1) == 1, 'bus released before the transfer ended')
    blocks = sequence(kind, length)
    done = 0
    for i, n in enumerate(blocks):
        check(len(pending) == 1, kind, length, 'one block at a time', pending)
        check(pending[0] == (kind, BUF + done, n), kind, length, pending, (BUF + done, n))
        check(call('events', 0) == 0, 'completion before the last block')
        spi_finish()
        done += n
        if i < len(blocks) - 1:
            check(call('busy', 1) == 1, 'bus released between blocks')
    check(not pending)
    check(call('events', 0) == 1, kind, length, 'one completion event')
    check(call('events', 1) == length, kind, length, 'count', call('events', 1))
    check(call('events', 3) == (BUF if kind == 'rx' else 0), 'event buffer')
    check(call('busy', 1) == 0, 'bus not released')
    if kind == 'tx':
        check(call('tx_ready', 1) == 1, 'bTxReady')
# Command then read: the command phase keeps the bus until the data arrives.
pending.clear()
call('init', 1, 0, 1, 1)
def spi_cmd_done():
    check(len(pending) == 1 and pending[0] == ('tx', pending[0][1], 1), pending)
    check(call('tx_ready', 1) == 0)
    spi_finish()
    check(call('tx_ready', 1) == 1, 'command phase not flagged ready')
    check(call('busy', 1) == 1, 'bus released after the command phase')
    check(call('events', 0) == 0, 'command phase reported as a completion')
r = call_break('read_reg', (1, 8), 'DeviceIntrfWaitTxComplete', spi_cmd_done)
check(r == -1, 'read returns', r)
check(pending == [('rx', BUF, 8)], pending)
spi_finish()
check(call('events', 0) == 1 and call('events', 1) == 8 and call('busy', 1) == 0, 'read completion')
active = None

# ---- TWIM master, interrupt and DMA
check(call('init', 0, 0, 1, 1) == 1, 'i2c init')
active = 0
base = BASE[0]
pending.clear(); actions.clear()
check(call('tx', 0, 4) == -1, 'i2c tx')
check(pending == [('tx', BUF, 4)] and call('busy', 0) == 1, pending)
put(base + o[11], 1)
isr(0)
check(call('tx_ready', 0) == 1 and call('busy', 0) == 0 and actions == ['stop'], 'i2c tx end', actions)
check(call('events', 2) == 1, 'TX_READY event')
put(base + o[9], 1)
isr(0)
check(call('events', 0) == 1, 'i2c tx completion')

# Register read: command with bNoStop, then the receive phase.
pending.clear(); actions.clear()
call('init', 0, 0, 1, 1)
def i2c_cmd_done():
    check(len(pending) == 1 and pending[0][0] == 'tx' and pending[0][2] == 1, pending)
    check(call('tx_ready', 0) == 0)
    put(base + o[11], 1)
    isr(0)
    check(call('tx_ready', 0) == 1, 'command phase not flagged ready')
    check(call('busy', 0) == 1 and actions == [], 'bus released or stopped after the command', actions)
r = call_break('read_reg', (0, 8), 'DeviceIntrfWaitTxComplete', i2c_cmd_done)
check(r == -1, 'i2c read returns', r)
check(pending[1:] == [('rx', BUF, 8)], pending)
put(base + o[12] + 8, 8)
put(base + o[14], 1)
put(base + o[10], 1)
isr(0)
check(call('busy', 0) == 0 and actions == ['stop'], 'receive end', actions)
put(base + o[9], 1)
isr(0)
check(call('events', 0) == 1 and call('events', 1) == 8, 'read completion', call('events', 0), call('events', 1))
active = None

# ---- Polled transfers still work when bIntEn is false
polled = True
pending.clear()
for bus in (1, 0):
    check(call('init', bus, 0, 1, 0) == 1, 'polled init', bus)
    active = bus
    for kind in ('tx', 'rx'):
        n = 2 * DMA_MAX + 5 if bus else 4
        check(call(kind, bus, n) == n, 'polled', bus, kind)
        check(call('busy', bus) == 0 and not pending, 'polled end', bus, kind, call('busy', bus), pending)
    active = None
polled = False
print('PASS: nRF9160 master SPI and TWIM interrupt transfers')
