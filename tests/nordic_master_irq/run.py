#!/usr/bin/env python3
"""Execute the real Arm SPIM and TWIM master drivers with bIntEn.

Usage: run.py fixture.elf [MCU name]
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
for base, size in [(0, 0x100000), (0x20000000, 0x100000), (0x40000000, 0x100000), (0x50000000, 0x200000), (0x50800000, 0x100000), (0xE0000000, 0x100000)]:
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

o = [get(symbols['offsets'] + i * 4) for i in range(24)]
CHIP = sys.argv[2] if len(sys.argv) > 2 else 'NRF9160_XXAA'
TWIM_MAX, SH_TX_SUSPEND, SH_TX_STOP, SH_RX_STOP, TWIM_INTEN = [get(symbols['twim'] + i * 4) for i in range(5)]
DMA_MAX = get(symbols['dma_max'])
BUF = call('buffer_address')
BASE = {bus: call('address', bus, 0) for bus in (0, 1)}   # SPIM and TWIM share the instance
pending = []
shorts = []
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
        if get(rx + 4) == 0 and get(tx + 4) == 0:
            # Empty start of the nRF52 anomaly 109 workaround
            put(base + o[23], 1)
            return
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
        shorts.append(get(base + o[19]))
        if polled:
            pending.pop()
            shorts.pop()
            put(reg + 8, get(reg + 4))
            put(base + (o[10] if kind == 'rx' else o[11]), 1)
            put(base + (o[9] if kind == 'rx' else o[17]), 1)
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

def sequence(kind, total, maximum=None):
    blocks = []
    left = total
    while left:
        blocks.append(min(left, maximum or DMA_MAX))
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
base = BASE[0]

def twim_reset():
    pending.clear(); shorts.clear(); actions.clear()
    check(call('init', 0, 0, 1, 1) == 1, 'i2c init')
    check(get(base + o[22]) == TWIM_INTEN, 'TWIM interrupt mask', hex(get(base + o[22])))

def twim_finish(amount=None, event=None, error=False):
    """Hardware ends the oldest TWIM block, then the interrupt runs."""
    kind, ptr, length = pending.pop(0)
    sh = shorts.pop(0)
    put(base + (o[12] if kind == 'rx' else o[13]) + 8, length if amount is None else amount)
    if error:
        put(base + o[18], 1)
        isr(0)
        return kind, sh
    if event is None:
        event = 'stopped' if kind == 'rx' or sh == SH_TX_STOP else 'suspended'
    put(base + (o[9] if event == 'stopped' else o[17]), 1)
    isr(0)
    check(get(base + o[9]) == 0 and get(base + o[17]) == 0, 'events not cleared')
    return kind, sh

active = 0
for kind, length in [('tx', 4), ('rx', 20), ('tx', 2 * TWIM_MAX + 5), ('rx', TWIM_MAX + 7), ('tx', TWIM_MAX)]:
    twim_reset()
    r = call(kind, 0, length)
    check(r == -1, 'i2c', kind, length, 'returns', r)
    blocks = sequence(kind, length, TWIM_MAX)
    done = 0
    for i, n in enumerate(blocks):
        last = i == len(blocks) - 1
        check(len(pending) == 1 and pending[0] == (kind, BUF + done, n), 'i2c', kind, length, pending)
        if kind == 'tx':
            check(shorts[0] == (SH_TX_STOP if last else SH_TX_SUSPEND), 'i2c tx shorts', i, hex(shorts[0]))
        else:
            check(shorts[0] == SH_RX_STOP, 'i2c rx shorts', hex(shorts[0]))
        check(call('busy', 0) == 1 and call('events', 0) == 0, 'i2c released early', kind, length, i)
        twim_finish()
        done += n
    check(not pending, pending)
    check(actions == [], 'STOP task issued by the driver after the hardware stop', actions)
    check(call('events', 0) == 1 and call('events', 1) == length, 'i2c completion', kind, length,
          call('events', 0), call('events', 1))
    check(call('events', 3) == (BUF if kind == 'rx' else 0), 'i2c event buffer')
    check(call('busy', 0) == 0, 'i2c bus not released')
    if kind == 'tx':
        check(call('tx_ready', 0) == 1, 'i2c bTxReady')
    check(call('events', 2) == 0, 'i2c TX_READY event')

# Register read: command with bNoStop keeps the bus suspended, then the receive.
twim_reset()
def i2c_cmd_done():
    check(len(pending) == 1 and pending[0] == ('tx', pending[0][1], 1), pending)
    check(shorts[0] == SH_TX_SUSPEND, 'command must suspend the bus', hex(shorts[0]))
    check(call('tx_ready', 0) == 0)
    twim_finish()
    check(call('tx_ready', 0) == 1, 'command phase not flagged ready')
    check(call('busy', 0) == 1 and actions == [], 'bus released or stopped after the command', actions)
    check(call('events', 0) == 0, 'command phase reported as a completion')
r = call_break('read_reg', (0, 8), 'DeviceIntrfWaitTxComplete', i2c_cmd_done)
check(r == -1, 'i2c read returns', r)
check(pending == [('rx', BUF, 8)] and shorts == [SH_RX_STOP], pending, shorts)
twim_finish()
check(actions == [] and call('busy', 0) == 0, 'receive end', actions)
check(call('events', 0) == 1 and call('events', 1) == 8 and call('events', 3) == BUF, 'read completion')

# Register read longer than one DMA block
twim_reset()
r = call_break('read_reg', (0, TWIM_MAX + 3), 'DeviceIntrfWaitTxComplete', lambda: twim_finish())
check(r == -1, 'long read returns', r)
twim_finish()
check(pending == [('rx', BUF + TWIM_MAX, 3)] and call('busy', 0) == 1, 'second block', pending)
twim_finish()
check(call('events', 0) == 1 and call('events', 1) == TWIM_MAX + 3 and call('busy', 0) == 0, 'long read')

# Bus error: the driver stops the bus, the completion reports what moved.
for kind in ('tx', 'rx'):
    twim_reset()
    check(call(kind, 0, 10) == -1)
    twim_finish(amount=3, error=True)
    check(actions == ['stop'], 'error must stop the bus', kind, actions)
    check(call('busy', 0) == 1 and call('events', 0) == 0, 'completion before STOPPED', kind)
    put(base + o[9], 1)
    isr(0)
    check(call('busy', 0) == 0 and call('events', 0) == 1 and call('events', 1) == 3, 'error end', kind,
          call('busy', 0), call('events', 0), call('events', 1))
    check(actions == ['stop'], 'second STOP task', actions)
    # The bus can be used again
    check(call('tx', 0, 2) == -1 and pending == [('tx', BUF, 2)], 'transfer after an error', pending)
    twim_finish()
    check(call('events', 0) == 2 and call('busy', 0) == 0, 'transfer after an error ended')

# Address NACK on the command of a register read: the read still ends.
twim_reset()
def i2c_cmd_nack():
    twim_finish(amount=0, error=True)
    put(base + o[9], 1)
    isr(0)
    check(call('tx_ready', 0) == 1 and call('events', 0) == 0, 'nacked command')
r = call_break('read_reg', (0, 4), 'DeviceIntrfWaitTxComplete', i2c_cmd_nack)
check(r == -1 and pending == [('rx', BUF, 4)], 'read after nack', pending)
twim_finish(amount=0, error=True)
put(base + o[9], 1)
isr(0)
check(call('events', 0) == 1 and call('events', 1) == 0 and call('busy', 0) == 0, 'nacked read end')
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
print('PASS: %s master SPI and TWIM interrupt transfers' % CHIP)
