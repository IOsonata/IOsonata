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
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_READ, UC_HOOK_CODE
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
    where = [v for k, v in symbols.items() if k == brk or k.startswith('_Z%d%s' % (len(brk), brk))][0] & ~1
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
T = [get(symbols['twi'] + i * 4) for i in range(16 if get(symbols['twi']) else 1)]
S = [get(symbols['spi_legacy'] + i * 4) for i in range(6 if get(symbols['spi_legacy']) else 1)]
TWIM_MAX, SH_TX_SUSPEND, SH_TX_STOP, SH_RX_STOP, TWIM_INTEN = [get(symbols['twim'] + i * 4) for i in range(5)]
DMA_MAX = get(symbols['dma_max'])
BUF = call('buffer_address')
BASE = {bus: call('address', bus, 0) for bus in (0, 1)}   # SPIM and TWIM share the instance
pending = []
shorts = []
actions = []
active = None
polled = False      # model hardware that finishes by itself, for bIntEn false

legacy = None      # 'twi' selects the TWI model, without EasyDMA
sent = []          # bytes written to TXD

at_start = []      # TXDSENT, RXDREADY and SHORTS seen by STARTTX and STARTRX

def legacy_write(addr, value):
    offset = addr - BASE[0]
    if offset == T[10]:
        sent.append(value & 0xFF)
        if legacy == 'twi_polled':
            put(BASE[0] + T[7], 1)     # TXDSENT for the byte just written
        return
    if value != 1:
        return
    name = {T[1]: 'startrx', T[2]: 'starttx', T[3]: 'stop', T[4]: 'resume'}.get(offset)
    if name:
        actions.append(name)
    if legacy == 'twi_polled':
        if name in ('starttx', 'startrx'):
            at_start.append((get(BASE[0] + T[7]), get(BASE[0] + T[6]), get(BASE[0] + T[11])))
        if name == 'stop':
            put(BASE[0] + T[5], 1)     # STOPPED

def legacy_read(uc, access, addr, size, value, data):
    # Polled TWI receive: a byte is always ready
    if legacy == 'twi_polled' and addr == BASE[0] + T[6]:
        put(addr, 1)

def write(uc, access, addr, size, value, data):
    if legacy in ('twi', 'twi_polled'):
        legacy_write(addr, value)
        return
    if legacy == 'spi':
        if addr - BASE[1] == S[3]:
            sent.append(value & 0xFF)
        return
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
u.hook_add(UC_HOOK_MEM_READ, legacy_read)

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

TXBUF_SIZE = [get(symbols['txbuf_size'] + i * 4) for i in range(2)]
TXBUF = [call('txbuf_address', bus, 0) for bus in range(2)]

def transfer(bus, kind, length, maximum, finish, block_check=None):
    """Run one interrupt transfer and its DMA blocks, return what TxData returned.

    A transmit that fits the driver copy is checked to be sent from the copy:
    the caller's buffer is overwritten as soon as the call returns. A longer
    transmit waits in the driver, its blocks are completed from that wait."""
    pattern = bytes((0x30 + i) & 0xFF for i in range(min(length, 4096)))
    u.mem_write(BUF, pattern)
    copied = kind == 'tx' and length <= TXBUF_SIZE[bus]
    src = TXBUF[bus] if copied else BUF
    blocks = sequence(kind, length, maximum)

    def run_blocks():
        done = 0
        for i, n in enumerate(blocks):
            check(len(pending) == 1 and pending[0] == (kind, src + done, n), bus, kind, length,
                  pending, (src + done, n))
            if block_check:
                block_check(i, i == len(blocks) - 1)
            check(call('busy', bus) == 1 and call('events', 0) == 0, 'released early', bus, kind, length, i)
            finish()
            done += n
        check(not pending, pending)

    if kind == 'tx' and not copied:
        r = call_break(kind, (bus, length), 'DeviceIntrfWaitTxComplete', run_blocks)
        check(call('events', 0) == 1, 'long transmit returned before its end', bus, length)
    else:
        r = call(kind, bus, length)
        if copied:
            u.mem_write(BUF, b'\x55' * len(pattern))
            check(bytes(u.mem_read(src, len(pattern))) == pattern, 'transmit not sent from a copy', bus, length)
        run_blocks()
    return r

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
for kind, length in [('tx', 10), ('rx', 20), ('tx', 2 * DMA_MAX + 5), ('rx', DMA_MAX + 7), ('tx', DMA_MAX),
                     ('tx', TXBUF_SIZE[1]), ('tx', TXBUF_SIZE[1] + 1)]:
    pending.clear()
    call('init', 1, 0, 1, 1)
    r = transfer(1, kind, length, DMA_MAX, spi_finish)
    check(r == -1, kind, length, 'returns', r)
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
def twim_block(i, last):
    kind = pending[0][0]
    if kind == 'tx':
        check(shorts[0] == (SH_TX_STOP if last else SH_TX_SUSPEND), 'i2c tx shorts', i, hex(shorts[0]))
    else:
        check(shorts[0] == SH_RX_STOP, 'i2c rx shorts', hex(shorts[0]))

for kind, length in [('tx', 4), ('rx', 20), ('tx', 2 * TWIM_MAX + 5), ('rx', TWIM_MAX + 7), ('tx', TWIM_MAX),
                     ('tx', TXBUF_SIZE[0]), ('tx', TXBUF_SIZE[0] + 1)]:
    twim_reset()
    r = transfer(0, kind, length, TWIM_MAX, twim_finish, twim_block)
    check(r == -1, 'i2c', kind, length, 'returns', r)
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
    check(call('tx', 0, 2) == -1 and pending == [('tx', TXBUF[0], 2)], 'transfer after an error', pending)
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

# ---- TWI master without EasyDMA, interrupt
if T[0]:
    base = BASE[0]
    legacy = 'twi'
    def twi_init():
        actions.clear(); sent.clear()
        check(call('init', 0, 0, 0, 1) == 1, 'twi init')
        check(get(base + T[14]) == T[15], 'TWI interrupt mask', hex(get(base + T[14])))
        actions.clear()
    def twi_event(index, rxd=None):
        if rxd is not None:
            put(base + T[9], rxd)
        put(base + T[index], 1)
        isr(0)
        check(get(base + T[index]) == 0, 'TWI event not cleared', index)

    for length in (1, 3):
        twi_init()
        u.mem_write(BUF, bytes(range(0x10, 0x10 + length)))
        check(call('tx', 0, length) == -1, 'twi tx')
        u.mem_write(BUF, b'\x55' * length)  # caller buffer gone
        check(actions == ['starttx'] and sent == [0x10], 'twi tx start', actions, sent)
        for i in range(length):
            check(call('events', 0) == 0 and call('busy', 0) == 1, 'twi tx early end')
            twi_event(7)
        check(sent == list(range(0x10, 0x10 + length)), 'twi tx bytes', sent)
        check(actions == ['starttx', 'stop'], 'twi tx stop', actions, sent)
        check(call('busy', 0) == 1 and call('events', 0) == 0, 'twi tx released before STOPPED')
        twi_event(5)
        check(call('busy', 0) == 0 and call('tx_ready', 0) == 1, 'twi tx end')
        check(call('events', 0) == 1 and call('events', 1) == length, 'twi tx completion', call('events', 1))

    for length in (1, 2, 4):
        twi_init()
        check(call('rx', 0, length) == -1, 'twi rx')
        check(actions == ['startrx'], 'twi rx start', actions)
        check(get(base + T[11]) == (T[12] if length > 1 else T[13]), 'twi rx shorts', length)
        for i in range(length):
            actions.clear()
            twi_event(6, 0x40 + i)
            left = length - i - 1
            check(actions == (['resume'] if left else []), 'twi rx resume', length, i, actions)
            if left:
                check(get(base + T[11]) == (T[13] if left == 1 else T[12]), 'twi rx shorts', length, i)
        check(call('busy', 0) == 1 and call('events', 0) == 0, 'twi rx released before STOPPED')
        twi_event(5)
        check(call('busy', 0) == 0 and call('events', 0) == 1 and call('events', 1) == length, 'twi rx end')
        check(call('events', 3) == BUF, 'twi rx buffer')
        check(bytes(u.mem_read(BUF, length)) == bytes(0x40 + i for i in range(length)), 'twi rx data')

    # Register read: the command keeps the bus, then the receive
    twi_init()
    def twi_cmd():
        check(actions == ['starttx'] and sent == [4], actions, sent)
        twi_event(7)
        check(call('tx_ready', 0) == 1 and call('busy', 0) == 1 and call('events', 0) == 0, 'twi command')
        check(actions == ['starttx'], 'twi command stopped the bus', actions)
    check(call_break('read_reg', (0, 2), 'DeviceIntrfWaitTxComplete', twi_cmd) == -1)
    check(actions == ['starttx', 'startrx'], 'twi read', actions)
    twi_event(6, 1)
    twi_event(6, 2)
    twi_event(5)
    check(call('events', 0) == 1 and call('events', 1) == 2 and call('busy', 0) == 0, 'twi read end')

    # Bus error
    twi_init()
    check(call('tx', 0, 4) == -1)
    twi_event(7)
    twi_event(8)
    check(actions == ['starttx', 'resume', 'stop'], 'twi error stop', actions)
    twi_event(5)
    check(call('events', 0) == 1 and call('events', 1) == 1 and call('busy', 0) == 0, 'twi error end')

    # Polled TWI after TWIM transfers: TWI and TWIM are one block, the events
    # a TWIM transfer leaves must not count for the TWI.
    legacy = 'twi_polled'
    for kind, length in (('tx', 3), ('rx', 3), ('rx', 1)):
        check(call('init', 0, 0, 0, 0) == 1, 'polled twi init')
        actions.clear(); sent.clear(); at_start.clear()
        put(base + T[7], 1)            # stale TXDSENT
        put(base + T[6], 1)            # stale RXDREADY
        r = call(kind, 0, length)
        check(r == length, 'polled twi', kind, length, r)
        check(len(at_start) == 1 and at_start[0][0 if kind == 'tx' else 1] == 0,
              'polled twi stale event at start', kind, length, at_start)
        if kind == 'tx':
            check(actions == ['starttx', 'resume', 'stop'], 'polled twi tx', actions)
        else:
            # One byte: STOP from the BB shortcut, before the byte is acknowledged
            check(at_start[0][2] == (T[13] if length == 1 else 0), 'polled twi rx shorts', length,
                  hex(at_start[0][2]))
    legacy = None

# ---- SPI master without EasyDMA, interrupt
if S[0]:
    base = BASE[1]
    legacy = 'spi'
    def spi_legacy_init():
        sent.clear()
        check(call('init', 1, 0, 0, 1) == 1, 'spi init')
        check(get(base + S[4]) == S[5], 'SPI interrupt mask', hex(get(base + S[4])))
    def ready(rxd=0):
        put(base + S[2], rxd)
        put(base + S[1], 1)
        isr(1)
        check(get(base + S[1]) == 0, 'READY not cleared')

    for length in (1, 3):
        spi_legacy_init()
        u.mem_write(BUF, bytes(range(0x10, 0x10 + length)))
        check(call('tx', 1, length) == -1, 'spi tx')
        u.mem_write(BUF, b'\x55' * length)  # caller buffer gone
        for i in range(length):
            check(sent == list(range(0x10, 0x11 + i)), 'spi tx bytes', sent)
            check(call('events', 0) == 0 and call('busy', 1) == 1, 'spi tx early end')
            ready()
        check(len(sent) == length, sent)
        check(call('events', 0) == 1 and call('events', 1) == length and call('busy', 1) == 0, 'spi tx end')
        check(call('tx_ready', 1) == 1 and call('events', 3) == 0, 'spi tx ready')

    for length in (1, 3):
        spi_legacy_init()
        check(call('rx', 1, length) == -1, 'spi rx')
        for i in range(length):
            check(sent == [0xA5] * (i + 1), 'spi rx dummy', sent)
            check(call('events', 0) == 0 and call('busy', 1) == 1, 'spi rx early end')
            ready(0x60 + i)
        check(call('events', 0) == 1 and call('events', 1) == length and call('busy', 1) == 0, 'spi rx end')
        check(call('events', 3) == BUF, 'spi rx buffer')
        check(bytes(u.mem_read(BUF, length)) == bytes(0x60 + i for i in range(length)), 'spi rx data')

    spi_legacy_init()
    def spi_legacy_cmd():
        check(sent == [4], sent)
        ready()
        check(call('tx_ready', 1) == 1 and call('busy', 1) == 1 and call('events', 0) == 0, 'spi command')
    check(call_break('read_reg', (1, 2), 'DeviceIntrfWaitTxComplete', spi_legacy_cmd) == -1)
    check(sent == [4, 0xA5], sent)
    ready(1)
    ready(2)
    check(call('events', 0) == 1 and call('events', 1) == 2 and call('busy', 1) == 0, 'spi read end')
    legacy = None

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
print('PASS: %s master interrupt transfers: SPIM, TWIM%s' % (CHIP, ', TWI, SPI' if T[0] and S[0] else ''))
