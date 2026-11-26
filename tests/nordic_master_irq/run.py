#!/usr/bin/env python3
"""Execute the real Arm SPIM and TWIM master drivers with bIntEn.

Usage: run.py fixture.elf [MCU name]
Requires unicorn and pyelftools. The register model starts nothing by itself:
each START or STARTTX/STARTRX task is recorded. An interrupt driven transfer
waits in the driver for its interrupt handler: when the firmware runs on
without returning, the test raises the next completion events and runs the
shared interrupt handler on another stack, which is what the hardware and
NVIC do, then lets the firmware continue. Pin configuration is stubbed; this
does not model electrical signalling, bus timing or hardware errata.
"""
import struct
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_READ
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0

u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
for base, size in [(0, 0x100000), (0x10000000, 0x1000), (0x20000000, 0x100000), (0x40000000, 0x100000), (0x50000000, 0x200000), (0x50800000, 0x100000), (0xE0000000, 0x100000)]:
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

stack = [0x200FF000]    # the calls made while a transfer waits use another stack

def call(name, *args, count=10000000):
    setup(name, args, stack[-1])
    u.emu_start(symbols[name] | 1, 0xF0000, count=count)
    assert u.reg_read(UC_ARM_REG_PC) == 0xF0000, (name, 'did not return')
    return signed(u.reg_read(UC_ARM_REG_R0))

QUANTUM = 20000     # instructions, far below the time limit of the driver
DONE = object()

def run(name, args, steps=None):
    """Call name, which waits for its interrupts.

    steps is a generator function. Each time the firmware has run QUANTUM
    instructions without returning, it is waiting for the interrupt handler:
    the generator is advanced to its next yield, where it checks what the
    driver has started, ends it as the hardware does and runs the interrupt.
    The call must return when the generator has nothing left, not before."""
    gen = steps() if steps else iter(())
    setup(name, args, stack[-1])
    pc = symbols[name] | 1
    while True:
        u.emu_start(pc, 0xF0000, count=QUANTUM)
        stopped = u.reg_read(UC_ARM_REG_PC)
        if stopped == 0xF0000:
            break
        ctx = u.context_save()
        stack.append(0x200F0000)
        try:
            next(gen)
        except StopIteration:
            raise AssertionError((name, args, 'still waits, nothing left to serve'))
        stack.pop()
        u.context_restore(ctx)
        pc = stopped | 1
    r = signed(u.reg_read(UC_ARM_REG_R0))
    assert next(gen, DONE) is DONE, (name, args, 'returned before the hardware was done', r)
    return r

o = [get(symbols['offsets'] + i * 4) for i in range(26)]
CHIP = sys.argv[2] if len(sys.argv) > 2 else 'NRF9160_XXAA'
T = [get(symbols['twi'] + i * 4) for i in range(16 if get(symbols['twi']) else 1)]
S = [get(symbols['spi_legacy'] + i * 4) for i in range(9 if get(symbols['spi_legacy']) else 1)]
if CHIP.startswith('NRF52832'):
    put(0x10000130, 0x06)      # FICR INFO.PART, nRF52832
    put(0x10000134, 0x05)      # FICR INFO.VARIANT, revision 3
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
nack_at = None     # polled TWI: the byte of this index is refused

def legacy_write(addr, value):
    offset = addr - BASE[0]
    if offset == T[10]:
        sent.append(value & 0xFF)
        if legacy == 'twi_polled':
            if nack_at is not None and len(sent) - 1 == nack_at:
                errorsrc[0] = 4
                put(BASE[0] + T[8], 1) # ERROR, data NACK
            else:
                put(BASE[0] + T[7], 1) # TXDSENT for the byte just written
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

intenclr = []      # values written to INTENCLR of the SPI and TWI instance
errorsrc = [0]     # ERRORSRC of the TWI instance: set with ERROR, cleared by writing ones

def read(uc, access, addr, size, value, data):
    if addr == BASE[0] + o[21]:
        put(addr, errorsrc[0])
    legacy_read(uc, access, addr, size, value, data)

def write(uc, access, addr, size, value, data):
    if addr == BASE[0] + o[21]:
        errorsrc[0] &= ~value
    if addr in (BASE[1] + o[24], BASE[0] + o[25]):
        intenclr.append(value & 0xFFFFFFFF)
    if legacy in ('twi', 'twi_polled'):
        legacy_write(addr, value)
        return
    if legacy == 'spi':
        if addr - BASE[1] == S[3]:
            sent.append(value & 0xFF)
        return
    if active == 1 and S[0] and addr - BASE[1] == S[3] and get(BASE[1] + S[6]) == S[7]:
        # SPI (without EasyDMA) on the SPIM instance: the byte is clocked
        sent.append(value & 0xFF)
        put(BASE[1] + S[2], 0x5A)
        put(BASE[1] + S[1], 1)
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
u.hook_add(UC_HOOK_MEM_READ, read)

def isr(bus):
    call('isr', bus, 0)

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

def transfer(bus, kind, length, maximum, finish, block_check=None):
    """Run one interrupt transfer and its DMA blocks, return what it returned.

    The driver has no copy of the data: every block is checked to use the
    caller's buffer, at the offset reached so far."""
    pattern = bytes((0x30 + i) & 0xFF for i in range(length))
    u.mem_write(BUF, pattern)
    blocks = sequence(kind, length, maximum)
    def steps():
        done = 0
        for i, n in enumerate(blocks):
            check(len(pending) == 1 and pending[0] == (kind, BUF + done, n), bus, kind, length,
                  pending, (BUF + done, n))
            if block_check:
                block_check(i, i == len(blocks) - 1)
            check(call('busy', bus) == 1 and call('events', 0) == 0, 'ended early', bus, kind, length, i)
            finish()
            done += n
            yield
    r = run(kind, (bus, length), steps)
    check(not pending, pending)
    return r

def once(finish):
    """One hardware step and its interrupt."""
    def steps():
        finish()
        yield
    return steps

def ended(bus, kind, length, name):
    """The transfer returned its count, reported it and released the bus."""
    check(call('events', 0) == 1 and call('events', 1) == length, name, 'completion', kind, length,
          call('events', 0), call('events', 1))
    check(call('events', 3) == (BUF if kind == 'rx' else 0), name, 'event buffer')
    check(call('busy', bus) == 0, name, 'bus not released')
    if kind == 'tx':
        check(call('tx_ready', bus) == 1, name, 'bTxReady')
    check(call('events', 2) == (1 if kind == 'tx' else 0), name, 'TX_READY event', kind, call('events', 2))

# ---- SPI master, interrupt and DMA
check(call('init', 1, 0, 1, 1) == 1, 'spi init')
active = 1
for kind, length in [('tx', 10), ('rx', 20), ('tx', min(2 * DMA_MAX + 5, 66000)), ('rx', DMA_MAX + 7), ('tx', DMA_MAX),
                     ('tx', 257)]:
    pending.clear()
    call('init', 1, 0, 1, 1)
    r = transfer(1, kind, length, DMA_MAX, spi_finish)
    check(r == length, 'spi', kind, length, 'returns', r)
    ended(1, kind, length, 'spi')
# Command then read: the command phase keeps the bus until the data arrives.
pending.clear()
call('init', 1, 0, 1, 1)
def spi_read():
    check(len(pending) == 1 and pending[0] == ('tx', pending[0][1], 1), pending)
    check(call('tx_ready', 1) == 0)
    spi_finish()
    check(call('tx_ready', 1) == 1, 'command phase not flagged ready')
    check(call('busy', 1) == 1, 'bus released after the command phase')
    check(call('events', 0) == 0, 'command phase reported as a completion')
    yield
    check(pending == [('rx', BUF, 8)], pending)
    spi_finish()
    yield
r = run('read_reg', (1, 8), spi_read)
check(r == 8, 'read returns', r)
check(call('events', 0) == 1 and call('events', 1) == 8 and call('busy', 1) == 0, 'read completion')
# No interrupt at all: the transfer ends on the time limit of the driver
pending.clear()
call('init', 1, 0, 1, 1)
r = call('tx', 1, 10, count=40000000)
check(r == 0 and call('busy', 1) == 0 and call('events', 0) == 0, 'spi transfer without interrupt', r)
pending.clear()
r = run('tx', (1, 4), once(spi_finish))
check(r == 4 and call('busy', 1) == 0, 'spi transfer after a time limit', r)
active = None

# A new configuration clears every interrupt of the previous one (SPI and SPIM,
# TWI and TWIM, are one block: an event of one is seen by the other).
for bus in (1, 0):
    for dma in ((0, 1) if S[0] else (1,)):
        intenclr.clear()
        check(call('init', bus, 0, dma, 0) == 1)
        check(0xFFFFFFFF in intenclr, 'interrupts of a previous configuration kept', bus, dma, intenclr)

# One byte SPIM receive on a part with anomaly 58: done by the SPI, the SPIM
# never starts and is enabled again after.
if S[0] and CHIP.startswith('NRF52832'):
    active = 1
    for irq in (0, 1):
        pending.clear(); sent.clear()
        call('init', 1, 0, 1, irq)
        u.mem_write(BUF, b'\x00')
        r = call('rx', 1, 1)
        check(r == 1, 'one byte receive', irq, r)
        check(not pending and sent == [0xA5], 'one byte receive on the SPIM', irq, pending, sent)
        check(u.mem_read(BUF, 1)[0] == 0x5A, 'one byte receive data', irq)
        check(get(BASE[1] + S[6]) == S[8], 'SPIM not enabled again', irq, get(BASE[1] + S[6]))
        check(call('busy', 1) == 0, 'one byte receive bus', irq)
    # Command then one byte: the command uses the SPIM, the byte the SPI
    pending.clear(); sent.clear()
    call('init', 1, 0, 1, 1)
    r = run('read_reg', (1, 1), once(spi_finish))
    check(r == 1 and sent == [0xA5] and call('busy', 1) == 0, 'command then one byte', r, sent)
active = None

# ---- TWIM master, interrupt and DMA
base = BASE[0]

def twim_reset():
    pending.clear(); shorts.clear(); actions.clear()
    check(call('init', 0, 0, 1, 1) == 1, 'i2c init')
    check(get(base + o[22]) == TWIM_INTEN, 'TWIM interrupt mask', hex(get(base + o[22])))

def twim_stopped():
    put(base + o[9], 1)
    isr(0)

def twim_finish(amount=None, event=None, error=False):
    """Hardware ends the oldest TWIM block, then the interrupt runs."""
    kind, ptr, length = pending.pop(0)
    sh = shorts.pop(0)
    put(base + (o[12] if kind == 'rx' else o[13]) + 8, length if amount is None else amount)
    if error:
        errorsrc[0] = 2     # address NACK
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

# A transmit has no size limit: it is sent from the caller's buffer, one DMA
# block after the other.
for kind, length in [('tx', 4), ('rx', 20), ('rx', TWIM_MAX + 7),
                     ('tx', TWIM_MAX), ('tx', TWIM_MAX + 5)]:
    twim_reset()
    r = transfer(0, kind, length, TWIM_MAX, twim_finish, twim_block)
    check(r == length, 'i2c', kind, length, 'returns', r)
    check(actions == [], 'STOP task issued by the driver after the hardware stop', actions)
    ended(0, kind, length, 'i2c')

# Register read: command with bNoStop keeps the bus suspended, then the receive.
twim_reset()
def i2c_read():
    check(len(pending) == 1 and pending[0] == ('tx', pending[0][1], 1), pending)
    check(shorts[0] == SH_TX_SUSPEND, 'command must suspend the bus', hex(shorts[0]))
    check(call('tx_ready', 0) == 0)
    twim_finish()
    check(call('tx_ready', 0) == 1, 'command phase not flagged ready')
    check(call('busy', 0) == 1 and actions == [], 'bus released or stopped after the command', actions)
    check(call('events', 0) == 0, 'command phase reported as a completion')
    yield
    check(pending == [('rx', BUF, 8)] and shorts == [SH_RX_STOP], pending, shorts)
    twim_finish()
    yield
r = run('read_reg', (0, 8), i2c_read)
check(r == 8, 'i2c read returns', r)
check(actions == [] and call('busy', 0) == 0, 'receive end', actions)
check(call('events', 0) == 1 and call('events', 1) == 8 and call('events', 3) == BUF, 'read completion')

# Combined write: DeviceIntrfWrite() joins command and data on its stack, the
# driver sends from that buffer and returns when it has gone out.
twim_reset()
def i2c_write():
    check(len(pending) == 1 and pending[0][0] == 'tx' and pending[0][2] == 6, pending)
    check(pending[0][1] != BUF, 'combined write not sent from the buffer of DeviceIntrfWrite')
    check(call('tx_ready', 0) == 0, 'combined write ended early')
    check(bytes(u.mem_read(pending[0][1], 6)) == bytes(u.mem_read(BUF, 1)) + bytes(u.mem_read(BUF + 1, 5)),
          'combined write data')
    twim_finish()
    yield
r = run('write_data', (0, 6), i2c_write)
check(r == 5, 'combined write returns', r)
check(call('events', 0) == 1 and call('events', 1) == 6 and call('busy', 0) == 0, 'combined write completion',
      call('events', 0), call('events', 1))

# Register read longer than one DMA block
twim_reset()
def i2c_long_read():
    twim_finish()
    yield
    check(pending == [('rx', BUF, TWIM_MAX)], 'first block', pending)
    twim_finish()
    yield
    check(pending == [('rx', BUF + TWIM_MAX, 3)] and call('busy', 0) == 1, 'second block', pending)
    twim_finish()
    yield
r = run('read_reg', (0, TWIM_MAX + 3), i2c_long_read)
check(r == TWIM_MAX + 3, 'long read returns', r)
check(call('events', 0) == 1 and call('events', 1) == TWIM_MAX + 3 and call('busy', 0) == 0, 'long read')

# Bus error: the driver stops the bus, the bytes moved before it are returned.
for kind in ('tx', 'rx'):
    twim_reset()
    def i2c_error():
        twim_finish(amount=3, error=True)
        check(actions == ['stop'], 'error must stop the bus', kind, actions)
        check(call('busy', 0) == 1 and call('events', 0) == 0, 'completion before STOPPED', kind)
        yield
        twim_stopped()
        yield
    r = run(kind, (0, 10), i2c_error)
    check(r == 3, 'error returns', kind, r)
    check(call('busy', 0) == 0 and call('events', 0) == 1 and call('events', 1) == 3, 'error end', kind,
          call('busy', 0), call('events', 0), call('events', 1))
    check(actions == ['stop'], 'second STOP task', actions)
    # The bus can be used again
    r = run('tx', (0, 2), once(twim_finish))
    check(r == 2 and call('events', 0) == 2 and call('busy', 0) == 0, 'transfer after an error', r)

# Address NACK on the command of a register read: the receive still runs.
twim_reset()
def i2c_cmd_nack():
    twim_finish(amount=0, error=True)
    twim_stopped()
    check(call('tx_ready', 0) == 1 and call('events', 0) == 0, 'nacked command')
    yield
    check(pending == [('rx', BUF, 4)], 'read after nack', pending)
    twim_finish()
    yield
r = run('read_reg', (0, 4), i2c_cmd_nack)
check(r == 4 and call('busy', 0) == 0, 'read after a nacked command', r)

# No device: every attempt is refused, the read is retried as in polling mode
# and returns 0.
twim_reset()
attempts = []
def i2c_absent():
    for i in range(6):          # MaxRetry is 5
        for phase in ('tx', 'rx'):
            check(len(pending) == 1 and pending[0][0] == phase, 'absent device', i, pending)
            twim_finish(amount=0, error=True)
            twim_stopped()
            yield
        attempts.append(i)
r = run('read_reg', (0, 4), i2c_absent)
check(r == 0 and len(attempts) == 6 and call('busy', 0) == 0, 'absent device', r, attempts)

# No interrupt at all: each attempt ends on the time limit of the driver,
# which stops the bus.
twim_reset()
r = call('tx', 0, 10, count=80000000)
check(r == 0 and call('busy', 0) == 0, 'i2c transfer without interrupt', r)
check(actions == ['stop'] * 6 and len(pending) == 6, 'time limit must stop the bus', actions, len(pending))
pending.clear(); shorts.clear(); actions.clear()
twim_stopped()
check(call('events', 0) == 0, 'late STOPPED reported')
r = run('rx', (0, 3), once(twim_finish))
check(r == 3 and call('busy', 0) == 0, 'i2c transfer after a time limit', r)
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
        def twi_tx():
            check(actions == ['starttx'] and sent == [0x10], 'twi tx start', actions, sent)
            for i in range(length):
                check(call('events', 0) == 0 and call('busy', 0) == 1, 'twi tx early end')
                twi_event(7)
                yield
            check(sent == list(range(0x10, 0x10 + length)), 'twi tx bytes', sent)
            check(actions == ['starttx', 'stop'], 'twi tx stop', actions, sent)
            check(call('events', 0) == 0, 'twi tx ended before STOPPED')
            twi_event(5)
            yield
        check(run('tx', (0, length), twi_tx) == length, 'twi tx')
        check(actions == ['starttx', 'stop'], 'twi tx second stop', actions)
        check(call('busy', 0) == 0 and call('tx_ready', 0) == 1, 'twi tx end')
        check(call('events', 0) == 1 and call('events', 1) == length, 'twi tx completion', call('events', 1))

    for length in (1, 2, 4):
        twi_init()
        def twi_rx():
            check(actions == ['startrx'], 'twi rx start', actions)
            check(get(base + T[11]) == (T[12] if length > 1 else T[13]), 'twi rx shorts', length)
            for i in range(length):
                actions.clear()
                twi_event(6, 0x40 + i)
                left = length - i - 1
                check(actions == (['resume'] if left else []), 'twi rx resume', length, i, actions)
                if left:
                    check(get(base + T[11]) == (T[13] if left == 1 else T[12]), 'twi rx shorts', length, i)
                yield
            check(call('busy', 0) == 1 and call('events', 0) == 0, 'twi rx ended before STOPPED')
            twi_event(5)
            yield
        check(run('rx', (0, length), twi_rx) == length, 'twi rx')
        check(call('busy', 0) == 0 and call('events', 0) == 1 and call('events', 1) == length, 'twi rx end')
        check(call('events', 3) == BUF, 'twi rx buffer')
        check(bytes(u.mem_read(BUF, length)) == bytes(0x40 + i for i in range(length)), 'twi rx data')

    # Register read: the command keeps the bus, then the receive
    twi_init()
    def twi_read():
        check(actions == ['starttx'] and sent == [4], actions, sent)
        twi_event(7)
        check(call('tx_ready', 0) == 1 and call('busy', 0) == 1 and call('events', 0) == 0, 'twi command')
        check(actions == ['starttx'], 'twi command stopped the bus', actions)
        yield
        check(actions == ['starttx', 'startrx'], 'twi read', actions)
        twi_event(6, 1)
        yield
        twi_event(6, 2)
        yield
        twi_event(5)
        yield
    check(run('read_reg', (0, 2), twi_read) == 2, 'twi read')
    check(call('events', 0) == 1 and call('events', 1) == 2 and call('busy', 0) == 0, 'twi read end')
    check(bytes(u.mem_read(BUF, 2)) == b'\x01\x02', 'twi read data')

    # Bus error
    twi_init()
    def twi_error():
        twi_event(7)
        yield
        errorsrc[0] = 4     # data NACK
        twi_event(8)
        check(actions == ['starttx', 'resume', 'stop'], 'twi error stop', actions)
        yield
        twi_event(5)
        yield
    check(run('tx', (0, 4), twi_error) == 1, 'twi error')
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

    # A refused byte stops the transmit: nothing more is written to TXD
    check(call('init', 0, 0, 0, 0) == 1)
    actions.clear(); sent.clear(); at_start.clear()
    nack_at = 2
    r = call('tx', 0, 5)
    nack_at = None
    check(r == 2 and len(sent) == 3, 'polled twi transmit after a NACK', r, sent)
    check(actions.count('stop') >= 1, 'polled twi NACK without STOP', actions)
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
        def spi_tx():
            for i in range(length):
                check(sent == list(range(0x10, 0x11 + i)), 'spi tx bytes', sent)
                check(call('events', 0) == 0 and call('busy', 1) == 1, 'spi tx early end')
                ready()
                yield
        check(run('tx', (1, length), spi_tx) == length, 'spi tx')
        check(len(sent) == length, sent)
        check(call('events', 0) == 1 and call('events', 1) == length and call('busy', 1) == 0, 'spi tx end')
        check(call('tx_ready', 1) == 1 and call('events', 3) == 0, 'spi tx ready')

    for length in (1, 3):
        spi_legacy_init()
        def spi_rx():
            for i in range(length):
                check(sent == [0xA5] * (i + 1), 'spi rx dummy', sent)
                check(call('events', 0) == 0 and call('busy', 1) == 1, 'spi rx early end')
                ready(0x60 + i)
                yield
        check(run('rx', (1, length), spi_rx) == length, 'spi rx')
        check(call('events', 0) == 1 and call('events', 1) == length and call('busy', 1) == 0, 'spi rx end')
        check(call('events', 3) == BUF, 'spi rx buffer')
        check(bytes(u.mem_read(BUF, length)) == bytes(0x60 + i for i in range(length)), 'spi rx data')

    spi_legacy_init()
    def spi_legacy_read():
        check(sent == [4], sent)
        ready()
        check(call('tx_ready', 1) == 1 and call('busy', 1) == 1 and call('events', 0) == 0, 'spi command')
        yield
        check(sent == [4, 0xA5], sent)
        ready(1)
        yield
        ready(2)
        yield
    check(run('read_reg', (1, 2), spi_legacy_read) == 2, 'spi read')
    check(call('events', 0) == 1 and call('events', 1) == 2 and call('busy', 1) == 0, 'spi read end')
    legacy = None

# ---- Polled transfers still work when bIntEn is false
polled = True
pending.clear()
for bus in (1, 0):
    check(call('init', bus, 0, 1, 0) == 1, 'polled init', bus)
    active = bus
    for kind in ('tx', 'rx'):
        n = min(2 * DMA_MAX + 5, 66000) if bus else 4
        check(call(kind, bus, n) == n, 'polled', bus, kind)
        check(call('busy', bus) == 0 and not pending, 'polled end', bus, kind, call('busy', bus), pending)
    active = None
polled = False
print('PASS: %s master interrupt transfers: SPIM, TWIM%s' % (CHIP, ', TWI, SPI' if T[0] and S[0] else ''))
