#!/usr/bin/env python3
"""Execute the real Arm SPIM and TWIM master drivers with bIntEn.

Usage: run.py fixture.elf [MCU name]
Requires unicorn and pyelftools. The register model starts nothing by itself:
each START or STARTTX/STARTRX task is recorded, and the test then raises the
completion events and runs the shared interrupt handler, which is what the
hardware and NVIC do. An I2C master transfer is synchronous and waits for its
interrupt: the test serves the controller and the interrupt each time the
firmware has run for a while without returning. Pin configuration is stubbed; this does not model
electrical signalling, bus timing or hardware errata.
"""
import struct
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_READ, UC_HOOK_CODE
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

def call(name, *args, sp=0x200FF000):
    setup(name, args, sp)
    u.emu_start(symbols[name] | 1, 0xF0000, count=10000000)
    assert u.reg_read(UC_ARM_REG_PC) == 0xF0000, (name, 'did not return')
    return signed(u.reg_read(UC_ARM_REG_R0))

def call_break(name, args, brk, action):
    """Run name until the function brk is entered, run action, then resume.

    The stop is asked for on each instruction of brk until the emulator takes
    it: a stop asked for on the entry alone can be dropped when the call
    follows a conditional block."""
    hit = []
    def code(uc, address, size, data):
        hit.append(address)
        uc.emu_stop()
    prefixes = ('_Z%d%s' % (len(brk), brk), '_ZL%d%s' % (len(brk), brk))
    where = [v for k, v in symbols.items() if k == brk or k.startswith(prefixes)][0] & ~1
    last = min(v & ~1 for v in symbols.values() if (v & ~1) > where) - 2
    h = u.hook_add(UC_HOOK_CODE, code, begin=where, end=last)
    u.ctl_flush_tb()    # blocks translated earlier do not run a new hook
    setup(name, args, 0x200FF000)
    u.emu_start(symbols[name] | 1, 0xF0000, count=10000000)
    u.hook_del(h)
    u.ctl_flush_tb()
    assert hit, (name, 'never reached', brk)
    stopped = u.reg_read(UC_ARM_REG_PC) & ~1
    assert where <= stopped <= last, (name, 'did not stop in', brk, hex(stopped))
    ctx = u.context_save()
    action()
    u.context_restore(ctx)
    u.emu_start(stopped | 1, 0xF0000, count=10000000)
    assert u.reg_read(UC_ARM_REG_PC) == 0xF0000, (name, 'did not return after', brk)
    return signed(u.reg_read(UC_ARM_REG_R0))

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
nack_at = None     # TWI: the byte of this index is refused
twi_queue = []     # TWI with interrupt: the events to come, in order
twi = {'rx': False, 'stop': False, 'n': 0}

def legacy_write(addr, value):
    offset = addr - BASE[0]
    if offset == T[10]:
        sent.append(value & 0xFF)
        if legacy == 'twi_int':
            twi_queue.append('error' if nack_at is not None and len(sent) - 1 == nack_at else 'txdsent')
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
    if legacy == 'twi_int':
        if name == 'starttx':
            twi['rx'] = False
        elif name == 'startrx':
            twi.update(rx=True, stop=get(BASE[0] + T[11]) & T[13] != 0, n=0)
            twi_queue.append('rxdready')
        elif name == 'stop':
            if twi['rx']:
                twi['stop'] = True         # STOPPED follows the byte being received
            elif 'stopped' not in twi_queue and not get(BASE[0] + T[5]):
                twi_queue.append('stopped')
    if legacy == 'twi_polled':
        if name in ('starttx', 'startrx'):
            at_start.append((get(BASE[0] + T[7]), get(BASE[0] + T[6]), get(BASE[0] + T[11])))
        if name == 'stop':
            put(BASE[0] + T[5], 1)     # STOPPED

def legacy_read(uc, access, addr, size, value, data):
    # Polled TWI receive: a byte is always ready
    if legacy == 'twi_polled' and addr == BASE[0] + T[6]:
        put(addr, 1)
    # TWI with interrupt: reading RXD lets the next byte come, or the STOP
    if legacy == 'twi_int' and addr == BASE[0] + T[9] and twi['rx']:
        if twi['stop']:
            twi['rx'] = False
            twi_queue.append('stopped')
        else:
            twi_queue.append('rxdready')

intenclr = []      # values written to INTENCLR of the SPI and TWI instance
errorsrc = [0]     # ERRORSRC of the TWI instance: set with ERROR, cleared by writing ones

def read(uc, access, addr, size, value, data):
    if addr == BASE[0] + o[21]:
        put(addr, errorsrc[0])
    legacy_read(uc, access, addr, size, value, data)

inten = [0]        # INTEN of the TWI instance, from INTENSET and INTENCLR
stop_req = [False] # TWIM: STOP task not answered yet

def write(uc, access, addr, size, value, data):
    if addr == BASE[0] + o[22]:
        inten[0] |= value
    if addr == BASE[0] + o[25]:
        inten[0] &= ~value
    if addr in (BASE[0] + o[22], BASE[0] + o[25]):
        put(BASE[0] + o[22], inten[0])
        put(BASE[0] + o[25], inten[0])
    if addr == BASE[0] + o[21]:
        errorsrc[0] &= ~value
    if addr in (BASE[1] + o[24], BASE[0] + o[25]):
        intenclr.append(value & 0xFFFFFFFF)
    if legacy in ('twi_int', 'twi_polled'):
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
        stop_req[0] = False
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
        stop_req[0] = True
        if polled:
            put(base + o[9], 1)

u.hook_add(UC_HOOK_MEM_WRITE, write)
u.hook_add(UC_HOOK_MEM_READ, read)

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

def transfer(bus, kind, length, maximum, finish, block_check=None):
    """Run one interrupt transfer and its DMA blocks, return what TxData returned.

    The driver has no copy of the data: every block is checked to use the
    caller's buffer, at the offset reached so far."""
    pattern = bytes((0x30 + i) & 0xFF for i in range(length))
    u.mem_write(BUF, pattern)
    blocks = sequence(kind, length, maximum)
    r = call(kind, bus, length)
    done = 0
    for i, n in enumerate(blocks):
        check(len(pending) == 1 and pending[0] == (kind, BUF + done, n), bus, kind, length,
              pending, (BUF + done, n))
        if block_check:
            block_check(i, i == len(blocks) - 1)
        check(call('busy', bus) == 1 and call('events', 0) == 0, 'released early', bus, kind, length, i)
        finish()
        done += n
    check(not pending, pending)
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
for kind, length in [('tx', 10), ('rx', 20), ('tx', min(2 * DMA_MAX + 5, 66000)), ('rx', DMA_MAX + 7), ('tx', DMA_MAX),
                     ('tx', 257)]:
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
    check(call('events', 2) == (1 if kind == 'tx' else 0), 'spi TX_READY event', kind, call('events', 2))
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
    r = call_break('read_reg', (1, 1), 'DeviceIntrfWaitTxComplete', spi_finish)
    check(r == 1 and sent == [0xA5] and call('busy', 1) == 0, 'command then one byte', r, sent)
active = None

# ---- I2C master with bIntEn. A transfer is synchronous, its wait functions
# wait for the interrupt. The model does nothing by itself: when the firmware
# has run QUANTUM instructions without returning it waits, the test then ends
# what the controller was doing and runs the interrupt handler when the event
# is enabled in INTEN, which is what the NVIC does.
base = BASE[0]
QUANTUM = 20000
irq_blocked = [False]  # the interrupt cannot run, as from a handler of higher priority
EVENTS = [o[9], o[10], o[11], o[17], o[18]] + ([T[5], T[6], T[7], T[8]] if T[0] else [])

def irq_pending():
    return not irq_blocked[0] and any(
        get(base + e) and inten[0] & (1 << ((e - 0x100) // 4)) for e in EVENTS)

def run_served(name, args, step):
    """Call name and serve it each time it waits. Returns what it returned,
    the number of controller steps served and of interrupts run."""
    setup(name, args, 0x200FF000)
    pc = symbols[name] | 1
    steps = irqs = 0
    while True:
        u.emu_start(pc, 0xF0000, count=QUANTUM)
        stopped = u.reg_read(UC_ARM_REG_PC)
        if stopped == 0xF0000:
            break
        ctx = u.context_save()
        if not irq_pending():
            check(step() is not None, name, args, 'waits with nothing to serve', steps, irqs)
            steps += 1
        if irq_pending():
            isr(0)
            irqs += 1
        u.context_restore(ctx)
        pc = stopped | 1
        check(steps + irqs < 2000, name, args, 'does not end')
    return signed(u.reg_read(UC_ARM_REG_R0)), steps, irqs

rx_value = lambda i: (0x40 + i) & 0xFF
rx_done = [0]
blocks_seen = []

def twim_step(error=False):
    """The TWIM ends its oldest block, or the STOP asked for."""
    if pending:
        kind, ptr, n = pending.pop(0)
        sh = shorts.pop(0)
        blocks_seen.append((kind, ptr, n, sh))
        if error:
            put(base + (o[12] if kind == 'rx' else o[13]) + 8, 0)
            errorsrc[0] = 2     # address NACK
            put(base + o[18], 1)
            return True
        if kind == 'rx':
            u.mem_write(ptr, bytes(rx_value(rx_done[0] + i) for i in range(n)))
            rx_done[0] += n
        put(base + (o[12] if kind == 'rx' else o[13]) + 8, n)
        if kind == 'rx':
            put(base + o[10], 1)                            # LASTRX
            put(base + o[9], 1)                             # STOPPED, LASTRX_STOP short
        else:
            put(base + o[11], 1)                            # LASTTX
            put(base + (o[17] if sh == SH_TX_SUSPEND else o[9]), 1)
        return True
    if stop_req[0]:
        stop_req[0] = False
        put(base + o[9], 1)
        return True
    return None

def twim_init():
    pending.clear(); shorts.clear(); actions.clear(); blocks_seen.clear()
    stop_req[0] = False; errorsrc[0] = 0; rx_done[0] = 0
    check(call('init', 0, 0, 1, 1) == 1, 'i2c init')
    check(inten[0] == TWIM_INTEN, 'TWIM interrupt mask', hex(inten[0]), hex(TWIM_INTEN))

active = 0
# Transmit and receive from the caller's buffer, one DMA block after the other.
# Each block suspends the bus (transmit) or ends with STOP (receive), as polled.
for kind, length in [('tx', 4), ('rx', 20), ('rx', TWIM_MAX + 7), ('tx', TWIM_MAX), ('tx', TWIM_MAX + 5)]:
    twim_init()
    u.mem_write(BUF, bytes((0x30 + i) & 0xFF for i in range(length)))
    r, steps, irqs = run_served(kind, (0, length), twim_step)
    check(r == length, 'i2c', kind, length, 'returns', r)
    done = 0
    for i, n in enumerate(sequence(kind, length, TWIM_MAX)):
        check(blocks_seen[i] == (kind, BUF + done, n, SH_RX_STOP if kind == 'rx' else SH_TX_SUSPEND),
              'i2c block', kind, length, i, blocks_seen[i])
        done += n
    check(len(blocks_seen) == i + 1, blocks_seen)
    # a transmit then waits for the STOP it asks for, a receive is stopped already
    check(steps == len(blocks_seen) + (kind == 'tx') and irqs >= steps, 'i2c steps', kind, length, steps, irqs)
    if kind == 'rx':
        check(bytes(u.mem_read(BUF, length)) == bytes(rx_value(i) for i in range(length)), 'i2c rx data')
    check(call('busy', 0) == 0 and call('tx_ready', 0) == 1, 'i2c end', kind, length)
    check(call('events', 0) == 0, 'i2c master reports an event', call('events', 0))

# Register read: the command suspends the bus, then the receive
twim_init()
u.mem_write(BUF, b'\x00' * 8)
r, steps, irqs = run_served('read_reg', (0, 8), twim_step)
check(r == 8, 'i2c read returns', r)
check([(k, n, sh) for k, p, n, sh in blocks_seen] == [('tx', 1, SH_TX_SUSPEND), ('rx', 8, SH_RX_STOP)],
      'i2c read blocks', blocks_seen)
check(blocks_seen[1][1] == BUF and bytes(u.mem_read(BUF, 8)) == bytes(rx_value(i) for i in range(8)), 'i2c read data')
check(steps == 2 and call('busy', 0) == 0, 'i2c read end', steps)

# Combined write: sent from the buffer DeviceIntrfWrite() builds on its stack
twim_init()
u.mem_write(BUF, bytes(range(0x50, 0x56)))
sent_data = []
def write_step():
    if pending:
        sent_data.append(bytes(u.mem_read(pending[0][1], pending[0][2])))
    return twim_step()
r, steps, irqs = run_served('write_data', (0, 6), write_step)
check(r == 5, 'combined write returns', r)
check(sent_data == [bytes(range(0x50, 0x56))] and blocks_seen[0][1] != BUF, 'combined write data', sent_data)
check(call('busy', 0) == 0, 'combined write end')

# Register read longer than one DMA block
twim_init()
r, steps, irqs = run_served('read_reg', (0, TWIM_MAX + 3), twim_step)
check(r == TWIM_MAX + 3 and [n for k, p, n, sh in blocks_seen] == [1, TWIM_MAX, 3], 'long read', r, blocks_seen)
check(blocks_seen[2][1] == BUF + TWIM_MAX and call('busy', 0) == 0, 'long read end')

# The wait takes an event only through its interrupt: with the interrupt held
# off, as from a handler of higher priority, the transfer ends on the time
# limit although the event is in the register.
twim_init()
irq_blocked[0] = True
r, steps, irqs = run_served('tx', (0, 4), lambda: twim_step() or 'idle')
irq_blocked[0] = False
check(r == 0 and irqs == 0 and call('busy', 0) == 0, 'i2c event without its interrupt', r, steps)
pending.clear(); shorts.clear()
for e in EVENTS:
    put(base + e, 0)
twim_init()
r, steps, irqs = run_served('tx', (0, 2), twim_step)
check(r == 2, 'i2c transfer after the time limit', r)

# Address NACK: the wait stops the bus, DeviceIntrfTx retries, then returns 0.
# The bus can be used again.
twim_init()
r, steps, irqs = run_served('tx', (0, 4), lambda: twim_step(error=bool(pending)))
check(r == 0 and len(blocks_seen) == 6 and call('busy', 0) == 0, 'i2c nack', r, len(blocks_seen))
errorsrc[0] = 0
r, steps, irqs = run_served('tx', (0, 2), twim_step)
check(r == 2 and call('busy', 0) == 0, 'i2c transfer after a nack', r)
active = None

# ---- TWI master without EasyDMA, interrupt
if T[0]:
    legacy = 'twi_int'
    def twi_step():
        if not twi_queue:
            return None
        ev = twi_queue.pop(0)
        if ev == 'rxdready':
            put(base + T[9], rx_value(twi['n']))
            twi['n'] += 1
        if ev == 'error':
            errorsrc[0] = 4     # data NACK
        put(base + {'rxdready': T[6], 'txdsent': T[7], 'stopped': T[5], 'error': T[8]}[ev], 1)
        return ev
    def twi_init():
        actions.clear(); sent.clear(); twi_queue.clear(); errorsrc[0] = 0
        twi.update(rx=False, stop=False, n=0)
        check(call('init', 0, 0, 0, 1) == 1, 'twi init')
        check(inten[0] == T[15], 'TWI interrupt mask', hex(inten[0]))
        actions.clear()

    for length in (1, 3):
        twi_init()
        u.mem_write(BUF, bytes(range(0x10, 0x10 + length)))
        r, steps, irqs = run_served('tx', (0, length), twi_step)
        check(r == length and sent == list(range(0x10, 0x10 + length)), 'twi tx', length, r, sent)
        check(steps == length + 1 and irqs >= steps, 'twi tx steps', length, steps, irqs)
        check(call('busy', 0) == 0 and call('tx_ready', 0) == 1, 'twi tx end')

    for length in (1, 2, 4):
        twi_init()
        u.mem_write(BUF, b'\x00' * length)
        r, steps, irqs = run_served('rx', (0, length), twi_step)
        check(r == length and bytes(u.mem_read(BUF, length)) == bytes(rx_value(i) for i in range(length)),
              'twi rx', length, r)
        check(steps == length + 1 and irqs >= steps and call('busy', 0) == 0, 'twi rx end', length, steps, irqs)

    # Register read: the command keeps the bus, then the receive
    twi_init()
    r, steps, irqs = run_served('read_reg', (0, 2), twi_step)
    check(r == 2 and sent == [4] and actions[:2] == ['starttx', 'startrx'], 'twi read', r, sent, actions)
    check(bytes(u.mem_read(BUF, 2)) == bytes(rx_value(i) for i in range(2)) and call('busy', 0) == 0, 'twi read end')

    # A refused byte ends the transmit with the bytes sent before it
    twi_init()
    nack_at = 1
    r, steps, irqs = run_served('tx', (0, 4), twi_step)
    nack_at = None
    check(r == 1 and len(sent) == 2 and call('busy', 0) == 0, 'twi nack', r, sent)

    base = BASE[0]
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
        check(call('tx', 1, length) == -1, 'spi tx')
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
        n = min(2 * DMA_MAX + 5, 66000) if bus else 4
        check(call(kind, bus, n) == n, 'polled', bus, kind)
        check(call('busy', bus) == 0 and not pending, 'polled end', bus, kind, call('busy', bus), pending)
    active = None
polled = False
print('PASS: %s master interrupt transfers: SPIM, TWIM%s' % (CHIP, ', TWI, SPI' if T[0] and S[0] else ''))
