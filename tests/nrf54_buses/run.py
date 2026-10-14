#!/usr/bin/env python3
"""Execute the real Arm bus drivers against a small DMA register model.

Usage: run.py fixture.elf [5|7]
Requires unicorn and pyelftools. Pin configuration is stubbed; this does not
model electrical signalling, bus timing, pin routing or hardware errata.
"""
import struct
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0

u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
for base, size in [(0, 0x100000), (0x20000000, 0x100000), (0x50000000, 0x200000), (0xE0000000, 0x100000)]:
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

def call(name, *args):
    for i, arg in enumerate(args):
        u.reg_write(UC_ARM_REG_R0 + i, arg)
    u.reg_write(UC_ARM_REG_SP, 0x200FF000)
    u.reg_write(UC_ARM_REG_LR, 0xF0001)
    u.emu_start(symbols[name] | 1, 0xF0000, count=10000000)
    assert u.reg_read(UC_ARM_REG_PC) == 0xF0000, (name, 'did not return')
    return u.reg_read(UC_ARM_REG_R0)

o = [get(symbols['offsets'] + i * 4) for i in range(17)]
bases = [0x50104000, 0x500C6000, 0x500C7000, 0x500C8000, 0x5004D000, 0x500ED000, 0x500EE000]
count = int(sys.argv[2]) if len(sys.argv) > 2 else 5
if count == 5:
    bases[4] = 0x5004A000
active_bus = None
chunks = []
actions = []

def write(uc, access, addr, size, value, data):
    if active_bus is None or value != 1:
        return
    base = bases[0]
    offset = addr - base
    if active_bus == 0:
        if offset == o[2]:
            actions.append("stop")
            put(base + o[3], 1)
        if offset in o[:2]:
            receive = offset == o[0]
            actions.append("rx" if receive else "tx")
            dma = base + o[5 if receive else 6]
            ptr, length = get(dma), get(dma + 4)
            chunks.append((ptr, length))
            put(dma + 8, length)
            put(base + o[3 if receive else 4], 1)
    elif offset == o[15]:
        actions.append("stop")
        put(base + o[16], 1)
    elif offset == o[7]:
        rx, tx = base + o[11], base + o[12]
        dma = rx if get(rx + 4) else tx
        ptr, length = get(dma), get(dma + 4)
        chunks.append((ptr, length))
        put(dma + 8, length)
        put(base + o[8], 1)
        put(base + o[9], 1)
        put(base + o[10], 1)

u.hook_add(UC_HOOK_MEM_WRITE, write)
for bus in [0, 1]:
    for dev in range(count):
        if bus == 0 and dev == 4:
            assert call('init', bus, dev, 0, 100000) == 0
            assert call('init', bus, dev, 1, 100000) == 0
            continue
        assert call('address', bus, dev) == bases[dev]
        assert call('init', bus, dev, 0, 100000) == 1
        for request in [0, 1, 100000, 175001, 240000, 700001, 1000000, 2700000, 9000000, 0xFFFFFFFF]:
            if bus == 0:
                choices = [100000, 250000, 400000, 1000000]
            else:
                clock = 128000000 if dev == 4 else 16000000
                choices = [clock // d for d in range(4 if dev == 4 else 2, 127, 2)]
            actual = call('rate', bus, request)
            assert abs(actual-request) == min(abs(c-request) for c in choices), (bus, dev, request, actual)
            if bus == 1:
                assert clock // get(bases[dev] + o[13]) == actual
        # Do not write a master clock register while operating as a slave.
        rate_register = bases[dev] + o[13 if bus else 14]
        put(rate_register, 0xA55A)
        assert call('init', bus, dev, 1, 400000) == 1
        assert get(rate_register) == 0xA55A
        assert call('slave_test', bus, dev) == 0, (bus, dev, 'slave callback')
    assert call('init', bus, count, 0, 100000) == 0
    # A final short DMA block must use the buffer after the full first block.
    for receive in [0, 1]:
        assert call('init', bus, 0, 0, 1000000) == 1
        chunks.clear()
        active_bus = bus
        assert call('transfer', bus, receive, 65540) == 65540, (bus, receive, chunks)
        active_bus = None
        assert len(chunks) == 2 and [x[1] for x in chunks] == [65535, 5], chunks
        assert chunks[1][0] == chunks[0][0] + 65535, chunks
assert call('init', 0, 0, 0, 400000) == 1
active_bus = 0
actions.clear()
assert call('register_read') == 8
active_bus = None
assert actions[:2] == ['tx', 'rx'] and actions[2:] == ['stop'], actions
assert call('init', 1, 0, 0, 1000000) == 1
active_bus = 1
actions.clear()
assert call('timeout_test') == 0
active_bus = None
assert actions == ['stop']
print('PASS: device mapping, closest rates, slave DMA/callbacks and master DMA chunking/repeated start')
