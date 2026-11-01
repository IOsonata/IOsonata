#!/usr/bin/env python3
"""Check real PRBS/Video service registration with 32-bit pointers and the 2 KB default.

Requires Linux x86 GCC headers, g++ with -m32 code generation, and Python
unicorn/pyelftools. No 32-bit libc is linked. This checks storage, not radio
behavior or an Arm firmware build.
"""
from pathlib import Path
import struct
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_X86, UC_MODE_32, UC_HOOK_INTR
from unicorn.x86_const import UC_X86_REG_EAX, UC_X86_REG_EBX, UC_X86_REG_ESP
from test_prbs_tx_services import service_source as prbs_source
from test_video_ble_services import service_source as video_source

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent


def execute(binary):
    machine = Uc(UC_ARCH_X86, UC_MODE_32)
    result = []
    with binary.open('rb') as stream:
        elf = ELFFile(stream)
        entry = elf.header.e_entry
        address = elf.get_section_by_name('.symtab').get_symbol_by_name('g_ServiceStorage')[0]['st_value']
        for segment in elf.iter_segments():
            if segment.header.p_type != 'PT_LOAD' or not segment.header.p_memsz:
                continue
            start = segment.header.p_vaddr
            base = start & ~4095
            size = ((start + segment.header.p_memsz + 4095) & ~4095) - base
            machine.mem_map(base, size)
            machine.mem_write(start, segment.data())
    machine.mem_map(0x70000000, 0x100000)
    machine.reg_write(UC_X86_REG_ESP, 0x700ffff0)

    def interrupt(cpu, number, _):
        assert number == 128 and cpu.reg_read(UC_X86_REG_EAX) == 1
        result.append(cpu.reg_read(UC_X86_REG_EBX))
        cpu.emu_stop()

    machine.hook_add(UC_HOOK_INTR, interrupt)
    machine.emu_start(entry, 0, count=10000000)
    assert result == [0], ('service registration failed', result)
    values = struct.unpack('<4I', machine.mem_read(address, 16))
    count, used, tail, pool = values
    assert count == 38 and tail == 28 and pool == 2048
    assert used + tail <= pool
    return values


with tempfile.TemporaryDirectory(prefix='att-services32-') as tmp:
    temp = Path(tmp)
    # These sources use only declarations from libc; the test runtime supplies
    # the functions. An empty ABI stub avoids requiring 32-bit libc packages.
    (temp / 'gnu').mkdir()
    (temp / 'gnu/stubs-32.h').touch()
    triple = subprocess.check_output(['g++', '-dumpmachine'], text=True).strip()
    version = subprocess.check_output(['g++', '-dumpversion'], text=True).strip().split('.')[0]
    flags = ['-U_FORTIFY_SOURCE', '-D_FORTIFY_SOURCE=0', '-m32', '-std=gnu++20', '-I' + str(temp),
             '-I/usr/include/' + triple + '/c++/' + version,
             '-I/usr/include/' + triple, '-I' + str(ROOT / 'include'),
             '-ffunction-sections', '-fdata-sections', '-fno-exceptions',
             '-fno-rtti', '-fno-builtin', '-fno-stack-protector',
             '-fno-pie', '-no-pie', '-nostdlib', '-Wl,--gc-sections']
    sources = [ROOT / 'src/bluetooth' / name for name in
               ('bt_gap.cpp', 'bt_gatt.cpp', 'bt_att.cpp', 'bt_uuid.cpp', 'services/bt_dis.cpp')]
    for name, source in [('PRBS', prbs_source()), ('VideoBLE', video_source())]:
        cpp = temp / (name + '.cpp')
        binary = temp / name
        cpp.write_text(source + '\nstatic_assert(sizeof(void *) == 4);\n'
                       'static_assert(offsetof(BtAttDBEntry_t, Data) == 24);\n')
        for opt in ('-O0', '-O2'):
            subprocess.run(['g++', *flags, opt, str(cpp),
                            str(HERE / 'att_services_32_runtime.cpp'),
                            *map(str, sources), '-o', str(binary)], check=True)
            print(name, opt, 'attributes/used/tail/pool:', execute(binary))
