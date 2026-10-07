#!/usr/bin/env python3
"""Run RA4M1 model and ELF layout checks. Needs Clang, LLD, and Python 3.

Reduced test headers are not a production CMSIS package. The link probe is not
an alternative to IOsonata ResetEntry. No hardware execution is performed.
"""
from pathlib import Path
import os
import shutil
import struct
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
TARGET = ROOT / 'ARM/Renesas/RA4M1'
BUILD = HERE / 'build'


def run(args, *, expect=True):
    p = subprocess.run([str(a) for a in args], cwd=ROOT, text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if expect != (p.returncode == 0):
        raise RuntimeError(f'Unexpected exit {p.returncode}: {args}\n{p.stdout}')
    return p.stdout


def elf(path):
    data = path.read_bytes()
    assert data[:7] == b'\x7fELF\x01\x01\x01', 'ELF32 little-endian required'
    header = struct.unpack_from('<HHIIIIIHHHHHH', data, 16)
    assert header[1] == 40, 'Not an ARM ELF'
    shoff, entsize, count, strindex = header[5], header[10], header[11], header[12]
    headers = [struct.unpack_from('<IIIIIIIIII', data, shoff + i * entsize)
               for i in range(count)]
    s = headers[strindex]
    strings = data[s[4]:s[4]+s[5]]
    def cstr(tab, off):
        return tab[off:tab.index(b'\0', off)].decode()
    sections = {cstr(strings, h[0]): h for h in headers}
    syms = {}
    sym = sections['.symtab']
    string_header = headers[sym[6]]
    names = data[string_header[4]:string_header[4]+string_header[5]]
    for off in range(sym[4], sym[4]+sym[5], sym[9]):
        name, value, size, info, other, idx = struct.unpack_from('<IIIBBH', data, off)
        if name:
            syms[cstr(names, name)] = value
    return data, sections, syms


def check_image(path, stack=0x800, heap=0x400):
    data, sections, symbols = elf(path)
    vec = sections['.ivector']
    assert vec[3] == 0 and vec[5] == 192 and vec[8] == 256
    words = struct.unpack_from('<48I', data, vec[4])
    assert words[0] == 0x20008000 and words[1] == symbols['ResetEntry']
    assert words[1] & 1
    for i, name in {2:'NMI_Handler',3:'HardFault_Handler',4:'MemManage_Handler',
                    5:'BusFault_Handler',6:'UsageFault_Handler',11:'SVC_Handler',
                    12:'DebugMon_Handler',14:'PendSV_Handler',15:'SysTick_Handler'}.items():
        assert words[i] == symbols[name] and words[i] & 1
    assert all(words[i] == 0 for i in (7,8,9,10,13))
    for i in range(32):
        assert words[i+16] == symbols[f'IEL{i}_IRQHandler'] and words[i+16] & 1
    assert words[27] != symbols['DEF_IRQHandler'], 'Strong IRQ11 override lost'
    opts = sections['.option_setting']
    assert opts[3] == 0x400 and opts[5] == 0x3C
    assert struct.unpack_from('<15I', data, opts[4]) == (0xFFFFFFFF,0xFFFF8EFF)+(0xFFFFFFFF,)*13
    for name in ('.Version','.AppStart','.text','.rodata','.ARM.exidx'):
        assert sections[name][3] >= 0x440
    assert 0x20000000 <= symbols['__data_start__'] < 0x20008000
    assert symbols['__data_loc__'] < 0x40000
    assert symbols['__data_size__'] == sections['.data'][5]
    assert symbols['__bss_size__'] == sections['.bss'][5]
    assert 0x20000000 <= symbols['probe_ram'] < 0x20008000
    assert symbols['__StackLimit'] == 0x20008000-stack
    assert symbols['__HeapLimit']-symbols['__HeapBase'] == heap
    assert symbols['__heap_size__'] == heap
    assert symbols['__HeapLimit'] <= symbols['__StackLimit']
    print('PASS: ELF vectors, options, RAM code, ResetEntry symbols, heap/stack:', path.name)


def main():
    clang = os.environ.get('CLANG') or shutil.which('clang')
    linker = os.environ.get('LD_LLD') or shutil.which('ld.lld')
    if not clang or not linker:
        raise SystemExit('Install Clang and LLD, or set CLANG and LD_LLD.')
    BUILD.mkdir(exist_ok=True)
    includes = ['-I'+str(HERE/'include'), '-I'+str(TARGET/'include')]
    for opt in ('-O0','-Os','-O2'):
        out = BUILD/('register_model'+opt)
        run([clang,'-std=c11',opt,'-Wall','-Wextra','-Werror',*includes,
             HERE/'register_model.c','-o',out])
        log = run([out])
        (BUILD/(out.name+'.log')).write_text(log)
        print(opt,log.splitlines()[-1])
    out = BUILD/'register_model_sanitized'
    run([clang,'-std=c11','-O1','-g','-fsanitize=address,undefined',
         '-fno-omit-frame-pointer',*includes,HERE/'register_model.c','-o',out])
    log = run([out]); (BUILD/'sanitizers.log').write_text(log)
    print('ASan/UBSan:',log.splitlines()[-1])
    objects = []
    flags = ['--target=arm-none-eabi','-mcpu=cortex-m4','-mthumb',
             '-mfpu=fpv4-sp-d16','-std=c11','-ffreestanding','-fno-builtin',
             '-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror']
    for abi in ('soft','hard'):
        for opt in ('-O0','-Os','-O2'):
            for name in ('system_ra4m1','vectors_ra4m1'):
                run([clang,*flags,'-mfloat-abi='+abi,opt,*includes,'-c',
                     TARGET/'src'/(name+'.c'),'-o',BUILD/(name+'_'+abi+opt+'.o')])
        print('PASS: ARM C11 compile, ABI',abi,'at O0/Os/O2 (test headers)')
    for name in ('system_ra4m1','vectors_ra4m1'):
        objects.append(BUILD/(name+'_soft-Os.o'))
    probe = BUILD/'probe.o'
    run([clang,*flags,'-mfloat-abi=soft','-Os',*includes,'-c',HERE/'link_probe.c','-o',probe])
    objects.append(probe)
    base=[linker,'-T',TARGET/'ldscript/gcc_ra4m1.ld','--gc-sections',*objects]
    image=BUILD/'link_probe.elf'
    run([*base,'-Map='+str(BUILD/'link_probe.map'),'-o',image]); check_image(image)
    image=BUILD/'custom_memory.elf'
    run([*base,'--defsym=__STACK_SIZE=4096','--defsym=__HEAP_SIZE=2048','-o',image])
    check_image(image,4096,2048)
    text=run([*base,'--defsym=__STACK_SIZE=32768','--defsym=__HEAP_SIZE=32768',
              '-o',BUILD/'must_not_link.elf'],expect=False)
    assert 'overlap' in text or 'overflow' in text
    print('PASS: impossible RAM reservation rejected by linker')
    run([linker,'-T',TARGET/'ldscript/gcc_ra4m1.ld',objects[0],objects[2],
         '-o',BUILD/'must_not_link.elf'],expect=False)
    print('PASS: missing vector/option object rejected by linker')
    print('Validation completed. Full CMSIS/newlib build and hardware boot NOT tested.')

if __name__ == '__main__':
    main()
