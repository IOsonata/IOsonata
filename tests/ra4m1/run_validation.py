#!/usr/bin/env python3
"""Run RA4M1 model and ELF layout checks. Needs Clang/Clang++, LLD, llvm-ar, and Python 3.

Reduced test headers are not a production CMSIS package. The link probe is not
an alternative to IOsonata ResetEntry. No hardware execution is performed.
"""
from pathlib import Path
import os
import shutil
import struct
import subprocess
import json
import re
import xml.etree.ElementTree as ET

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
    assert words[27] != symbols.get('Ra4m1DefaultIEL11', 0), 'Strong IRQ11 override lost'
    assert words[16] == symbols['Ra4m1DefaultIEL0'], 'IRQ dispatcher not extracted'
    assert 'IOPinConfig' in symbols and 'IOPinSetDir' in symbols
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


def check_ioc():
    project = TARGET/'lib/ioc'
    xml = ET.parse(project/'.project').getroot()
    assert xml.findtext('name') == 'IOsonata_RA4M1'
    source_names = set()
    for link in xml.findall('linkedResources/link'):
        name, uri = link.findtext('name'), link.findtext('locationURI')
        assert 'tests/ra4m1' not in uri
        if uri == 'virtual:/virtual': continue
        match = re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.+)', uri)
        assert match, uri
        path = project.parents[int(match[1])-1]/match[2]
        # The probe checkout need not carry the rest of the production tree.
        # RA4M1 links are all required to exist locally; shared paths were
        # read from the live repository and retain the existing project links.
        if 'RA4M1' in path.parts: assert path.is_file() or path.is_dir(), path
        if name.endswith(('.c', '.cpp')): source_names.add(name)
    assert source_names == {'src/ResetEntry.c', 'src/system_ra4m1.c',
                            'src/vectors_ra4m1.c', 'src/iopincfg_ra4m1.c',
                            'src/interrupt_ra4m1.cpp', 'src/uart_ra4m1.cpp',
                            'src/timer_ra4m1.cpp', 'src/timer.cpp',
                            'src/cfifo.c', 'src/device_intrf.cpp', 'src/uart.cpp'}
    cxml = ET.parse(project/'.cproject').getroot()
    configs = cxml.findall('.//cconfiguration')
    assert len(configs) == 2
    names = set()
    for cfg in configs:
        config = cfg.find("storageModule[@moduleId='cdtBuildSystem']/configuration")
        names.add(config.attrib['name'])
        assert config.attrib['artifactExtension'] == 'a'
        assert config.attrib['artifactName'] == '${ProjName}'
        options = {o.attrib.get('superClass'):o for o in cfg.findall('.//option')}
        prefix = 'ilg.gnuarmeclipse.managedbuild.cross.option.'
        assert options[prefix+'arm.target.family'].attrib['value'].endswith('cortex-m4')
        assert options[prefix+'arm.target.fpu.abi'].attrib['value'].endswith('.hard')
        assert options[prefix+'arm.target.fpu.unit'].attrib['value'].endswith('.fpv4spd16')
        for language in ('c', 'cpp'):
            paths = options[prefix+language+'.compiler.include.paths'].findall('listOptionValue')
            assert all('tests/ra4m1' not in v.attrib['value'] for v in paths)
        assert not any(e.attrib.get('excluding') for e in cfg.findall('.//sourceEntries/entry'))
    assert names == {'Debug','Release'}
    wizard = json.loads((project/'wizard.json').read_text())
    assert wizard['lib'] == 'IOsonata_RA4M1' and wizard['mcu'] == 'RA4M1'
    assert wizard['lib_path'] == '${iosonata}/ARM/Renesas/RA4M1/lib/ioc/${config}'
    print('PASS: IOC XML, eleven linked sources, target links, hard-float configurations and wizard metadata')


def main():
    clang = os.environ.get('CLANG') or shutil.which('clang')
    linker = os.environ.get('LD_LLD') or shutil.which('ld.lld')
    clangxx = os.environ.get('CLANGXX') or shutil.which('clang++')
    ar = os.environ.get('LLVM_AR') or shutil.which('llvm-ar')
    if not all((clang, clangxx, linker, ar)):
        raise SystemExit('Install Clang/Clang++, LLD and llvm-ar, or set CLANG, CLANGXX, LD_LLD, LLVM_AR.')
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
    run([clang,'-std=c11','-O1','-g','-fsanitize=address,undefined', '-fno-sanitize-recover=all',
         '-fno-omit-frame-pointer',*includes,HERE/'register_model.c','-o',out])
    log = run([out]); (BUILD/'sanitizers.log').write_text(log)
    print('ASan/UBSan:',log.splitlines()[-1])
    for opt in ('-O0', '-Os', '-O2'):
        out = BUILD / ('io_model' + opt)
        run([clangxx, '-std=c++11', opt, '-Wall', '-Wextra', '-Werror',
             *includes, HERE/'io_model.cpp', '-o', out])
        log = run([out]); (BUILD/(out.name+'.log')).write_text(log)
        print(opt, log.strip())
    out = BUILD/'io_model_sanitized'
    run([clangxx, '-std=c++11', '-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
         '-fno-omit-frame-pointer', *includes, HERE/'io_model.cpp', '-o', out])
    log = run([out]); (BUILD/'io_sanitizers.log').write_text(log)
    print('ASan/UBSan:', log.strip())
    for package in (40, 48, 64, 100):
        out = BUILD/('package_'+str(package))
        run([clang, '-std=c17', '-DRA4M1_PACKAGE_PINS='+str(package),
             *includes, HERE/'package_probe.c', '-o', out])
        print(run([out]).strip())
    run([clang, '-std=c17', '-DRA4M1_PACKAGE_PINS=32', *includes,
         '-c', HERE/'package_probe.c', '-o', BUILD/'bad_package.o'], expect=False)
    print('PASS: unsupported package definition rejected')
    flags = ['--target=arm-none-eabi','-mcpu=cortex-m4','-mthumb',
             '-mfpu=fpv4-sp-d16','-std=c17','-ffreestanding','-fno-builtin',
             '-ffunction-sections','-fdata-sections','-Wall','-Wextra','-Werror']
    for abi in ('soft','hard'):
        for opt in ('-O0','-Os','-O2'):
            for name in ('system_ra4m1','vectors_ra4m1','iopincfg_ra4m1'):
                run([clang,*flags,'-mfloat-abi='+abi,opt,*includes,'-c',
                     TARGET/'src'/(name+'.c'),'-o',BUILD/(name+'_'+abi+opt+'.o')])
            cppflags = [f if f != '-std=c17' else '-std=c++11' for f in flags]
            run([clangxx, *cppflags, '-fno-exceptions', '-fno-rtti',
                 '-mfloat-abi='+abi, opt, *includes, '-c',
                 TARGET/'src/interrupt_ra4m1.cpp', '-o', BUILD/('interrupt_ra4m1_'+abi+opt+'.o')])
        print('PASS: ARM C17/C++11 compile, ABI',abi,'at O0/Os/O2 (test headers)')
    objects = [BUILD/(name+'_soft-Os.o') for name in
               ('system_ra4m1','vectors_ra4m1','iopincfg_ra4m1','interrupt_ra4m1')]
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
    run([linker,'-T',TARGET/'ldscript/gcc_ra4m1.ld',objects[0],objects[2],objects[3],probe,
         '-o',BUILD/'must_not_link.elf'],expect=False)
    print('PASS: missing vector/option object rejected by linker')
    archive = BUILD/'libIOsonata_RA4M1_probe.a'
    if archive.exists(): archive.unlink()
    run([ar, 'rcs', archive, *objects[:-1]])
    image = BUILD/'archive_probe.elf'
    run([linker, '-T', TARGET/'ldscript/gcc_ra4m1.ld', '--gc-sections',
         probe, archive, '-Map='+str(BUILD/'archive_probe.map'), '-o', image])
    check_image(image)
    print('PASS: static archive extraction without whole-archive; strong IEL override retained')
    check_ioc()
    print('Validation completed. Full CMSIS/newlib build, IOC import and hardware boot NOT tested.')

if __name__ == '__main__':
    main()
