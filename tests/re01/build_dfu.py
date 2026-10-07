#!/usr/bin/env python3
"""Build the real RE01 stage-0 boot and exercise its FACI target in Unicorn."""
import argparse
import hashlib
from pathlib import Path
import random
import re
import struct
import subprocess
import sys

from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from elftools.elf.elffile import ELFFile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tool-prefix', default='arm-none-eabi-')
parser.add_argument('--package', choices=['DBN', 'CFB', 'CFP'], default='CFB')
parser.add_argument('--build-only', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
out = Path(__file__).resolve().parent / 'build' / ('dfu_' + args.package)
out.mkdir(parents=True, exist_ok=True)
port = root / 'ARM/Renesas/RE01/src'
lddir = root / 'ARM/Renesas/RE01/ldscript'
board = root / 'ARM/Renesas/RE01/RE01_1500KB/exemples/UartPrbsTxTest/src'
includes = ['include', 'ARM/include', 'ARM/CMSIS/Core/Include', 'micro-ecc',
            'ARM/Renesas/RE01/include', 'ARM/Renesas/RE01/RE01_1500KB/lib/include']
flags = ['-mcpu=cortex-m0plus', '-mthumb', '-DRE01_1500KB',
         '-DRE01_1500KB_' + args.package, '-DuECC_SUPPORTS_secp256r1=1',
         '-DuECC_SUPPORTS_secp160r1=0', '-DuECC_SUPPORTS_secp192r1=0',
         '-DuECC_SUPPORTS_secp224r1=0', '-DuECC_SUPPORTS_secp256k1=0',
         '-DuECC_SUPPORT_COMPRESSED_POINT=0', '-DuECC_OPTIMIZATION_LEVEL=2',
         '-DuECC_PLATFORM=0', '-DuECC_WORD_SIZE=4', '-Os', '-g',
         '-ffunction-sections', '-fdata-sections', '-Wall', '-Wextra',
         '-Wno-unused-parameter', '-Wno-missing-field-initializers']
flags += ['-I' + str(root / p) for p in includes]


def run(cmd):
    return subprocess.check_output(cmd, text=True)


def compile_source(source, name, board_dir=board):
    cpp = source.suffix == '.cpp'
    opts = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu11']
    opts.append('-I' + str(board_dir))
    obj = out / (name + '.o')
    run([args.tool_prefix + ('g++' if cpp else 'gcc'), *flags, *opts,
         '-c', str(source), '-o', str(obj)])
    return obj


# An independent MCUboot-format fixture, signed with an ephemeral P-256 key.
# The real boot example receives that public key as any board project does.
key = ec.generate_private_key(ec.SECP256R1())
der = key.public_key().public_bytes(serialization.Encoding.DER,
                                   serialization.PublicFormat.SubjectPublicKeyInfo)
keyfile = out / 'test_key.c'
keyfile.write_text('const unsigned char ecdsa_pub_key[] = {' +
                   ','.join(str(b) for b in der) + '};\n' +
                   'const unsigned int ecdsa_pub_key_len = sizeof(ecdsa_pub_key);\n')
# Flash commands require ICLK <= 32 MHz. Keep this test configuration in
# the boot image only; Blinky supplies its own board oscillator definition.
clockfile = out / 'test_clock.c'
clockfile.write_text('#include "coredev/system_core_clock.h"\n' +
                     'McuOsc_t g_McuOsc = { {OSC_TYPE_RC, 32000000, 20}, ' +
                     '{OSC_TYPE_RC, 32768, 20}, false };\n')
payload = bytearray(random.Random(1).randbytes(8195))
struct.pack_into('<II', payload, 0, 0x20008000, 0x18101)
hdr = struct.pack('<IIHHIIBBHII', 0x96F3B83D, 0, 32, 0, len(payload), 0,
                  1, 0, 0, 0, 0)
signed = hdr + payload
signature = key.sign(signed, ec.ECDSA(hashes.SHA256()))
tlvs = b''
for tag, value in ((1, hashlib.sha256(der).digest()),
                   (0x10, hashlib.sha256(signed).digest()), (0x22, signature)):
    tlvs += struct.pack('<BBH', tag, 0, len(value)) + value
app = out / 'app_signed.bin'
app.write_bytes(signed + struct.pack('<HH', 0x6907, len(tlvs) + 4) + tlvs)

sources = sorted(p for p in port.iterdir() if p.suffix in ('.c', '.cpp'))
common = ['ARM/src/ResetEntry.c', 'ARM/src/iatomic.c', 'src/cfifo.c',
          'src/coredev/uart.cpp', 'src/coredev/timer.cpp', 'src/device_intrf.cpp',
          'src/device.cpp', 'src/crc.c', 'src/pulse_train.c',
          'src/slip_intrf.cpp', 'src/crypto/crypto_softsha256.cpp',
          'src/crypto/crypto_uecc.cpp', 'micro-ecc/uECC.c',
          'src/dfu/dfu_image.cpp', 'src/dfu/dfu_layout.cpp', 'src/dfu/dfu_writer.cpp',
          'src/dfu/dfu_boot.cpp', 'src/dfu/dfu_mgr.cpp', 'src/dfu/dfu_wire.cpp',
          'exemples/dfu/dfu_boot_main.cpp']
objects = [compile_source(p, p.stem) for p in sources]
objects += [compile_source(root / p, Path(p).stem + '_generic') for p in common]
objects.append(compile_source(keyfile, 'test_key'))
objects.append(compile_source(clockfile, 'test_clock'))
elf = out / 'DfuBoot.elf'
run([args.tool_prefix + 'g++', '-mcpu=cortex-m0plus', '-mthumb',
     '--specs=nano.specs', '--specs=nosys.specs', '-Wl,--gc-sections',
     '-L' + str(root / 'ARM/ldscript'), '-L' + str(lddir),
     '-T' + str(lddir / 'dfu_boot_re01_1500kb.ld'), *map(str, objects), '-o', str(elf)])
sections = {}
with elf.open('rb') as f:
    allocated = {s.name for s in ELFFile(f).iter_sections() if s['sh_flags'] & 2}
for line in run([args.tool_prefix + 'objdump', '-h', str(elf)]).splitlines():
    m = re.match(r'\s*\d+\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)', line)
    if m:
        sections[m[1]] = tuple(int(v, 16) for v in m.groups()[1:])
assert sections['.ivector'][1] == 0 and sections['.ivector'][0] <= 0x400
assert sections['.osm'][:2] == (0x40, 0x400)
assert sections['.Version'][1] >= 0x440
assert sections['.data'][1] >= 0x20000400
for name, (size, addr, lma) in sections.items():
    if name not in allocated:
        continue
    if size and 0 <= lma < 0x180000:
        assert lma + size <= 0x10000, (name, lma, size)
        if name not in ('.ivector', '.osm'):
            assert lma >= 0x440, (name, lma)
osm = out / 'option_bytes.bin'
run([args.tool_prefix + 'objcopy', '--only-section=.osm', '-O', 'binary', str(elf), str(osm)])
assert osm.read_bytes() == b'\xff' * 64
symbols = {}
for line in run([args.tool_prefix + 'nm', '-n', str(elf)]).splitlines():
    fields = line.split()
    if len(fields) == 3:
        symbols[fields[2]] = int(fields[0], 16)
assert symbols['__dfu_state_unit'] == 256
assert symbols['__StackTop'] == symbols['__dfu_flag'] == 0x2003FFF0
for name, addr in symbols.items():
    if 'DfuTgtRam' in name or 'DfuTgtPe' in name or 'DfuTgtRecover' in name:
        assert 0x20000400 <= addr < 0x2003FFF0, (name, addr)
print(elf.name + ': link, option memory, DFU layout and RAM routines passed', flush=True)
appobj = compile_source(root / 'exemples/misc/blinky.c', 'Blinky',
                        root / 'ARM/Renesas/RE01/RE01_1500KB/exemples/Blinky/src')
appelf = out / 'BlinkyDfu.elf'
appobjects = [obj for obj in objects
              if obj.stem not in ('dfu_boot_main_generic', 'test_key', 'test_clock')]
run([args.tool_prefix + 'g++', '-mcpu=cortex-m0plus', '-mthumb',
     '--specs=nano.specs', '--specs=nosys.specs', '-Wl,--gc-sections',
     '-L' + str(root / 'ARM/ldscript'), '-L' + str(lddir),
     '-T' + str(lddir / 'gcc_re01_1500kb_dfu.ld'), *map(str, appobjects),
     str(appobj), '-o', str(appelf)])
with appelf.open('rb') as f:
    image = ELFFile(f)
    assert image.get_section_by_name('.ivector')['sh_addr'] == 0x18000
    assert image.get_section_by_name('.osm') is None
    for seg in image.iter_segments():
        if seg['p_type'] == 'PT_LOAD' and seg['p_filesz'] and seg['p_paddr'] < 0x180000:
            assert 0x18000 <= seg['p_paddr']
            assert seg['p_paddr'] + seg['p_filesz'] <= 0x180000
    appsym = {s.name: s['st_value'] for s in image.get_section_by_name('.symtab').iter_symbols()}
    assert appsym['__StackTop'] == 0x2003FFF0
    assert appsym['__dfu_state_unit'] == 256
print(appelf.name + ': application slot and reserved RAM passed', flush=True)
if not args.build_only:
    for scenario in ('empty', 'start', 'tamper', 'legacy', 'write', 'fail', 'checks',
                     'dbfull', 'timeout', 'exitfail', 'stopfail'):
        subprocess.run([sys.executable, str(root / 'tests/dfu/boot_emu_renesas.py'),
                        're01', str(elf), str(app), scenario], check=True)
