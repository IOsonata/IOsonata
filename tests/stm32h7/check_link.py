#!/usr/bin/env python3
"""Check H753 bare-metal ELF vectors and DMA placement after linking.

Usage: python3 tests/stm32h7/check_link.py BUILD_DIRECTORY
The directory contains libIOsonata_STM32H753.a and example .elf files.
Requires GNU Arm nm and pyelftools. Does not execute firmware on hardware.
"""
from pathlib import Path
import argparse
import subprocess
import os

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('directory', type=Path)
args = parser.parse_args()
directory = args.directory
archive = directory / 'libIOsonata_STM32H753.a'
nm = os.environ.get('ARM_NM', 'arm-none-eabi-nm')
symbols = subprocess.check_output([nm, '--defined-only', str(archive)], text=True)
assert sum(line.split()[-2:] == ['T', 'TimerInit'] for line in symbols.splitlines()) == 1, \
                'library must contain exactly one strong TimerInit implementation'
images = sorted(directory.glob('*.elf'))
assert images, 'no firmware images supplied'
import struct
from elftools.elf.elffile import ELFFile
for path in images:
    with path.open('rb') as f:
        e=ELFFile(f);syms={s.name:s for s in e.get_section_by_name('.symtab').iter_symbols()}
        def word(addr):
            for seg in e.iter_segments():
                o=addr-seg['p_vaddr']
                if seg['p_type']=='PT_LOAD' and 0<=o and o+4<=seg['p_filesz']:return struct.unpack_from('<I',seg.data(),o)[0]
            raise AssertionError('address missing')
        assert word(0x08000000)==0x20020000
        assert word(0x08000004)==syms['ResetEntry']['st_value']
        assert not any('SystickSetFrequency' in x for x in syms)
        assert e.header['e_entry']==syms['ResetEntry']['st_value']
        if path.stem=='TimerDemo': assert 'Stm32TimInit' in syms
        checks=[]
        if path.stem.startswith('Usb'):checks.append((101,'OTG_FS_IRQHandler'))
        if path.stem.startswith('Ethernet'):checks.append((61,'ETH_IRQHandler'))
        if path.stem.startswith('Uart'):checks.extend([(39,'USART3_IRQHandler'),(15,'DMA1_Stream4_IRQHandler'),(16,'DMA1_Stream5_IRQHandler')])
        for irq,name in checks:
            assert syms[name]['st_info']['bind']=='STB_GLOBAL',(path,name)
            assert word(0x08000000+4*(16+irq))==syms[name]['st_value'],(path,name)
        for name in ['.uart_dma','.eth_dma']:
            sec=e.get_section_by_name(name)
            if sec and sec['sh_size']:
                assert sec['sh_type']=='SHT_NOBITS'
                assert sec['sh_addr']%32==0
                assert 0x24000000<=sec['sh_addr']<sec['sh_addr']+sec['sh_size']<=0x24080000
    print(path.stem,'startup, vectors, timer selection, DMA placement PASS')
