#!/usr/bin/env python3
"""Run the nRF52840 PWM register checks (requires unicorn and pyelftools)."""
import sys
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

for path in sys.argv[1:]:
 for report_stop in (True, False):
  uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
  for address,size in [(0,0x200000),(0x20000000,0x40000),(0x40000000,0x100000),(0x50000000,0x100000),(0xe000e000,0x2000)]:
   uc.mem_map(address,size)
  with open(path,'rb') as file:
   elf=ELFFile(file)
   for segment in elf.iter_segments():
    if segment['p_type']=='PT_LOAD': uc.mem_write(segment['p_vaddr'],segment.data())
   symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
  # Model STOP completion. The second run exercises the bounded-timeout fallback.
  def write_hook(cpu,access,address,size,value,context):
   if report_stop and address==0x4001c004 and value==1:
    cpu.mem_write(0x4001c104,(1).to_bytes(4,'little'))
  uc.hook_add(UC_HOOK_MEM_WRITE,write_hook)
  end=0x1fff00
  uc.reg_write(UC_ARM_REG_SP,0x2003f000)
  uc.reg_write(UC_ARM_REG_LR,end|1)
  uc.emu_start(symbols['main']|1,end,count=10000000)
  assert uc.reg_read(UC_ARM_REG_PC)==end, 'firmware did not return'
  result=uc.reg_read(UC_ARM_REG_R0)
  assert result==0, f'{path}: assertion at fixture line {result}'
  print(path+': PWM idle polarity, stop/disable, restart and close passed; STOP event='+str(report_stop))
