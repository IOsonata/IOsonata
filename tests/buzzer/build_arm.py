#!/usr/bin/env python3
"""Build the nRF52840 buzzer example and PWM register fixture."""
import argparse
from pathlib import Path
import subprocess

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--tool-prefix', default='arm-none-eabi-')
p.add_argument('--mdk', type=Path, required=True, help='Nordic MDK directory containing nrf.h')
p.add_argument('--cmsis', type=Path, required=True)
a=p.parse_args()
root=Path(__file__).resolve().parents[2]
includes=[root/'include',root/'ARM/include',root/'ARM/Nordic/include',
          root/'ARM/Nordic/nRF52/nRF52840/lib/include',
          root/'ARM/Nordic/nRF52/nRF52840/exemples/BuzzerDemo/src',a.cmsis,a.mdk,
          *[x for x in a.mdk.rglob('*') if x.is_dir()]]
common=['src/audio/buzzer.cpp','src/coredev/timer.cpp',
        'ARM/Nordic/src/pwm_nrfx.cpp','ARM/Nordic/src/iopincfg_nrfx.c',
        'ARM/Nordic/src/timer_nrfx.cpp','ARM/Nordic/src/timer_lf_nrfx.cpp',
        'ARM/Nordic/src/timer_hf_nrfx.cpp','ARM/Nordic/nRF52/src/system_nrf52.c',
        'ARM/src/ResetEntry.c','ARM/Nordic/nRF52/nRF52840/lib/src/Vectors_nRF52840.c']
for opt,name in [('-O0','Debug'),('-Os','Release')]:
 out=root/'tests/buzzer/build'/name
 out.mkdir(parents=True,exist_ok=True)
 flags=['-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard',
        '-DNRF52840_XXAA',opt,'-ffunction-sections','-fdata-sections']
 flags+=['-I'+str(x) for x in includes]
 def compile_file(source):
  obj=out/(Path(source).stem+'.o')
  cpp=source.endswith('.cpp')
  subprocess.run([a.tool_prefix+('g++' if cpp else 'gcc'),*flags,
    *(['-std=gnu++23','-fno-rtti','-fno-exceptions'] if cpp else ['-std=gnu17']),
    '-c',str(root/source),'-o',str(obj)],check=True)
  return str(obj)
 objects=[compile_file(s) for s in common]
 for source in ['exemples/audio/buzzer_demo.cpp','tests/buzzer/pwm_nrf52840_test.cpp']:
  obj=compile_file(source)
  elf=out/(Path(source).stem+'.elf')
  subprocess.run([a.tool_prefix+'g++',*flags,'--specs=nano.specs','--specs=nosys.specs',
   '-Wl,--gc-sections','-L'+str(root/'ARM/ldscript'),
   '-T'+str(root/'ARM/Nordic/nRF52/nRF52840/ldscript/gcc_nrf52840_xxaa.ld'),
   *objects,obj,'-o',str(elf)],check=True)
  subprocess.run([a.tool_prefix+'size',str(elf)],check=True)
