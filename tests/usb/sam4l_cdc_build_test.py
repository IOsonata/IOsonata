#!/usr/bin/env python3
"""Cross-build the SAM4L CDC examples' source closure with Arm GNU.

This is a compile/link check, not a replacement for the IOcomposer library
project or a hardware test. TaktOS is an optional sibling checkout; its
standard M4 Debug/Release build uses the base AAPCS (softfp) ABI.
"""
from pathlib import Path
import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--config', choices=['Debug', 'Release'], default='Release')
parser.add_argument('--toolchain-prefix', default='arm-none-eabi-')
parser.add_argument('--taktos', type=Path)
parser.add_argument('--build-dir', type=Path, default=root / 'tests/usb/build/sam4l')
args = parser.parse_args()
mode = args.config
trace_flags = ['-DUSB_DEBUG_TRACE'] if mode == 'Debug' else []
takt = args.taktos.resolve() if args.taktos else None
out = args.build_dir.resolve() / mode
out.mkdir(parents=True, exist_ok=True)
cc = args.toolchain_prefix
base=['-mcpu=cortex-m4','-mthumb','-mfloat-abi=soft',('-Os' if mode=='Release' else '-O0'),'-g','-ffunction-sections','-fdata-sections','-fsigned-char','-D__PROGRAM_START','-D__SAM4LC8C__',('-DNDEBUG' if mode=='Release' else '-DDEBUG')]
inc=['-I'+str(root/p) for p in ['include','ARM/include','ARM/Microchip/SAM4L/include','ARM/CMSIS/Core/Include']]
if takt:
 inc+=['-I'+str(takt/p) for p in ['include','ARM/include']]
def run(args):
 p=subprocess.run(args,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
 if p.stdout:print(p.stdout)
 if p.returncode:sys.exit(p.returncode)
def compile(p,targetdir=out,extra=[]):
 o=targetdir/(p.name+'.o')
 flags=['-std=gnu++23','-fno-rtti','-fno-exceptions'] if p.suffix=='.cpp' else ['-std=gnu17'] if p.suffix=='.c' else ['-x','assembler-with-cpp']
 run([cc+('g++' if p.suffix=='.cpp' else 'gcc'),*base,*flags,*extra,*inc,'-c',str(p),'-o',str(o)])
 return str(o)
files=['src/CppRuntimeOverload.cpp','src/syslog.cpp','ARM/src/ResetEntry.c','ARM/Microchip/SAM4L/src/vectors_sam4l.c','ARM/Microchip/SAM4L/src/system_sam4l.c','ARM/Microchip/SAM4L/src/iopincfg_sam4l.c','ARM/Microchip/SAM4L/src/usb_ctrlr_sam4l.cpp','src/cfifo.c','src/device_intrf.cpp','src/device.cpp','src/app_evt_handler.cpp','src/app_run.cpp','src/usb/usb.cpp','src/usb/usb_intrf.cpp','src/usb/usbd_epalloc.cpp','src/usb/usbd_cdc.cpp','src/usb/usbd_cdc_desc.cpp','src/usb/usbd_hid.cpp','src/usb/usb_int.cpp','src/usb/usbd_bulk.cpp','src/usb/usbd_msc.cpp']
objs=[compile(root/p, extra=trace_flags) for p in files]
run([cc+'ar','rcs',str(out/'libIOsonata_SAM4LCxC.a'),*objs])
def check_project(name):
 project = root / 'ARM/Microchip/SAM4L/SAM4LCxC/exemples' / name / 'ioc'
 for link in ET.parse(project / '.project').findall('.//link'):
  uri = link.findtext('locationURI')
  if not uri.startswith('PARENT-'):
   continue
  prefix, relative = uri.split('-PROJECT_LOC/', 1)
  target = project
  for _ in range(int(prefix.split('-')[1])):
   target = target.parent
  if not (target / relative).is_file():
   raise RuntimeError('Missing project source: ' + str(target / relative))
 for option in ET.parse(project / '.cproject').findall('.//option'):
  if option.get('superClass', '').endswith('cpp.linker.scriptfile'):
   for entry in option:
    target = (project / mode / entry.get('value').strip('"')).resolve()
    if not target.is_file():
     raise RuntimeError('Missing linker script: ' + str(target))

examples = [(False, 'UsbCdcLoopback')]
if takt:
 examples.append((True, 'UsbCdcLoopbackTaktOS'))
for task,name in examples:
 check_project(name)
 app=compile(root/('exemples/usb/usb_cdc_loopback_taktos.cpp' if task else 'exemples/usb/usb_cdc_loopback.cpp'),extra=['-I'+str(root/'ARM/Microchip/SAM4L/SAM4LCxC/exemples'/name/'src'),*(trace_flags if not task else [])])
 libs=[]
 if task:
  td=out/'taktos';td.mkdir(exist_ok=True)
  objs=[compile(p,td,['-DTAKT_ARCH_CM4','-mfloat-abi=softfp','-mfpu=fpv4-sp-d16']) for p in sorted((takt/'src').glob('taktos*.cpp'))]
  objs+=[compile(takt/'ARM/src/TaktKernelCM.cpp',td,['-DTAKT_ARCH_CM4','-mfloat-abi=softfp','-mfpu=fpv4-sp-d16']),compile(takt/'ARM/cm4/PendSV_M4.S',td,['-DTAKT_ARCH_CM4','-mfloat-abi=softfp','-mfpu=fpv4-sp-d16'])]
  run([cc+'ar','rcs',str(out/'libTaktOS_M4.a'),*objs]);libs=['-lTaktOS_M4']
 elf=out/(name+'.elf')
 run([cc+'g++',*base,'--specs=nano.specs',('--specs=rdimon.specs' if mode=='Debug' and not task else '--specs=nosys.specs'),'-Wl,--gc-sections','-Wl,-Map='+str(out/(name+'.map')),'-L'+str(root/'ARM/ldscript'),'-T'+str(root/'ARM/Microchip/SAM4L/ldscript/gcc_sam4lx8.ld'),app,'-L'+str(out),*libs,'-lIOsonata_SAM4LCxC','-Wl,--start-group','-lgcc','-lc','-lm','-Wl,--end-group','-o',str(elf)])
 run([cc+'size',str(elf)])
