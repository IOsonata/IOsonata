#!/usr/bin/env python3
"""Cortex-M0 compilation and static-archive link checks (no hardware execution)."""
from pathlib import Path
import subprocess, os, tempfile
root=Path(__file__).resolve().parents[2]
work=tempfile.TemporaryDirectory(prefix='f030-arm-');out=Path(work.name)
cc=os.environ.get('ARM_TOOLCHAIN_PREFIX','arm-none-eabi-')
inc=['-I'+str(root/'ARM/ST/STM32F0xx/include'),'-I'+str(root/'include'),'-I'+str(root/'ARM/include'),'-I'+str(root/'ARM/CMSIS/Core/Include')]
for opt in ['-O0','-Os']:
 objs=[]
 for name in ['ARM/ST/STM32F0xx/src/timer_stm32f030x8.cpp','ARM/ST/STM32F0xx/STM32F030x8/lib/src/Vectors_STM32F030x8.c']:
  cpp=name.endswith('cpp');obj=out/(Path(name).stem+opt+'.o');objs.append(str(obj))
  subprocess.run([cc+('g++' if cpp else 'gcc'),'-std='+('gnu++17' if cpp else 'gnu11'),'-mcpu=cortex-m0','-mthumb','-DSTM32F030x8',opt,'-ffunction-sections','-fdata-sections',*(['-fno-exceptions','-fno-rtti'] if cpp else []),*inc,'-c',str(root/name),'-o',str(obj)],check=True)
 archive=out/('libtimer'+opt+'.a');subprocess.run([cc+'ar','rcs',str(archive),*objs],check=True)
 for variant in ['timer','benchmark','systick']:
  src=out/'main.cpp';src.write_text('''#include "coredev/timer.h"
extern "C" {uint32_t SystemPeriphClockGet(int) {return 48000000;}
void SysTick_Handler() {}
'''+('void TIM17_IRQHandler() {}\n' if variant=='benchmark' else '')+'''void ResetEntry() {
'''+('TimerDev_t t{};TimerCfg_t cfg{5,TIMER_CLKSRC_DEFAULT,1000000,2,nullptr,false};TimerInit(&t,&cfg);\n' if variant!='systick' else '')+'''while(1){} }
}
''')
  exe=out/(variant+opt+'.elf')
  subprocess.run([cc+'g++','-mcpu=cortex-m0','-mthumb',opt,'-fno-exceptions','-fno-rtti','-ffunction-sections',*inc,'-nostartfiles',str(src),'-Wl,-u,__Vectors','-Wl,-e,ResetEntry','-Wl,--defsym=__StackTop=0x20002000','-Wl,--gc-sections',str(archive),'-Wl,--start-group','-lc','-lgcc','-lnosys','-Wl,--end-group','-o',str(exe)],check=True)
  symbols=subprocess.check_output([cc+'nm',str(exe)],text=True)
  assert (' T TimerInit' in symbols)==(variant!='systick')
  if variant=='benchmark':assert ' T TIM17_IRQHandler' in symbols
  assert ' T SysTick_Handler' in symbols
  print('PASS Cortex-M0 archive link',opt,variant)
