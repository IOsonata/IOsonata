/* Link-layout probe only, NOT an alternative firmware startup/runtime. */
#include <stdint.h>
#include "ra4m1xxx.h"
extern unsigned long __data_loc__, __data_start__, __data_size__, __bss_start__, __bss_size__;
uint32_t SystemMicroSecLoopCnt = 1;
volatile uint32_t probe_data = 0x12345678;
volatile uint32_t probe_bss;
__attribute__((section(".fastrun"),noinline)) uint32_t probe_ram(void) {return probe_data;}
/* Strong override must replace exactly external IRQ11's weak alias. */
void IEL11_IRQHandler(void) {probe_bss++;}
__attribute__((section(".AppStart"))) void ResetEntry(void)
{
 /* Keep all IOsonata ResetEntry linker contract symbols observable. */
 probe_bss=(uint32_t)(uintptr_t)&__data_loc__+(uint32_t)(uintptr_t)&__data_start__+
           (uint32_t)(uintptr_t)&__data_size__+(uint32_t)(uintptr_t)&__bss_start__+
           (uint32_t)(uintptr_t)&__bss_size__;
 SystemInit();
 SystemCoreClockUpdate();
 for(;;){probe_bss=probe_ram();}
}
