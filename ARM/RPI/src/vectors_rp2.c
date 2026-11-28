// Core 0 Arm exception and interrupt vectors.
#include <stdint.h>
#include "hardware/platform_defs.h"
#include "hardware/regs/intctrl.h"
typedef void (*Vector_t)(void);
extern uint32_t __StackTop;
extern void ResetEntry(void);
extern void RpTimerIrq(void);
extern void RpUartIrq(void);
extern void RpPinIrq(void);
extern void SysTick_Handler(void);
static void DefaultHandler(void) { for (;;) {} }

// Weak handlers let an application that only uses GPIO omit UART and timers.
#pragma weak RpTimerIrq
#pragma weak RpUartIrq
#pragma weak RpPinIrq
#pragma weak SysTick_Handler
__attribute__((section(".vectors"), used))
Vector_t const __Vectors[16 + NUM_IRQS] = {
	[0] = (Vector_t)&__StackTop, [1] = ResetEntry,
	[2 ... 14] = DefaultHandler, [15] = SysTick_Handler,
#ifdef RP2350
	[16 + TIMER0_IRQ_0 ... 16 + TIMER1_IRQ_3] = RpTimerIrq,
#else
	[16 + TIMER_IRQ_0 ... 16 + TIMER_IRQ_3] = RpTimerIrq,
#endif
	[16 + IO_IRQ_BANK0] = RpPinIrq,
	[16 + UART0_IRQ] = RpUartIrq, [16 + UART1_IRQ] = RpUartIrq,
};
