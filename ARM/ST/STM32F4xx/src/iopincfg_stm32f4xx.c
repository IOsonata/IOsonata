/* STM32F401 GPIO clock and EXTI integration. GPIO registers use ST CMSIS. */
#include <stdbool.h>
#include <stdint.h>
#include "iopinctrl.h"

bool Stm32GpioEnableClock(int port, int pin, int op, IOPINDIR dir)
{
    (void)pin; (void)op; (void)dir;
    uint32_t mask = 0U;
    switch (port) {
#ifdef RCC_AHB1ENR_GPIOAEN
    case IOPORTA: mask = RCC_AHB1ENR_GPIOAEN; break;
#endif
#ifdef RCC_AHB1ENR_GPIOBEN
    case IOPORTB: mask = RCC_AHB1ENR_GPIOBEN; break;
#endif
#ifdef RCC_AHB1ENR_GPIOCEN
    case IOPORTC: mask = RCC_AHB1ENR_GPIOCEN; break;
#endif
#ifdef RCC_AHB1ENR_GPIODEN
    case IOPORTD: mask = RCC_AHB1ENR_GPIODEN; break;
#endif
#ifdef RCC_AHB1ENR_GPIOEEN
    case IOPORTE: mask = RCC_AHB1ENR_GPIOEEN; break;
#endif
#ifdef RCC_AHB1ENR_GPIOHEN
    case IOPORTH: mask = RCC_AHB1ENR_GPIOHEN; break;
#endif
    default: return false;
    }
    if (!Stm32Gpio(port)) return false;
    RCC->AHB1ENR |= mask;
    (void)RCC->AHB1ENR;
    return true;
}

typedef struct { IOPinEvtHandler_t cb; void *ctx; uint8_t port; } Stm32F401ExtiHook;
static Stm32F401ExtiHook s_Exti[16];

static IRQn_Type Stm32F401ExtiIrq(unsigned pin)
{
    static const IRQn_Type irqs[16] = {
        EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn,
        EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
        EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
        EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn
    };
    return irqs[pin];
}
static void Stm32F401ExtiDispatch(unsigned first, unsigned last)
{
    uint32_t pending = EXTI->PR & EXTI->IMR;
    for (unsigned pin = first; pin <= last; ++pin) {
        uint32_t bit = 1UL << pin;
        if (!(pending & bit)) continue;
        EXTI->PR = bit;
        if (s_Exti[pin].cb) s_Exti[pin].cb((int)pin, s_Exti[pin].ctx);
    }
}
void EXTI0_IRQHandler(void) { Stm32F401ExtiDispatch(0, 0); }
void EXTI1_IRQHandler(void) { Stm32F401ExtiDispatch(1, 1); }
void EXTI2_IRQHandler(void) { Stm32F401ExtiDispatch(2, 2); }
void EXTI3_IRQHandler(void) { Stm32F401ExtiDispatch(3, 3); }
void EXTI4_IRQHandler(void) { Stm32F401ExtiDispatch(4, 4); }
void EXTI9_5_IRQHandler(void) { Stm32F401ExtiDispatch(5, 9); }
void EXTI15_10_IRQHandler(void) { Stm32F401ExtiDispatch(10, 15); }

void IOPinDisableInterrupt(int intNo)
{
    if ((unsigned)intNo >= 16U) return;
    unsigned pin = (unsigned)intNo;
    uint32_t bit = 1UL << pin;
    EXTI->IMR &= ~bit;
    EXTI->RTSR &= ~bit;
    EXTI->FTSR &= ~bit;
    EXTI->PR = bit;
    s_Exti[pin].cb = 0;
    s_Exti[pin].ctx = 0;
    uint32_t group = pin < 5U ? pin : (pin < 10U ? 5U : 10U);
    unsigned last = group == 5U ? 9U : group == 10U ? 15U : group;
    bool used = false;
    for (unsigned i = group; i <= last; ++i) used |= s_Exti[i].cb != 0;
    if (!used) NVIC_DisableIRQ(Stm32F401ExtiIrq(pin));
}

bool IOPinEnableInterrupt(int intNo, int prio, uint32_t port, uint32_t pin,
                          IOPINSENSE sense, IOPinEvtHandler_t cb, void *ctx)
{
    if (pin >= 16U || intNo != (int)pin || !Stm32Gpio((int)port) ||
        !cb || (unsigned)prio >= (1UL << __NVIC_PRIO_BITS) ||
        (sense != IOPINSENSE_LOW_TRANSITION &&
         sense != IOPINSENSE_HIGH_TRANSITION && sense != IOPINSENSE_TOGGLE) ||
        s_Exti[pin].cb) return false;
    uint32_t bit = 1UL << pin;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;
    unsigned index = pin >> 2, shift = (pin & 3U) * 4U;
    EXTI->IMR &= ~bit;
    SYSCFG->EXTICR[index] = (SYSCFG->EXTICR[index] & ~(15UL << shift)) |
                            ((port & 15U) << shift);
    EXTI->RTSR = (EXTI->RTSR & ~bit) |
                 ((sense != IOPINSENSE_LOW_TRANSITION) ? bit : 0U);
    EXTI->FTSR = (EXTI->FTSR & ~bit) |
                 ((sense != IOPINSENSE_HIGH_TRANSITION) ? bit : 0U);
    EXTI->PR = bit;
    s_Exti[pin].port = (uint8_t)port;
    s_Exti[pin].ctx = ctx;
    s_Exti[pin].cb = cb;
    IRQn_Type irq = Stm32F401ExtiIrq(pin);
    NVIC_SetPriority(irq, (uint32_t)prio);
    NVIC_ClearPendingIRQ(irq);
    EXTI->IMR |= bit;
    NVIC_EnableIRQ(irq);
    return true;
}
int IOPinAllocateInterrupt(int prio, int port, int pin, IOPINSENSE sense,
                           IOPinEvtHandler_t cb, void *ctx)
{
    return IOPinEnableInterrupt(pin, prio, (uint32_t)port, (uint32_t)pin,
                                sense, cb, ctx) ? pin : -1;
}
void IOPinSetSense(int port, int pin, IOPINSENSE sense)
{
    if ((unsigned)pin >= 16U || s_Exti[pin].cb == 0 ||
        s_Exti[pin].port != (uint8_t)port) return;
    uint32_t bit = 1UL << (unsigned)pin;
    EXTI->RTSR = (EXTI->RTSR & ~bit) |
        ((sense == IOPINSENSE_HIGH_TRANSITION || sense == IOPINSENSE_TOGGLE) ? bit : 0U);
    EXTI->FTSR = (EXTI->FTSR & ~bit) |
        ((sense == IOPINSENSE_LOW_TRANSITION || sense == IOPINSENSE_TOGGLE) ? bit : 0U);
}
