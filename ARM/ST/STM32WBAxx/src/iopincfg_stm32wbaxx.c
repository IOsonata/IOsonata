extern "C" bool Stm32GpioEnableClock(int port, int pin, int op, IOPINDIR dir)
{
    (void)pin; (void)op; (void)dir;
    uint32_t mask = Stm32WbaGpioClockMask(port);
    if (!mask || !Stm32Gpio(port)) return false;
    RCC->AHB2ENR |= mask;
    (void)RCC->AHB2ENR;
    return true;
}

/* STM32WBA GPIO configuration, shared by peripheral-compatible WBA targets. */
#include "iopinctrl.h"

static uint32_t Stm32WbaGpioClockMask(int port)
{
    switch (port)
    {
        case IOPORTA: return RCC_AHB2ENR_GPIOAEN;
        case IOPORTB: return RCC_AHB2ENR_GPIOBEN;
        case IOPORTC: return RCC_AHB2ENR_GPIOCEN;
#ifdef RCC_AHB2ENR_GPIODEN
        case IOPORTD: return RCC_AHB2ENR_GPIODEN;
#endif
#ifdef RCC_AHB2ENR_GPIOEEN
        case IOPORTE: return RCC_AHB2ENR_GPIOEEN;
#endif
#ifdef RCC_AHB2ENR_GPIOFEN
        case IOPORTF: return RCC_AHB2ENR_GPIOFEN;
#endif
#ifdef RCC_AHB2ENR_GPIOGEN
        case IOPORTG: return RCC_AHB2ENR_GPIOGEN;
#endif
#ifdef RCC_AHB2ENR_GPIOHEN
        case IOPORTH: return RCC_AHB2ENR_GPIOHEN;
#endif
        default: return 0U;
    }
}

void IOPinDisableInterrupt(int intNo)
{
    if ((unsigned)intNo >= 16U) return;
    uint32_t bit = 1UL << (unsigned)intNo;
    NVIC_DisableIRQ(s_ExtiIrq[intNo]);
    *WbaExtiReg(0x80U) &= ~bit;
    *WbaExtiReg(0x00U) &= ~bit;
    *WbaExtiReg(0x04U) &= ~bit;
    *WbaExtiReg(0x0CU) = bit;
    *WbaExtiReg(0x10U) = bit;
    s_Exti[intNo].cb = 0;
    s_Exti[intNo].ctx = 0;
}

bool IOPinEnableInterrupt(int intNo, int priority, uint32_t port,
                          uint32_t pin, IOPINSENSE sense,
                          IOPinEvtHandler_t cb, void *ctx)
{
    if (pin >= 16U || intNo != (int)pin || cb == 0 ||
        Stm32Gpio((int)port) == 0 ||
        (unsigned)priority >= (1U << __NVIC_PRIO_BITS) ||
        (sense != IOPINSENSE_LOW_TRANSITION &&
         sense != IOPINSENSE_HIGH_TRANSITION &&
         sense != IOPINSENSE_TOGGLE))
    {
        return false;
    }

    uint32_t bit = 1UL << pin;
    if (s_Exti[pin].cb != 0) return false;
    NVIC_DisableIRQ(s_ExtiIrq[pin]);

    /* EXTICR1..4 at EXTI + 0x60, one 8-bit field per line. */
    volatile uint32_t *exticr = WbaExtiReg(0x60U + 4U * (pin >> 2));
    uint32_t shift = 8U * (pin & 3U);
    *exticr = (*exticr & ~(0xFFUL << shift)) |
              ((port & 0xFFU) << shift);
    *WbaExtiReg(0x80U) &= ~bit;
    *WbaExtiReg(0x00U) &= ~bit;
    *WbaExtiReg(0x04U) &= ~bit;
    *WbaExtiReg(0x0CU) = bit;
    *WbaExtiReg(0x10U) = bit;
    if (sense == IOPINSENSE_HIGH_TRANSITION || sense == IOPINSENSE_TOGGLE)
        *WbaExtiReg(0x00U) |= bit;
    if (sense == IOPINSENSE_LOW_TRANSITION || sense == IOPINSENSE_TOGGLE)
        *WbaExtiReg(0x04U) |= bit;

    s_Exti[pin].port = (uint8_t)port;
    s_Exti[pin].ctx = ctx;
    s_Exti[pin].cb = cb;
    NVIC_ClearPendingIRQ(s_ExtiIrq[pin]);
    NVIC_SetPriority(s_ExtiIrq[pin], (uint32_t)priority);
    *WbaExtiReg(0x80U) |= bit;
    NVIC_EnableIRQ(s_ExtiIrq[pin]);
    return true;
}

int IOPinAllocateInterrupt(int priority, int port, int pin,
                           IOPINSENSE sense, IOPinEvtHandler_t cb, void *ctx)
{
    return IOPinEnableInterrupt(pin, priority, (uint32_t)port,
                                (uint32_t)pin, sense, cb, ctx) ? pin : -1;
}

void IOPinSetSense(int port, int pin, IOPINSENSE sense)
{
    (void)port;
    if ((unsigned)pin >= 16U || s_Exti[pin].cb == 0) return;
    uint32_t bit = 1UL << (unsigned)pin;
    *WbaExtiReg(0x00U) &= ~bit;
    *WbaExtiReg(0x04U) &= ~bit;
    if (sense == IOPINSENSE_HIGH_TRANSITION || sense == IOPINSENSE_TOGGLE)
        *WbaExtiReg(0x00U) |= bit;
    if (sense == IOPINSENSE_LOW_TRANSITION || sense == IOPINSENSE_TOGGLE)
        *WbaExtiReg(0x04U) |= bit;
}

