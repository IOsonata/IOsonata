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

void IOPinConfig(int port, int pin, int op, IOPINDIR dir,
                 IOPINRES resistor, IOPINTYPE type)
{
    GPIO_TypeDef *gpio = Stm32WbaGpio(port);
    uint32_t clock = Stm32WbaGpioClockMask(port);
    if (!gpio || !clock || (unsigned)pin >= 16U)
    {
        return;
    }

    RCC->AHB2ENR |= clock;
    (void)RCC->AHB2ENR;

    uint32_t pinMask = 1UL << (unsigned)pin;
    uint32_t shift = (uint32_t)pin * 2U;
    uint32_t mode = 0U;

    if (op == IOPINOP_GPIO)
    {
        mode = dir == IOPINDIR_OUTPUT ? 1U : 0U;
    }
    else if (op >= IOPINOP_FUNC0 && op <= IOPINOP_FUNC15)
    {
        mode = 2U;
        uint32_t afShift = ((uint32_t)pin & 7U) * 4U;
        uint32_t afrIndex = (uint32_t)pin >> 3;
        uint32_t af = (uint32_t)(op - IOPINOP_FUNC0);
        gpio->AFR[afrIndex] = (gpio->AFR[afrIndex] & ~(15UL << afShift)) |
                              (af << afShift);
    }
    else
    {
        mode = 3U;  // Analog
    }

    uint32_t pull = 0U;
    if (resistor == IOPINRES_PULLUP || resistor == IOPINRES_FOLLOW)
    {
        pull = 1U;
    }
    else if (resistor == IOPINRES_PULLDOWN)
    {
        pull = 2U;
    }

    gpio->OTYPER = type == IOPINTYPE_OPENDRAIN ?
                   (gpio->OTYPER | pinMask) : (gpio->OTYPER & ~pinMask);
    gpio->PUPDR = (gpio->PUPDR & ~(3UL << shift)) | (pull << shift);
    gpio->MODER = (gpio->MODER & ~(3UL << shift)) | (mode << shift);
}

void IOPinDisable(int port, int pin)
{
    GPIO_TypeDef *gpio = Stm32WbaGpio(port);
    if (gpio && (unsigned)pin < 16U)
    {
        uint32_t shift = (uint32_t)pin * 2U;
        gpio->MODER |= 3UL << shift;
        gpio->PUPDR &= ~(3UL << shift);
    }
}

/* EXTI is not enabled until the routing/dispatch implementation is added. */
void IOPinDisableInterrupt(int intNo) { (void)intNo; }

bool IOPinEnableInterrupt(int intNo, int priority, uint32_t port,
                          uint32_t pin, IOPINSENSE sense,
                          IOPinEvtHandler_t cb, void *ctx)
{
    (void)intNo; (void)priority; (void)port; (void)pin;
    (void)sense; (void)cb; (void)ctx;
    return false;
}

int IOPinAllocateInterrupt(int priority, int port, int pin,
                           IOPINSENSE sense, IOPinEvtHandler_t cb, void *ctx)
{
    (void)priority; (void)port; (void)pin; (void)sense; (void)cb; (void)ctx;
    return -1;
}

void IOPinSetSense(int port, int pin, IOPINSENSE sense)
{
    (void)port; (void)pin; (void)sense;
}

void IOPinSetStrength(int port, int pin, IOPINSTRENGTH strength)
{
    (void)port; (void)pin; (void)strength;
}

void IOPinSetSpeed(int port, int pin, IOPINSPEED speed)
{
    GPIO_TypeDef *gpio = Stm32WbaGpio(port);
    if (!gpio || (unsigned)pin >= 16U) return;
    uint32_t shift = (uint32_t)pin * 2U;
    uint32_t v = speed == IOPINSPEED_LOW ? 0U :
                 speed == IOPINSPEED_MEDIUM ? 1U :
                 speed == IOPINSPEED_HIGH ? 2U : 3U;
    gpio->OSPEEDR = (gpio->OSPEEDR & ~(3UL << shift)) | (v << shift);
}
