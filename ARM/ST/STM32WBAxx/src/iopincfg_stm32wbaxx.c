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
    GPIO_TypeDef *gpio = Stm32Gpio(port);
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
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    if (gpio && (unsigned)pin < 16U)
    {
        uint32_t shift = (uint32_t)pin * 2U;
        gpio->MODER |= 3UL << shift;
        gpio->PUPDR &= ~(3UL << shift);
    }
}

/* WBA6 GPIO interrupt lines map one-to-one to EXTI0..EXTI15.
 * The pin number selects the IRQ; the EXTI port multiplexer selects GPIOx.
 * Only one GPIO port can own a given EXTI line at a time.
 */
typedef struct {
    IOPinEvtHandler_t cb;
    void *ctx;
    uint8_t port;
} WbaExtiHook_t;
static WbaExtiHook_t s_Exti[16];

static const IRQn_Type s_ExtiIrq[16] = {
    EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn,
    EXTI4_IRQn, EXTI5_IRQn, EXTI6_IRQn, EXTI7_IRQn,
    EXTI8_IRQn, EXTI9_IRQn, EXTI10_IRQn, EXTI11_IRQn,
    EXTI12_IRQn, EXTI13_IRQn, EXTI14_IRQn, EXTI15_IRQn
};

/* STM32WBA6 EXTI registers, RM0515 chapter 19. */
static volatile uint32_t *WbaExtiReg(uint32_t offset)
{
    return (volatile uint32_t *)(EXTI_BASE + offset);
}

static void WbaExtiHandler(unsigned line)
{
    uint32_t bit = 1UL << line;
    volatile uint32_t *rpr = WbaExtiReg(0x0CU);
    volatile uint32_t *fpr = WbaExtiReg(0x10U);
    uint32_t pending = (*rpr | *fpr) & bit;
    if (pending == 0U) return;
    *rpr = bit;
    *fpr = bit;
    IOPinEvtHandler_t cb = s_Exti[line].cb;
    if (cb) cb((int)line, s_Exti[line].ctx);
}

void EXTI0_IRQHandler(void) { WbaExtiHandler(0U); }
void EXTI1_IRQHandler(void) { WbaExtiHandler(1U); }
void EXTI2_IRQHandler(void) { WbaExtiHandler(2U); }
void EXTI3_IRQHandler(void) { WbaExtiHandler(3U); }
void EXTI4_IRQHandler(void) { WbaExtiHandler(4U); }
void EXTI5_IRQHandler(void) { WbaExtiHandler(5U); }
void EXTI6_IRQHandler(void) { WbaExtiHandler(6U); }
void EXTI7_IRQHandler(void) { WbaExtiHandler(7U); }
void EXTI8_IRQHandler(void) { WbaExtiHandler(8U); }
void EXTI9_IRQHandler(void) { WbaExtiHandler(9U); }
void EXTI10_IRQHandler(void) { WbaExtiHandler(10U); }
void EXTI11_IRQHandler(void) { WbaExtiHandler(11U); }
void EXTI12_IRQHandler(void) { WbaExtiHandler(12U); }
void EXTI13_IRQHandler(void) { WbaExtiHandler(13U); }
void EXTI14_IRQHandler(void) { WbaExtiHandler(14U); }
void EXTI15_IRQHandler(void) { WbaExtiHandler(15U); }

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

void IOPinSetStrength(int port, int pin, IOPINSTRENGTH strength)
{
    (void)port; (void)pin; (void)strength;
}

void IOPinSetSpeed(int port, int pin, IOPINSPEED speed)
{
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    if (!gpio || (unsigned)pin >= 16U) return;
    uint32_t shift = (uint32_t)pin * 2U;
    uint32_t v = speed == IOPINSPEED_LOW ? 0U :
                 speed == IOPINSPEED_MEDIUM ? 1U :
                 speed == IOPINSPEED_HIGH ? 2U : 3U;
    gpio->OSPEEDR = (gpio->OSPEEDR & ~(3UL << shift)) | (v << shift);
}
