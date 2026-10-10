/* Common STM32 GPIO configuration using ST CMSIS GPIO_TypeDef.
 * RCC and EXTI registers are selected using ST CMSIS device definitions.
 */
#include "iopinctrl.h"

void IOPinConfig(int port, int pin, int op, IOPINDIR dir,
                            IOPINRES resistor, IOPINTYPE type)
{
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    if (!gpio || (unsigned)pin >= 16U || !Stm32GpioEnableClock(port, pin, op, dir))
        return;
    uint32_t shift = (unsigned)pin * 2U;
    uint32_t mode = dir == IOPINDIR_OUTPUT ? 1U : 0U;
    if (op >= IOPINOP_FUNC0 && op <= IOPINOP_FUNC15) {
        mode = 2U;
        uint32_t afShift = ((unsigned)pin & 7U) * 4U;
        uint32_t af = (unsigned)(op - IOPINOP_FUNC0);
        uint32_t index = (unsigned)pin >> 3;
        gpio->AFR[index] = (gpio->AFR[index] & ~(15UL << afShift)) |
                           (af << afShift);
    } else if (op != IOPINOP_GPIO) {
        mode = 3U;
    }
    uint32_t pull = resistor == IOPINRES_PULLDOWN ? 2U :
                    (resistor == IOPINRES_PULLUP || resistor == IOPINRES_FOLLOW) ? 1U : 0U;
    uint32_t mask = 1UL << (unsigned)pin;
    gpio->OTYPER = (gpio->OTYPER & ~mask) |
                   (type == IOPINTYPE_OPENDRAIN ? mask : 0U);
    gpio->PUPDR = (gpio->PUPDR & ~(3UL << shift)) | (pull << shift);
    gpio->MODER = (gpio->MODER & ~(3UL << shift)) | (mode << shift);
}
void IOPinDisable(int port, int pin)
{
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    if (!gpio || (unsigned)pin >= 16U) return;
    uint32_t shift = (unsigned)pin * 2U;
    gpio->MODER |= 3UL << shift;
    gpio->PUPDR &= ~(3UL << shift);
}
void IOPinSetStrength(int port, int pin, IOPINSTRENGTH strength)
{
    (void)port; (void)pin; (void)strength;
}
void IOPinSetSpeed(int port, int pin, IOPINSPEED speed)
{
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    if (!gpio || (unsigned)pin >= 16U) return;
    uint32_t shift = (unsigned)pin * 2U;
    uint32_t value = speed == IOPINSPEED_LOW ? 0U :
                     speed == IOPINSPEED_MEDIUM ? 1U :
                     speed == IOPINSPEED_HIGH ? 2U : 3U;
    gpio->OSPEEDR = (gpio->OSPEEDR & ~(3UL << shift)) | (value << shift);
}


/* GPIO clock selection uses each family's CMSIS RCC definitions. */
bool Stm32GpioEnableClock(int PortNo, int PinNo, int PinOp, IOPINDIR Dir)
{
    (void)PinNo;
    (void)PinOp;
    (void)Dir;
    if (!Stm32Gpio(PortNo)) return false;

#if defined(IOSONATA_STM32_F0)
    uint32_t Mask = 0;
#ifdef RCC_AHBENR_GPIOAEN
    if (PortNo == IOPORTA) Mask = RCC_AHBENR_GPIOAEN;
#endif
#ifdef RCC_AHBENR_GPIOBEN
    if (PortNo == IOPORTB) Mask = RCC_AHBENR_GPIOBEN;
#endif
#ifdef RCC_AHBENR_GPIOCEN
    if (PortNo == IOPORTC) Mask = RCC_AHBENR_GPIOCEN;
#endif
#ifdef RCC_AHBENR_GPIODEN
    if (PortNo == IOPORTD) Mask = RCC_AHBENR_GPIODEN;
#endif
#ifdef RCC_AHBENR_GPIOFEN
    if (PortNo == IOPORTF) Mask = RCC_AHBENR_GPIOFEN;
#endif
    if (!Mask) return false;
    RCC->AHBENR |= Mask;
    (void)RCC->AHBENR;
#elif defined(IOSONATA_STM32_F4)
    uint32_t Mask = 0;
#ifdef RCC_AHB1ENR_GPIOAEN
    if (PortNo == IOPORTA) Mask = RCC_AHB1ENR_GPIOAEN;
#endif
#ifdef RCC_AHB1ENR_GPIOBEN
    if (PortNo == IOPORTB) Mask = RCC_AHB1ENR_GPIOBEN;
#endif
#ifdef RCC_AHB1ENR_GPIOCEN
    if (PortNo == IOPORTC) Mask = RCC_AHB1ENR_GPIOCEN;
#endif
#ifdef RCC_AHB1ENR_GPIODEN
    if (PortNo == IOPORTD) Mask = RCC_AHB1ENR_GPIODEN;
#endif
#ifdef RCC_AHB1ENR_GPIOEEN
    if (PortNo == IOPORTE) Mask = RCC_AHB1ENR_GPIOEEN;
#endif
#ifdef RCC_AHB1ENR_GPIOHEN
    if (PortNo == IOPORTH) Mask = RCC_AHB1ENR_GPIOHEN;
#endif
    if (!Mask) return false;
    RCC->AHB1ENR |= Mask;
    (void)RCC->AHB1ENR;
#elif defined(IOSONATA_STM32_L4)
    if (PortNo == IOPORTG) {
#if defined(RCC_APB1ENR1_PWREN) && defined(PWR_CR2_IOSV)
        RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
        PWR->CR2 |= PWR_CR2_IOSV;
#endif
    }
    RCC->AHB2ENR |= 1UL << (unsigned)PortNo;
    (void)RCC->AHB2ENR;
#elif defined(IOSONATA_STM32_WBA)
    uint32_t Mask = 0;
#ifdef RCC_AHB2ENR_GPIOAEN
    if (PortNo == IOPORTA) Mask = RCC_AHB2ENR_GPIOAEN;
#endif
#ifdef RCC_AHB2ENR_GPIOBEN
    if (PortNo == IOPORTB) Mask = RCC_AHB2ENR_GPIOBEN;
#endif
#ifdef RCC_AHB2ENR_GPIOCEN
    if (PortNo == IOPORTC) Mask = RCC_AHB2ENR_GPIOCEN;
#endif
#ifdef RCC_AHB2ENR_GPIODEN
    if (PortNo == IOPORTD) Mask = RCC_AHB2ENR_GPIODEN;
#endif
#ifdef RCC_AHB2ENR_GPIOEEN
    if (PortNo == IOPORTE) Mask = RCC_AHB2ENR_GPIOEEN;
#endif
#ifdef RCC_AHB2ENR_GPIOFEN
    if (PortNo == IOPORTF) Mask = RCC_AHB2ENR_GPIOFEN;
#endif
#ifdef RCC_AHB2ENR_GPIOGEN
    if (PortNo == IOPORTG) Mask = RCC_AHB2ENR_GPIOGEN;
#endif
#ifdef RCC_AHB2ENR_GPIOHEN
    if (PortNo == IOPORTH) Mask = RCC_AHB2ENR_GPIOHEN;
#endif
    if (!Mask) return false;
    RCC->AHB2ENR |= Mask;
    (void)RCC->AHB2ENR;
#else
    return false;
#endif
    return true;
}

typedef struct {
    IOPinEvtHandler_t Handler;
    void *pContext;
    uint8_t PortNo;
} Stm32ExtiHook_t;

static Stm32ExtiHook_t s_ExtiHook[16];

static IRQn_Type Stm32ExtiIRQ(int PinNo)
{
#if defined(IOSONATA_STM32_F0)
    if (PinNo < 2) return EXTI0_1_IRQn;
    if (PinNo < 4) return EXTI2_3_IRQn;
    return EXTI4_15_IRQn;
#elif defined(IOSONATA_STM32_WBA)
    static const IRQn_Type Irq[16] = {
        EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn,
        EXTI4_IRQn, EXTI5_IRQn, EXTI6_IRQn, EXTI7_IRQn,
        EXTI8_IRQn, EXTI9_IRQn, EXTI10_IRQn, EXTI11_IRQn,
        EXTI12_IRQn, EXTI13_IRQn, EXTI14_IRQn, EXTI15_IRQn
    };
    return Irq[PinNo];
#else
    static const IRQn_Type Irq[5] = {
        EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn, EXTI4_IRQn
    };
    return PinNo < 5 ? Irq[PinNo] :
           (PinNo < 10 ? EXTI9_5_IRQn : EXTI15_10_IRQn);
#endif
}

static unsigned Stm32ExtiGroupStart(int PinNo)
{
#if defined(IOSONATA_STM32_F0)
    return PinNo < 2 ? 0U : PinNo < 4 ? 2U : 4U;
#elif defined(IOSONATA_STM32_WBA)
    return (unsigned)PinNo;
#else
    return PinNo < 5 ? (unsigned)PinNo : PinNo < 10 ? 5U : 10U;
#endif
}

static unsigned Stm32ExtiGroupEnd(int PinNo)
{
#if defined(IOSONATA_STM32_F0)
    return PinNo < 2 ? 1U : PinNo < 4 ? 3U : 15U;
#elif defined(IOSONATA_STM32_WBA)
    return (unsigned)PinNo;
#else
    return PinNo < 5 ? (unsigned)PinNo : PinNo < 10 ? 9U : 15U;
#endif
}

static bool Stm32ExtiGroupUsed(int PinNo)
{
    for (unsigned i = Stm32ExtiGroupStart(PinNo);
         i <= Stm32ExtiGroupEnd(PinNo); ++i) {
        if (s_ExtiHook[i].Handler) return true;
    }
    return false;
}

#if defined(IOSONATA_STM32_WBA)
/* WBA6 requires verification of the EXTI register layout against the
 * installed ST CMSIS device revision before EXTI can be enabled.
 * Do not write offsets inferred from unrelated STM32 EXTI blocks.
 */
static bool Stm32ExtiAvailable(void) { return false; }
static uint32_t Stm32ExtiPending(void) { return 0U; }
static void Stm32ExtiClear(uint32_t Mask) { (void)Mask; }
static void Stm32ExtiMask(uint32_t Mask, bool Enable) { (void)Mask; (void)Enable; }
static void Stm32ExtiEdges(uint32_t Mask, IOPINSENSE Sense) { (void)Mask; (void)Sense; }
static void Stm32ExtiSelect(int PortNo, int PinNo) { (void)PortNo; (void)PinNo; }
#else
static bool Stm32ExtiAvailable(void) { return true; }
static uint32_t Stm32ExtiPending(void)
{
#if defined(IOSONATA_STM32_L4)
    return EXTI->PR1 & EXTI->IMR1;
#else
    return EXTI->PR & EXTI->IMR;
#endif
}
static void Stm32ExtiClear(uint32_t Mask)
{
#if defined(IOSONATA_STM32_L4)
    EXTI->PR1 = Mask;
#else
    EXTI->PR = Mask;
#endif
}
static void Stm32ExtiMask(uint32_t Mask, bool Enable)
{
#if defined(IOSONATA_STM32_L4)
    if (Enable) EXTI->IMR1 |= Mask;
    else EXTI->IMR1 &= ~Mask;
#else
    if (Enable) EXTI->IMR |= Mask;
    else EXTI->IMR &= ~Mask;
#endif
}
static void Stm32ExtiEdges(uint32_t Mask, IOPINSENSE Sense)
{
#if defined(IOSONATA_STM32_L4)
    EXTI->RTSR1 = (EXTI->RTSR1 & ~Mask) |
        ((Sense == IOPINSENSE_HIGH_TRANSITION || Sense == IOPINSENSE_TOGGLE) ? Mask : 0U);
    EXTI->FTSR1 = (EXTI->FTSR1 & ~Mask) |
        ((Sense == IOPINSENSE_LOW_TRANSITION || Sense == IOPINSENSE_TOGGLE) ? Mask : 0U);
#else
    EXTI->RTSR = (EXTI->RTSR & ~Mask) |
        ((Sense == IOPINSENSE_HIGH_TRANSITION || Sense == IOPINSENSE_TOGGLE) ? Mask : 0U);
    EXTI->FTSR = (EXTI->FTSR & ~Mask) |
        ((Sense == IOPINSENSE_LOW_TRANSITION || Sense == IOPINSENSE_TOGGLE) ? Mask : 0U);
#endif
}
static void Stm32ExtiSelect(int PortNo, int PinNo)
{
    uint32_t Shift = ((unsigned)PinNo & 3U) * 4U;
    uint32_t Index = (unsigned)PinNo >> 2;
    SYSCFG->EXTICR[Index] = (SYSCFG->EXTICR[Index] & ~(15UL << Shift)) |
                           (((unsigned)PortNo & 15U) << Shift);
}
#endif

static void Stm32ExtiDispatch(unsigned First, unsigned Last)
{
    uint32_t Pending = Stm32ExtiPending();
    for (unsigned i = First; i <= Last; ++i) {
        uint32_t Mask = 1UL << i;
        if ((Pending & Mask) == 0U) continue;
        Stm32ExtiClear(Mask);
        if (s_ExtiHook[i].Handler)
            s_ExtiHook[i].Handler((int)i, s_ExtiHook[i].pContext);
    }
}

void IOPinDisableInterrupt(int IntNo)
{
    if ((unsigned)IntNo >= 16U || !s_ExtiHook[IntNo].Handler) return;
    uint32_t Mask = 1UL << (unsigned)IntNo;
    IRQn_Type Irq = Stm32ExtiIRQ(IntNo);
    Stm32ExtiMask(Mask, false);
    Stm32ExtiEdges(Mask, IOPINSENSE_DISABLE);
    Stm32ExtiClear(Mask);
    s_ExtiHook[IntNo].Handler = NULL;
    s_ExtiHook[IntNo].pContext = NULL;
    if (!Stm32ExtiGroupUsed(IntNo)) NVIC_DisableIRQ(Irq);
}

bool IOPinEnableInterrupt(int IntNo, int IntPrio, uint32_t PortNo, uint32_t PinNo,
                          IOPINSENSE Sense, IOPinEvtHandler_t Handler, void *pContext)
{
    if (PinNo >= 16U || IntNo != (int)PinNo ||
        !Stm32Gpio((int)PortNo) || !Handler || !Stm32ExtiAvailable() ||
        (unsigned)IntPrio >= (1UL << __NVIC_PRIO_BITS) ||
        (Sense != IOPINSENSE_LOW_TRANSITION &&
         Sense != IOPINSENSE_HIGH_TRANSITION && Sense != IOPINSENSE_TOGGLE) ||
        s_ExtiHook[PinNo].Handler)
        return false;

#if defined(IOSONATA_STM32_F0)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
#elif defined(IOSONATA_STM32_F4) || defined(IOSONATA_STM32_L4)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
#endif
    uint32_t Mask = 1UL << PinNo;
    IRQn_Type Irq = Stm32ExtiIRQ((int)PinNo);
    Stm32ExtiMask(Mask, false);
    Stm32ExtiSelect((int)PortNo, (int)PinNo);
    Stm32ExtiEdges(Mask, Sense);
    Stm32ExtiClear(Mask);
    s_ExtiHook[PinNo].PortNo = (uint8_t)PortNo;
    s_ExtiHook[PinNo].pContext = pContext;
    s_ExtiHook[PinNo].Handler = Handler;
    NVIC_SetPriority(Irq, (uint32_t)IntPrio);
    NVIC_ClearPendingIRQ(Irq);
    Stm32ExtiMask(Mask, true);
    NVIC_EnableIRQ(Irq);
    return true;
}

int IOPinAllocateInterrupt(int IntPrio, int PortNo, int PinNo, IOPINSENSE Sense,
                           IOPinEvtHandler_t Handler, void *pContext)
{
    return IOPinEnableInterrupt(PinNo, IntPrio, (uint32_t)PortNo,
                                (uint32_t)PinNo, Sense, Handler, pContext) ? PinNo : -1;
}

void IOPinSetSense(int PortNo, int PinNo, IOPINSENSE Sense)
{
    if ((unsigned)PinNo >= 16U || !s_ExtiHook[PinNo].Handler ||
        s_ExtiHook[PinNo].PortNo != (uint8_t)PortNo) return;
    Stm32ExtiEdges(1UL << (unsigned)PinNo, Sense);
}

#if defined(IOSONATA_STM32_F0)
void EXTI0_1_IRQHandler(void) { Stm32ExtiDispatch(0, 1); }
void EXTI2_3_IRQHandler(void) { Stm32ExtiDispatch(2, 3); }
void EXTI4_15_IRQHandler(void) { Stm32ExtiDispatch(4, 15); }
#elif defined(IOSONATA_STM32_WBA)
void EXTI0_IRQHandler(void) { Stm32ExtiDispatch(0, 0); }
void EXTI1_IRQHandler(void) { Stm32ExtiDispatch(1, 1); }
void EXTI2_IRQHandler(void) { Stm32ExtiDispatch(2, 2); }
void EXTI3_IRQHandler(void) { Stm32ExtiDispatch(3, 3); }
void EXTI4_IRQHandler(void) { Stm32ExtiDispatch(4, 4); }
void EXTI5_IRQHandler(void) { Stm32ExtiDispatch(5, 5); }
void EXTI6_IRQHandler(void) { Stm32ExtiDispatch(6, 6); }
void EXTI7_IRQHandler(void) { Stm32ExtiDispatch(7, 7); }
void EXTI8_IRQHandler(void) { Stm32ExtiDispatch(8, 8); }
void EXTI9_IRQHandler(void) { Stm32ExtiDispatch(9, 9); }
void EXTI10_IRQHandler(void) { Stm32ExtiDispatch(10, 10); }
void EXTI11_IRQHandler(void) { Stm32ExtiDispatch(11, 11); }
void EXTI12_IRQHandler(void) { Stm32ExtiDispatch(12, 12); }
void EXTI13_IRQHandler(void) { Stm32ExtiDispatch(13, 13); }
void EXTI14_IRQHandler(void) { Stm32ExtiDispatch(14, 14); }
void EXTI15_IRQHandler(void) { Stm32ExtiDispatch(15, 15); }
#else
void EXTI0_IRQHandler(void) { Stm32ExtiDispatch(0, 0); }
void EXTI1_IRQHandler(void) { Stm32ExtiDispatch(1, 1); }
void EXTI2_IRQHandler(void) { Stm32ExtiDispatch(2, 2); }
void EXTI3_IRQHandler(void) { Stm32ExtiDispatch(3, 3); }
void EXTI4_IRQHandler(void) { Stm32ExtiDispatch(4, 4); }
void EXTI9_5_IRQHandler(void) { Stm32ExtiDispatch(5, 9); }
void EXTI15_10_IRQHandler(void) { Stm32ExtiDispatch(10, 15); }
#endif
