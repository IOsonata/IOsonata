/* STM32WBA GPIO access. Port numbers use IOsonata's A=0 ... H=7. */
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include "stm32wbaxx.h"
#include "coredev/iopincfg.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline GPIO_TypeDef *Stm32WbaGpio(int port)
{
    switch (port)
    {
        case IOPORTA: return GPIOA;
        case IOPORTB: return GPIOB;
        case IOPORTC: return GPIOC;
#ifdef GPIOD
        case IOPORTD: return GPIOD;
#endif
#ifdef GPIOE
        case IOPORTE: return GPIOE;
#endif
#ifdef GPIOF
        case IOPORTF: return GPIOF;
#endif
#ifdef GPIOG
        case IOPORTG: return GPIOG;
#endif
#ifdef GPIOH
        case IOPORTH: return GPIOH;
#endif
        default: return (GPIO_TypeDef *)0;
    }
}

static inline void IOPinSetDir(int port, int pin, IOPINDIR dir)
{
    GPIO_TypeDef *gpio = Stm32WbaGpio(port);
    if (!gpio || (unsigned)pin >= 16U) return;
    uint32_t shift = (uint32_t)pin * 2U;
    uint32_t mode = dir == IOPINDIR_OUTPUT ? 1U : 0U;
    gpio->MODER = (gpio->MODER & ~(3UL << shift)) | (mode << shift);
}

static inline int IOPinRead(int port, int pin)
{
    GPIO_TypeDef *gpio = Stm32WbaGpio(port);
    return gpio && (unsigned)pin < 16U ?
        (int)((gpio->IDR >> (unsigned)pin) & 1U) : 0;
}

static inline void IOPinSet(int port, int pin)
{
    GPIO_TypeDef *gpio = Stm32WbaGpio(port);
    if (gpio && (unsigned)pin < 16U) gpio->BSRR = 1UL << (unsigned)pin;
}

static inline void IOPinClear(int port, int pin)
{
    GPIO_TypeDef *gpio = Stm32WbaGpio(port);
    if (gpio && (unsigned)pin < 16U) gpio->BSRR = 1UL << ((unsigned)pin + 16U);
}

static inline void IOPinToggle(int port, int pin)
{
    GPIO_TypeDef *gpio = Stm32WbaGpio(port);
    if (gpio && (unsigned)pin < 16U)
    {
        gpio->BSRR = (gpio->ODR & (1UL << (unsigned)pin)) ?
                     (1UL << ((unsigned)pin + 16U)) :
                     (1UL << (unsigned)pin);
    }
}

static inline uint32_t IOPinReadPort(int port)
{
    GPIO_TypeDef *gpio = Stm32WbaGpio(port);
    return gpio ? gpio->IDR : 0U;
}

static inline void IOPinWritePort(int port, uint32_t data)
{
    GPIO_TypeDef *gpio = Stm32WbaGpio(port);
    if (gpio) gpio->ODR = data;
}

#ifdef __cplusplus
}
#endif
#endif
