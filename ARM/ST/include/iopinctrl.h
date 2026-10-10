/* Common STM32 GPIO fast access. Uses ST CMSIS GPIO_TypeDef. */
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__
#include <stdint.h>
#include "stm32.h"
#include "coredev/iopincfg.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline GPIO_TypeDef *Stm32Gpio(int port)
{
    switch (port) {
#ifdef GPIOA
    case IOPORTA: return GPIOA;
#endif
#ifdef GPIOB
    case IOPORTB: return GPIOB;
#endif
#ifdef GPIOC
    case IOPORTC: return GPIOC;
#endif
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
#ifdef GPIOI
    case IOPORTI: return GPIOI;
#endif
#ifdef GPIOJ
    case IOPORTJ: return GPIOJ;
#endif
    default: return (GPIO_TypeDef *)0;
    }
}

static inline void IOPinSetDir(int port, int pin, IOPINDIR dir)
{
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    if (!gpio || (unsigned)pin >= 16U) return;
    uint32_t shift = (unsigned)pin * 2U;
    gpio->MODER = (gpio->MODER & ~(3UL << shift)) |
                  ((dir == IOPINDIR_OUTPUT ? 1UL : 0UL) << shift);
}
static inline int IOPinRead(int port, int pin)
{
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    return gpio && (unsigned)pin < 16U ?
           (int)((gpio->IDR >> (unsigned)pin) & 1UL) : 0;
}
static inline void IOPinSet(int port, int pin)
{
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    if (gpio && (unsigned)pin < 16U) gpio->BSRR = 1UL << (unsigned)pin;
}
static inline void IOPinClear(int port, int pin)
{
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    if (gpio && (unsigned)pin < 16U) gpio->BSRR = 1UL << ((unsigned)pin + 16U);
}
static inline void IOPinToggle(int port, int pin)
{
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    if (gpio && (unsigned)pin < 16U) {
        uint32_t mask = 1UL << (unsigned)pin;
        gpio->BSRR = (gpio->ODR & mask) ? (mask << 16U) : mask;
    }
}
static inline uint32_t IOPinReadPort(int port)
{
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    return gpio ? (gpio->IDR & 0xFFFFU) : 0U;
}
static inline void IOPinWritePort(int port, uint32_t data)
{
    GPIO_TypeDef *gpio = Stm32Gpio(port);
    if (gpio) gpio->ODR = data & 0xFFFFU;
}
#ifdef __cplusplus
}
#endif
#endif
