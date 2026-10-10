/* Common STM32 GPIO configuration using ST CMSIS GPIO_TypeDef.
 * GPIO clock/IO supply activation is supplied by the MCU family.
 * EXTI routing and dispatch remain family-specific.
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
