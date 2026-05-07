/**-------------------------------------------------------------------------
@file	ws2812_blinky.c

@brief	Tier-2 GPIO bring-up diagnostic — bypasses IOPinConfig.

	Replaces the IOPinConfig/IOPinSet/IOPinClear path with direct
	register writes so we can isolate whether the issue is in main's
	reachability vs. in the IOsonata GPIO abstraction.  Toggles
	GPIO 5 only (cleanly broken-out pin on DevKitM-1, no boot-strap
	side-effects).

	If a scope on GPIO 5 shows toggling: main is reached and the
	hardware works — the previous "no toggle" was an IOPinConfig
	side-effect (something IOPinConfig writes is breaking the pad).

	If no toggle: code is hung before main (init_array, _start, or
	earlier).  Next step is to add UART output at the top to find
	exactly where.
----------------------------------------------------------------------------*/
#include <stdint.h>

#define IOMUX_GPIO5_REG     (*(volatile uint32_t *)0x60009018UL)  /* MTDI pad */
#define IOMUX_GPIO8_REG     (*(volatile uint32_t *)0x60009024UL)
#define GPIO_OUT_W1TS_REG   (*(volatile uint32_t *)0x60004008UL)
#define GPIO_OUT_W1TC_REG   (*(volatile uint32_t *)0x6000400CUL)
#define GPIO_ENABLE_W1TS    (*(volatile uint32_t *)0x60004024UL)
#define GPIO_FUNC5_OUT_SEL  (*(volatile uint32_t *)0x60004568UL)  /* base+0x554 + 5*4 */
#define GPIO_FUNC8_OUT_SEL  (*(volatile uint32_t *)0x60004574UL)

static void Delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 400000UL; i++) { __asm volatile("nop"); }
}

int main(void)
{
    /* Force IOMUX: MCU_SEL = 1 (GPIO mode), FUN_DRV = 2.  Same as IOPinConfig
     * but bypasses any IOPinConfig side effects we might be missing. */
    IOMUX_GPIO5_REG = 0x1800UL;
    IOMUX_GPIO8_REG = 0x1800UL;

    /* Force GPIO matrix: simple GPIO output (signal 128). */
    GPIO_FUNC5_OUT_SEL = 128UL;
    GPIO_FUNC8_OUT_SEL = 128UL;

    /* Enable output drivers for GPIO 5 and 8. */
    GPIO_ENABLE_W1TS = (1UL << 5) | (1UL << 8);

    /* Toggle forever. */
    while (1)
    {
        GPIO_OUT_W1TS_REG = (1UL << 5) | (1UL << 8);
        Delay();
        GPIO_OUT_W1TC_REG = (1UL << 5) | (1UL << 8);
        Delay();
    }
}
