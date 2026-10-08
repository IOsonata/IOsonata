/* Example pin-map checks against the native MCU package masks, not wiring.
 * Copyright (c) 2026 I-SYST inc. MIT License.
 */
#include <cassert>
#include <cstdio>
#include "example_api.h"
#include "ra4m1_ioregs.h"
#include "board.h"
int main()
{
#ifdef LED_PINS_MAP
    static_assert(LED0_PINOP == IOPINOP_GPIO && LED1_PINOP == IOPINOP_GPIO &&
                  LED2_PINOP == IOPINOP_GPIO, "named LED pin operations");
    const IOPinCfg_t leds[] = LED_PINS_MAP;
    assert(leds[0].PortNo == LED0_PORT && leds[0].PinNo == LED0_PIN);
    assert(leds[1].PortNo == LED1_PORT && leds[1].PinNo == LED1_PIN);
    assert(leds[2].PortNo == LED2_PORT && leds[2].PinNo == LED2_PIN);
    static_assert(sizeof(leds)/sizeof(leds[0]) == 3, "three example outputs");
    for (const auto &pin : leds) {
        assert(Ra4m1PinValid(pin.PortNo, pin.PinNo));
        assert(pin.PinOp == IOPINOP_GPIO && pin.PinDir == IOPINDIR_OUTPUT);
    }
#endif
#ifdef BUTTON_PINS_MAP
    const IOPinCfg_t buttons[] = BUTTON_PINS_MAP;
    assert(buttons[0].PortNo == BUT1_PORT && buttons[0].PinNo == BUT1_PIN &&
           buttons[0].PinOp == BUT1_PINOP);
    assert(Ra4m1PinValid(buttons[0].PortNo, buttons[0].PinNo));
    assert(buttons[0].PortNo == 0 && buttons[0].PinNo == 0 && BUT1_INT == 6);
    assert(buttons[0].PinDir == IOPINDIR_INPUT && buttons[0].Res == IOPINRES_PULLUP);
#endif
#ifdef UART_PINS
    const IOPinCfg_t pins[] = UART_PINS;
    assert(pins[0].PortNo == UART_RX_PORT && pins[0].PinNo == UART_RX_PIN &&
           pins[0].PinOp == UART_RX_PINOP);
    assert(pins[1].PortNo == UART_TX_PORT && pins[1].PinNo == UART_TX_PIN &&
           pins[1].PinOp == UART_TX_PINOP);
    static_assert(sizeof(pins)/sizeof(pins[0]) == 2, "RX and TX only");
    for (const auto &pin : pins) assert(Ra4m1PinValid(pin.PortNo, pin.PinNo));
    assert(pins[0].PortNo == 1 && pins[0].PinNo == 0 && pins[0].PinOp == 4);
    assert(pins[1].PortNo == 1 && pins[1].PinNo == 1 && pins[1].PinOp == 4);
    assert(UART_NO == 0 && UART_DEVNO == 0 && !UART_DMA_MODE);
#endif
    printf("PASS: example pin map, %d-pin package\n", RA4M1_PACKAGE_PINS);
}
