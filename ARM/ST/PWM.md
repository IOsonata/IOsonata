# STM32 PWM

`ARM/ST/src/pwm_stm32.cpp` implements the generic PWM API of
`include/coredev/pwm.h` (`PWMInit` and the `Pwm` class) on the output
channels of the STM32 TIM. One source serves every family of the shared
timer driver; the differences are keyed on the CMSIS device header (32 bit
counter, break function, center aligned mode, OCxM bit 3).

## Device and channels

`PwmCfg_t::DevNo` is the timer DevNo of `ARM/ST/include/timer_stm32.h`: the
LPTIM first, then the TIM from the basic timers to the advanced timers. The
timer driver gives the TIM of a DevNo (`Stm32TimHwGet`). The LPTIM and the
basic timers TIM6 and TIM7 have no output and are refused, as are a TIM bound
by `TimerInit` and a TIM counting for other code.

| Library | DevNo and TIM with outputs |
|---|---|
| STM32F030x8 | 1 TIM14, 2 TIM16, 3 TIM17, 4 TIM15, 5 TIM3, 6 TIM1 |
| STM32F401xC | 0 TIM10, 1 TIM11, 2 TIM9, 3 TIM2, 4 TIM3, 5 TIM4, 6 TIM5, 7 TIM1 |
| STM32L152RE | 2 TIM10, 3 TIM11, 4 TIM9, 5 TIM2, 6 TIM3, 7 TIM4, 8 TIM5 |
| STM32L432 | 4 TIM16, 5 TIM15, 6 TIM2, 7 TIM1 |
| STM32L476, L496, L4S9 | 4 TIM16, 5 TIM17, 6 TIM15, 7 TIM2, 8 TIM3, 9 TIM4, 10 TIM5, 11 TIM1, 12 TIM8 |
| STM32G474 | 3 TIM16, 4 TIM17, 5 TIM15, 6 TIM2, 7 TIM3, 8 TIM4, 9 TIM5, 10 TIM1, 11 TIM8, 12 TIM20 |
| STM32H753 | 2 TIM13, 3 TIM14, 4 TIM16, 5 TIM17, 6 TIM12, 7 TIM15, 8 TIM2, 9 TIM3, 10 TIM4, 11 TIM5, 12 TIM1, 13 TIM8 |
| STM32WBA65 | 2 TIM16, 3 TIM17, 4 TIM2, 5 TIM3, 6 TIM4, 7 TIM1 |

`PwmChanCfg_t::Chan` 0 to 3 is the output CH1 to CH4 of the TIM, as many as
the TIM has (1, 2 or 4). `PwmChanCfg_t::PinOp` is the alternate function of
the pin for that output, `IOPINOP_FUNC0` to `IOPINOP_FUNC15` for AF0 to AF15,
from the datasheet of the part. The complementary outputs (CHxN) are not
supported. MCUs that route any pin to their PWM (Nordic) ignore PinOp.

## Operation

The counter runs from Init or Enable to Disable. An open channel drives its
pin at its inactive level, and PWM mode 1 while started: active while the
counter is below the compare value, the polarity from CCxP. Edge mode counts
up, center mode up then down (center aligned mode 1, on TIM1 to TIM5, TIM8
and TIM20). The prescaler is the smallest that fits the period in the 16 or
32 bit counter, for the best duty resolution, and the period count is the
closest to the frequency asked; `PwmDev_t::Freq` gives the frequency
obtained. The duty cycle is a percentage of the period, 0 keeps the output
inactive and 100 active. New compare values take effect at the next period;
a new frequency at once. On the TIM with a break function (TIM1, TIM8,
TIM15 to TIM17, TIM20) Init sets MOE.

Closing a channel returns its pin to GPIO output at the inactive level, then
turns the TIM output off. Init again closes the open channels. Init of
another `PwmDev_t` on the DevNo takes the TIM over; Init of a `PwmDev_t` on
another DevNo releases its previous TIM (channels closed, counter stopped,
clock off). Not supported, Init or Start fail: interrupt and events
(`bIntEn`), Start with a duration.

## Libraries, examples and tests

The F030x8, F401xC, L152RE, L432, L476, L496, L4S9, G474, H753 and WBA65
library projects build the driver. Example: `exemples/pwm/pwm_demo.cpp`,
with `PWM_DEVNO` and `PWM_CHAN_CFG` in `board.h`, in
`STM32L4xx/STM32L476/exemples/PwmDemo` (B-L475E-IOT01A, TIM2 on LD1 and
Arduino D0) and `STM32G4xx/STM32G474/exemples/PwmDemo` (NUCLEO-G474RE, TIM2
on LD2 and Arduino A1).

`tests/stm32/run_pwm.py` builds the driver with the TIM driver for F030x8,
F072xB, F401xC, F446xx, L152xE, L432, L476, L4S9, G474, G431, H753 and WBA65
at -O0 and -Os, and runs it in Unicorn against a model of the TIM registers:
clock and reset, register widths, the registers each TIM has, and the pin
level of each channel computed from the registers. The firmware checks the
period and the high time of the outputs against the frequency and duty cycle
asked. The L476 and G474 PwmDemo Release images, run in Unicorn up to the
first delay with the RCC ready flags modeled, leave TIM2 at 1 kHz (ARR 79999
from 80 MHz on the L476, 169999 from 170 MHz on the G474), CCR1 and CCR2 at 50
and 25 percent in PWM mode 1, and PA5 and PA1 in alternate function 1. Not
run on hardware yet.
