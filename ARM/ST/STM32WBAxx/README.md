# STM32WBA MCU port - STM32WBA65 priority

IOsonata contains a standalone STM32WBA65 MCU library and an ST Bluetooth ACI
adapter. **Hardware validation is incomplete.** The bring-up target is the
STM32WBA65I-DK1 Discovery Kit with STM32WBA65RIV7 (2 MB flash / 512 KB SRAM).
The current application layout assumes TrustZone disabled (TZEN = 0); it does
not install a secure bootloader or configure secure/nonsecure partitions.
Confirm the board's option bytes before using this layout. WBA5x linker
scripts must not be used for WBA65.

The detailed build dependencies, examples and validation status are in
[`STM32WBA65/README.md`](STM32WBA65/README.md).

## Existing IOsonata sources

- `include/stm32wbaxx.h` and `include/system_stm32wbaxx.h`:
  device selection and system declarations. `include/stm32wba65xx.h` supplies
  the ST device register map and IRQ definitions.
- `src/bt_{app,adv,scan,gap,gatt,smp_bond,pds}_stm32wba.cpp` and
  `include/bt_wba_event_router.h`, `include/bt_wba_stack.h`: ST BLE middleware
  integration, compile checked against STM32CubeWBA 1.10.0, see
  `STM32WBA65/README.md`.
- `ldscript/gcc_stm32wba5xxx.ld`: *1 MB / 128 KB* WBA5x-specific memory
  layout with reserved NVM regions; not a universal WBA linker script.

## UART

The WBA65 library links the shared STM32 UART driver,
`ARM/ST/src/uart_stm32.cpp`. UART indices are 0 for LPUART1, 1 for USART1,
2 for USART2 and 3 for USART3. USART1 to USART3 run from PCLK2 or PCLK1, the
driver writes their CCIPR1 kernel clock selection to 0. LPUART1 is on APB7 and
selects LSE, HSI16 or PCLK7 for the requested rate. `SystemPeriphClockGet` in
`src/system_stm32wbaxx.c` reads the APB1, APB2 and APB7 prescalers.

Polling, interrupt and DMA (`bDMAMode`, with or without `bIntMode`) are
supported. GPDMA1 takes any request on any channel; the driver uses two
channels for each instance and defines their vectors. GPDMA has no circular
mode, the RX ring is a linked list node that loads the same block again at
the end of each block. The driver leaves the GPDMA1 security and privilege
settings at their reset value, which suits a device with TrustZone disabled
(TZEN = 0). UART PRBS TX and loopback have passed on hardware. The STM32WBA65I-DK1 test projects are
`STM32WBA65/exemples/UartLoopback` and `UartPrbsTxTest`, described in
`STM32WBA65/README.md`.

| DevNo | Hardware | RX DMA | TX DMA | Requests RX, TX |
|---|---|---|---|---|
| 0 | LPUART1 | GPDMA1 channel 4 | GPDMA1 channel 5 | 15, 16 |
| 1 | USART1 | GPDMA1 channel 0 | GPDMA1 channel 1 | 11, 12 |
| 2 | USART2 | GPDMA1 channel 2 | GPDMA1 channel 3 | 13, 14 |
| 3 | USART3 | GPDMA1 channel 6 | GPDMA1 channel 7 | 65, 66 |

## Timer

The WBA65 library builds the STM32 timer driver shared with the other
families: `ARM/ST/src/timer_stm32.cpp` (TimerInit), `timer_lptim_stm32.cpp`
(LPTIM) and `timer_tim_stm32.cpp` (TIM), see `ARM/ST/include/timer_stm32.h`.
DevNo 0 is LPTIM1 (APB7, CCIPR3 LPTIM1SEL), DevNo 1 LPTIM2 (APB1, CCIPR1
LPTIM2SEL), then 2 TIM16, 3 TIM17, 4 TIM2, 5 TIM3, 6 TIM4, 7 TIM1.
`TimerGetLowFreqDevCount` returns 2, `TimerGetHighFreqDevCount` 6.

Each LPTIM counts 0 to 0xFFFF, extended to 64 bits in software, and has two
compare triggers, CCR1 and CCR2, single shot or continuous. A continuous
trigger is scheduled one period after its previous compare value, so it does
not drift with the interrupt latency. Clock: LSI (`TIMER_CLKSRC_LFRC`,
32 kHz, 250 Hz with the LSI1 prescaler), LSE (`TIMER_CLKSRC_LFXTAL`, crystal,
or bypass when `g_McuOsc.LowPwrOsc` is `OSC_TYPE_TCXO`; LSESYS is enabled for
the peripherals) or HSI16. `TIMER_CLKSRC_DEFAULT` takes LSI or LSE from
`g_McuOsc.LowPwrOsc`. The LSI and LSE bits are in the backup domain: the
driver enables the PWR clock and sets `PWR_DBPR` DBP. The frequency is the
closest of the kernel clock divided by 1 to 128.

This LPTIM takes CFGR writes with the timer disabled and DIER, ARR, CCR1 and
CCR2 writes with the timer enabled, each confirmed by DIEROK, ARROK, CMP1OK
or CMP2OK; the waits are bounded and a write not confirmed makes `TimerInit`,
`Enable` or `EnableTrigger` fail. DIER is written right after ENABLE, as in
the ST HAL, and written again if DIEROK does not come.

The TIM count at the APB timer clock divided by 1 to 65536,
`TIMER_CLKSRC_DEFAULT` only, with their compare channels as triggers: 1 on
TIM16 and TIM17, 4 on TIM1 to TIM4. External trigger and the tick interrupt
are not supported. The timer vectors of `STM32WBA65/src/vectors_stm32wba65.c`
are weak aliases of `Stm32TimerVector`; TimerInit refuses a timer whose
vector is replaced by an application handler. TimerDemo callbacks have passed
on hardware for the LPTIM and TIM engines; absolute frequency has not been
measured against an external reference.

## STM32CubeWBA dependency

Use an ST STM32CubeWBA distribution matched to the selected device and BLE
middleware version. The existing adapter includes `stm32wbaxx_hal.h`,
`ble_gap_aci.h`, `ble_gatt_aci.h`, `host_stack_if.h`,
`ll_sys.h`, `bpka.h`, `scm.h` and `stm32_timer.h`. The project must provide
their official headers, linked middleware objects/libraries, and any required
BLE/link-layer initialization components. Do not substitute invented register
definitions or mix headers from incompatible CubeWBA releases.

The WBA65 library uses `stm32wba65xx.h`, IOsonata `ARM/src/ResetEntry.c`,
`src/system_stm32wbaxx.c` and `STM32WBA65/src/vectors_stm32wba65.c`. Do not
add a second Cube startup or vector table to the application.

## Implementation and validation inventory

| Area | Existing implementation | Remaining validation or limitation |
|---|---|---|
| Startup and clocks | ResetEntry, WBA65 vectors, FPU enable, PLL1R at 100 MHz | Debug/Release minimal images reach main in the emulator; Blinky boots on hardware; option bytes and clock frequency unverified |
| Memory layout | WBA65 I: 2 MB flash / 512 KB SRAM; two reserved 16 KB NVM regions | Confirm security configuration on the board; no secure image supplied |
| GPIO | Shared STM32 pin configuration, atomic output writes, EXTI dispatch | LED output and external input/EXTI confirmed on hardware |
| UART | LPUART1, USART1/2/3; polling, interrupt and GPDMA | 1 Mbaud PRBS TX and loopback passed; loopback reported zero errors |
| Timers | Two LPTIM and six TIM instances through the shared STM32 driver | LPTIM and TIM callbacks passed; no external frequency reference used |
| I2C | Master polling/interrupt/DMA/DMA+interrupt; slave interrupt/DMA | I2CMasterSlave: all eight combinations passed on DK1 |
| SPI | Master polling/interrupt/DMA/DMA+interrupt; slave interrupt/DMA | SPIMasterSlave: all eight combinations passed; see detailed README |
| BLE | CubeWBA host/link-layer integration, GAP/GATT, security and bond storage | Advertising and counter updates pass; BleSecurityDemo reported working October 18, 2026; central validation remains pending |

WBA65 projects include GPIO, UART, timer, I2C, SPI and BLE examples with Debug
and Release configurations. UartRetargetDemo, UartSlipPrbsTx and UartSlipPrbsRx
now link the shared UART examples. Their builds and hardware tests are pending.
The library includes the BLE glue and vendor platform sources; Bluetooth
applications also link the two ST middleware archives. See
`STM32WBA65/README.md` for the project list, wiring and recorded test results.

Remaining validation includes BLE central scan/connect/discovery/read/subscribe
and optional peripherals not covered by the reported tests. Do not repeat every
timer instance when the same timer engine has already been tested.

Keep the release unmerged until hardware validation and final approval.

ST's firmware package: https://github.com/STMicroelectronics/STM32CubeWBA

## STM32WBA65 hardware priority (2026-10-09)

The maintainer has a WBA65 development board available for target testing.
WBA65 is now the first STM32WBA bring-up target. ST documents WBA65 family
variants with up to 2 MB flash and 512 KB SRAM, a 100 MHz Cortex-M33 and
Bluetooth LE support. Confirm the exact MCU marking before selecting the
flash/RAM layout; do not assume every WBA65 has the maximum memory.

For STM32WBA65I-DK1, use the WBA65 device header and board-local pin
assignments in examples; no Discovery Kit wiring belongs in the MCU library.
The example board.h selects the board pin map of `ARM/ST/include/stm32_boards.h`
with `#define BOARD STM32WBA65I_DK1`: USART1 (DevNo 1) PA8 RX / PB12 TX to the
ST-LINK virtual COM port, USART2 (DevNo 2) on Arduino D0 PA11 / D1 PA12, LD6
green PD8 and LD5 red PD9 (active low). The library does not include it.
The Discovery Kit has STLINK-V3EC (including USB virtual COM), OLED display,
three LEDs and joystick, enabling startup, UART, GPIO and interactive BLE
security validation. Board support must not be confused with the NUCLEO kit.

References:
- https://www.st.com/en/evaluation-tools/stm32wba65i-dk1.html
- https://www.st.com/resource/en/user_manual/DM01147159.pdf
- https://www.st.com/en/microcontrollers-microprocessors/stm32wba65mi.html

