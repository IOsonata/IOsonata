# STM32WBA MCU port

IOsonata contains a standalone STM32WBA65 MCU library and an ST Bluetooth ACI
adapter. GPIO, UART, timers, I2C/SPI, BLE security, crypto and USB CDC are
hardware validated. BLE central scan/connect/discovery/notification/reconnect
also pass on hardware. OLED, analog joystick, microphone level capture and
stereo headphone tone examples work on the DK1. Full-duplex microphone audio
has an unresolved clock-associated headphone buzz; further investigation is
closed for this bring-up. See the detailed status and optional limitations.
The target is the
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

The library builds the shared STM32 PWM driver, `ARM/ST/src/pwm_stm32.cpp`:
the generic PWM API on the output channels of TIM16, TIM17, TIM2, TIM3, TIM4
and TIM1, with the timer DevNo above. See [STM32 PWM](../PWM.md). No example
project; not run on hardware yet.

The library builds the shared STM32 watchdog driver,
`ARM/ST/src/wdt_stm32.cpp`, on the IWDG: timeout up to 131 s from the 32 kHz
LSI1 (128 times longer with LSI1PREDIV), window, and the timeout event from
the early wakeup interrupt, one counter tick before the reset. See
[STM32 watchdog](../WDT.md). Example `STM32WBA65/exemples/WdtDemo`
(`exemples/misc/wdt_demo.cpp`) on the first ST-LINK virtual COM port (USART1,
115200 8N1) in Release, with semihosting in Debug. Not run on hardware yet.

The library builds the shared STM32 RTC driver, `RtcStm32`
(`ARM/ST/src/rtc_stm32.cpp`), the generic RTC device of `miscdev/rtc.h`
(`src/miscdev/rtc.cpp`): calendar in the C runtime types, alarms A and B,
wakeup timer, 32 backup registers in TAMP, from the LSE or the 32 kHz LSI1
(250 Hz with LSI1PREDIV), which keeps running over a system reset. See [STM32
RTC](../RTC.md). Example `STM32WBA65/exemples/RtcDemo`
(`exemples/misc/rtc_demo.cpp`) on the first ST-LINK virtual COM port (USART1,
115200 8N1) in Release, with semihosting in Debug. Not run on hardware yet.

## Crypto

Hardware validated on STM32WBA65I-DK1, October 18, 2026: **51 passed, 0 failed**.
Tests cover RNG, AES ECB/CBC/CTR/CMAC/CCM/GCM, P-256 ECDH and ECDSA,
invalid-input rejection, output clearing and comparison with software engines.
P-256 scalar multiplication averaged 31072 us on the engine and 188099 us in
software over 8 rounds. BLE uses CryptoWbaAes and CryptoWbaPka, with software
AES fallback when the AES peripheral is busy. See the [test results](STM32WBA65/README.md#crypto-hardware-validation).

## USB device

The WBA65 library links the shared STM32 OTG FIFO driver. UsbCdcLoopback and
UsbCdcPrbsTx projects use the common IOsonata applications on the native CN5
connector. Control, bulk and interrupt transfers are implemented; ISO and USB
host mode are unsupported. CDC PRBS TX runs at about 194.5 kB/s in Debug and
649 kB/s in Release. Release CDC loopback passed 60 seconds at about 1.20 MB/s
in each direction: 72094720 bytes sent and received, zero errors and zero
pending bytes. Reconnect, suspend/resume and FS fallback remain pending. See
[`../USB.md`](../USB.md) for family coverage, clock requirements and testing.

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
| Crypto | RNG, AES and PKA engines | Hardware validated: 51 passed, 0 failed; see Crypto above |
| USB | Shared STM32 OTG device controller; control, bulk and interrupt transfers | CDC hardware validated: Debug/Release PRBS TX and Release 60-second loopback; additional modes pending |
| BLE | CubeWBA host/link-layer integration, GAP/GATT, security and bond storage | Advertising and counter updates pass; Numeric Comparison, encrypted transfer and bonded reconnect after reset hardware validated; central scan/connect/discovery/notification/reconnect tested; explicit characteristic reads unvalidated |
| ADC / joystick | Polling single-round ADC, external inputs; menu, stopwatch, countdown and Pong | Menu/stopwatch/countdown hardware tested; Pong hardware pending; continuous/DMA/internal-temperature modes unsupported |
| OLED | Generic SSD1315 through SPI3 | Console, graphics and interactive examples exercised |
| EEPROM | M95P32 through generic Nvm; SeepDemo | Builds and command-model tests pass; hardware result not recorded |
| Audio | Generic WM8904 control and SAI1 I2S; shared SAI/GPDMA native PDM | Level meter and headphone tones hardware tested; clock-associated loopback buzz unresolved; native PDM hardware untested |

WBA65 projects include GPIO, UART, timer, I2C, SPI and BLE examples with Debug
and Release configurations. UartRetargetDemo, UartSlipPrbsTx and UartSlipPrbsRx
link the shared UART examples. They compile and link in Debug/Release;
hardware results are not recorded.
The library includes the BLE glue and vendor platform sources; Bluetooth
applications also link the two ST middleware archives. See
`STM32WBA65/README.md` for the project list, wiring and recorded test results.

Remaining validation includes explicit BLE characteristic reads, additional
USB modes and optional peripherals not covered by the reported tests. Do not repeat every
timer instance when the same timer engine has already been tested.

ST's firmware package: https://github.com/STMicroelectronics/STM32CubeWBA

## STM32WBA65I-DK1 board mapping

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


