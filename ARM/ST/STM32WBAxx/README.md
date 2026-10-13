# STM32WBA MCU port — STM32WBA65 priority

IOsonata currently contains the STM32WBA Bluetooth ACI adapter and linker
scripts, but **not** a complete standalone STM32WBA MCU port. The hardware bring-up target is **STM32WBA65**. The target board is **STM32WBA65I-DK1 Discovery Kit**, with the
**STM32WBA65RIV7** device (2 MB flash / 512 KB SRAM), as confirmed by ST's
UM3462 and the board product page. Match its TrustZone configuration and
middleware version. The WBA5x linker scripts
are retained solely for WBA5x devices and must **not** be used for WBA65.

## Existing IOsonata sources

- `include/stm32wbaxx.h` and `include/system_stm32wbaxx.h`:
  ST-compatible entry headers; these do **not** contain the device register map.
- `src/bt_{app,adv,scan,gap,gatt,smp_bond,pds}_stm32wba.cpp` and
  `src/bt_wba_event_router.h`: initial ST BLE middleware integration.
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
(TZEN = 0). Not tested on hardware. The STM32WBA65I-DK1 test projects are
`STM32WBA65/exemples/UartLoopback` and `UartPrbsTxTest`, described in
`STM32WBA65/README.md`.

| DevNo | Hardware | RX DMA | TX DMA | Requests RX, TX |
|---|---|---|---|---|
| 0 | LPUART1 | GPDMA1 channel 4 | GPDMA1 channel 5 | 15, 16 |
| 1 | USART1 | GPDMA1 channel 0 | GPDMA1 channel 1 | 11, 12 |
| 2 | USART2 | GPDMA1 channel 2 | GPDMA1 channel 3 | 13, 14 |
| 3 | USART3 | GPDMA1 channel 6 | GPDMA1 channel 7 | 65, 66 |

## STM32CubeWBA dependency

Use an ST STM32CubeWBA distribution matched to the selected device and BLE
middleware version. The existing adapter includes `stm32wbaxx_hal.h`,
`ble_gap_aci.h`, `ble_gatt_aci.h`, `host_stack_if.h`,
`ll_sys.h`, `bpka.h`, `scm.h` and `stm32_timer.h`. The project must provide
their official headers, linked middleware objects/libraries, and any required
BLE/link-layer initialization components. Do not substitute invented register
definitions or mix headers from incompatible CubeWBA releases.

The complete device header (for example, `stm32wba55xx.h`) and matching
CMSIS startup/system implementation are required. The presence of
`stm32wbaxx.h` alone does not satisfy them.

## Work required before claiming an IOsonata MCU port

1. For STM32WBA65, add the exact WBA65 CMSIS device package, device-specific
   system clock and reset/startup implementation, vector
   table and interrupt dispatch using the existing IOsonata MCU pattern.
2. Implement and test MCU-owned GPIO/pinmux, UART and Timer drivers using
   the target reference manual and the correct STM32WBA device definitions.
3. Bind the existing BLE adapter to a pinned STM32CubeWBA release, resolve the
   ACI and link-layer dependencies, and verify controller/stack bring-up.
4. Create an MCU library project and small bring-up applications with
   application-owned `board.h` files; do not put board wiring in the library.
5. Create a dedicated STM32WBA65 linker script (the 1 MB/128 KB WBA5x
   script is incompatible). Verify FLASH/RAM sizes and secure/non-secure placement against the
   exact WBA part and chosen provisioning configuration.
6. Run compiler/link checks and hardware tests for startup, UART, timers,
   BLE advertising, connecting, GATT, pairing/bonding and NVM persistence.

**Validation status:** these steps are not yet complete. Bluetooth source
presence must not be interpreted as a buildable or hardware-validated port.
The BLE adapter is designed for ST's CubeWBA host/controller middleware,
not a replacement of its radio controller.

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
