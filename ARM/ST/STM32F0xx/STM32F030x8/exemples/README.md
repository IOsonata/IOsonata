# STM32F030x8 example projects

Import each example's `ioc` directory into IOcomposer. Build
`STM32F030x8/lib/ioc` (`IOsonata_STM32F030x8`) in the same Debug or Release configuration
first. Projects link the shared sources under the repository's `exemples/`
directory; pin assignments and target settings belong in each application's
`src/board.h`.

## Peripheral and kernel examples

| Project directory | Shared source | Purpose |
|---|---|---|
| Blinky | `misc/blinky.c` | GPIO LEDs; check its board-specific pin map |
| PulseTrain | `misc/pulse_train_test.c` | Pulse train across all 55 GPIOs exposed by the STM32F030R8 package |
| TimerDemo | `timer/timer_demo.cpp` | All seven virtual TIM devices, selected in board.h |
| UartPrbsTest | `uart/uart_prbs_tx.cpp` | Raw PRBS transmitter |
| UartPrbsRxTest | `uart/uart_prbs_rx.cpp` | Raw PRBS receiver and error reporting |
| UartLoopback | `uart/uart_loopback.cpp` | Echo received UART bytes |
| UartRetargetDemo | `uart/uart_retarget_demo.cpp` | printf/scanf through UART |
| UartSlipPrbsTxTest | `uart/uart_slip_prbs_tx.cpp` | SLIP-framed PRBS transmitter |
| UartSlipPrbsRxTest | `uart/uart_slip_prbs_rx.cpp` | Streaming SLIP decoder and PRBS checking |
| UartPrbsTxTestTaktOS | `uart/uart_prbs_tx_taktos.cpp` | PRBS transmit from a TaktOS thread |

New UART projects use USART1, virtual device 0, TX PA9 and RX PA10 at
115200 baud, 8N1, interrupt mode without DMA or flow control. Connect an external
3.3 V UART adapter with a common ground; the STM32F0308-DISCO ST-LINK/V2 is not
assumed to provide a virtual COM port. Use another board/transmitter to feed the
RX examples. Raw PRBS and SLIP are separate wire formats.

The new projects other than TimerDemo and UartRetargetDemo use `rdimon` semihosting: run with a
debugger that enables semihosting. PRBS/SLIP diagnostics go to the debugger, not
into the test UART stream. Per-byte logging is disabled for the F030 raw receiver
to avoid stalling reception. TimerDemo and UartRetargetDemo use `nosys` with
IOsonata's UART stdio retargeting and do not require semihosting.

For UartPrbsTxTestTaktOS, place the TaktOS checkout beside IOsonata, import
`TaktOS/ARM/cm0/ioc`, and build `TaktOS_M0` in the matching configuration before
building the example. The kernel owns SysTick and PendSV; the example does not
replace the shared ARM SysTick infrastructure or claim a peripheral TIM.

## Software-only tests

These projects use the debugger console and require no external wiring.

| Project directory | Shared source | Coverage |
|---|---|---|
| CryptoSoftAesTest | `crypto/crypto_softaes_test.cpp` | Software AES engine |
| CryptoSoftSha256Test | `crypto/crypto_softsha256_test.cpp` | SHA-256 and HMAC |
| CryptoSoftRngTest | `crypto/crypto_softrng_test.cpp` | Software PRNG checks |
| CryptoUeccTest | `crypto/crypto_uecc_test.cpp` | Software P-256 engine |
| DfuImageVerify | `crypto/dfu_image_verify.cpp` | Signature verification only; does not flash an image |

Software PRNG test fixtures are not a cryptographic entropy source.
The library project links the existing portable AES and PRNG sources
needed by these tests; their implementations are unchanged.

## Existing boot project and exclusions

RF-tag example projects are deferred until the required MCU transport support
is implemented. Simulated RF-tag exercisers are not included as F030 examples.

DfuBoot remains a separate existing project with its own flash layout and boot
requirements; it is not covered by the new example link checks.

Examples requiring unimplemented F030 drivers (I2C, SPI, ADC/comparator, PWM,
watchdog, USB, Bluetooth, LTE or hardware crypto/RNG) are not added. The generic
NDEF demo needs a board-supplied `RFTagDemoGetIntrf()` adapter and is not a
standalone software exerciser. Storage host tests use simulated memories larger
than this target's RAM and are not target firmware projects. The shared FreeRTOS
PRBS source has no scheduler/task startup or F030 kernel/configuration project;
it is not represented as a working FreeRTOS example.

## Validation

The 12 retained shared example sources were compiled for Cortex-M0 with Arm GNU 14.3.1
at `-O0` and `-Os`. Their 24 firmware images linked using the real startup, vector,
clock and required library sources, with the 64 KB flash / 8 KB RAM linker
regions. The TaktOS image also links the real Cortex-M0 kernel and context
switch implementation. These command-line checks use the necessary library
source subset; they are not full IOcomposer library builds or board execution.
Static image fit does not prove worst-case runtime stack usage.

The shared SLIP RX regression runs the actual example and decoder with buffered
and byte-fragmented UART input, including empty, exact-buffer and oversized
frames and split escape sequences:

```sh
python3 tests/stm32f0/run_slip_example.py
```

The maintainer confirmed startup, LED GPIO, USART1 UART output/retargeting,
TIM6 (virtual device 0) and TIM16 (virtual device 2) on STM32F0308-DISCO on
2026-10-08. TIM6 printed 115 consecutive 100 ms periods over 11.5 seconds.
TimerDemo uses UART retargeting; semihosting pauses can disturb measurements.
If an external UART adapter uses a level shifter, power that circuit as well.

The maintainer also confirmed UART TX DMA with the Release PRBS transmitter
at approximately 81.0 kB/s at 1 Mbaud and zero drops in the supplied output
([PR 77](https://github.com/IOsonata/IOsonata/pull/77)). UART RX/loopback,
SLIP data integrity, the remaining timer devices and software crypto examples
still need their own hardware checks. The minimum
MCU port is complete; this does not mean every example has been run on board.

The full GPIO pulse train includes PA13/PA14 (SWD) and oscillator pins. Use an
internal clock and isolate conflicting ST-LINK/oscillator connections for this
standalone test. Blinky enters the pulse train after B1 and disables its button
interrupts before reusing the pins as outputs.
