# STM32F030x8 example projects

Import each example's `ioc` directory into IOcomposer. Build
`STM32F030x8/lib/ioc` (`IOsonata_STM32F030x8`) in the same Debug or Release configuration
first. Projects link the shared sources under the repository's `exemples/`
directory; pin assignments and target settings belong in each application's
`src/board.h`.

## Peripheral and kernel examples

| Project directory | Shared source | Purpose |
|---|---|---|
| Blinky (existing) | `misc/blinky.c` | GPIO LEDs; check its board-specific pin map |
| PulseTrain | `misc/pulse_train_test.c` | Moving pulse on the DISCO PC9/PC8 LEDs |
| TimerDemo (existing) | `timer/timer_demo.cpp` | All seven virtual TIM devices, selected in board.h |
| UartPrbsTest (existing) | `uart/uart_prbs_tx.cpp` | Raw PRBS transmitter |
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

The new projects other than UartRetargetDemo use `rdimon` semihosting: run with a
debugger that enables semihosting. PRBS/SLIP diagnostics go to the debugger, not
into the test UART stream. Per-byte logging is disabled for the F030 raw receiver
to avoid stalling reception. UartRetargetDemo uses `nosys` with IOsonata's UART
stdio retargeting and does not require semihosting.

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
| RfReaderPn532Test | `rftag/rfreader_pn532_exerciser.cpp` | PN532 framing with a simulated transport |
| RfTagControllerTest | `rftag/rftag_controller_exerciser.cpp` | Controller framing with a simulated transport |
| RfTagIso15693Test | `rftag/rftag_iso15693_exerciser.cpp` | ISO15693 protocol over memory-backed tags |
| RfTagT2Test | `rftag/rftag_t2_exerciser.cpp` | Type 2 tag protocol over memory-backed tags |
| RfTagT4Test | `rftag/rftag_t4_exerciser.cpp` | Type 4 tag protocol over memory-backed tags |
| St25dvTest | `rftag/st25dv_exerciser.cpp` | ST25DV operations with a simulated transport |

Software PRNG test fixtures are not a cryptographic entropy source. RF-tag
exercisers do not establish physical NFC/RF, I2C or SPI support on this MCU.
The library project links the existing portable AES, PRNG and RF-tag sources
needed by these tests; their implementations are unchanged.

## Existing boot project and exclusions

DfuBoot remains a separate existing project with its own flash layout and boot
requirements; it is not covered by the new example link checks.

Examples requiring unimplemented F030 backends (I2C, SPI, ADC/comparator, PWM,
watchdog, USB, Bluetooth, LTE or hardware crypto/RNG) are not added. The generic
NDEF demo needs a board-supplied `RFTagDemoGetIntrf()` adapter and is not a
standalone software exerciser. Storage host tests use simulated memories larger
than this target's RAM and are not target firmware projects. The shared FreeRTOS
PRBS source has no scheduler/task startup or F030 kernel/configuration project;
it is not represented as a working FreeRTOS example.

## Validation

All 18 added shared example sources compile for Cortex-M0 with Arm GNU 14.3.1
at `-O0` and `-Os`. All 36 firmware images link using the real startup, vector,
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

Hardware UART data integrity, debugger console behavior and GPIO/timer activity
still require on-board checks.
