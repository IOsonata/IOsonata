# SAM4L UART receive wake

Call `UARTEnableWake(&uart, true)` or `uart.EnableWake(true)` on
an initialized, enabled USART0 with RX mapped to PB00, then call
`SystemSleep()`. The application does not switch UART clocks or baud
rates, disable USART0, configure PICOUART or restore the transport.
The SAM4L UART driver prepares PICOUART for WAIT sleep and restores
USART0 before `SystemSleep()` returns. If PICOUART received a byte,
the driver copies it into the normal UART RX FIFO for `UARTRx()`.

This is a UART wake-source selection, not a UART sleep register or
an application-defined sleep mode. The UART selects its wake capability
and the system power implementation selects a compatible power state.
Other targets without receive wake return false from
`UARTEnableWake()`.

## Hardware connections and rates

PICOUART shares USART0 RX on PB00, not the EDBG USART1 console.

| External adapter | SAM4L |
| --- | --- |
| 3.3 V TX | EXT1/EXT4 pin 13, PB00 (USART0 RX / PICOUART RX) |
| 3.3 V RX | EXT1/EXT4 pin 14, PB01 (USART0 TX) |
| GND | EXT1/EXT4 pin 19 (or pin 2) |

EXT2 pin 13/14 is PC26/PC27 USART1, not USART0; do not use EXT2
for this profile. PB00/PB01 are available on both EXT1 and EXT4.

USART0 normal operation retains its configured baud, 1,000,000 by
default. PICOUART uses 9600 baud, 8-N-1, with a 32768 Hz clock
from RC32K or OSC32K. The sender must wait for application
acknowledgement before sending more data. PICOUART holds only one
received byte during the transition.

## CorePowerTest profile

Set `POWER_DEMO_UART_WAKE=1` in SAM4LCxC or SAM4LSxC CorePowerTest
`src/board.h` (or the application project symbols). The SAM4LCxC board.h selects PICOUART wake by default on the 0.14
review branch. Override `POWER_DEMO_UART_WAKE=0` to return to its
previously validated EIC button-wake profile. Rebuild the matching
library and application using the same Eclipse managed Debug or
Release configuration. The UART-wake
profile uses the external adapter for **all console output**, not EDBG.

Start the host helper before resetting the board:

```sh
python3 Python/sam4l_uart_wake.py --port /dev/cu.usbserial-ADAPTER --baud 1000000 --cycles 3
```

The helper first sends a ready byte `R` at the normal 1 Mbaud console
rate. After the MCU's setup prompt, the host changes the adapter to
9600 baud while USART0 is still active. The MCU waits 750 ms before
arming PICOUART in `SystemSleep()`; the host waits 1.25 s by default
before sending `U`, then restores the console baud. It checks the
recovered UART byte, rate, CPU/APB clock, sequential resume counter
and sampled SRAM after every wake. The script stops after the requested
number of complete cycles or a configured timeout.

## Driver behavior and limits

Only an enabled USART0 configured on PB00 for 8-N-1 without flow
control can request PICOUART wake. The driver checks the RX pin
route, frame format, clock source, PM wake ownership and pending
UART traffic. A busy receiver, conflicting wake registration or
unavailable low-frequency clock may prevent sleep. UART stays
operational if preparation fails; `SystemSleep()` has no independent
success return value.

Disabling, powering off or reinitializing UART clears wake selection.
The driver restores PICOUART, USART interrupt settings, clock gates,
the asynchronous PICOUART wake selection and the captured RX byte.
SystemSleep alone manages the shared PM wake interrupt. Any receive callback
is called with interrupts masked and must not block. The ordinary
USART divider is not rewritten for PICOUART. This is resumable WAIT
sleep, not Backup-mode UART wake.

## Validation status

SAM4L register tests passed before this refactor. Earlier LC/LS
Debug/Release checks compiled and linked using cached library
archives with updated UART and system objects; these were not
clean full-library rebuilds. On November 18, the maintainer reported
10 consecutive physical PICOUART wakes using 9600-baud `U`. Every
cycle passed USART0 baud restoration, CPU/APB restoration, sequential
resume counting (1 through 10), and 128 static plus 128 stack bytes
of sampled SRAM retention. The host helper required every check
before accepting a cycle. Sleep-current measurement, extended wake
reliability and mixed-peripheral sleep behavior remain pending.

## Measuring PICOUART sleep current

The SAM4L8 Xplained Pro has a dedicated 1x2 MCU-current measurement
header near the board's upper edge. All MCU supply current passes
through this header (Atmel-42103B-SAM4L8-Xplained-Pro User Guide,
section 4.1.3). **Disconnect board power before removing the MCU
supply jumper or inserting/removing an ammeter.** Removing it while
powered may back-power the MCU through I/O pins and cause damage.

Connect a low-burden ammeter or power analyzer *in series across the
jumper header*, then power the board. Use an appropriate starting
current range to accommodate startup current. Check that the MCU
still starts and that the analyzer does not cause a supply brownout.
The external USB-UART adapter must share ground and idle HIGH on PB00.
Measure MCU supply current, not the total debugger/board current.

The host helper accepts `--wake-delay`, the delay from switching the
FTDI adapter to 9600 baud until sending the wake byte. The default is
1.25 seconds, matching the ten-cycle validation. Choose a longer
interval to observe a stable low-power plateau. The MCU remains in
its independently selected sleep mode; the host controls only when it
sends the wake stimulus.

```sh
python3 Python/sam4l_uart_wake.py \
    --port /dev/cu.usbserial-A8UKQC7S \
    --baud 1000000 \
    --cycles 1 \
    --wake-delay 20
```

After the host prints "Adapter switched to 9600 before PICOUART
activation", wait approximately one second for the firmware's
750 ms pre-sleep guard to elapse, then measure the steady-state
current before the host sends `U`. The helper performs the same
post-wake functional checks. Record the supply voltage, mean current
and peak/transient current, supply configuration, debugger connection,
adapter wiring and measurement duration.

A successful wake does not itself establish the current or even the
exact power state reached. Compare the plateau with an active-idle
baseline, and repeat separately for EIC/AST wake if needed. Do not
attribute board-wide or debugger current to the target MCU.

## Extended wake validation (November 18, 2026)

The maintainer reported a successful single hardware wake using
`--wake-delay 20`, with the normal 1 Mbaud RX preflight and the
9600-baud PICOUART wake followed by UART rate, CPU/APB clock,
resume-counter and 128-static/128-stack-byte SRAM checks all PASS.
The earlier ten-cycle default-delay run also passed every check.
The extended delay is host-controlled test traffic only; PICOUART
and USART0 transition management remains inside the MCU driver.

No supply-current measurement was possible with the available tools.
The exact low-power hardware state and consumption remain unverified.
