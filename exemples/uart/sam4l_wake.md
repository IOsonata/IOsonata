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

## PowerOffDemo profile

Set `POWER_DEMO_UART_WAKE=1` in SAM4LCxC or SAM4LSxC PowerOffDemo
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
arming PICOUART in `SystemSleep()`; the host waits 1.25 s before
sending `U`, then restores the console baud. It checks the recovered
UART byte, rate and CPU/APB clock
after every wake. The script stops after the requested number of
complete cycles or a configured timeout.

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
one physical PICOUART wake with the 9600-baud `U` delivered and
successful USART0 rate and CPU/APB restoration. The host helper now
requires a sequential resume counter and SRAM PASS on each cycle.
Ten consecutive physical wake cycles, SRAM verification under the
stricter host check, and sleep-current measurement remain pending.
