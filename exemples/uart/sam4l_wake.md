# SAM4L UART wake

Enable receive wake with `UARTEnableWake(&uart, true)` or the C++
`uart.EnableWake(true)` wrapper, then call `SystemSleep()`. Configuration does
not stop the UART or enter sleep. During sleep entry, the SAM4L driver uses
PICOUART to receive one wake byte in WAIT. Before `SystemSleep()` returns,
the driver restores USART0 and places the captured byte in its normal RX FIFO.
Ports without receive-wake support return false from `UARTEnableWake()`.

PICOUART is wired to PB00, shared with USART0 RX. It cannot monitor the
USART1 EDBG console. Its frame format is fixed: 9600 baud, eight data bits,
no parity, one stop bit, with a 32768 Hz CLK32 source. RC32K or OSC32K must
already be running. The helper does not change the LF clock selection or
the normal USART baud-rate divider. USART0 resumes at its configured rate.

## ClockDemo profile

In the SAM4LCxC or SAM4LSxC ClockDemo `src/board.h`, set
`CLOCK_DEMO_UART_WAKE` to `1`, or define it as `1` in the compiler symbols.
Clean and rebuild the matching IOsonata library and ClockDemo. The default
profile remains unchanged when this option is zero.

Connect a 3.3 V UART adapter:

| Adapter | MCU pin |
| --- | --- |
| TX | PB00, USART0 RX |
| RX | PB01, USART0 TX |
| GND | GND |

Open the adapter at the normal console rate (1000000 baud by default),
8-N-1, without flow control. This profile uses
that adapter for all console output, not the EDBG virtual serial port. Check
that other connected peripherals do not drive PB00 or PB01.

At each `Entering UART sleep` prompt, change the adapter to 9600 baud, send
one uppercase `U` without a newline, drain the adapter TX, and immediately
restore the normal baud rate. The MCU resumes and prints at the normal rate.
The demo waits 100 ms after return before printing so the host adapter can
change rate. The driver itself introduces no such delay. A manually operated
terminal may still miss the first output during this switch.
`Python/sam4l_uart_wake.py` performs this sequence automatically. Wait for
`UART wake byte: PASS` before sending payload. The demo checks CPU/APB/LF restoration, timer IRQs,
msDelay and nsDelay after each wake. When the ordinary `UART RX` prompt
appears, send another single `U` for that separate receive check.

With pyserial installed, run the host helper before resetting the board:

```sh
python3 Python/sam4l_uart_wake.py --port /dev/cu.usbserial-ADAPTER --baud 1000000
```

The helper also answers the ordinary UART RX prompts at the normal baud rate.

The LC profile repeats UART wake after each selected CPU source and LF
selection, six times in total. The LS profile runs three wake cycles after
its clock sweep. A successful run ends with `ClockDemo complete: 0 failures`.

## Driver use and limits

Include `coredev/uart.h` and `coredev/system_core_clock.h`. Initialize USART0
on PB00 at the application baud rate, 8-N-1, with no flow control. Only the
sleep receiver uses 9600 baud. Drain TX and RX and stop other active peripherals
before entering sleep from thread context with interrupts enabled.

```cpp
if (UARTEnableWake(&uart, true))
{
	SystemSleep();
	UARTEnableWake(&uart, false);
}
```

Wake remains selected across repeated `SystemSleep()` calls until disabled.
Disabling, powering off or reinitializing the UART also clears the selection.
The example disables wake after each sleep check. No target header or target
function is needed by the application.

At sleep entry, the driver checks CLK32, PICOUART ownership, the PM WAKE
interrupt and UART activity. If it cannot safely prepare reception, it leaves
the UART running and `SystemSleep()` returns without sleeping. Other interrupt
sources can also wake the MCU. Always check received data using `UARTRx()`;
accepting the wake configuration does not guarantee that a byte was received.

The driver restores the USART interrupt mask, clock gates, PICOUART
configuration and asynchronous wake mask before returning from sleep. A fresh
PM wake event and PICOUART data-ready status are both required before copying
a byte into the RX FIFO. An installed RX callback runs during restoration with
interrupts masked and must not block.

The sender must wait for an application acknowledgement before sending payload:
PICOUART has only one receive holding register, and uninterrupted reception
during clock restoration is not supported. This provides resumable WAIT wake,
not Backup-mode UART wake.

## Validation

Both profiles compile and link for SAM4LCxC and SAM4LSxC in Debug and Release
with GNU Arm GCC 15.2.1 and GNU++11. The UART, generic UART and system-power sources were rebuilt into
the existing library archives for these eight links. Existing SAM4L power
and clock register tests also pass. UART wake, first-byte recovery and repeat
wake behavior still require hardware validation.

Reference: Microchip ATSAM4L8/L4/L2 datasheet 42023H, chapter 25 (PICOUART),
chapter 10 (Power Manager) and the I/O multiplexing tables.
