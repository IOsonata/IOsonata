# SAM4L UART wake

`Sam4lUartSleep()` uses PICOUART to receive one wake byte while the CPU is
in WAIT. USART0 resumes after wake, and `UARTRx()` returns the captured byte
through its normal receive FIFO. The generic UART API is unchanged.

PICOUART is wired to PB00, shared with USART0 RX. It cannot monitor the
USART1 EDBG console. Its frame format is fixed: 9600 baud, eight data bits,
no parity, one stop bit, with a 32768 Hz CLK32 source. RC32K or OSC32K must
already be running. The helper does not change the LF clock selection.

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

Open the adapter at 9600 baud, 8-N-1, without flow control. This profile uses
that adapter for all console output, not the EDBG virtual serial port. Check
that other connected peripherals do not drive PB00 or PB01.

At each `Entering UART sleep; send one U at 9600 baud` prompt, send a single
uppercase `U`, without a newline. Wait for `UART wake byte: PASS` before
sending anything else. The demo checks CPU/APB/LF restoration, timer IRQs,
msDelay and nsDelay after each wake. When the ordinary `UART RX` prompt
appears, send another single `U` for that separate receive check.

The LC profile repeats UART wake after each selected CPU source and LF
selection, six times in total. The LS profile runs three wake cycles after
its clock sweep. A successful run ends with `ClockDemo complete: 0 failures`.

## Driver use and limits

Include `uart_sam4l.h`. Initialize USART0 on PB00 at 9600 baud, 8-N-1, with
no flow control. Drain TX and RX, stop other active peripherals, and call
`Sam4lUartSleep(&uart)` from thread context with interrupts enabled.

The helper temporarily owns PICOUART and the PM WAKE interrupt. It refuses
an already enabled PICOUART, an application-owned PM WAKE interrupt, a busy
UART, or an unsupported configuration. It restores the USART interrupt mask,
clock gates, PICOUART configuration and asynchronous wake mask on return.
A fresh PM wake event and PICOUART data-ready status are both required before
copying a byte into the RX FIFO. An unrelated interrupt can return false;
callers must handle this result rather than assuming UART reception occurred.

A successful return means one byte is in the RX FIFO. An installed UART RX
callback is invoked in the caller context after restoration. The sender must
wait for an application acknowledgement before sending payload: PICOUART has
only one receive holding register, and uninterrupted reception during clock
restoration is not supported. This helper provides resumable WAIT wake, not
Backup-mode UART wake.

## Validation

Both profiles compile and link for SAM4LCxC and SAM4LSxC in Debug and Release
with GNU Arm GCC 15.2.1 and GNU++11. The updated UART driver was rebuilt into
the existing library archives for these eight links. Existing SAM4L power
and clock register tests also pass. UART wake, first-byte recovery and repeat
wake behavior still require hardware validation.

Reference: Microchip ATSAM4L8/L4/L2 datasheet 42023H, chapter 25 (PICOUART),
chapter 10 (Power Manager) and the I/O multiplexing tables.
