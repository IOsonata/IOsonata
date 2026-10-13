# LPC1769 UART PRBS transmit test

Build the matching Debug or Release IOsonata_LPC1769 library, then import
`ioc` into IOcomposer. The project links `exemples/uart/uart_prbs_tx.cpp`
and continuously transmits the shared PRBS byte sequence.

Connect a 3.3 V USB-UART adapter RX to P0.2 (UART0 TX) and a common ground.
P0.3 is UART0 RX; connecting adapter TX is optional for this transmit test.
These are the same MCU pins as UartLoopback and TimerDemo; locate them on
your board's headers. No RX-to-TX jumper is needed.

Use 1000000 baud, 8N1, no flow control. Interrupt mode is enabled by default
and uses the driver's default FIFOs. Set UART_INT_MODE to false for polling.
UART DMA is unsupported and disabled. The shared application sends one byte
at a time and advances the PRBS sequence only after the byte is accepted.

From the repository root, with pyserial installed:

```
python3 Python/uartprbs_rx.py --baud 1000000 --port /dev/cu.usbserial-DEVICE
```

Replace the port with your adapter's serial port. Check the reported receive
rate and dropped bytes over a sustained run. Stop with Ctrl-C. Compare Debug
and Release separately; their throughput can differ.

Debug uses semihosting and requires an attached debugger with semihosting
enabled. Release uses nosys without semihosting. Neither build redirects
console output into the PRBS stream.

Debug and Release application compilation and links against the complete
LPC1769 library pass. Hardware PRBS integrity and throughput testing remain
pending.
