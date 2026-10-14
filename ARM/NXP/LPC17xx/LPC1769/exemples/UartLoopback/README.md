# LPC1769 UART loopback

Build the matching Debug or Release IOsonata_LPC1769 library, then import
`ioc` into IOcomposer. The project links `exemples/uart/uart_loopback.cpp`
and echoes received data through the common UART interface.

Connect a 3.3 V USB-UART adapter TX to P0.3 (UART0 RX), adapter RX to P0.2
(UART0 TX), and a common ground. These are the same MCU pins as TimerDemo
and DfuBoot; locate them on your board's headers. Do not jumper RX to TX.

Use 1000000 baud, 8N1, no flow control. The default uses interrupt-driven
RX/TX with 256-byte FIFOs. UART DMA is unsupported and disabled. Set
UART_INT_MODE to false to exercise polling operation.

From the repository root, with pyserial installed:

```
python3 Python/uartprbs_lb.py --baud 1000000 --port /dev/cu.usbserial-DEVICE
```

Replace the port with your adapter's port. Check for sustained echo with no
errors or lost bytes. Stop with Ctrl-C.

Debug uses semihosting and requires an attached debugger with semihosting
enabled. Release uses nosys without semihosting. Neither build redirects
console output into the UART data stream.

Debug and Release application compilation and links against the complete
LPC1769 library pass. Physical UART testing and throughput measurements
remain pending.
