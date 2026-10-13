# LPC11U35 UART loopback

Build the matching Debug or Release IOsonata_LPC11U35 library, then import
`ioc` into IOcomposer. This project links `exemples/uart/uart_loopback.cpp`.
The application echoes received bytes using the common UART interface.

On IBK-IDAP-M revision 0, connect a 3.3 V USB-UART adapter:

| Adapter | Board |
| --- | --- |
| TX | P2 pin 3, P0.18 UART0 RX |
| RX | P2 pin 4, P0.19 UART0 TX |
| GND | P2 or P3 pin 13 |

Use 1000000 baud, 8N1, no flow control. Do not jumper the board RX and TX
together: the host sends data and the firmware returns it. The default uses
interrupt-driven RX/TX with 256-byte FIFOs. UART DMA is unsupported and off.
Set UART_INT_MODE to false to exercise polling operation.

From the repository root, with pyserial installed:

```
python3 Python/uartprbs_lb.py --baud 1000000 --port /dev/cu.usbserial-DEVICE
```

Replace the port with your adapter's port. Check for sustained echo with no
errors or lost bytes. Stop with Ctrl-C. No throughput result is claimed yet.

Debug retains default semihosting and requires an attached debugger with
semihosting enabled. Release uses nosys without semihosting. Neither build
redirects console output into the UART data stream.

Debug and Release application compilation and links against the complete
LPC11U35 library pass. Physical UART testing remains pending.
