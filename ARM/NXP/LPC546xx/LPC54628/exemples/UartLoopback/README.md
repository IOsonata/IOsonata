# LPC54628 UART loopback

Build the matching Debug or Release IOsonata_LPC54628 library, then import
`ioc` into IOcomposer. The project links `exemples/uart/uart_loopback.cpp`
and echoes received data through the common UART interface.

The pin configuration matches LPCXpresso54628 UartPrbsTxTest: Flexcomm 0,
P0.29 RX and P0.30 TX, both function 1, connected to LPC-Link2 VCOM.
Connect the board's LPC-Link2 USB port to the host and select its serial
port. No RX-to-TX jumper is needed. For a board using an external 3.3 V
USB-UART adapter instead, connect adapter TX to P0.29, adapter RX to P0.30,
and a common ground; avoid connecting two transmitters to the same signal.

Use 1000000 baud, 8N1, no flow control. The default uses interrupt-driven
RX/TX with 256-byte FIFOs. UART DMA is unsupported and disabled. Set
UART_INT_MODE to false to exercise polling operation.

From the repository root, with pyserial installed:

```
python3 Python/uartprbs_lb.py --baud 1000000 --port /dev/cu.usbmodemDEVICE
```

Replace the port with the board's serial port. Check for sustained echo
with no errors or lost bytes. Stop with Ctrl-C.

Debug uses semihosting and requires an attached debugger with semihosting
enabled. Release uses nosys without semihosting. Neither build redirects
console output into the UART data stream.

Debug and Release application compilation and firmware links with the
production startup, UART driver, and linker script pass. Physical UART
loopback and throughput measurements remain pending.
