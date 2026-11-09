# ESP32-C3 UART0 RX / loopback test

**Hardware:** ESP32-C3-DevKitM-1, USB cable to the on-board CP2102N.

**Serial path:** UART0 TX = GPIO21, UART0 RX = GPIO20, 8-N-1 at
**1,000,000 baud**. No jumper is needed: the built-in USB/UART bridge
provides both transmit and receive paths. Do **not** short GPIO21 to
GPIO20; that can contend with the bridge's TX output.

## Firmware

Open `RISCV/Espressif/ESP32C/ESP32C3/exemples/UartLoopback/ioc` in
IOcomposer / Eclipse Embedded CDT, select Release, and clean-build
the ESP32-C3 IOsonata library and this application from the same branch.

The project links the existing **generic**
`exemples/uart/uart_loopback.cpp`. It does not copy or modify it.
The default C++ `UART` interface runs in interrupt mode with caller-owned
256-byte RX/TX CFIFOs and DMA disabled. The example has no console
output: UART0 is used entirely for binary echo.

Flash the application with the usual ESP32-C3 tool. Then close any
serial monitor or flashing program still holding the CP2102N port.

## Host acceptance

Python 3 and `pyserial` are required on the host:

```sh
python3 -m pip install pyserial
python3 RISCV/Espressif/ESP32C/ESP32C3/exemples/UartLoopback/tools/verify_uart_loopback.py --port /dev/cu.SLAB_USBtoUART --seconds 30
```

On macOS, identify the actual device with `ls /dev/cu.*`. On Windows,
replace the port with `COM5` or the appropriate assigned COM number.

The host script ignores the initial ROM boot output, synchronizes on
a binary 4-byte echo, then checks individual frames containing nulls,
0xFF, 0x55, 0xAA, CR/LF, high-bit values, and deterministic random
payloads of various lengths. It then runs a sustained **30-second
stop-and-wait** binary echo with a 64-byte frame, verifying every byte.
Use `--seconds 60` for a longer run and `--stress` to add unpaced
256-, 511- and 1024-byte frames.

Reported rate is **stop-and-wait host echo throughput**, not maximum
one-direction UART throughput. A failed large burst is valuable evidence
of a UART RX overflow, FIFO scheduling issue, or host-transfer limit
and should not be ignored.

For completeness, the generic `uart_loopback.cpp` also supports
its existing C mode via the `DEMO_C` compile-time symbol. The provided
managed-project configuration tests the C++ path first.

## Status

**Prepared for hardware testing, not yet validated.** The prior
UART PRBS TX demo and CryptoTest console confirm parts of UART0 TX,
but they do not establish UART0 interrupt-driven RX correctness.
No UART driver was changed by this project.
