# ESP32-C6 UartLoopback

ESP32-C6-DevKitC-1 N8, UART0 through the board USB-to-UART bridge:
GPIO16 TX, GPIO17 RX, 1,000,000 baud, 8N1, interrupts enabled, DMA disabled.
Use the bridge USB connector; native USB Serial/JTAG is not this UART path.

The managed project links the shared `exemples/uart/uart_loopback.cpp` example.
Only pin assignments and UART options are in `src/board.h`.

Import `ioc` in IOcomposer and build the ESP32-C6 library and example with
matching Debug or Release configurations. Flash the resulting image through
IOcomposer. Use Release for both projects when measuring throughput.
The project selects `rv32imc_zicsr`, `ilp32`, and the C6 linker script.

Run `python3 Python/uartprbs_lb.py --port <bridge-port> --baud 1000000 --window 256`.
This echoes host data in software; do not short TX and RX together.
Compare errors and sustained throughput using the same host window.

Define `DEMO_C` to exercise the shared example's C UART API path.
The default uses the C++ wrapper over the same driver.
Hardware validation on C6 remains pending.
