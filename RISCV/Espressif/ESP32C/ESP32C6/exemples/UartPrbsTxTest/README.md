# ESP32-C6 UartPrbsTxTest

ESP32-C6-DevKitC-1 N8, UART0 through the board USB-to-UART bridge:
GPIO16 TX, GPIO17 RX, 1,000,000 baud, 8N1, interrupts enabled, DMA disabled.
Use the bridge USB connector; native USB Serial/JTAG is not this UART path.

The managed project links the shared `exemples/uart/uart_prbs_tx.cpp` example.
Only pin assignments and UART options are in `src/board.h`.

Import `ioc` in IOcomposer and build the ESP32-C6 library and example with
matching Debug or Release configurations. Flash the resulting image through
IOcomposer. Use Release for both projects when measuring throughput.
The project selects `rv32imc_zicsr`, `ilp32`, and the C6 linker script.

Run `python3 Python/uartprbs_rx.py --port <bridge-port> --baud 1000000`.
The shared example currently uses byte mode. The 8N1 wire limit is 100,000 B/s.
Validate TX before running UartLoopback.

Define `DEMO_C` to exercise the shared example's C UART API path.
The default uses the C++ wrapper over the same driver.
## Hardware result

ESP32-C6-DevKitC-1, UART driver at `ffe3e035`, Release, byte mode, 1 Mbaud,
interrupts enabled, DMA disabled, CPU on the 40 MHz crystal (direct boot):
`uartprbs_rx.py` reported 99,983 to 100,017 B/s per second, cumulative
average 99,996.54 B/s. The drop count stayed at 7 from the first report to
the end of the run.

Before that commit the TX start armed the TX empty interrupt on every call,
one interrupt per byte at this CPU clock, and the same test measured below
50,000 B/s.
