# ESP32C61 UartPrbsRxTest

Uses the shared `exemples/uart/uart_prbs_rx.cpp`. Set connections in `src/board.h`.
Build the matching library and example in the same Debug or Release configuration.
Flash the binary at offset 0. UART uses 1,000,000 baud, 8N1, no flow control.

See [the example guide](../../../EXAMPLES.md) for peer hardware and host commands.
Hardware validation is pending.
