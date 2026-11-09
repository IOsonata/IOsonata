# ESP32C61 bare-metal bring-up

Import `lib/ioc` and either `exemples/UartPrbsTxTest/ioc` or
`exemples/UartLoopback/ioc` into IOcomposer. Build the library and example
with matching Debug or Release configurations. Both use `rv32imc_zicsr`
and `ilp32`; no ESP-IDF runtime is required.

The examples link the shared generic UART applications. Connect a 3.3 V UART
adapter RX to GPIO4, TX to GPIO5, and common ground. These are external
adapter pins, not an assumed onboard USB bridge. Edit only `src/board.h`
for board wiring. Tests use 1,000,000 baud, 8N1, interrupts, no DMA.

Flash the generated raw `.bin` at offset zero using IOcomposer Direct Boot.
The linker supplies the two ROM magic words and entry at `0x42000008`.
The current layout allows 2 MiB flash and reserves upper SRAM for ROM.
Do not wrap this binary in an ESP-IDF application image.

Startup keeps the ROM clock configuration, disables boot watchdogs, enables
the delay counter, and installs the interrupt matrix/vector handler before
applications run. The core frequency is decoded from hardware.

This library contains startup, GPIO, UART, and their generic dependencies.
Timer and other peripheral integration is still pending. Build and host MMIO
tests do not establish hardware validation; TX must be checked before RX.

The default crystal descriptor is 40 MHz. For a board with a different
crystal, supply `MCUOSC` in `board.h` using the existing `McuOsc_t` fields;
the generic application supplies `g_McuOsc` from that descriptor.

Register and memory references: Espressif ESP-IDF v5.5.1, per-chip
`clk_tree_ll.h`, `pcr_reg.h` or `system_reg.h`, watchdog registers,
`gpio_reg.h`, and `memory.ld.in`.

Reproduce build and image checks from the repository root:

```sh
python3 tests/esp32/build_startup.py --output /tmp/esp32-startup-build
```

Use `--tool-prefix /path/to/bin/riscv-none-elf-` if the compiler is not on PATH.
The script builds every source linked by the new library projects, then both
UART applications with C++ and C API selection. It checks the raw boot header,
entry, vector alignment, IRAM copy address, and stack boundary. It does not
run IOcomposer or emulate CPU/ROM execution. Host MMIO checks require Linux.
