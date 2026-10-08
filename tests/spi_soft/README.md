# SPISoft host tests

Run from the repository root:

```sh
g++ -std=c++17 -Wall -Wextra -fsanitize=address,undefined -g \
  -Itests/spi_soft/shim -Iinclude \
  tests/spi_soft/test_spi_soft.cpp src/coredev/spi_soft.cpp \
  src/device_intrf.cpp -o /tmp/test_spi_soft
ASAN_OPTIONS=detect_leaks=0 /tmp/test_spi_soft
```

The test uses the production driver and generic DeviceIntrf implementation.
GPIO/delay shims check 104 combinations of clock mode, word width and bit
order, plus data values, exact in-place buffers, CS selection, command/RX
continuity, generic Write, reset/disable/power-off, rates, shared-data direction
and invalid configurations. It also links without a hardware SPIInit provider.
This is a logic test, not a clock-frequency or physical GPIO timing measurement.
