# LPC tests

Run all LPC host regression suites:

```sh
python3 tests/lpc/run.py
```

Each subfolder also has its own `run.py` and README for focused checks:

- `clock`: LPC11Uxx clock decoding and linkage.
- `gpio`: LPC11Uxx pin configuration and interrupts.
- `uart`: shared LPC11Uxx/LPC17xx UART.
- `ssp`: shared SSP rates and LPC11Uxx transfers.
- `i2c`: shared LPC11Uxx/LPC17xx I2C master.
- `timer`: LPC11Uxx timers and generic timer helpers.
- `lpc546xx`: LPC546xx timer, I2C and SPI.
- `startup`: LPC11U35 linker layout and checksum arithmetic.
- `iap`: LPC11Uxx IAP compatibility and interrupt state.
- `iap_shared`: shared IAP commands and interrupt state.

The host suites require Python 3, GCC/G++, and GNU binutils. They use temporary
build directories and can be launched from any working directory. Register
models and host linker checks do not replace firmware builds or hardware tests.
