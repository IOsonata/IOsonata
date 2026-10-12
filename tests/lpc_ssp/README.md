# Legacy LPC SSP rate checks

Run from the repository root with Linux, Python 3 and g++:

```sh
python3 tests/lpc_ssp/run.py
```

The common rate setter is compiled with each family's vendor CMSIS register
type. An independent oracle evaluates all 127 * 256 legal prescaler/serial
divider pairs for each requested rate. Tests include zero, below-range,
above-range and UINT32_MAX requests, several source clocks, preserved frame
format and enable state, and refusal to change a busy or queued transmitter.
Zero requests select the minimum achievable SCK; rates are reported rounded
to integer Hz.

The LPC11Uxx test also exercises SPIInit with SSP1CLKDIV still zero while
initializing SSP0, and with the CPU clock divided below the main clock.
It checks both SSP instances and invalid device numbers. Tests map peripheral
memory and use the existing UART tests' CMSIS core shim; they do not simulate
SSP FIFO transfers or validate electrical timing. UBSan is enabled.

This change fixes rate selection and uses vendor SSP types. LPC17xx still
has no SPIInit implementation. The existing LPC11Uxx transfer paths, chip
select handling, clock polarity, lifecycle hooks and unsupported mode
handling need separate fixes before the SPI port can be considered complete.

Hardware reference: NXP UM10462, clock diagram and SSP chapter, especially
CR0 SCR and CPSR CPSDVSR. SCK = PCLK / (CPSDVSR * (SCR + 1)); CPSDVSR is even
from 2 through 254, and SCR is from 0 through 255.
