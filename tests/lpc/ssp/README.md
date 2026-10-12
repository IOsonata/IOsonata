# Legacy LPC SSP driver checks

Run from the repository root with Linux, Python 3 and g++:

```sh
python3 tests/lpc/ssp/run.py
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

The LPC11Uxx transfer test uses the real generic DeviceIntrf helpers and
models SSP FIFO/status side effects and GPIO chip-select transitions. It runs
at O0 and O2 with UBSan. Cases cover all four clock modes, 129-byte transfers,
command-to-read with continuous CS, first command response, 4-bit masking,
unaligned 16-bit buffers, odd-length rejection, FIFO timeouts, command failure,
recovery, final-busy timeout, stale RX draining, reset preservation, power-off,
enable reference counting, manual CS and rejected execution modes.

The LPC11Uxx implementation supports polling master, normal separate MOSI/MISO,
MSB-first, 4-16 bit frames, and GPIO or application-controlled chip selects.
For frames above 8 bits, buffers contain little-endian two-byte words and must
have even byte lengths. Both bytes may be unaligned. Receive clocks repeat the
configured DummyByte in both halves of a wide frame. GPIO chip selects must
use IOPINOP_GPIO and output direction in the pin map. Alternate signal pins
use IOPINOP_FUNCn for hardware function n. Reset/debug pins may be repurposed
as GPIO CS, but doing so removes their reset/debug function.

Slave, interrupt, DMA, LSB-first and other physical interfaces are rejected.
Rate changes during a selected transaction retain the existing rate. A stalled
transfer returns its completed byte count (zero if the final idle wait fails)
and Stop resets the peripheral before releasing CS. The generic Start/Stop
helpers continue to own the busy flag. PowerOff requires full reinitialization.

LPC17xx still has no SPIInit implementation. These host tests and ARM compiler
checks do not establish a working board port. Pin muxing, electrical timing,
and transfers on real hardware still require validation.

Hardware reference: NXP UM10462, clock diagram and SSP chapter, especially
CR0 SCR and CPSR CPSDVSR. SCK = PCLK / (CPSDVSR * (SCR + 1)); CPSDVSR is even
from 2 through 254, and SCR is from 0 through 255.

LPC11Uxx initialization validates the SCK/MISO/MOSI routes for the selected
controller before touching registers, ownership, or pins. It rejects duplicate
pads, including a CS that would replace a bus signal. Route checks follow
UM10462 chapter 7, tables 67, 69-71, 82-83, 100, 105-107 and 114.

| Controller | SCK | MISO | MOSI |
| --- | --- | --- | --- |
| SSP0 | P0.6 FUNC2, P0.10 FUNC2, P1.29 FUNC1 | P0.8 FUNC1 | P0.9 FUNC1 |
| SSP1 | P1.15 FUNC3, P1.20 FUNC2 | P0.22 FUNC3, P1.21 FUNC2 | P0.21 FUNC2, P1.22 FUNC2 |

Choose pins bonded out on the actual package. Tests exercise every combination
above and check that invalid functions, swapped data pins, wrong-controller
maps and overlapping CS pins leave registers, pin calls and driver state
unchanged. Rate tests now use real routes for each controller.
