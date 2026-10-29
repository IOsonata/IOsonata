# SDRAM User Guide

`Sdram` (`include/coredev/sdram.h`) sets up an SDRAM on the external memory
controller of the MCU: the controller for the part, the power up sequence of
the part, then the memory mapped in the address space. C data `SdramDev_t`,
C init `SdramInit()` in the MCU port, C++ class `Sdram`.

## Use

```cpp
#include "coredev/sdram.h"

static Sdram s_Sdram;

// cfg contains the complete pin map and timings for the fitted memory.
static bool InitSdram(const SdramCfg_t &cfg)
{
	return s_Sdram.Init(cfg);
}
```

Use the complete configuration in
[ext_ram_test.cpp](../exemples/misc/ext_ram_test.cpp) with the target's
[board.h](../ARM/NXP/LPC546xx/LPC54628/exemples/SdramTest/src/board.h).
After successful initialization, `Base()` returns the mapped address,
`Size()` its byte capacity and `Rate()` the selected clock. Keep the pin map
alive. Do not use a partially initialized timing structure as a board setup.

The configuration is the organization and the timings of the data sheet of
the part: banks, row and column address bits, data width, CAS latency, and
tRCD, tRP, tRAS, tRC, tRFC, tRRD, tWR, tAPR, tSREX, tXSR in ns, tMRD in
clocks. Several parts side by side make the data bus (`BusWidth` /
`DevWidth`). `RefreshNs` is the time between two refresh commands: 64 ms /
4096 rows gives 15625 ns.

The SDRAM clock is the closest the controller makes to `Rate`; `Rate()` gives
it. The timings are converted to clocks of that rate, rounded up. The mode
register gets the CAS latency and a sequential burst chosen by the port.
`Init` returns false, the controller untouched, for an organization or a
value the controller does not take. Run `Init` before any access to the
memory, from code and data in the internal memories.

| MCU port | Controller | Notes |
| --- | --- | --- |
| LPC546xx | EMC (`ARM/NXP/LPC546xx/src/emc_lpc546xx.cpp`) | one SDRAM on EMC_DYCS0 to EMC_DYCS3 (`DevNo` 0 to 3), 16 or 32 bit bus, the 16 Mb to 512 Mb parts of the user manual table (x32 on a 32 bit bus only), 128 MB per chip select, CAS latency 1 to 3, tRCD 1 to 3 clocks, tMRD 1 to 16 clocks; burst of 8 on a 16 bit bus, 4 on a 32 bit bus; the EMC clock is the CPU clock divided by 1 to 256 |

## Example

`exemples/misc/ext_ram_test.cpp` sets up the SDRAM from the `SDRAM_` macros of
`board.h` (`SDRAM_PINS`, `SDRAM_RATE`, the organization and the timings) and
checks the data lines, the address lines, the byte lanes and the whole
memory. `exemples/display/fb_display_demo.cpp` sets up the SDRAM of the frame
buffer from the same macros. See the LPCXpresso54628
[SdramTest](../ARM/NXP/LPC546xx/LPC54628/exemples/SdramTest/README.md).

## Build and run

1. Build `ARM/NXP/LPC546xx/LPC54628/lib/ioc` in the desired configuration.
2. Import `ARM/NXP/LPC546xx/LPC54628/exemples/SdramTest/ioc` and use the
   matching Debug or Release configuration.
3. Keep startup code, stack and test buffers in internal memory until SDRAM
   initialization succeeds. A linker placement alone does not initialize SDRAM.
4. Enable semihosting for Debug. Release reports over UART at 1000000 baud, 8N1.
5. Check the reported address, capacity and clock, then the final test result.

The supplied memory test overwrites the entire SDRAM. Do not run it over a
live framebuffer, stack, heap or another application's data. For a product,
reserve non-overlapping regions for each user of the mapped memory.

## Validation

`tests/lpc/lpc546xx/emc_test.cpp` runs the LPC546xx port on a register model
of the EMC and of the SDRAM power up sequence: the register values for each
part of the user manual table on both bus widths and each chip select, the
mode register and its row address, the base and size, the timings, the clock
and the configurations refused. 42 of 42 mutants of the port are killed.

No hardware result yet.
