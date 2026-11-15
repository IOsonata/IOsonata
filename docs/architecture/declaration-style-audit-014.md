# Declaration-style audit for IOsonata 0.14

This is a repository-wide **worklist**, not a claim that all files have
been reviewed. The initial scope is files touched during the earlier power,
clock, PEI and MCU driver work.

## Mandatory coding rules

- Declare local variables at the **beginning of the function**, before
  executable statements. A separate block, conditional, or loop is not a
  new place for local declarations.
- Use tab indentation, ASCII, one instruction per line, no `goto`.
- Do not add global objects or symbols when function-local/static scope is
  sufficient. Keep interfaces explicit and avoid unnecessary cross-file
  declarations.
- Do not introduce `extern "C"` solely for debugger symbol naming.
  Where C implementation is appropriate, use C source instead.
- Keep this mechanical style cleanup separate from functional changes and
  validate all applicable target variants before merge.

## Source findings

The first source inspection identified local declarations away from function
entry in these existing files:

| File | Representative location before cleanup | Status |
| --- | --- | --- |
| `ARM/Nordic/src/system_power_nrfx.cpp` | `SystemResetReason()` inner block, `SystemRamRetain()` loop and `SystemPowerOff()` after a guard | First cleanup committed |
| `ARM/Nordic/nRF52/src/system_power_nrf52.cpp` | `SystemResetReason()` inner block and `SystemRamRetain()` retention loop | First cleanup committed |
| `ARM/ST/src/system_power_stm32.cpp` | `SystemPowerOff()` after peripheral clock access and inside per-pin loop; reset mapping loop | First cleanup committed |
| `ARM/Microchip/SAMD/src/uart_samd.cpp` | `SamdUartPollTx()` and other loops with local declarations | Pending systematic cleanup |
| `ARM/Microchip/SAMD/src/iopincfg_samd.c` | `EIC_Handler()` loop-local mask | Pending; pending hardware validation on separate PR #141 |
| `ARM/NXP/src/uart_lpcxx.cpp` | Nested calculation locals in baud-rate search loop | Pending |
| `ARM/Nordic/nRF53/src/pei_nrf53.c` | Must inspect all PEI changes, beyond declaration regex | Pending |
| `ARM/Nordic/src/system_clock_nrfx.cpp` | Must inspect functions and global symbol scope, not just sampled patterns | Pending |

## Current commits on this branch

The first three corrections move existing locals to the entry of their
functions without intentionally changing algorithms or hardware behavior.
They are **not compiled or hardware-validated yet**. MCU/SoftDevice/SDK
variant builds must pass before squash-merge.

## Scope and process

1. Finish full source enumeration for all MCU power and clock sources,
   then PEI/event drivers, UART, SPI/I2C, timer/RTC and remaining drivers.
2. Audit **global/static symbol necessity** separately from local-variable
   declaration placement. Global symbol decisions require call-site and
   linkage review, not just moving lines.
3. Apply small file-family commits only after auditing every function in
   each selected file. Avoid mixing driver changes with style cleanup.
4. Add maintainable static checks rather than relying on permissive
   compiler acceptance; regex-only checks are candidate detectors, not
   authoritative C/C++ parsers.
5. Run relevant host tests and a target-specific build matrix before
   merging. Do not rewrite or invalidate the independently hardware-
   validated SAMD SW0 wake path in PR #141.

## Hardware evidence from user

SAMD11 onboard SW0/PA14 NMI Standby wake produced five consecutive
resumes, with counters `0x00000001` through `0x00000005`, then the
next `Entering Standby` output. This validates repeated resume for the
tested build; sleep current and separate SRAM retention checks are pending.
