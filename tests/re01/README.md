# RE01 port regression checks

Run the Linux host checks from the repository root:

```sh
make -C tests/re01 test
```

The test compiles the real RE01 port and generated register header. A small
CMSIS shim replaces CPU instructions and NVIC operations; anonymous memory
backs the peripheral addresses. It checks clock reporting and divider bounds,
ICU ownership and reserved events, IRQ8/9 routing, GPIO validation and failed
allocation cleanup, TMR resume, AGT reset/divider limits, comparator allocation
and reuse, UART polling and interrupt paths, SCI9 module control, repeated
initialization, RX drops, error clearing, and partial IRQ allocation cleanup.
The baud-rate check uses an independent exhaustive BRR/MDDR calculation.

Build and link Blinky, TimerDemo and UartPrbsTxTest for Cortex-M0+ with both
normal linker scripts, then inspect the resulting ELF sections and option bytes:

```sh
python3 tests/re01/build_examples.py
```

Use `--tool-prefix /path/to/bin/arm-none-eabi-` if the compiler is not on PATH.
`--package DBN`, `--package CFB` or `--package CFP` selects the package define.
The check requires vectors below 0x400, 64 erased option bytes at 0x400,
code/data load addresses at or above 0x440, and heap/stack limits inside RAM.
These are direct compiler/linker checks of the example sources, not an
IOcomposer or Eclipse project build. The script uses newlib-nano and nosys.

Validated with GCC 14.3.Rel1 for all three package defines. The affected RE01
sources compile without warnings. Existing shared `ARM/src/iatomic.c` builtin
declaration warnings and example printf/unused-variable warnings are still
reported by the example builds.

The host test does not emulate FIFO side effects, oscillator stabilization, voltage
transitions, asynchronous AGT stop acknowledgement, interrupt timing, or flash
programming. Board validation is still required for normal/boost startup, PLL
fallback, SCI loopback and burst traffic, GPIO wake, timer rate/reset/resume,
and simultaneous AGT compare events. I2C/SPI support is outside this repair pass.

Build the real stage-0 boot example and a Blinky application with the DFU
linker scripts, then run the boot and target flash routines in Unicorn:

```sh
python3 -m pip install cryptography pyelftools unicorn
python3 tests/re01/build_dfu.py
```

The same `--tool-prefix` and `--package` options apply. `--build-only` skips
emulation. The script supplies an ephemeral P-256 public key and a 32 MHz
RC oscillator configuration to the boot example, independently signs an
MCUboot-format fixture, and verifies option memory, boot/application load
addresses, reserved recovery RAM, and placement of flash routines in RAM.
GCC 14.3.Rel1 builds both images for DBN, CFB and CFP (six images). Shared
atomic builtin warnings and the boot linker's RAM code RWX warning remain.

The 11 emulator scenarios cover empty, valid, tampered and legacy records;
payload writes from flash and unaligned RAM; program errors and retries;
all 384 erase-block addresses, neighbouring block isolation and all-ones
erase; address/length limits; exact clock bounds and MHz rounding; restoration
of protection, flash interrupts, VTOR and PRIMASK; DBFULL/command timeouts;
and RAM reset when forced stop or return to read mode fails. The model rejects
flash fetches/reads during P/E mode and checks RAM NMI/HardFault vectors.
SystemInit is skipped; this is a command model, not hardware validation of
flash timing, voltage, startup or asynchronous exception delivery.

Flash geometry and commands follow Renesas's official
[RE01 driver package](https://github.com/renesas/re-driver-package/tree/d67d8f1410421e33923e65a776add5401fbb11b8/SDK_RE01_1500KB/RE01_1500KB_DFP/Device/Driver):
`Include/r_flash_re01_1500kb.h` defines 384 uniform 4 KB blocks and 8/256-byte
programming; `Src/r_flash/r_flash_lowlevel.c` defines the FACI address,
command sequence and 1-32 MHz ICLK requirement. The accompanying driver
specification is R01AN4768EJ0103, revision 1.03. DFU uses 256-byte programming
and retains the existing boot, record and application addresses.

A DFU boot project must select ICLK between 1 and 32 MHz, for example by
overriding `g_McuOsc` with a 32 MHz RC configuration and `bUSBClk = false`.
The port's weak default is 48 MHz; flash erase/write calls reject that clock.
The target does not switch clocks during a flash operation.

The record magic now occupies 256 bytes (`__dfu_state_unit = 256`). Records
made with the previous 128-byte assumption are refused. After installing the
updated boot, reupload the application or regenerate its factory record with
the new state unit. Keep the recovery path available during this migration.
