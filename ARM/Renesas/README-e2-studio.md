# IOsonata Renesas e2 studio projects

The `e2` directories contain Eclipse CDT / GNU Arm project configurations
for Renesas e2 studio. They preserve the existing IOsonata MCU-centric source
links and Debug/Release settings from the corresponding `ioc` configurations.

## ARM projects

- `RA4M1/lib/e2` plus Blinky, TimerDemo, UartLoopback and UartPrbsTxTest
  example projects in their respective `exemples/<name>/e2` directories.
- `RE01/RE01_1500KB/lib/e2` plus Blinky, TimerDemo, UartRetargetDemo,
  UartPrbsTxTest, I2CMasterDemo, SPIMasterDemo, PulseTrain and
  UartSlipPrbsRxTest example projects.
- Existing `RISCV/Renesas/R9A02/R9A02G021/lib/e2` is separate and uses
  its own RISC-V toolchain.

Import the `lib/e2` project and desired example `e2` projects with
**File > Import > Existing Projects into Workspace**. Install an e2 studio
version with the Eclipse CDT managed-build / GNU Arm Embedded toolchain
integration and select an Arm GNU toolchain compatible with each target.
The generated projects are renamed with the `_e2` suffix to coexist
with the IOcomposer projects. Build the matching library configuration
before the example; the copied linker search paths select `lib/e2`.

RE01 configurations distinguish CFB, CFP and DBN package variants; select
the same variant and Debug/Release configuration in library and example.
Use application-local `board.h` for wiring. Library projects add no board
initialization.

These are **portable CDT import configurations**, not projects generated
by Renesas FSP/Smart Configurator. They have not been compiled inside licensed
or separately installed e2 studio, and executable/debugger/device-pack setup
must be validated with the installed IDE and the target hardware.
